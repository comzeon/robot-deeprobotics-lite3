#!/usr/bin/env python3
"""Lite3 chassis primitive — registered with Atlas via robonix-api."""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import struct
import sys
import threading
import time

from robonix_api import Primitive, Ok, Err, Deferred

# ── Protocol constants ──
ETH_CMD_FMT = '<III'
ETH_CMD_SIZE = 12
CMD_ROBOT_DATA = 0x00000906
CMD_PORT = 43893
STATE_PORT = 43897
ROBOT_IP = '192.168.2.1'

JOINT_NAMES = [
    'FL_HipX','FL_HipY','FL_Knee',
    'FR_HipX','FR_HipY','FR_Knee',
    'HL_HipX','HL_HipY','HL_Knee',
    'HR_HipX','HR_HipY','HR_Knee',
]

# ── Primitive instance ──
lite3 = Primitive(id="lite3_chassis", namespace="robonix/primitive/chassis")

# ── Global state ──
_state_sock = None
_cmd_sock = None
_running = threading.Event()
_running.set()
_last_state = {}
_last_cmd = None


def parse_robot_data(payload: bytes) -> dict:
    off = 0
    tick = struct.unpack_from('<I', payload, off)[0]; off += 4
    imu_raw = struct.unpack_from('<i9f', payload, off); off += 40
    imu = {
        'roll': imu_raw[1], 'pitch': imu_raw[2], 'yaw': imu_raw[3],
        'angvel_r': imu_raw[4], 'angvel_p': imu_raw[5], 'angvel_y': imu_raw[6],
        'acc_x': imu_raw[7], 'acc_y': imu_raw[8], 'acc_z': imu_raw[9],
    }
    joints = []
    for i in range(12):
        p, v, t, tmp = struct.unpack_from('<ffff', payload, off); off += 16
        joints.append({'name': JOINT_NAMES[i], 'pos': p, 'vel': v, 'trq': t, 'temp': tmp})
    contact = list(struct.unpack_from('<12d', payload, off))
    return {'tick': tick, 'imu': imu, 'joints': joints, 'contact': contact}


def build_joint_cmd(pos=0.0, vel=0.0, trq=0.0, kp=30.0, kd=1.0) -> bytes:
    return struct.pack('<fffff', pos, vel, trq, kp, kd)


def build_robot_cmd() -> bytes:
    return b''.join(build_joint_cmd() for _ in range(12))


def udp_loop():
    """Background thread: receive Lite3 state, send keep-alive commands."""
    global _state_sock, _cmd_sock, _last_state, _last_cmd

    _state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    _state_sock.settimeout(0.5)
    _state_sock.bind(('0.0.0.0', STATE_PORT))

    _cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    robot_ip = os.environ.get('LITE3_ROBOT_IP', ROBOT_IP)
    cmd_port = int(os.environ.get('LITE3_CMD_PORT', CMD_PORT))
    state_port = int(os.environ.get('LITE3_STATE_PORT', STATE_PORT))

    _last_cmd = build_robot_cmd()
    last_send = 0.0
    report_interval = 50
    tick_count = 0

    print(f"[lite3_chassis] UDP thread started: listen :{state_port} ← {robot_ip}", flush=True)

    while _running.is_set():
        try:
            data, addr = _state_sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break

        if len(data) < ETH_CMD_SIZE:
            continue
        code = struct.unpack_from('<I', data, 0)[0]
        if code != CMD_ROBOT_DATA:
            continue

        payload = data[ETH_CMD_SIZE:]
        if len(payload) < 332:
            continue

        _last_state = parse_robot_data(payload)
        tick_count += 1

        if tick_count % report_interval == 0:
            s = _last_state
            print(f"  [{s['tick']}] r={s['imu']['roll']:.1f}° "
                  f"p={s['imu']['pitch']:.1f}° y={s['imu']['yaw']:.1f}°",
                  flush=True)

        # Send keep-alive command every 200ms
        now = time.time()
        if now - last_send > 0.2:
            _cmd_sock.sendto(_last_cmd, (robot_ip, cmd_port))
            last_send = now

    _state_sock.close()
    _cmd_sock.close()
    print("[lite3_chassis] UDP thread stopped", flush=True)


# ── Lifecycle handlers ──

@lite3.on_init
def init(cfg: dict):
    global _last_cmd

    robot_ip = cfg.get('robot_ip', ROBOT_IP)
    cmd_port = cfg.get('cmd_port', CMD_PORT)
    state_port = cfg.get('state_port', STATE_PORT)

    os.environ['LITE3_ROBOT_IP'] = robot_ip
    os.environ['LITE3_CMD_PORT'] = str(cmd_port)
    os.environ['LITE3_STATE_PORT'] = str(state_port)

    # Capabilities: declare what this primitive offers
    lite3.declare_ros2_topic('robonix/primitive/chassis/odom', '/odom', qos='reliable')
    lite3.declare_ros2_topic('robonix/primitive/chassis/twist_in', '/cmd_vel', qos='reliable')

    # Start UDP thread
    t = threading.Thread(target=udp_loop, daemon=True)
    t.start()
    print(f"[lite3_chassis] init OK — UDP bridge to {robot_ip}:{cmd_port}", flush=True)
    return Ok()


@lite3.on_shutdown
def shutdown():
    global _running
    _running.clear()
    print("[lite3_chassis] shutdown", flush=True)
    return Ok()


@lite3.on_activate
def activate():
    print("[lite3_chassis] activated", flush=True)
    return Ok()


@lite3.on_deactivate
def deactivate():
    print("[lite3_chassis] deactivated", flush=True)
    return Ok()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-ip', default=ROBOT_IP)
    parser.add_argument('--cmd-port', type=int, default=CMD_PORT)
    parser.add_argument('--state-port', type=int, default=STATE_PORT)
    args, remaining = parser.parse_known_args()

    # Set defaults for cfg if not provided via env/manifest
    if 'LITE3_ROBOT_IP' not in os.environ:
        os.environ['LITE3_ROBOT_IP'] = args.robot_ip
    if 'LITE3_CMD_PORT' not in os.environ:
        os.environ['LITE3_CMD_PORT'] = str(args.cmd_port)
    if 'LITE3_STATE_PORT' not in os.environ:
        os.environ['LITE3_STATE_PORT'] = str(args.state_port)

    print(f"[lite3_chassis] starting — connecting to Atlas...", flush=True)
    lite3.run()
