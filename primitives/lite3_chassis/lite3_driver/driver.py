#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Lite3 chassis primitive — topic-bridge driver (ADR-0005).

The Lite3 Motion Host is driven by the OFFICIAL DeepRobotics `transfer` package
(ROS 2, foxy branch adapted to Humble), which owns the Motion-Host UDP sockets
(`:43893` commands / `:43897` telemetry). This primitive is a pure ROS 2 topic
bridge into the robonix/atlas graph — it no longer speaks UDP:

  robonix/primitive/chassis/move      rpc    publishes a bounded Twist burst on
                                    /cmd_vel (the `transfer` node forwards it to
                                    the Motion Host), then an explicit zero.
                                    Velocity-only with ADR-0003 clamps.
  robonix/primitive/chassis/twist_in  topic_in  subscribe /cmd_vel. External
                                    Twist (Nav2) is consumed directly by
                                    `transfer` in the shared ROS 2 graph; the
                                    primitive only observes it for the contract —
                                    it must NOT republish onto /cmd_vel, or it
                                    would echo its own move() bursts.
  robonix/primitive/chassis/odom      topic_out  relay `transfer`'s /leg_odom2
                                    → /odom (nav_msgs/Odometry), verbatim.
  robonix/primitive/chassis/driver    lifecycle.

This supersedes the direct-Motion-Host-UDP bridge (ADR-0001) for the chassis:
the protocol fidelity notes there still describe the wire format, but socket
ownership moves to `transfer`. The Motion Host must be in AUTO mode (velocity
commands), not SDK mode, for this path to respond.
"""
from __future__ import annotations

import json
import os
import threading
import time

from robonix_api import Primitive, Ok
from tf2_msgs.msg import TFMessage  # type: ignore
from geometry_msgs.msg import TransformStamped  # type: ignore

# ── Provider instance ───────────────────────────────────────────────────────
# Onboarding §4.2: read the boot-injected instance name; fall back to the
# manifest default so a bare `python3 -m lite3_driver.driver` still works.
lite3 = Primitive(
    id=os.environ.get("RBNX_INSTANCE_NAME", "lite3_chassis"),
    namespace="robonix/primitive/chassis",
)

# ── Velocity safety limits (ADR-0003) — applied to the move() RPC path. ─────
# The Nav2 /cmd_vel path is clamped by the Motion Host's built-in limits and by
# Nav2's own controller params (see ADR-0005).
MAX_LIN_X = 0.6   # m/s, well under the documented motion ceiling
MAX_LIN_Y = 0.3   # m/s lateral
MAX_ANG_Z = 1.5   # rad/s yaw

# -- Direct Motion-Host action channel ------------------------------
# `transfer` only forwards /cmd_vel (velocity); action & mode commands are
# 12-byte MotionSimpleCMD frames sent straight to the Motion-Host command port
# (send-only socket -- no bind, so no clash with `transfer`'s telemetry bind).
DEFAULT_ROBOT_IP = "192.168.1.120"      # RK3588 (jy_exe) command endpoint
DEFAULT_ROBOT_PORT = 43893
CMD_AUTONOMOUS = 0x21010C03             # autonomous mode: respond to perception-host velocity
CMD_HEARTBEAT = 0x21040001              # keep-alive (>=2 Hz)
CMD_STAND = 0x21010202                  # stand up
CMD_STAND_DOWN = 0x21010203             # lie down
CMD_STOP = 0x21010C0B                   # stop current action (send value 0 AND 1)

_robot_ip = DEFAULT_ROBOT_IP
_robot_port = DEFAULT_ROBOT_PORT
_udp_sock = None

# ── Topic bridge defaults (official `transfer` package, ADR-0005) ───────────
DEFAULT_ODOM_SOURCE = "/leg_odom2"     # transfer publishes leg odom+twist here
DEFAULT_ODOM_TOPIC = "/odom"           # robonix odom capability output topic
DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"     # transfer subscribes here → Motion Host
DEFAULT_MOVE_DURATION_S = 1.0          # single move() burst length before stop

_move_duration_s = DEFAULT_MOVE_DURATION_S

# ── Bridge state ────────────────────────────────────────────────────────────
_running = threading.Event()
_running.set()

_move_lock = threading.Lock()
_move_target: tuple[float, float, float] | None = None  # (vx, vy, wz) clamped
_move_deadline = 0.0                                   # monotonic time
_move_active = False

odom_pub = None          # /odom publisher (nav_msgs/Odometry) — odom capability
tf_pub = None            # /tf publisher (tf2_msgs/TFMessage) — odom->base_link
tf_base_link = "base_link"  # child frame of the odom->base_link transform
odom_src_sub = None      # subscription on transfer's /leg_odom2 (kept alive)
cmd_vel_pub = None       # /cmd_vel publisher (geometry_msgs/Twist) — move() sink
cmd_vel_sub = None       # /cmd_vel subscription — twist_in capability (passive)
_last_twist = None       # last external Twist observed (telemetry only)


def _publish_twist(vx: float, vy: float, wz: float) -> None:
    from geometry_msgs.msg import Twist, Vector3  # type: ignore
    msg = Twist()
    msg.linear = Vector3(x=float(vx), y=float(vy), z=0.0)
    msg.angular = Vector3(x=0.0, y=0.0, z=float(wz))
    cmd_vel_pub.publish(msg)


def _send_simple(code: int, param: int = 0) -> None:
    """Send a 12-byte MotionSimpleCMD {code, param, type=0} to the Motion Host."""
    import socket
    import struct
    global _udp_sock
    try:
        if _udp_sock is None:
            _udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _udp_sock.sendto(struct.pack("<iii", code, param, 0), (_robot_ip, _robot_port))
    except Exception:
        pass


def _begin_move(vx: float, vy: float, wz: float, duration_s: float) -> None:
    """Arm a velocity burst: /cmd_vel is streamed at ~10 Hz for `duration_s`,
    then an explicit zero is published. All values hard-clamped (ADR-0003)."""
    global _move_target, _move_deadline, _move_active
    with _move_lock:
        _move_target = (
            max(-MAX_LIN_X, min(MAX_LIN_X, vx)),
            max(-MAX_LIN_Y, min(MAX_LIN_Y, vy)),
            max(-MAX_ANG_Z, min(MAX_ANG_Z, wz)),
        )
        _move_deadline = time.monotonic() + max(0.1, duration_s)
        _move_active = True


def _relay_loop() -> None:
    """Publish the active move() burst to /cmd_vel at ~10 Hz, then a zero.

    Silent when no burst is armed so Nav2 (the other /cmd_vel writer, consumed
    directly by `transfer`) is not flooded or overridden.
    """
    global _move_active, _move_target  # assigned below → must be module-global
    last_pub = 0.0
    while _running.is_set():
        now = time.monotonic()
        with _move_lock:
            active = _move_active
            target = _move_target
            deadline = _move_deadline
        if active:
            if now < deadline:
                if cmd_vel_pub is not None and now - last_pub >= 0.1:
                    _publish_twist(*target)
                    last_pub = now
            else:
                # Burst over → one explicit stop, then silence. The Motion Host
                # stops the robot when velocity packets stop arriving, so the
                # zero must be sent before we go quiet.
                if cmd_vel_pub is not None:
                    _publish_twist(0.0, 0.0, 0.0)
                with _move_lock:
                    _move_active = False
                    _move_target = None
        time.sleep(0.01)


def _on_cmd_vel(msg):
    """External Twist on /cmd_vel — consumed by `transfer` in the shared graph.

    Observed only for the twist_in contract; NOT fed back into the output (that
    would create an echo loop with our own move() bursts on the same topic).
    """
    global _last_twist
    _last_twist = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))


def _on_leg_odom(msg):
    """Relay `transfer`'s /leg_odom2 → /odom verbatim (frames/stamps preserved),
    and publish the odom→base_link dynamic TF (RTAB-Map SLAM / Nav2 input)."""
    if odom_pub is not None:
        odom_pub.publish(msg)
    if tf_pub is not None:
        try:
            ts = TransformStamped()
            # transfer's /leg_odom2 arrives with EMPTY frame_id — force odom.
            ts.header.stamp = msg.header.stamp
            ts.header.frame_id = "odom"
            ts.child_frame_id = tf_base_link
            ts.transform.translation.x = msg.pose.pose.position.x
            ts.transform.translation.y = msg.pose.pose.position.y
            ts.transform.translation.z = msg.pose.pose.position.z
            ts.transform.rotation = msg.pose.pose.orientation
            tf_pub.publish(TFMessage(transforms=[ts]))
        except Exception as e:
            print(f"[lite3_chassis] tf publish failed: {e}", flush=True)


# ════════════════════════════════════════════════════════════════════════════
# MCP: robonix/primitive/chassis/move — velocity-only burst
# ════════════════════════════════════════════════════════════════════════════
import chassis_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402


@lite3.mcp("robonix/primitive/chassis/move")
def move(req: "chassis_mcp.ExecuteMoveCommand_Request") -> "chassis_mcp.ExecuteMoveCommand_Response":
    """Publish a bounded velocity burst to /cmd_vel (`transfer` forwards it).

    Velocity-only (ADR-0003): only linear_x / linear_y / angular_z are honoured,
    hard-clamped to the safety limits. forward_m / rotate_deg / linear_z are
    REJECTED (not silently ignored) so a caller misreading the contract fails
    loudly rather than commanding an unintended motion. The burst length is
    MoveCommand.duration_sec when > 0, else the manifest default; the Motion
    Host stops the robot when velocity packets stop arriving, and this driver
    additionally publishes an explicit zero at the end of the burst.
    """
    cmd = req.command
    forward_m = float(getattr(cmd, "forward_m", 0.0))
    rotate_deg = float(getattr(cmd, "rotate_deg", 0.0))
    linear_z = float(getattr(cmd, "linear_z", 0.0))
    if forward_m != 0.0 or rotate_deg != 0.0 or linear_z != 0.0:
        return chassis_mcp.ExecuteMoveCommand_Response(
            status=std_msgs_mcp.String(
                data=json.dumps(
                    {"error": "move is velocity-only on Lite3; "
                              "forward_m/rotate_deg/linear_z rejected"}
                )
            ),
        )
    vx = float(getattr(cmd, "linear_x", 0.0))
    vy = float(getattr(cmd, "linear_y", 0.0))
    wz = float(getattr(cmd, "angular_z", 0.0))
    duration = float(getattr(cmd, "duration_sec", 0.0) or 0.0)
    if duration <= 0.0:
        duration = _move_duration_s
    # Lite3 requires autonomous mode (0x21010C03) for the perception host's
    # velocity to be honoured -- otherwise the handle/remote holds control.
    _send_simple(CMD_AUTONOMOUS)
    _begin_move(vx, vy, wz, duration)
    return chassis_mcp.ExecuteMoveCommand_Response(
        status=std_msgs_mcp.String(data=json.dumps({
            "status": "moving",
            "linear_x": vx, "linear_y": vy, "angular_z": wz,
            "duration_sec": duration,
            "limits": {"max_lin_x": MAX_LIN_X, "max_lin_y": MAX_LIN_Y, "max_ang_z": MAX_ANG_Z},
        })),
    )


# ════════════════════════════════════════════════════════════════════════════
@lite3.mcp("robonix/primitive/chassis/stand")
def stand(msg: "std_msgs_mcp.Empty") -> "std_msgs_mcp.String":
    """Stand the robot up. Robot must be lying down; takes ~2-7 s."""
    _send_simple(CMD_STAND)
    return std_msgs_mcp.String(data=json.dumps({"status": "stand_cmd_sent"}))


@lite3.mcp("robonix/primitive/chassis/down")
def down(msg: "std_msgs_mcp.Empty") -> "std_msgs_mcp.String":
    """Lie the robot down. Robot must be standing (force control)."""
    _send_simple(CMD_STAND_DOWN)
    return std_msgs_mcp.String(data=json.dumps({"status": "down_cmd_sent"}))


@lite3.mcp("robonix/primitive/chassis/stop")
def stop(msg: "std_msgs_mcp.Empty") -> "std_msgs_mcp.String":
    """Stop the current action (0x21010C0B -- protocol requires value 0 AND 1)."""
    _send_simple(CMD_STOP, 0)
    _send_simple(CMD_STOP, 1)
    return std_msgs_mcp.String(data=json.dumps({"status": "stop_cmd_sent"}))


# Lifecycle
# ════════════════════════════════════════════════════════════════════════════


@lite3.on_init
def init(cfg: dict):
    global odom_pub, odom_src_sub, cmd_vel_pub, cmd_vel_sub, _move_duration_s, _robot_ip, _robot_port
    cfg = cfg or {}
    _move_duration_s = float(cfg.get("move_duration_sec", DEFAULT_MOVE_DURATION_S))
    _robot_ip = str(cfg.get("robot_ip", DEFAULT_ROBOT_IP))
    _robot_port = int(cfg.get("robot_port", DEFAULT_ROBOT_PORT))
    odom_source = str(cfg.get("odom_source", DEFAULT_ODOM_SOURCE))
    odom_topic = str(cfg.get("odom_topic", DEFAULT_ODOM_TOPIC))
    cmd_vel_topic = str(cfg.get("cmd_vel_topic", DEFAULT_CMD_VEL_TOPIC))

    # /odom (nav_msgs/Odometry) — onboarding §6.1 requires odom. Relayed verbatim
    # from the official `transfer` package's /leg_odom2 (ADR-0005); the odom→
    # base_link dynamic TF comes from the message's child_frame_id, while the
    # static chain (base_link→TORSO→sensors) is published by robot_state_publisher.
    from nav_msgs.msg import Odometry  # type: ignore
    odom_pub = lite3.create_publisher(
        "robonix/primitive/chassis/odom",
        topic=odom_topic, msg_type=Odometry, qos="reliable",
    )
    # odom->base_link dynamic TF: RTAB-Map SLAM and Nav2 need this to project
    # sensor data into the odom frame. Published from each /leg_odom2 pose.
    global tf_pub
    tf_pub = lite3.create_publisher(
        "robonix/primitive/chassis/tf",
        topic="/tf", msg_type=TFMessage, qos="reliable", declare=False,
    )
    odom_src_sub = lite3.create_subscription(
        "robonix/primitive/chassis/odom_src",
        topic=odom_source, msg_type=Odometry,
        callback=_on_leg_odom, qos="reliable", declare=False,
    )

    # /cmd_vel — output sink for move() (transfer forwards it to the Motion
    # Host). Undeclared: it is the physical implementation of the move RPC, not
    # a robonix capability. `transfer` is the OTHER /cmd_vel writer when Nav2
    # drives the robot.
    from geometry_msgs.msg import Twist  # type: ignore
    cmd_vel_pub = lite3.create_publisher(
        "robonix/primitive/chassis/move_out",
        topic=cmd_vel_topic, msg_type=Twist, qos="reliable", declare=False,
    )
    # /cmd_vel subscription — twist_in capability (passive observer).
    cmd_vel_sub = lite3.create_subscription(
        "robonix/primitive/chassis/twist_in",
        topic=cmd_vel_topic, msg_type="Twist", callback=_on_cmd_vel, qos="reliable",
    )

    t = threading.Thread(target=_relay_loop, daemon=True)
    t.start()
    print(
        f"[lite3_chassis] init OK — topic bridge: {odom_source}→{odom_topic}, "
        f"move→{cmd_vel_topic} (burst {_move_duration_s}s)",
        flush=True,
    )
    return Ok()


@lite3.on_shutdown
def shutdown():
    global _running
    _running.clear()
    # Publish an explicit stop so the robot does not inherit a stale set-point.
    if cmd_vel_pub is not None:
        try:
            _publish_twist(0.0, 0.0, 0.0)
        except Exception:  # noqa: BLE001
            pass
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


if __name__ == "__main__":
    print("[lite3_chassis] starting — connecting to Atlas...", flush=True)
    lite3.run()
