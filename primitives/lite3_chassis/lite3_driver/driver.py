#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Lite3 chassis primitive — Capability-based driver.

Speaks the Lite3 Motion Host UDP protocol directly (no separate `transfer`
bridge process). Owns `robonix/primitive/chassis/*`:

  primitive/chassis/move      rpc   gRPC ExecuteMoveCommand — velocity-only,
                                    hard-clamped vx/vy/wz; no joint-position
                                    commands are ever emitted (ADR-0003).
  primitive/chassis/odom       topic_out  ROS 2 /odom (nav_msgs/Odometry)
  primitive/chassis/twist_in   topic_in   ROS 2 /cmd_vel
  primitive/chassis/ultrasound topic_out  ROS 2 /ultrasound (sensor_msgs/Range
                                    [2]) — published but NOT a mapping input.

Protocol fidelity mirrors DeepRoboticsLab/Lite3_ROS `protocol.hpp` and
`Jetson2Motion.cpp` field-for-field (`#pragma pack(4)`). Telemetry frames are
dispatched by packet length + `code` exactly as the C++ bridge's
`switch(recv_num_)`; commands are MotionSimpleCMD/MotionComplexCMD to :43893,
velocity as three ComplexCMD packets (codes 320/325/321, yaw negated). See
ADR-0001.
"""
from __future__ import annotations

import ctypes
import json
import math
import os
import socket
import threading
import time
from typing import Optional

from robonix_api import Primitive, Ok

# ── Provider instance ───────────────────────────────────────────────────────
# Onboarding §4.2: read the boot-injected instance name; fall back to the
# manifest default so a bare `python3 -m lite3_driver.driver` still works.
lite3 = Primitive(
    id=os.environ.get("RBNX_INSTANCE_NAME", "lite3_chassis"),
    namespace="robonix/primitive/chassis",
)

# ── Network defaults (DeepRobotics factory) ─────────────────────────────────
#   motion host  : 192.168.1.1   (commands → :43893, telemetry ← :43897)
#   SDK/perc host: 192.168.1.120 (peer that binds the sockets)
DEFAULT_ROBOT_IP = os.environ.get("LITE3_ROBOT_IP", "192.168.1.1")
DEFAULT_CMD_PORT = 43893
DEFAULT_STATE_PORT = 43897

# ── Velocity safety limits (ADR-0003) ───────────────────────────────────────
MAX_LIN_X = 0.6   # m/s, well under the documented motion ceiling
MAX_LIN_Y = 0.3   # m/s lateral
MAX_ANG_Z = 1.5   # rad/s yaw

ULTRASOUND_MIN_M = 0.28
ULTRASOUND_MAX_M = 4.50

# ── Joint names (for JointState publishing), DeepRobotics Leg/RF/LB/RB order. ──
JOINT_NAMES = (
    "LF_Joint", "LF_Joint_1", "LF_Joint_2",
    "RF_Joint", "RF_Joint_1", "RF_Joint_2",
    "LB_Joint", "LB_Joint_1", "LB_Joint_2",
    "RB_Joint", "RB_Joint_1", "RB_Joint_2",
)

# ════════════════════════════════════════════════════════════════════════════
# Motion Host UDP protocol — ctypes structs, _pack_ = 4 (protocol.hpp)
# ════════════════════════════════════════════════════════════════════════════


class SimpleCMD(ctypes.Structure):
    """12-byte command: cmd_code / cmd_value / type (3× int32)."""
    _pack_ = 4
    _fields_ = [
        ("cmd_code", ctypes.c_int32),
        ("cmd_value", ctypes.c_int32),
        ("type", ctypes.c_int32),
    ]


class ComplexCMD(ctypes.Structure):
    """20-byte command: SimpleCMD + 8-byte double data."""
    _pack_ = 4
    _fields_ = [
        ("cmd_code", ctypes.c_int32),
        ("cmd_value", ctypes.c_int32),
        ("type", ctypes.c_int32),
        ("data", ctypes.c_double),
    ]


class RobotState(ctypes.Structure):
    """Robot base state — pose, IMU, velocities, battery, ultrasound.

    Field order matches protocol.hpp struct RobotState exactly. Four fields
    are documented invalid placeholders (kept only for layout correctness):
    touch_down_and_stair_trot, is_charging, error_state, task_state.
    """
    _pack_ = 4
    _fields_ = [
        ("robot_basic_state", ctypes.c_int),
        ("robot_gait_state", ctypes.c_int),
        ("rpy", ctypes.c_double * 3),            # IMU angle, degrees
        ("rpy_vel", ctypes.c_double * 3),        # angular velocity, rad/s
        ("xyz_acc", ctypes.c_double * 3),        # acceleration, m/s^2
        ("pos_world", ctypes.c_double * 3),      # {x, y, yaw_rad}
        ("vel_world", ctypes.c_double * 3),      # world-frame velocity
        ("vel_body", ctypes.c_double * 3),       # body-frame {vx, vy, wz}
        ("touch_down_and_stair_trot", ctypes.c_uint),  # INVALID placeholder
        ("is_charging", ctypes.c_bool),          # INVALID placeholder
        ("error_state", ctypes.c_uint),          # INVALID placeholder
        ("robot_motion_state", ctypes.c_int),
        ("battery_level", ctypes.c_double),      # percentage
        ("task_state", ctypes.c_int),            # INVALID placeholder
        ("is_robot_need_move", ctypes.c_bool),
        ("zero_position_flag", ctypes.c_bool),
        ("ultrasound", ctypes.c_double * 2),     # {front, rear} metres
    ]


class ImuData(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("timestamp", ctypes.c_uint32),
        ("buffer", ctypes.c_float * 9),  # {rpy, rpy_vel, acc}
    ]


# Framed telemetry packets (code / size / cons_code header + payload).
def _received(extra_fields):
    _pack_ = 4
    return [
        ("code", ctypes.c_int),
        ("size", ctypes.c_int),
        ("cons_code", ctypes.c_int),
    ] + extra_fields


class RobotStateReceived(ctypes.Structure):
    _pack_ = 4
    _fields_ = _received([("data", RobotState)])


class RobotStateReceivedWithPolicy(ctypes.Structure):
    """Newer transfer package inserts robot_policy_state after gait_state,
    making the frame 4 bytes larger. Dispatch differentiates by length."""
    _pack_ = 4
    _fields_ = [
        ("code", ctypes.c_int),
        ("size", ctypes.c_int),
        ("cons_code", ctypes.c_int),
        ("robot_basic_state", ctypes.c_int),
        ("robot_gait_state", ctypes.c_int),
        ("robot_policy_state", ctypes.c_int),
        ("rpy", ctypes.c_double * 3),
        ("rpy_vel", ctypes.c_double * 3),
        ("xyz_acc", ctypes.c_double * 3),
        ("pos_world", ctypes.c_double * 3),
        ("vel_world", ctypes.c_double * 3),
        ("vel_body", ctypes.c_double * 3),
        ("touch_down_and_stair_trot", ctypes.c_uint),
        ("is_charging", ctypes.c_bool),
        ("error_state", ctypes.c_uint),
        ("robot_motion_state", ctypes.c_int),
        ("battery_level", ctypes.c_double),
        ("task_state", ctypes.c_int),
        ("is_robot_need_move", ctypes.c_bool),
        ("zero_position_flag", ctypes.c_bool),
        ("ultrasound", ctypes.c_double * 2),
    ]


class JointState(ctypes.Structure):
    _pack_ = 4
    _fields_ = [(n, ctypes.c_double) for n in JOINT_NAMES]


class JointStateReceived(ctypes.Structure):
    _pack_ = 4
    _fields_ = _received([("data", JointState)])


class HandleState(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("left_axis_forward", ctypes.c_double),
        ("left_axis_side", ctypes.c_double),
        ("right_axis_yaw", ctypes.c_double),
        ("goal_vel_forward", ctypes.c_double),
        ("goal_vel_side", ctypes.c_double),
        ("goal_vel_yaw", ctypes.c_double),
    ]


class HandleStateReceived(ctypes.Structure):
    _pack_ = 4
    _fields_ = _received([("data", HandleState)])


class ImuDataReceived(ctypes.Structure):
    _pack_ = 4
    _fields_ = _received([("data", ImuData)])


# Telemetry `code` values (Jetson2Motion.cpp Parse*).
ROBOT_STATE_CODE = 2305
JOINT_STATE_CODE = 2306
HANDLE_STATE_CODE = 2309
IMU_DATA_CODE = 0x010901

# Packet sizes — dispatch mirrors the C++ switch(recv_num_).
ROBOT_STATE_SIZE = ctypes.sizeof(RobotStateReceived)
ROBOT_STATE_WITH_POLICY_SIZE = ctypes.sizeof(RobotStateReceivedWithPolicy)
JOINT_STATE_SIZE = ctypes.sizeof(JointStateReceived)
HANDLE_STATE_SIZE = ctypes.sizeof(HandleStateReceived)
IMU_DATA_SIZE = ctypes.sizeof(ImuDataReceived)


class CmdCode:
    """Motion Host command codes (Jueying Lite3 Motion Host Interface)."""
    VEL_FORWARD = 320   # linear x  (ComplexCMD)
    VEL_YAW = 321        # angular z — sent negated (bridge convention)
    VEL_LATERAL = 325    # linear y


# ════════════════════════════════════════════════════════════════════════════
# Protocol encode/decode
# ════════════════════════════════════════════════════════════════════════════


def encode_velocity(vx: float, vy: float, wz: float) -> list[bytes]:
    """Encode a base velocity as the three ComplexCMD packets the Lite3 expects.

    vx forward, vy left, wz counter-clockwise positive; yaw is negated to match
    the DeepRobotics bridge convention (Jetson2Motion.cpp target_addr_ send).
    """
    return [
        bytes(ComplexCMD(CmdCode.VEL_FORWARD, 8, 1, float(vx))),
        bytes(ComplexCMD(CmdCode.VEL_LATERAL, 8, 1, float(vy))),
        bytes(ComplexCMD(CmdCode.VEL_YAW, 8, 1, float(-wz))),
    ]


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# ════════════════════════════════════════════════════════════════════════════
# Driver state
# ════════════════════════════════════════════════════════════════════════════

_state_lock = threading.Lock()
_latest_robot: Optional[RobotState] = None
_latest_joints: Optional[JointState] = None
_latest_imu: Optional[ImuData] = None
_last_state_seq = 0  # monotonic odom sequence counter

_cmd_sock: Optional[socket.socket] = None
_state_sock: Optional[socket.socket] = None
_cmd_target: tuple[str, int] = ("", 0)
_running = threading.Event()
_running.set()

# Twist command received via /cmd_vel or the move() rpc; no input ⇒ stop.
_twist_lock = threading.Lock()
_target_vx = 0.0
_target_vy = 0.0
_target_wz = 0.0
_last_cmd_recv = 0.0       # monotonic time of last velocity reception
_cmd_timeout_s = 0.5       # no velocity for this long ⇒ zero velocity sent

odom_pub = None          # /odom publisher (nav_msgs/Odometry)
joint_pub = None          # /joint_states publisher (sensor_msgs/JointState)
us_front_pub = None       # /ultrasound/front (sensor_msgs/Range)
us_rear_pub = None        # /ultrasound/rear  (sensor_msgs/Range)
cmd_vel_sub = None       # /cmd_vel subscription


def _set_velocity(vx: float, vy: float, wz: float) -> None:
    global _target_vx, _target_vy, _target_wz, _last_cmd_recv
    vx = clamp(vx, -MAX_LIN_X, MAX_LIN_X)
    vy = clamp(vy, -MAX_LIN_Y, MAX_LIN_Y)
    wz = clamp(wz, -MAX_ANG_Z, MAX_ANG_Z)
    with _twist_lock:
        _target_vx = vx
        _target_vy = vy
        _target_wz = wz
        _last_cmd_recv = time.monotonic()


def _current_velocity() -> tuple[float, float, float]:
    """Return (vx, vy, wz) to send, applying the no-input→stop timeout."""
    with _twist_lock:
        vx = _target_vx
        vy = _target_vy
        wz = _target_wz
        last = _last_cmd_recv
    if time.monotonic() - last > _cmd_timeout_s:
        return 0.0, 0.0, 0.0
    return vx, vy, wz


# ════════════════════════════════════════════════════════════════════════════
# UDP receive/parse + odom publish thread
# ════════════════════════════════════════════════════════════════════════════


def _apply_robot_state(frame) -> Optional[RobotState]:
    """Store the latest RobotState from either wire layout variant.

    `RobotStateReceived` nests the payload as `frame.data` (a RobotState);
    the newer `RobotStateReceivedWithPolicy` flattens the same fields onto the
    frame itself. Copy field-by-field by name so both parse identically.
    """
    global _latest_robot, _last_state_seq
    if frame.code != ROBOT_STATE_CODE:
        return _latest_robot
    state = RobotState()
    src = frame.data if hasattr(frame, "data") else frame
    for name, _ in RobotState._fields_:
        try:
            setattr(state, name, getattr(src, name))
        except AttributeError:
            pass
    with _state_lock:
        _latest_robot = state
        _last_state_seq += 1
    return state


def _publish_robot_state() -> None:
    """Publish /odom and /ultrasound from the latest RobotState. Kept on the
    socket thread to avoid a second lock on writers."""
    with _state_lock:
        state = _latest_robot
    if state is None or odom_pub is None:
        return
    try:
        from nav_msgs.msg import Odometry  # type: ignore
        from geometry_msgs.msg import Twist, Vector3  # type: ignore
        from sensor_msgs.msg import Range  # type: ignore
        from std_msgs.msg import Header  # type: ignore

        now = _stamp()
        x = state.pos_world[0]
        y = state.pos_world[1]
        yaw = state.pos_world[2]  # already radians per the protocol field
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = os.environ.get("LITE3_BASE_FRAME", "base_link")
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = _yaw_quat(yaw)
        odom.twist.twist = Twist()
        odom.twist.twist.linear = Vector3(
            x=float(state.vel_body[0]), y=float(state.vel_body[1]), z=0.0
        )
        odom.twist.twist.angular = Vector3(
            x=0.0, y=0.0, z=float(state.vel_body[2])
        )
        odom_pub.publish(odom)

        _publish_range(us_front_pub, "ultrasonic_front", now,
                       float(state.ultrasound[0]), Range)
        _publish_range(us_rear_pub, "ultrasonic_rear", now,
                       float(state.ultrasound[1]), Range)
    except Exception as exc:  # noqa: BLE001
        print(f"[lite3_chassis] odom publish failed: {exc}", flush=True)


def _publish_range(pub, frame_id: str, stamp, distance_m: float, Range_cls) -> None:
    if pub is None:
        return
    msg = Range_cls()
    from std_msgs.msg import Header  # type: ignore
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.radiation_type = Range_cls.ULTRASOUND
    msg.field_of_view = 0.5
    msg.min_range = ULTRASOUND_MIN_M
    msg.max_range = ULTRASOUND_MAX_M
    msg.range = clamp(float(distance_m), 0.0, ULTRASOUND_MAX_M)
    pub.publish(msg)


def _publish_joint_state() -> None:
    with _state_lock:
        joints = _latest_joints
    if joints is None or joint_pub is None:
        return
    try:
        from sensor_msgs.msg import JointState  # type: ignore
        from std_msgs.msg import Header  # type: ignore
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = _stamp()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(getattr(joints, n)) for n in JOINT_NAMES]
        # velocity/effort are not present in the Lite3 JointState frame; leave empty.
        msg.velocity = []
        msg.effort = []
        joint_pub.publish(msg)
    except Exception as exc:  # noqa: BLE001
        print(f"[lite3_chassis] joint publish failed: {exc}", flush=True)


def _stamp():
    from builtin_interfaces.msg import Time  # type: ignore
    utc = time.time()
    t = Time()
    t.sec = int(utc)
    t.nanosec = int((utc - int(utc)) * 1e9) % 1_000_000_000
    return t


def _yaw_quat(yaw_rad: float):
    from geometry_msgs.msg import Quaternion  # type: ignore
    h = yaw_rad * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(h)
    q.w = math.cos(h)
    return q


def udp_loop(robot_ip: str, cmd_port: int, state_port: int) -> None:
    """Receive Lite3 state frames, dispatch by length+code, publish to ROS 2,
    and stream velocity commands to the Motion Host on :cmd_port.

    No keep-alive is sent: the Motion Host stops the robot when velocity
    packets stop arriving (same behaviour as the official transfer node), and
    the no-input→stop timeout below sends explicit zero velocity first."""
    global _state_sock, _cmd_sock, _cmd_target
    _state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    _state_sock.settimeout(0.5)
    _state_sock.bind(("0.0.0.0", state_port))

    _cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _cmd_target = (robot_ip, cmd_port)

    print(
        f"[lite3_chassis] UDP thread: listen :{state_port} ← motion host, "
        f"commands → {robot_ip}:{cmd_port}",
        flush=True,
    )

    last_send = 0.0
    while _running.is_set():
        # Send current velocity (zero when timed out) ~10 Hz.
        now = time.monotonic()
        if now - last_send >= 0.1:
            vx, vy, wz = _current_velocity()
            for pkt in encode_velocity(vx, vy, wz):
                _cmd_sock.sendto(pkt, _cmd_target)
            last_send = now

        try:
            data, _addr = _state_sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break

        n = len(data)
        if n == ROBOT_STATE_SIZE:
            frame = RobotStateReceived.from_buffer_copy(data)
            _apply_robot_state(frame)
            _publish_robot_state()
        elif n == ROBOT_STATE_WITH_POLICY_SIZE:
            frame = RobotStateReceivedWithPolicy.from_buffer_copy(data)
            _apply_robot_state(frame)
            _publish_robot_state()
        elif n == JOINT_STATE_SIZE:
            frame = JointStateReceived.from_buffer_copy(data)
            if frame.code == JOINT_STATE_CODE:
                with _state_lock:
                    _latest_joints = frame.data
                _publish_joint_state()
        elif n == HANDLE_STATE_SIZE:
            # Handle state is telemetry only; not consumed by nav. Accept silently.
            pass
        elif n == IMU_DATA_SIZE:
            frame = ImuDataReceived.from_buffer_copy(data)
            if frame.code == IMU_DATA_CODE:
                with _state_lock:
                    _latest_imu = frame.data
        # Unknown length → ignore (the C++ bridge logs these; we stay quiet).

    try:
        _state_sock.close()
        _cmd_sock.close()
    except OSError:
        pass
    print("[lite3_chassis] UDP thread stopped", flush=True)


# ════════════════════════════════════════════════════════════════════════════
# gRPC: robonix/primitive/chassis/move — velocity-only
# ════════════════════════════════════════════════════════════════════════════
import chassis_pb2  # noqa: E402
import std_msgs_pb2  # noqa: E402


@lite3.grpc("robonix/primitive/chassis/move")
def move(req: "chassis_pb2.ExecuteMoveCommand_Request") -> "chassis_pb2.ExecuteMoveCommand_Response":
    """Publish a velocity command to the Motion Host.

    Velocity-only (ADR-0003): only linear_x / linear_y / angular_z are honoured,
    hard-clamped to the safety limits. forward_m / rotate_deg / linear_z are
    REJECTED (not silently ignored) so a caller misreading the contract fails
    loudly rather than commanding an unintended motion. The Motion Host stops the
    robot when velocity packets stop arriving (no keep-alive).
    """
    cmd = req.command
    forward_m = float(getattr(cmd, "forward_m", 0.0))
    rotate_deg = float(getattr(cmd, "rotate_deg", 0.0))
    linear_z = float(getattr(cmd, "linear_z", 0.0))
    if forward_m != 0.0 or rotate_deg != 0.0 or linear_z != 0.0:
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(
                data=json.dumps(
                    {"error": "move is velocity-only on Lite3; "
                              "forward_m/rotate_deg/linear_z rejected"}
                )
            ),
        )
    vx = float(getattr(cmd, "linear_x", 0.0))
    vy = float(getattr(cmd, "linear_y", 0.0))
    wz = float(getattr(cmd, "angular_z", 0.0))
    _set_velocity(vx, vy, wz)
    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps({
            "status": "moving",
            "linear_x": vx, "linear_y": vy, "angular_z": wz,
            "limits": {"max_lin_x": MAX_LIN_X, "max_lin_y": MAX_LIN_Y, "max_ang_z": MAX_ANG_Z},
        })),
    )


# ════════════════════════════════════════════════════════════════════════════
# ROS 2 /cmd_vel subscription callback
# ════════════════════════════════════════════════════════════════════════════


def _on_cmd_vel(msg):
    """Forward a Twist on /cmd_vel into the velocity set-point (clamped)."""
    _set_velocity(float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))


# ════════════════════════════════════════════════════════════════════════════
# Lifecycle
# ════════════════════════════════════════════════════════════════════════════


@lite3.on_init
def init(cfg: dict):
    global odom_pub, joint_pub, us_front_pub, us_rear_pub, cmd_vel_sub
    cfg = cfg or {}
    robot_ip = str(cfg.get("robot_ip", DEFAULT_ROBOT_IP))
    cmd_port = int(cfg.get("cmd_port", DEFAULT_CMD_PORT))
    state_port = int(cfg.get("state_port", DEFAULT_STATE_PORT))
    base_frame = str(cfg.get("base_frame", os.environ.get("LITE3_BASE_FRAME", "base_link")))
    os.environ["LITE3_BASE_FRAME"] = base_frame

    # /odom (nav_msgs/Odometry) — onboarding §6.1 requires odom. The odom→
    # base_link STATIC transform is published by robot_state_publisher inside
    # the robonix_lite3_ros container (from this package's URDF; ADR-0004); the
    # leg odom drives the base pose via /odom.
    from nav_msgs.msg import Odometry  # type: ignore
    odom_pub = lite3.create_publisher(
        "robonix/primitive/chassis/odom",
        topic=str(cfg.get("odom_topic", "/odom")),
        msg_type=Odometry, qos="reliable",
    )
    # /cmd_vel subscription — onboarding §6.1: chassis accepts Twist.
    cmd_vel_sub = lite3.create_subscription(
        "robonix/primitive/chassis/twist_in",
        topic=str(cfg.get("cmd_vel_topic", "/cmd_vel")),
        msg_type="Twist", callback=_on_cmd_vel, qos="reliable",
    )
    # /joint_states — nav/scene read joint position for foot telemetry. There is
    # no robonix contract for it, so publish without an atlas declare.
    from sensor_msgs.msg import JointState  # type: ignore
    joint_pub = lite3.create_publisher(
        "robonix/primitive/chassis/joint_states",
        topic="/joint_states", msg_type=JointState, qos="reliable", declare=False,
    )
    # Front/rear ultrasonic rangefinders — published for telemetry/obstacle
    # awareness; NOT consumed by mapping (ADR-0002). Declared under the chassis
    # namespace as raw ROS topics (no atlas contract for them).
    from sensor_msgs.msg import Range  # type: ignore
    us_front_pub = lite3.create_publisher(
        "robonix/primitive/chassis/ultrasound_front",
        topic="/ultrasound/front", msg_type=Range, qos="best_effort", declare=False,
    )
    us_rear_pub = lite3.create_publisher(
        "robonix/primitive/chassis/ultrasound_rear",
        topic="/ultrasound/rear", msg_type=Range, qos="best_effort", declare=False,
    )

    t = threading.Thread(
        target=udp_loop, args=(robot_ip, cmd_port, state_port), daemon=True
    )
    t.start()
    print(f"[lite3_chassis] init OK — bridge to {robot_ip}:{cmd_port}", flush=True)
    return Ok()


@lite3.on_shutdown
def shutdown():
    global _running
    _running.clear()
    # Send an explicit stop so the robot does not inherit a stale set-point.
    if _cmd_sock is not None and _cmd_target[0]:
        try:
            for pkt in encode_velocity(0.0, 0.0, 0.0):
                _cmd_sock.sendto(pkt, _cmd_target)
        except OSError:
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