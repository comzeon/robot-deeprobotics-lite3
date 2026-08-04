#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Lite3 lidar primitive — MID-360S 3D lidar → 2D LaserScan.

Owns `robonix/primitive/lidar/*`. The Livox MID-360S driver
(`livox_ros_driver2`, a colcon package inside the robonix_lite3_ros container)
publishes PointCloud2 on `/livox/lidar`; Nav2 / mapping / scene need a planar
LaserScan, so this primitive slices the point cloud into a horizontal band and
projects it to a 2D scan on `/scan`:

  primitive/lidar/lidar      topic_out  ROS 2 LaserScan stream (/scan)
  primitive/lidar/snapshot   rpc        MCP one-shot LaserScan capture
  primitive/lidar/driver     rpc        gRPC lifecycle

The vendor driver is launched out-of-band (same container / ROS_DOMAIN_ID +
RMW, like orbbec_camera). The slice band height and /scan topic are configurable
via package config (defaults: ±0.15 m around the lidar plane, /scan).
"""
from __future__ import annotations

import math
import os
import threading
import time

import numpy as np

from robonix_api import Primitive, Ok, Err

# ── Provider instance (onboarding §4.2) ──────────────────────────────────────
lite3_lidar = Primitive(
    id=os.environ.get("RBNX_INSTANCE_NAME", "lite3_lidar"),
    namespace="robonix/primitive/lidar",
)

# ── shared state — latest rclpy LaserScan we produced ───────────────────────
state_lock = threading.Lock()
latest_scan = None

# Slice band around the lidar's mounting plane (z in [-band, +band], m).
# MID-360S vertical FOV is ±29.5°; for a planar 2D scan we keep points within a
# thin horizontal slab. Points beyond the slab (top/bottom of the FOV) are
# dropped — mapping uses the horizontal slice only.
SCAN_BAND_M = float(os.environ.get("LITE3_LIDAR_BAND_M", "0.15"))
SCAN_RANGE_MIN = float(os.environ.get("LITE3_LIDAR_RANGE_MIN", "0.1"))
SCAN_RANGE_MAX = float(os.environ.get("LITE3_LIDAR_RANGE_MAX", "40.0"))
# Fixed 2D bins across a full 360° turn (MID-360S is 360° horizontal).
SCAN_BINS = int(os.environ.get("LITE3_LIDAR_BINS", "720"))  # 0.5°/bin
SCAN_ANGLE_MIN = float(os.environ.get("LITE3_LIDAR_ANGLE_MIN", "-3.14159265"))
SCAN_ANGLE_MAX = float(os.environ.get("LITE3_LIDAR_ANGLE_MAX", "3.14159265"))
SCAN_FRAME = os.environ.get("LITE3_LIDAR_FRAME", "lidar_link")


def pointcloud_to_scan(msg) -> "object | None":
    """Slice a sensor_msgs/PointCloud2 into a sensor_msgs/LaserScan.

    Extracts x/y/z from the point cloud, keeps points within the horizontal
    band, and bins by azimuth angle (min range per bin). Returns a LaserScan or
    None if no usable points.
    """
    h = msg.height
    w = msg.width
    pc = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, msg.point_step)
    # Field offsets from the message layout (Livox: x@0,y@4,z@8 float32).
    offsets = {}
    for f in msg.fields:
        offsets[f.name] = f.offset
    if not {"x", "y", "z"}.issubset(offsets):
        return None
    x = pc[:, :, offsets["x"]:offsets["x"] + 4].reshape(-1).astype(np.float32)
    y = pc[:, :, offsets["y"]:offsets["y"] + 4].reshape(-1).astype(np.float32)
    z = pc[:, :, offsets["z"]:offsets["z"] + 4].reshape(-1).astype(np.float32)

    mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[mask], y[mask], z[mask]
    if x.size == 0:
        return None
    # Horizontal band around the lidar plane.
    band = np.abs(z) <= SCAN_BAND_M
    x, y = x[band], y[band]
    if x.size == 0:
        return None
    r = np.hypot(x, y)
    in_range = (r >= SCAN_RANGE_MIN) & (r <= SCAN_RANGE_MAX)
    x, y, r = x[in_range], y[in_range], r[in_range]
    if x.size == 0:
        return None
    ang = np.arctan2(y, x)

    bin_idx = np.clip(
        (ang - SCAN_ANGLE_MIN) / (SCAN_ANGLE_MAX - SCAN_ANGLE_MIN) * SCAN_BINS,
        0, SCAN_BINS - 1,
    ).astype(int)
    ranges = np.full(SCAN_BINS, np.inf, dtype=np.float32)
    # min range per bin via np.minimum.at (duplicates handled correctly)
    np.minimum.at(ranges, bin_idx, r)
    ranges[ranges == np.inf] = 0.0  # empty bins → 0 (no return)

    from sensor_msgs.msg import LaserScan  # type: ignore
    from std_msgs.msg import Header  # type: ignore
    scan = LaserScan()
    scan.header = Header()
    scan.header.stamp = _stamp()
    scan.header.frame_id = SCAN_FRAME
    scan.angle_min = SCAN_ANGLE_MIN
    scan.angle_max = SCAN_ANGLE_MAX
    scan.angle_increment = (SCAN_ANGLE_MAX - SCAN_ANGLE_MIN) / SCAN_BINS
    scan.time_increment = 0.0
    scan.scan_time = 1.0 / 10.0  # MID-360S ~10 Hz
    scan.range_min = SCAN_RANGE_MIN
    scan.range_max = SCAN_RANGE_MAX
    scan.ranges = ranges.tolist()
    scan.intensities = []
    return scan


def _stamp():
    from builtin_interfaces.msg import Time  # type: ignore
    utc = time.time()
    t = Time()
    t.sec = int(utc)
    t.nanosec = int((utc - int(utc)) * 1e9) % 1_000_000_000
    return t


# ── ROS2 point cloud callback ────────────────────────────────────────────────
scan_pub = None  # /scan publisher (LaserScan)


def on_pointcloud(msg):
    global latest_scan
    scan = pointcloud_to_scan(msg)
    if scan is None:
        return
    with state_lock:
        latest_scan = scan
    if scan_pub is not None:
        try:
            scan_pub.publish(scan)
        except Exception as e:  # noqa: BLE001
            print(f"[lite3_lidar] scan publish failed: {e}", flush=True)


# ── MCP snapshot tool ────────────────────────────────────────────────────────
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import LaserScan  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def ros_to_mcp(ros) -> LaserScan:
    h = ros.header
    stamp = builtin_interfaces_mcp.Time(sec=int(h.stamp.sec), nanosec=int(h.stamp.nanosec))
    header = std_msgs_mcp.Header(stamp=stamp, frame_id=str(h.frame_id))
    return LaserScan(
        header=header,
        angle_min=float(ros.angle_min),
        angle_max=float(ros.angle_max),
        angle_increment=float(ros.angle_increment),
        time_increment=float(ros.time_increment),
        scan_time=float(ros.scan_time),
        range_min=float(ros.range_min),
        range_max=float(ros.range_max),
        ranges=[float(r) for r in ros.ranges],
        intensities=[float(i) for i in ros.intensities],
    )


@lite3_lidar.mcp("robonix/primitive/lidar/snapshot")
def snapshot(msg: Empty) -> LaserScan:
    """Get the latest 2D laser scan (sliced from the MID-360S point cloud).
    Returns sensor_msgs/LaserScan; `ranges[i]` is the distance (m) at angle
    `angle_min + i*angle_increment`. Useful for "obstacle in front?".
    Contract: robonix/primitive/lidar/snapshot."""
    _ = msg
    with state_lock:
        ros_scan = latest_scan
    if ros_scan is None:
        raise RuntimeError("no LaserScan produced yet (no point cloud?)")
    return ros_to_mcp(ros_scan)


# ── lifecycle ────────────────────────────────────────────────────────────────
@lite3_lidar.on_init
def init(cfg):
    global scan_pub, SCAN_FRAME, SCAN_BAND_M, SCAN_RANGE_MIN, SCAN_RANGE_MAX, SCAN_BINS
    cfg = cfg or {}
    topic = cfg.get("scan_topic") or os.environ.get("LITE3_LIDAR_SCAN_TOPIC", "/scan")
    cloud_topic = cfg.get("cloud_topic") or os.environ.get("LITE3_LIDAR_CLOUD_TOPIC", "/livox/lidar")
    SCAN_FRAME = str(cfg.get("frame_id", SCAN_FRAME))
    SCAN_BAND_M = float(cfg.get("band_m", SCAN_BAND_M))
    SCAN_RANGE_MIN = float(cfg.get("range_min_m", SCAN_RANGE_MIN))
    SCAN_RANGE_MAX = float(cfg.get("range_max_m", SCAN_RANGE_MAX))
    SCAN_BINS = int(cfg.get("bins", SCAN_BINS))

    # Subscribe the vendor PointCloud2; publish the sliced LaserScan on /scan.
    from sensor_msgs.msg import LaserScan  # type: ignore
    scan_pub = lite3_lidar.create_publisher(
        "robonix/primitive/lidar/lidar",
        topic=topic, msg_type=LaserScan, qos="best_effort",
    )
    lite3_lidar.create_subscription(
        "robonix/primitive/lidar/lidar3d",
        topic=cloud_topic, msg_type="PointCloud2",
        callback=on_pointcloud, qos="best_effort",
        declare=False,  # the vendor driver owns the cloud; we only consume it
    )
    if not lite3_lidar.wait_for_topic(cloud_topic, "PointCloud2",
                                      float(cfg.get("sentinel_timeout_s", 30.0))):
        return Err(f"no PointCloud2 on {cloud_topic} within timeout "
                   "(is livox_ros_driver2 running?)")
    print(f"[lite3_lidar] init OK — {cloud_topic} → {topic} (band ±{SCAN_BAND_M} m)", flush=True)
    return Ok()


@lite3_lidar.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    print("[lite3_lidar] starting — connecting to Atlas...", flush=True)
    lite3_lidar.run()