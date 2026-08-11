---
description: Lite3 lidar — Livox MID-360S 3D point cloud sliced to a 2D LaserScan.
---
# Lite3 lidar (`robonix/primitive/lidar`)
Consumes the Livox MID-360S vendor driver's PointCloud2 (`/livox/lidar`), keeps
points in a horizontal band around the lidar plane (±`band_m`), bins by azimuth
across a full 360°, and publishes the result as `/scan` (sensor_msgs/LaserScan).
Exposes `robonix/primitive/lidar/lidar` (topic_out), `snapshot` (MCP), and
`driver` (gRPC lifecycle).

The vendor `livox_ros_driver2` node runs out-of-band in the same
`robonix_lite3_ros` container / ROS 2 domain (ADR-0004), built into the image.
MID-360S is 3D (360°×59° FOV); mapping uses the horizontal slice only — the
top/bottom of the FOV is intentionally dropped.
