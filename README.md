# robot-deeprobotics-lite3

A Robonix deployment package for the **DeepRobotics Jueying Lite3** quadruped
(*Explorer* SKU: onboard monocular RGB + front/rear ultrasonics) augmented with
an **external Orbbec Gemini 335 RGB-D camera** on `eth1` and a **Livox MID-360S
3D lidar** to enable SLAM + Nav2.

## What this package does

| Layer | Mechanism |
| --- | --- |
| Chassis | Speaks the Lite3 **Motion Host UDP protocol** directly (no `transfer` node hop) — `RobotState`/`JointState`/`Imu`/`Handle` telemetry from `:43897`, velocity commands to `:43893`. See [ADR-0001](docs/adr/0001-direct-udp-protocol.md). |
| Perception | A single **RGB-D head** (Orbbec Gemini 335) supplies rgb + depth + CameraInfo; the `lite3_camera` primitive bridges the vendor driver and provides JPEG snapshots. See [ADR-0002](docs/adr/0002-single-rgbd-camera.md). |
| Lidar | **Livox MID-360S** 3D lidar via `livox_ros_driver2`; `lite3_lidar` slices the point cloud into a planar `/scan` (LaserScan) for Nav2/mapping. |
| Mapping / Nav / Explore | `service-map-rbnx` (RTAB-Map, `lidar + rgbd + odom`), `service-navigation-rbnx` (Nav2), `skill-explore-rbnx`. |
| TF | `odom → base_link → TORSO → … → head_camera_* / ultrasonic_* / lidar_link` published by `robot_state_publisher` inside the container. See [ADR-0004](docs/adr/0004-containerized-primitives-tf.md). |

## Hardware & network (DeepRobotics factory topology)

| Host | IP | Role |
| --- | --- | --- |
| Motion Host (QNX) | **192.168.1.1** | Real-time leg control. Commands UDP `:43893`, telemetry UDP `:43897`. |
| Perception host (Jetson) | **192.168.1.120** | Runs robonix + ROS 2; binds the telemetry socket, sends commands. |
| Orbbec Gemini 335 | on `eth1` | External RGB-D head camera atop the onboard RGB. |
| Livox MID-360S | UDP on the LAN | 3D lidar (360°×59°); default config IP 192.168.1.100, data on the motion-network subnet. |

> The package defaults assume a stock DeepRobotics network. If your unit uses a
> different subnet, set `robot_ip` / the perception-host address in
> `robonix_manifest.yaml` before boot.

## Computing platform / dependencies

- **Host (Thor):** bare L4T — runs only the robonix Rust system components.
- `robonix_lite3_ros` docker container (built from `robonix-ros:humble-ros-base`
  + `ros-humble-rmw-zenoh-cpp` + `ros-humble-orbbec-camera` + source-built
  `livox_ros_driver2`): holds ROS 2 Humble + rclpy + rclcpp +
  `robot_state_publisher` + `tf2_ros`. All primitive processes run in it via
  `docker exec`. `--network host` + `--ipc host` so the zenoh ROS 2 graph is
  shared with the host's atlas and the mapping/nav2/scene containers.
- `orbbec_camera` ROS 2 driver for the Gemini 335 — **must be started before
  `rbnx boot`** (systemd unit or external launch, inside the same ROS 2 /
  zenoh domain). The `lite3_camera` primitive *bridges* that driver's topics;
  it does not spawn it.
- `livox_ros_driver2` for the MID-360S — built into the image, **must be
  launched before `rbnx boot`** in the same container/domain. `lite3_lidar`
  slices its `/livox/lidar` point cloud into `/scan`.
- RMW is `rmw_zenoh_cpp` with `ROS_DOMAIN_ID=0` (matches the working webots
  example and the sibling containers).

## Coordinate frames

```
odom  (leg odometry, from Motion Host pos_world)
└── base_link                (motion root)
    └── TORSO                 (vendor URDF body root; co-located with base_link)
        ├── FL_HIP / FR_HIP / HL_HIP / HR_HIP  (4 legs, vendor URDF)
        ├── head_camera_link
        │   ├── head_camera_rgb_optical_frame
        │   └── head_camera_depth_optical_frame
        ├── ultrasonic_front_link
        ├── ultrasonic_rear_link
        └── lidar_link        (MID-360S — /scan frame)
```

`base_link → TORSO` is an identity fixed joint so the vendor joint limits are
unchanged. The sensor link origins are **estimates** — measure them on the
physical unit and update `urdf/Lite3.urdf` before relying on extrinsics for SLAM.

## Safety boundaries (read before first motion)

The chassis `move` capability is **velocity-only** with hard in-code limits
([ADR-0003](docs/adr/0003-velocity-only-safe-move.md)):

| Limit | Value |
| --- | --- |
| `max_lin_x` (forward) | **0.6 m/s** |
| `max_lin_y` (lateral) | **0.3 m/s** |
| `max_ang_z` (yaw) | **1.5 rad/s** |
| No-input stop | **0.5 s** without a velocity command ⇒ zero velocity sent |

- The driver **never** emits joint-position / pose / gait commands.
- The Motion Host stops the robot when velocity packets stop arriving; the
  primitive additionally sends explicit zero velocity on the no-input timeout.
- **First motion test:** confirm the physical E-stop works, operate in an open
  area, and keep a finger on the app's manual-mode fallback. Do NOT do a first
  `move` test from the LLM until you have verified `/odom` is updating and the
  velocity clamp values are appropriate for your environment.

## Build & boot

The host is bare L4T (no ROS 2). Primitives run inside the `robonix_lite3_ros`
docker container; bring it up **before** `rbnx boot`. `robot_state_publisher`
(the single TF publisher for the URDF's fixed chain) runs in that container
from its entrypoint (ADR-0004).

```bash
# 1. Build/package the robonix primitives (host-side codegen for proto/MCP stubs):
rbnx validate ./primitives/lite3_chassis
rbnx validate ./primitives/lite3_camera
rbnx validate ./primitives/lite3_lidar
rbnx build -f robonix_manifest.yaml     # expect Failed:0 / Skipped:0

# 2. Host-side udev rules for the Orbbec Gemini 335 (needed once; the container
#    passes through /dev/bus/usb via compose `devices:`):
sudo cp /opt/ros/humble/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 3. Bring up the ROS 2 container. The entrypoint starts the WHOLE environment:
#    zenoh router + robot_state_publisher + orbbec_camera + livox_ros_driver2.
#    Hardware absent / bring-up? Disable a vendor driver with
#    LITE3_ENABLE_ORBBEC=0 / LITE3_ENABLE_LIVOX=0 (start.sh passes them through).
bash container/start.sh
#    (stop with: bash container/stop.sh)
#    start.sh waits for robot_state_publisher + the two vendor topics, WARNs
#    (does not fail) if a sensor is missing — the primitives gate on first frame.

# 4. Boot the robonix deployment — primitives docker-exec into the container:
rbnx boot -f robonix_manifest.yaml
rbnx caps -v                            # expect lite3_chassis/camera/lidar ACTIVE
rbnx logs -t soma -l warn

# 5. Inside the container, confirm protocol parsing + TF + scans:
docker exec robonix_lite3_ros ros2 topic echo /odom --once
docker exec robonix_lite3_ros ros2 run tf2_ros tf2_echo odom base_link
docker exec robonix_lite3_ros ros2 topic echo /ultrasound/front --once
docker exec robonix_lite3_ros ros2 topic echo /scan --once
```

If `/odom` is empty: check the Motion Host IP/subnet, and that UDP `:43897` on
the perception host is free (the official `transfer` node must not be running).
If `/scan` is empty: check the MID-360S IP in `MID360s_config.json` and that the
lidar is reachable from the host network; the vendor driver log is
`docker exec robonix_lite3_ros tail -80 /tmp/livox_ros_driver2.log` (orbbec:
`/tmp/orbbec_camera.log`).

## Package layout

```
robonix_manifest.yaml      deployment manifest (primitives, services, skills)
soma.yaml                 robot model + components + capability exports
urdf/Lite3.urdf           vendor body + appended sensor links
config/                   rtabmap_params.yaml, nav2_params.yaml
container/                the robonix_lite3_ros container (compose/Dockerfile/start.sh)
primitives/lite3_chassis/ Motion-Host UDP driver (docker-exec'd into the container)
primitives/lite3_camera/  RGB-D topic bridge + snapshot (docker-exec'd into the container)
primitives/lite3_lidar/   MID-360S PointCloud2 → /scan slice (docker-exec'd into the container)
docs/adr/                 architectural decisions
CONTEXT.md                project glossary
```

## Known limits (also in `soma.yaml description.cannot_do`)

- `move` is velocity-only; pose/gait/joint commands are out of scope.
- Leg odometry drifts over long distance; relocalize within mapped areas.
- No audio primitive is deployed.