# robot-deeprobotics-lite3

A Robonix deployment package for the **DeepRobotics Jueying Lite3** quadruped
(*Explorer* SKU: onboard monocular RGB + front/rear ultrasonics) augmented with
an **external Orbbec Gemini 335 RGB-D camera** on `eth1` to enable SLAM.

## What this package does

| Layer | Mechanism |
| --- | --- |
| Chassis | Speaks the Lite3 **Motion Host UDP protocol** directly (no `transfer` node hop) — `RobotState`/`JointState`/`Imu`/`Handle` telemetry from `:43897`, velocity commands to `:43893`. See [ADR-0001](docs/adr/0001-direct-udp-protocol.md). |
| Perception | A single **RGB-D head** (Orbbec Gemini 335) supplies rgb + depth + CameraInfo; the `lite3_camera` primitive bridges the vendor driver and provides JPEG snapshots. See [ADR-0002](docs/adr/0002-single-rgbd-camera.md). |
| Mapping / Nav / Explore | `service-map-rbnx` (RTAB-Map, `rgbd + odom`), `service-navigation-rbnx` (Nav2), `skill-explore-rbnx`. |
| TF | `odometer → base_link → TORSO → … → head_camera_* / ultrasonic_*` from `primitive-robot-description-rbnx`. |

## Hardware & network (DeepRobotics factory topology)

| Host | IP | Role |
| --- | --- | --- |
| Motion Host (QNX) | **192.168.1.1** | Real-time leg control. Commands UDP `:43893`, telemetry UDP `:43897`. |
| Perception host (Jetson) | **192.168.1.120** | Runs robonix + ROS 2; binds the telemetry socket, sends commands. |
| Orbbec Gemini 335 | on `eth1` | External RGB-D head camera atop the onboard RGB. |

> The package defaults assume a stock DeepRobotics network. If your unit uses a
> different subnet, set `robot_ip` / the perception-host address in
> `robonix_manifest.yaml` before boot.

## Computing platform / dependencies

- Linux (Jetson-class) perception host with ROS 2 + the robonix runtime (`rbnx`).
- `orbbec_camera` ROS 2 driver for the Gemini 335 — **must be started before
  `rbnx boot`** (e.g. systemd unit or external launch). The `lite3_camera`
  primitive *bridges* that driver's topics; it does not spawn it.
- CycloneDDS is the default RMW (`env: RMW_IMPLEMENTATION: rmw_cyclonedds_cpp`).

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
        └── ultrasonic_rear_link
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

```bash
rbnx validate ./primitives/lite3_chassis
rbnx validate ./primitives/lite3_camera
rbnx build -f robonix_manifest.yaml     # expect Failed:0 / Skipped:0
# Start the Orbbec driver first, then:
rbnx boot -f robonix_manifest.yaml
rbnx caps -v                            # expect lite3_chassis / lite3_camera ACTIVE
rbnx logs -t soma -l warn
ros2 topic echo /odom                   # confirm protocol parsing
```

## Package layout

```
robonix_manifest.yaml      deployment manifest (primitives, services, skills)
soma.yaml                 robot model + components + capability exports
urdf/Lite3.urdf           vendor body + appended sensor links
config/                   rtabmap_params.yaml, nav2_params.yaml
primitives/lite3_chassis/ Motion-Host UDP driver
primitives/lite3_camera/  RGB-D topic bridge + snapshot
docs/adr/                 architectural decisions
CONTEXT.md                project glossary
```

## Known limits (also in `soma.yaml description.cannot_do`)

- No planar lidar — occupancy comes from registered depth.
- `move` is velocity-only; pose/gait/joint commands are out of scope.
- Leg odometry drifts over long distance; relocalize within mapped areas.
- No audio primitive is deployed.