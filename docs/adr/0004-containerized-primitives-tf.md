# Primitives run in a bring-up ROS 2 container; TF via in-container robot_state_publisher

The host is a bare L4T board — no ROS 2 on it. Every ROS 2 process (the
chassis/camera primitives, `robot_state_publisher`) must run inside a container.
The robonix runtime launches a primitive by `docker exec`-ing the driver module
into a **pre-running** `robonix_lite3_ros` container, exactly as the Webots
example `docker exec`s drivers into its `robonix_tiago_sim` container — only the
sim container is replaced by a real-robot ROS 2 container.

Consequences driven by this:

- We do **not** use the external `primitive-robot-description-rbnx` package. Its
  `build.sh` sources `/opt/ros/humble/setup.bash` on the host, which a bare-L4T
  host cannot satisfy, so `rbnx build` fails. Instead the URDF + TF tree
  (odom→base_link→TORSO→head_camera/ultrasonic) is published by
  `robot_state_publisher`, installed and launched **inside** the same container,
  so TF and topics share one DDS/zenoh domain with the drivers.
- `container/` (compose + Dockerfile + `start.sh`/`stop.sh`) brings up that
  container *before* `rbnx boot`, mirroring `examples/webots/sim/`. Each
  primitive `scripts/start.sh` becomes a foreground `docker exec` wrapper into
  `robonix_lite3_ros`.
- The base image is `robonix-ros:humble-ros-base`; the Dockerfile additionally
  `apt install ros-humble-rmw-zenoh-cpp` (verified installable). RMW is
  `rmw_zenoh_cpp` with `ROS_DOMAIN_ID=0` to match the mapping/nav2/scene
  containers and the working webots example.
- `network_mode: host` and `ipc: host` so the ROS 2 graph is shared with the
  host's atlas and the sibling containers (SHM/DDS reach each other), and
  `../primitives` + the repo `pylib` are bind-mounted into `/robonix_pkgs/...`
  so drivers `docker exec` from the host without rebuilding.

**Considered Options** (rejected): building/running `primitive-robot-description-rbnx`
on the host (impossible — no host ROS); making the chassis primitive publish TF
itself (onboarding §3.1 prefers a single TF publisher, and mixing TF into the
UDP driver muddies the seam); running primitives on the bare host with an
external ROS (host has none).