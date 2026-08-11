# Primitives run in a bring-up ROS 2 container; TF via in-container robot_state_publisher

The host is a bare L4T board — no ROS 2 on it. Every ROS 2 process (the
chassis/camera primitives, `robot_state_publisher`) must run inside a container.
The robonix runtime launches a primitive by `docker exec`-ing the driver module
into a **pre-running** `robonix_lite3_ros` container, exactly as the Webots
example `docker exec`s drivers into its `robonix_tiago_sim` container — only the
sim container is replaced by a real-robot ROS 2 container.

Consequences driven by this:

- The container **entrypoint** brings up the whole ROS 2 environment: the zenoh
  router, `robot_state_publisher` (single TF publisher), and the two **vendor
  drivers** (`orbbec_camera` for the Gemini 335, `livox_ros_driver2` for the
  MID-360S). Vendor drivers are out-of-band processes the robonix primitives
  bridge — they belong in the entrypoint (toggleable via `LITE3_ENABLE_*`) so
  `container/start.sh` brings up the full sensor stack in one step, rather than
  leaving "start the camera driver" as an unexplained manual step.
- We do **not** use the external `primitive-robot-description-rbnx` package. Its
  `build.sh` sources `/opt/ros/humble/setup.bash` on the host, which a bare-L4T
  host cannot satisfy, so `rbnx build` fails. Instead the URDF + TF tree
  (odom→base_link→TORSO→head_camera/ultrasonic/lidar) is published by
  `robot_state_publisher`, installed and launched **inside** the same container,
  so TF and topics share one DDS/zenoh domain with the drivers.
- `container/` (compose + Dockerfile + `start.sh`/`stop.sh`) brings up that
  container *before* `rbnx boot`, mirroring `examples/webots/sim/`. Each
  primitive `scripts/start.sh` becomes a foreground `docker exec` wrapper into
  `robonix_lite3_ros`.
- The base image is `robonix-ros:humble-ros-base`; the Dockerfile additionally
  `apt install` rmw_zenoh_cpp + orbbec_camera, source-builds Livox-SDK2 +
  livox_ros_driver2, and adds python3-numpy (drivers use it). RMW is
  `rmw_zenoh_cpp` with `ROS_DOMAIN_ID=0` to match the mapping/nav2/scene
  containers and the working webots example.
- `network_mode: host` and `ipc: host` so the ROS 2 graph is shared with the
  host's atlas and the sibling containers (SHM/DDS reach each other), and
  `../primitives` + the repo `pylib` are bind-mounted into `/robonix_pkgs/...`
  so drivers `docker exec` from the host without rebuilding. The container gets
  `/dev/bus/usb` (compose `devices:`) so `orbbec_camera` can open the camera.

**Considered Options** (rejected): building/running `primitive-robot-description-rbnx`
on the host (impossible — no host ROS); making the chassis primitive publish TF
itself (onboarding §3.1 prefers a single TF publisher, and mixing TF into the
UDP driver muddies the seam); running primitives on the bare host with an
external ROS (host has none).