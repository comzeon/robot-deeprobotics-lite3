# A single external RGB-D camera replaces the onboard monocular RGB and enables mapping

The Lite3 Explorer's factory sensors — one monocular RGB plus two ultrasonic
rangefinders — are not enough for RTAB-Map: the robonix mapping service's 2-D
occupancy grid needs either a planar lidar or registered depth, and two discrete
ultrasound distances cannot produce that grid. So the mapping/nav2/explore
capabilities the package advertises could not actually run on the shipped kit.

The deployment adds a depth camera atop the RGB (an Orbbec Gemini 335 on `eth1`),
and the whole sensing head is modeled as **one** `rgbd_camera` component in soma
(not two primitives). The `lite3_camera` primitive is extended to bridge an
already-running vendor ROS driver's topics (`orbbec_camera` — color + depth +
CameraInfo) into robonix contracts and to provide `snapshot`; it does **not**
spawn the vendor driver. Mapping is then wired with `sensor_providers:
{rgb: lite3_camera, depth: lite3_camera, odom: lite3_chassis}`,
`rtabmap_inputs: [rgbd, odom]`.

This reverses a deliberate choice: the original package dropped a `:8554` RTSP
monocular stream for `ffmpeg` snapshots. That path is removed — an RTSP single
frame cannot serve SLAM depth, and keeping a second image source would split the
robot's single optical frame. The factory ultrasonics stay (published from the
chassis RobotState `ultrasound[2]`) but are telemetry, not a mapping input.

**Consequences**: the package now depends on the `orbbec_camera` driver being
started out-of-band (systemd / external launch) before `rbnx boot`; the README and
`on_init` sentinel must document and wait for its topics.