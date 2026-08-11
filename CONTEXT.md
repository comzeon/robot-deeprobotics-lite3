# robot-deeprobotics-lite3

## Language

**Motion Host (运动主机)**:
The QNX-based real-time controller aboard the Lite3 that owns leg control. It receives commands on UDP port `43893` and streams telemetry from `43897`. Factory default IP `192.168.1.1`.
_Avoid_: robot (overloaded — used for the whole quadruped), SDK host, perception host.

**Perception Host (感知主机)**:
The companion Linux computer (Jetson-class) where robonix runs. It is the peer that opens UDP sockets to the Motion Host and runs ROS 2 / the robonix primitive processes. Factory default IP `192.168.1.120`.
_Avoid_: SDK host (DeepRobotics' SDK examples bind a Sender to the perception-host IP), app host.

**Lite3 Explorer (Lite3 探索版)**:
The Lite3 SKU carrying the onboard monocular RGB camera plus front and rear ultrasonic rangefinders. The base vendor URDF and protocol structs model only the quadruped body; these sensors are not described by DeepRobotics' shipped URDF.
_Avoid_: base Lite3, standard Lite3.

**RobotState frame**:
A `#pragma pack(4)` telemetry frame `{int code,size,cons_code; RobotState data}` with `code==2305`, carrying rpy/rpy_vel/xyz_acc/pos_world/vel_world/vel_body/battery_level/ultrasound[2]. Parsed by packet length dispatch (`switch(recv_num_)`), not by a single magic code filter.
_Avoid_: robot state (generic), CMD_ROBOT_DATA (the wrong 0x906 code used in the original driver).

**MotionSimpleCMD / MotionComplexCMD**:
The two command wire formats sent to `:43893`. SimpleCMD is 12 bytes `{cmd_code,cmd_value,type}`; ComplexCMD is SimpleCMD plus an 8-byte double `data`. Velocity is three ComplexCMD packets with codes 320(vx)/325(vy)/321(wz); yaw is sent negated.
_Avoid_: joint-position packet, robot_cmd (the meaningless 240-byte blob the original driver sent).

**Leg odom**:
The pose-only odometry the Motion Host reports (`pos_world[3]` = {x, y, yaw_rad}; `vel_body[3]` = body-frame velocity). Origin is footfall-estimated, drifts over distance — sufficient for short-range Nav2 but not for absolute global position.
_Avoid_: wheel odometry, IMU-only odom.

**Onboarding guide**:
The Robonix vendor-integration specification at `docs/integration-guide/vendor-onboarding` of the `syswonder/robonix-book` repo; its numbered sections (§1 catalog, §3.1 URDF/TF, §3.2 soma↔manifest, §4.2 RBNX_INSTANCE_NAME, §4.4 capability-must-be-declared-in-code, §5 services, §6.1 odom+TF, §7 validation) are the acceptance bar for this package.
_Avoid_: the README, the manual (DeepRobotics' Jueying manual).