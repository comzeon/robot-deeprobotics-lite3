# Chassis primitive speaks the Lite3 Motion Host UDP protocol directly

The original `lite3_chassis` driver invented its own byte layout — a `0x906`
state-frame filter, a 332-byte monolithic state payload, and a 240-byte joint-
position command — none of which appear in the official `DeepRoboticsLab/Lite3_ROS`
`protocol.hpp`/`Jetson2Motion.cpp`. On a real Lite3 every state packet was
dropped and every command was ignored (or worse, misinterpreted), so the robot
could neither report state nor move.

We rewrite the primitive to mirror the official protocol field-for-field using
`ctypes` structs with `_pack_ = 4` (the same approach the independent
`automatika-robotics/emos-plugin-lite3` uses). Telelemetry is dispatched by
packet length + `code` (2305 RobotState, 2306 JointState, 2309 HandleState,
`0x010901` ImuData) exactly as the C++ bridge's `switch(recv_num_)` does.
Commands are `MotionSimpleCMD`/`MotionComplexCMD` to `:43893`; velocity is three
`ComplexCMD` packets (codes 320/325/321, yaw negated). No separate bridge process
is introduced — the prime's UDP thread is the bridge, so there is no extra hop
between robonix and the Motion Host.

**Considered Options** (rejected): wrapping the official `transfer` ROS 2 node
as a subprocess (adds a ROS 2 build dependency and an extra process just to
shuffle UDP→ROS→UDP); vendoring the full `Lite3_ROS` transfer package.