---
description: Lite3 quadruped base — ROS 2 topic bridge over the official transfer node; relays leg odom, publishes velocity bursts.
---
# Lite3 chassis (`robonix/primitive/chassis`)
The quadruped robot base. A topic bridge (ADR-0005): the official DeepRobotics `transfer` node owns the Motion-Host UDP sockets and the velocity channel; this primitive relays `/leg_odom2` → `/odom`, publishes `move()` velocity bursts on `/cmd_vel`, and exposes `twist_in`.
