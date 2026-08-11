# Chassis is a topic bridge over the official `transfer` node

The direct-Motion-Host-UDP bridge chosen in ADR-0001 gave the `lite3_chassis`
primitive sole ownership of the Motion-Host sockets (`:43893` commands /
`:43897` telemetry). In practice the self-written driver is thin and hard to
certify against the vendor controller, and it duplicates what DeepRobotics
already ships and maintains: the official `Lite3_ROS` `transfer` package
(`Jetson2Motion.cpp` + `protocol.hpp`), which converts `/cmd_vel` ⇄ UDP and
Motion-Host telemetry ⇄ `/leg_odom` `/leg_odom2` `/imu/data` `/joint_states`,
and also handles the App handle channel (auto/manual mode switching) that the
self-written driver lacks.

We reverse the "no transfer-node hop" decision **for the chassis only**: the
official `transfer` node (foxy branch, adapted to Humble, colcon-built into the
`robonix_lite3_ros` image) becomes the single owner of the Motion-Host UDP
sockets. `lite3_chassis` is reduced to a pure ROS 2 topic bridge into the
robonix/atlas graph:

- **`odom`** — relay `transfer`'s `/leg_odom2` → `/odom` verbatim
  (nav_msgs/Odometry). Frames/stamps come from the source.
- **`move`** — publish a bounded, hard-clamped velocity burst on `/cmd_vel`
  (ADR-0003), then an explicit zero. `transfer` forwards it to the Motion Host.
- **`twist_in`** — subscribe `/cmd_vel`; external Twist (Nav2) is consumed
  directly by `transfer` in the shared graph, so the primitive only observes it
  for the contract and does **not** republish onto the same topic (which would
  echo its own `move` bursts).

The protocol-fidelity notes in ADR-0001 still describe the wire format; socket
ownership simply moves to `transfer`. The Motion Host must be in **AUTO mode**
(velocity commands), not SDK mode — a running `MotionSDK` process grabs control
and makes the velocity path unresponsive.

## Safety consequences (supersedes part of ADR-0003)

- The `move` RPC path keeps the ADR-0003 hard clamps (0.6/0.3/1.5) and the
  loud rejection of `forward_m`/`rotate_deg`/`linear_z`.
- The Nav2 `/cmd_vel` path is no longer clamped in this primitive; it relies on
  Nav2's own controller limits and the Motion Host's built-in velocity bounds.
  If a deployment needs the tighter ADR-0003 envelope on the Nav2 path, put a
  clamp node on `/cmd_vel` (or remap Nav2 onto a `/cmd_vel_safe` input).
- No-input→stop is inherited from the Motion Host (stops when velocity packets
  stop) plus the explicit zero at the end of every `move` burst.

## Considered Options (rejected)

- **Keeping the direct-UDP driver** — duplicates the vendor bridge, lacks the
  App auto/manual channel, and is the least-tested component of the stack.
- **Vendoring `transfer` into the repo as a primitive** — `transfer` is a ROS 2
  node, not a robonix capability; running it from the container entrypoint
  (like `robot_state_publisher`, ADR-0004) keeps a single UDP owner and a
  single ROS 2 graph.
