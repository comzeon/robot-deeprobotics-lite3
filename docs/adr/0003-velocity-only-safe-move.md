# Velocity-only, hard-clamped `move` with no-input-→-stop as the safety boundary

The Lite3 is a real quadruped; an unrestricted velocity command can move the robot
into people or obstacles, and the original driver's "keep-alive joint-position
packet every 200 ms" sent zero-position targets irrespective of what a caller
asked for. We deliberately constrain the `move` capability to the narrowest
behaviour that serves navigation and teleop safely:

- `move` maps only `linear_x/linear_y/angular_z` (the `MoveCommand` velocity
  fields). `forward_m`/`rotate_deg`/`linear_z` and other fields are **rejected**,
  not silently ignored, so a caller misconstruing the contract gets a loud error.
- Velocities are hard-clamped in-code to ``MAX_LIN_X/MAX_LIN_Y/MAX_ANG_Z``
  (0.6 m/s, 0.3 m/s lateral, 1.5 rad/s), which sit well under DeepRobotics'*
  documented motion limits and are tuned for indoor exploration.
- The primitive sends no keep-alive and no commands while idle; the Motion Host
  stops the robot when it stops receiving velocity commands (the official
  `transfer` node's `MotionSender` works the same way — no `cmd_vel` ⇒ no motion).
- The driver **never** emits joint-position commands (`MotionSimpleCMD` pose-mode
  codes, sit/stand, gaits). Those are intentionally out of scope for this package.

**Considered Options** (rejected): adopting `tiago_chassis`'s distance/angle
modes (`forward_m`/`rotate_deg`) — they need a leg-odom closed loop whose
correctness on a 12-DOF quadruped we are not prepared to certify this round.