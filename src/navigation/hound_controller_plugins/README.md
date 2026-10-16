# hound_controller_plugins

Nav2 controller plugin `hound_controller_plugins::PidLookaheadController`: steers with a PID on
the heading error to a look-ahead point on the Nav2 path, under Ackermann limits (the
robot never turns in place). Used as `FollowPath` in `config/hound/nav2_params.yaml`.

Each control cycle (`controller_frequency`, 20 Hz):

1. Closest path pose to the robot, searching forward from last cycle's (progress is kept
   per plan); then the **look-ahead point**: the first point on the path
   `L = clamp(speed x lookahead_time, min_lookahead_dist, max_lookahead_dist)` from the robot.
2. **Tracking error** = heading error to that point, `atan2(y, x)` in the robot frame.
3. `angular.z = kp*e + ki*integral(e) + kd*de/dt` -- conditional integration (no windup
   while saturated), low-pass filtered derivative; reset after a gap (new goal).
4. `linear.x = desired_linear_vel * cos(e)`, slowed within `approach_dist` of the goal,
   never below `min_linear_vel` (the motors stall below it), capped by Nav2 speed limits.
5. **Ackermann**: `|angular.z| <= linear.x / min_turning_radius`.

Keep `min_turning_radius` equal to the planner's `GridBased.minimum_turning_radius` and
`hound_base`'s `min_turning_radius` (with `ackermann: true`), and `min_lookahead_dist`
at least that radius.

## Tuning

Watch `/controller_server/pid_tracking` (`[error, p, i, d, angular, linear, lookahead]`)
and `/lookahead_point` (add a PointStamped display in RViz):

- Wanders slowly back and forth around the path -> raise `kp`; too twitchy -> lower it.
- Overshoots after turns -> raise `kd` (or `derivative_filter` if it's jittery).
- Stays offset to one side on straights (e.g. a pulling motor) -> small `ki` (0.05-0.2).
- Cuts corners -> lower `min_lookahead_dist` (not below `min_turning_radius`);
  weaves at speed -> raise `lookahead_time`.

Parameters can be changed live: `ros2 param set /controller_server FollowPath.kp 2.0`
takes effect on the next `controller_server` restart (they're read at configure).
