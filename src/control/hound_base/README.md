# hound_base

`base_driver` turns `/cmd_vel` (`geometry_msgs/Twist`) into Sabertooth motor commands,
using the same packets and wiring as the keyboard teleop script.

```bash
ros2 launch hound_base base.launch.py
ros2 run hound_base teleop_wasd    # WASD in this terminal; robot on blocks first
```

`teleop_wasd` publishes on `/cmd_vel_teleop`. When stopped, hold A/D to spin in place.
When driving, hold W/S, and hold A/D to steer: the steering is momentary and returns to
centre as soon as A/D is released. Space/X stops, +/- change speed, Ctrl-C quits. Starts at
0.3 m/s and 0.8 rad/s. A terminal has no key-release events, so a key counts as released
~0.12 s after its autorepeat stops (up to 0.6 s after a single tap). Only the last key
pressed repeats: holding A while driving keeps the robot driving while A is held, and on
releasing A it stops (the terminal can't see that W is still down) -- press W again.

**Drive mode** (`ackermann` in `config/base_driver.yaml`): `false` (default) is
differential drive -- Nav2's `/cmd_vel` is applied as-is, so it can turn in place.
`true` drives it like an Ackermann vehicle instead: `|angular.z| <= |linear.x| /
min_turning_radius` (no turning in place); keep `min_turning_radius` equal to Nav2's
`GridBased.minimum_turning_radius`, and turn RPP's `use_rotate_to_heading` off.

The teleop topic `/cmd_vel_teleop` is not clamped, and takes priority: while it keeps
arriving, `/cmd_vel` is ignored, and Nav2 regains control `cmd_timeout` (0.5 s) after
the teleop goes quiet or quits.

It stops the motors when `/cmd_vel` goes quiet for `cmd_timeout` (0.5 s), when the
Nav2 behaviour tree sends a handbrake on `/vehicle_control_roads`, and on shutdown.
If the process is killed hard (`kill -9`, crash, USB unplugged), the Sabertooths keep
their last command, so keep the e-stop in reach.

## Calibration (do before running Nav2)

`track_width` and `max_wheel_speed` in `config/base_driver.yaml` start as guesses, and
the drive is open loop (no encoders), so `calibrate_base` measures them against RKO-LIO's
`/odom`. With `base_driver` and the sensors (RKO-LIO) running, robot stationary, ~10 m of
clear space on the surface you'll drive on, e-stop in reach:

```bash
ros2 run hound_base calibrate_base
```

It drives ~22 short moves (~100 s) on `/cmd_vel_teleop` -- straight runs at 0.2/0.4/0.6
m/s, arcs, an accel/brake step, slow runs -- each followed by its exact reverse, so the
robot ends near the start. Any key stops it; so does losing `/odom` or getting more than
`--max-radius` (6 m) from the start. It then prints `max_wheel_speed`, `track_width`
(effective: skid-steer slips, so it's larger than the physical width), the deadband and
acceleration, and the matching Nav2 limits; the raw log goes to `~/hound_calibration_*.csv`.

Put the values in `config/base_driver.yaml`, rebuild `hound_base`, restart `base_driver`
(it reads parameters at startup), and run it again: measured should match commanded
within a few %. `--speeds 0.3 0.6 0.9` etc. to test faster before raising Nav2's limits.

If a side drives backwards for positive `linear.x`, set `invert_left`/`invert_right` first.
