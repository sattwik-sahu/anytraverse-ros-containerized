"""Closed-loop velocity correction for base_driver, with RKO-LIO's /odom as feedback.

The hound has no wheel encoders and its speed response isn't proportional to the
Sabertooth command (stall zone at low speed, steeper above -- see calibrate_base), so the
open-loop conversion is only right near the speeds it was calibrated at. This adds a PI
correction on top of it, separately for forward speed and yaw rate:

    v_cmd = v_ref + Kp_v * e_v + Ki_v * integral(e_v)      e_v = v_ref - v_measured
    w_cmd = w_ref + Kp_w * e_w + Ki_w * integral(e_w)      e_w = w_ref - w_measured

v_ref / w_ref is the requested /cmd_vel (the feedforward); the integral term also pushes
through the stall zone. The measured velocity comes from differentiating /odom poses.
"""
import math


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomVelocity:
    """Body-frame forward speed and yaw rate from consecutive odometry poses.

    Uses the messages' own stamps (not arrival time) for dt, and an exponential moving
    average, since RKO-LIO publishes at the lidar rate (10 Hz) with some pose noise.
    """

    def __init__(self, alpha=0.6, max_dt=0.5):
        self.alpha = alpha
        self.max_dt = max_dt
        self.prev = None          # (t, x, y, yaw)
        self.v = self.w = 0.0
        self.valid = False
        self.last_update = None   # caller's clock, for staleness

    def update(self, stamp, x, y, yaw, now):
        if self.prev is not None:
            t0, x0, y0, yaw0 = self.prev
            dt = stamp - t0
            if 1e-3 < dt <= self.max_dt:
                dyaw = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))
                heading = yaw0 + dyaw / 2.0
                v = ((x - x0) * math.cos(heading) + (y - y0) * math.sin(heading)) / dt
                w = dyaw / dt
                a = self.alpha if self.valid else 1.0
                self.v = a * v + (1.0 - a) * self.v
                self.w = a * w + (1.0 - a) * self.w
                self.valid = True
            elif dt > self.max_dt or dt <= 0.0:
                self.valid = False  # gap or time jump: restart the estimate
        self.prev = (stamp, x, y, yaw)
        self.last_update = now

    def fresh(self, now, timeout):
        return self.valid and self.last_update is not None and now - self.last_update < timeout


class PI:
    """PI with a clamped integral and conditional integration (anti-windup)."""

    def __init__(self, kp, ki, i_limit, out_limit):
        self.kp, self.ki = kp, ki
        self.i_limit, self.out_limit = i_limit, out_limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, dt, saturated_sign=0, integrate=True):
        """saturated_sign: +1/-1 if the actuator is saturated in that direction last
        step -- don't integrate further into the saturation. integrate=False: P only."""
        if integrate and not (saturated_sign and math.copysign(1.0, error) == saturated_sign):
            self.integral += error * dt
            self.integral = max(-self.i_limit, min(self.integral, self.i_limit))
        out = self.kp * error + self.ki * self.integral
        return max(-self.out_limit, min(out, self.out_limit))


class VelocityController:
    """Corrects a (v, w) request using measured (v, w)."""

    def __init__(self, kp_v, ki_v, kp_w, ki_w, max_corr_v, max_corr_w, i_limit_v, i_limit_w,
                 stop_threshold_v=0.02, stop_threshold_w=0.02, steady_accel_v=0.1, steady_accel_w=0.2):
        self.pid_v = PI(kp_v, ki_v, i_limit_v, max_corr_v)
        self.pid_w = PI(kp_w, ki_w, i_limit_w, max_corr_w)
        self.stop_v, self.stop_w = stop_threshold_v, stop_threshold_w
        # Integrate only while the request is steady: while Nav2's velocity smoother is
        # ramping it, the lagging robot always looks behind and the integral would wind
        # up, then overshoot once the ramp ends (P still acts during the ramp).
        self.steady_v, self.steady_w = steady_accel_v, steady_accel_w
        self.prev_v_ref = self.prev_w_ref = 0.0
        self.sat_v = self.sat_w = 0

    def reset(self):
        self.pid_v.reset()
        self.pid_w.reset()
        self.sat_v = self.sat_w = 0
        self.prev_v_ref = self.prev_w_ref = 0.0

    def update(self, v_ref, w_ref, v_meas, w_meas, dt):
        """Returns the corrected (v, w). A component requested as ~zero gets no
        correction and its integrator is reset, so a stop is a stop (no creeping)."""
        steady_v = dt > 0 and abs(v_ref - self.prev_v_ref) / dt <= self.steady_v
        steady_w = dt > 0 and abs(w_ref - self.prev_w_ref) / dt <= self.steady_w
        self.prev_v_ref, self.prev_w_ref = v_ref, w_ref
        if abs(v_ref) < self.stop_v:
            self.pid_v.reset()
            v_cmd = v_ref
        else:
            v_cmd = v_ref + self.pid_v.update(v_ref - v_meas, dt, self.sat_v, steady_v)
        if abs(w_ref) < self.stop_w:
            self.pid_w.reset()
            w_cmd = w_ref
        else:
            w_cmd = w_ref + self.pid_w.update(w_ref - w_meas, dt, self.sat_w, steady_w)
        return v_cmd, w_cmd

    def set_saturation(self, sat_v, sat_w):
        """Which way (v, w) hit the motor limit after mixing: -1, 0 or +1 each."""
        self.sat_v, self.sat_w = sat_v, sat_w
