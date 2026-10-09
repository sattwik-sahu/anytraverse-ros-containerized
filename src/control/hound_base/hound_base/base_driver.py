"""/cmd_vel -> Sabertooth motor commands for the hound (differential/skid-steer).

Two Sabertooths, wired as in the hound teleop script:
  A (left_port,  address_a): motor 1 = left,  motor 2 = right
  B (right_port, address_b): motor 1 = right, motor 2 = left

Open loop: wheel speed (m/s) maps linearly to the Sabertooth command, with
max_wheel_speed being the wheel speed at max_command. Calibrate it and
track_width on the robot (see README.md).

Differential drive by default: /cmd_vel is applied as-is, so it can turn in place.
Ackermann mimic (ackermann: true) instead clamps the turn rate to
|linear.x| / min_turning_radius -- no turning in place, never tighter than that
radius; keep min_turning_radius equal to Nav2's GridBased.minimum_turning_radius.

Teleop (teleop_wasd) publishes on /cmd_vel_teleop instead, which is NOT clamped
(it can turn in place) and takes priority over /cmd_vel while it keeps arriving:
Nav2 is ignored until the teleop has been quiet for cmd_timeout.

Closed loop (closed_loop: true): a PI correction on forward speed and yaw rate, with
RKO-LIO's /odom as feedback (see velocity_pid.py), on top of the open-loop conversion.
Falls back to open loop whenever /odom is stale (odom_timeout), and resets its
integrators when the request is zero, on the handbrake and on a cmd timeout.

Stops the motors when:
  - no /cmd_vel or /cmd_vel_teleop for cmd_timeout seconds (Nav2 or teleop died),
  - a roads_msgs/VehicleControl with handbrake=True arrives on
    /vehicle_control_roads (zeta_navigation's PauseIfUncertain/ApplyHandbrake BT
    nodes), held for handbrake_hold seconds after the last one,
  - the node shuts down.
"""
import math
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from hound_base.sabertooth import set_motor_packet
from hound_base.velocity_pid import OdomVelocity, VelocityController, yaw_of

try:
    from roads_msgs.msg import VehicleControl
except ImportError:  # roads_ros not built: no BT handbrake input
    VehicleControl = None


class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')
        p = self.declare_parameter
        self.left_port = p('port_a', '/dev/ttyACM0').value
        self.right_port = p('port_b', '/dev/ttyACM1').value
        self.baudrate = int(p('baudrate', 9600).value)
        self.address_a = int(p('address_a', 128).value)
        self.address_b = int(p('address_b', 129).value)
        self.track_width = float(p('track_width', 0.6).value)
        self.ackermann = bool(p('ackermann', False).value)
        self.min_turning_radius = float(p('min_turning_radius', 1.5).value)
        self.max_wheel_speed = float(p('max_wheel_speed', 2.0).value)
        self.max_command = float(p('max_command', 2000).value)
        self.ramp_rate = float(p('ramp_rate', 3000.0).value)  # command units per second
        self.cmd_timeout = float(p('cmd_timeout', 0.5).value)
        self.handbrake_hold = float(p('handbrake_hold', 0.3).value)
        self.rate = float(p('rate', 30.0).value)
        self.invert_left = bool(p('invert_left', False).value)
        self.invert_right = bool(p('invert_right', False).value)
        # Closed-loop velocity correction (PI on v and w, /odom as feedback).
        self.closed_loop = bool(p('closed_loop', False).value)
        self.odom_timeout = float(p('odom_timeout', 0.3).value)
        self.cmd_vel_topic = str(p("cmd_vel_topic", "/cmd_vel").value)
        self.controller = VelocityController(
            kp_v=float(p('kp_v', 0.1).value), ki_v=float(p('ki_v', 0.5).value),
            kp_w=float(p('kp_w', 0.2).value), ki_w=float(p('ki_w', 0.6).value),
            max_corr_v=float(p('max_correction_v', 0.6).value),
            max_corr_w=float(p('max_correction_w', 1.5).value),
            i_limit_v=float(p('integral_limit_v', 1.0).value),
            i_limit_w=float(p('integral_limit_w', 2.0).value))
        self.odom_velocity = OdomVelocity()
        self.v_ref = self.w_ref = 0.0
        self.using_feedback = None  # for logging changes

        self.ser_a = None
        self.ser_b = None
        self.left = 0.0
        self.right = 0.0
        self.target_left = 0.0
        self.target_right = 0.0
        self.last_cmd_time = -float('inf')
        self.last_teleop_time = -float('inf')
        self.handbrake_until = -float('inf')
        self.last_tick = time.monotonic()

        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Twist, p('teleop_topic', '/cmd_vel_teleop').value, self.teleop_callback, 10)
        if VehicleControl is not None:
            self.create_subscription(VehicleControl, p('vehicle_control_topic', '/vehicle_control_roads').value,
                                     self.vehicle_control_callback, 10)
        else:
            self.get_logger().warn('roads_msgs not found: BT handbrake on /vehicle_control_roads is ignored.')
        if self.closed_loop:
            self.create_subscription(Odometry, p('odom_topic', '/odom').value, self.odom_callback, 10)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'track_width={self.track_width} m, max_wheel_speed={self.max_wheel_speed} m/s '
            f'at command {self.max_command:.0f}, '
            + (f'Ackermann mimic, min turning radius {self.min_turning_radius} m' if self.ackermann
               else 'differential (turns in place)')
            + (', closed loop on /odom' if self.closed_loop else ', open loop'))

    def cmd_vel_callback(self, msg):
        if time.monotonic() - self.last_teleop_time < self.cmd_timeout:
            return  # teleop has the robot
        v, w = msg.linear.x, msg.angular.z
        if self.ackermann:
            max_w = abs(v) / self.min_turning_radius
            w = max(-max_w, min(w, max_w))
        self.set_target(v, w)

    def teleop_callback(self, msg):
        self.last_teleop_time = time.monotonic()
        self.set_target(msg.linear.x, msg.angular.z)

    def set_target(self, v, w):
        """Store the requested velocity; tick() corrects (closed loop) and mixes it."""
        self.v_ref, self.w_ref = v, w
        self.last_cmd_time = time.monotonic()

    def mix(self, v, w):
        """(v, w) -> Sabertooth commands, and whether it hit the motor limit."""
        half = w * self.track_width / 2.0
        left, right = self.to_command(v - half), self.to_command(v + half)
        # Scale both sides together so a saturated turn keeps its curvature.
        peak = max(abs(left), abs(right))
        saturated = peak > self.max_command
        if saturated:
            left, right = left * self.max_command / peak, right * self.max_command / peak
        return left, right, saturated

    def odom_callback(self, msg):
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_velocity.update(stamp, p.position.x, p.position.y, yaw_of(p.orientation), time.monotonic())

    def vehicle_control_callback(self, msg):
        if msg.handbrake:
            self.handbrake_until = time.monotonic() + self.handbrake_hold

    def to_command(self, wheel_speed):
        return wheel_speed / self.max_wheel_speed * self.max_command

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        if now < self.handbrake_until:
            # Brake: no ramp, and ignore cmd_vel that was queued before it.
            self.left = self.right = self.target_left = self.target_right = 0.0
            self.v_ref = self.w_ref = 0.0
            self.controller.reset()
        else:
            if now - self.last_cmd_time > self.cmd_timeout:
                self.v_ref = self.w_ref = 0.0
            v, w = self.v_ref, self.w_ref
            feedback = self.closed_loop and self.odom_velocity.fresh(now, self.odom_timeout)
            if feedback:
                v, w = self.controller.update(v, w, self.odom_velocity.v, self.odom_velocity.w, dt)
            else:
                self.controller.reset()  # no stale integral when feedback comes back
            if self.closed_loop and feedback != self.using_feedback:
                # Separate calls: an rclpy log call site can't change severity.
                if feedback:
                    self.get_logger().info('closed loop: using /odom feedback')
                else:
                    self.get_logger().warn(f'closed loop: /odom stale (> {self.odom_timeout} s) -- open loop')
                self.using_feedback = feedback
            self.target_left, self.target_right, saturated = self.mix(v, w)
            self.controller.set_saturation(
                int(math.copysign(1, v)) if saturated and abs(v) > 1e-3 else 0,
                int(math.copysign(1, w)) if saturated and abs(w) > 1e-3 else 0)
            step = self.ramp_rate * dt
            self.left = self.ramp(self.left, self.target_left, step)
            self.right = self.ramp(self.right, self.target_right, step)
        self.send(self.left, self.right)

    @staticmethod
    def ramp(current, target, step):
        return min(current + step, target) if current < target else max(current - step, target)

    def send(self, left, right):
        left = -left if self.invert_left else left
        right = -right if self.invert_right else right
        try:
            if self.ser_a is None:
                self.ser_a = serial.Serial(self.left_port, self.baudrate, timeout=0.1, write_timeout=0.1)
                self.get_logger().info(f'Connected to {self.left_port}')
            if self.ser_b is None:
                self.ser_b = serial.Serial(self.right_port, self.baudrate, timeout=0.1, write_timeout=0.1)
                self.get_logger().info(f'Connected to {self.right_port}')
            self.ser_a.write(set_motor_packet(self.address_a, 1, left)
                             + set_motor_packet(self.address_a, 2, right))
            self.ser_b.write(set_motor_packet(self.address_b, 1, right)
                             + set_motor_packet(self.address_b, 2, left))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Sabertooth serial: {exc}; reconnecting', throttle_duration_sec=2.0)
            self.close()

    def close(self):
        for ser in (self.ser_a, self.ser_b):
            if ser is not None:
                try:
                    ser.close()
                except (serial.SerialException, OSError):
                    pass
        self.ser_a = self.ser_b = None

    def stop(self):
        self.send(0, 0)
        self.close()


def main():
    rclpy.init()
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
