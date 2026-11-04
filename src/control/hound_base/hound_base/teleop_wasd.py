"""WASD keyboard teleop in the terminal -> /cmd_vel_teleop (driven by base_driver).

base_driver doesn't apply its Ackermann limit to this topic, and it takes priority
over Nav2's /cmd_vel while it keeps arriving.

  Stopped:  hold A/D to spin in place.
  Driving:  hold W/S; hold A/D to steer -- the steering returns to centre as soon
            as A/D is released (no memory of the last steering command).
  Space or X stops, +/- change the speed, Ctrl-C quits (and stops).

Full steering lock turns at angular_speed while driving at linear_speed.

A terminal sends no key-release events, so "held" means "key repeats still
arriving": a key counts as released a moment after its repeats stop. Only the last
key pressed repeats, so W and A can't both be seen at once: pressing A while
driving keeps the robot driving (and steering) for as long as A is held, and
pressing W/S again re-centres the steering.
"""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

DRIVE = {'w': 1, 's': -1}
TURN = {'a': 1, 'd': -1}
HELP = """
  Stopped:  A/D  hold to spin in place left / right
  Driving:  W/S  hold to drive forward / back
            A/D  hold to steer left / right (back to centre when released)
  space or X  stop     +/-  speed up / down     Ctrl-C  quit
"""


class HeldKey:
    """One key held via terminal autorepeat. The first press waits out the autorepeat
    delay (~0.25-0.6 s) before calling the key released; once repeats arrive,
    release is detected within repeat_timeout."""

    def __init__(self, first_timeout, repeat_timeout):
        self.first_timeout, self.repeat_timeout = first_timeout, repeat_timeout
        self.key = None
        self.pressed_at = self.last_seen = 0.0
        self.repeating = False

    def press(self, ch, now):
        # An autorepeat event follows the previous one within ~30-50 ms; anything later
        # (or another key) is a fresh press, which waits out the autorepeat delay again.
        if ch == self.key and now - self.last_seen < 0.1:
            self.repeating = True
        else:
            self.key, self.pressed_at, self.repeating = ch, now, False
        self.last_seen = now

    def keep_alive(self, now):
        """Still held, but its repeats were interrupted by another key."""
        self.last_seen, self.pressed_at, self.repeating = now, now, False

    def release(self):
        self.key = None

    def expire(self, now):
        """Release the key if its repeats stopped. Returns True if it was released."""
        timeout = self.repeat_timeout if self.repeating else self.first_timeout
        if self.key and now - self.last_seen > timeout:
            self.key = None
            return True
        return False


class TeleopWASD(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.linear = float(self.declare_parameter('linear_speed', 0.3).value)
        self.angular = float(self.declare_parameter('angular_speed', 0.8).value)
        self.max_linear = float(self.declare_parameter('max_linear_speed', 3.0).value)
        self.max_angular = float(self.declare_parameter('max_angular_speed', 5.0).value)
        self.first_timeout = float(self.declare_parameter('first_press_timeout', 0.6).value)
        self.repeat_timeout = float(self.declare_parameter('repeat_timeout', 0.15).value)
        self.pub = self.create_publisher(
            Twist, self.declare_parameter('topic', '/cmd_vel_teleop').value, 10)

    def publish(self, drive, turn):
        msg = Twist()
        if drive:
            msg.linear.x = DRIVE[drive] * self.linear
            # Car-like in reverse: steering left still swings the front left.
            msg.angular.z = DRIVE[drive] * TURN.get(turn, 0) * self.angular
        elif turn:
            msg.angular.z = TURN[turn] * self.angular  # spin in place
        self.pub.publish(msg)

    def status(self, drive, turn):
        mode = {'w': 'forward', 's': 'reverse'}.get(drive) or ('spin' if turn else 'stopped')
        steer = {'a': 'LEFT  ', 'd': 'RIGHT '}.get(turn, 'centre')
        sys.stdout.write(f'\r  {mode:<8} steer {steer}  {self.linear:.2f} m/s  {self.angular:.2f} rad/s   ')
        sys.stdout.flush()


def main():
    if not sys.stdin.isatty():
        sys.exit('teleop_wasd needs an interactive terminal: use `ros2 run`, not a launch file.')
    # No rclpy SIGINT handler: Ctrl-C must raise KeyboardInterrupt here, so the stop
    # messages go out and the terminal is restored before rclpy shuts down.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopWASD()
    settings = termios.tcgetattr(sys.stdin)
    drive = HeldKey(node.first_timeout, node.repeat_timeout)
    turn = HeldKey(node.first_timeout, node.repeat_timeout)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.status(None, None)
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)  # 20 Hz publish
            now = time.monotonic()
            if ready:
                ch = sys.stdin.read(1).lower()
                if ch in DRIVE:
                    if turn.key:
                        # W/S after steering is a new press (its repeats stopped while
                        # A/D was held), so it waits out the autorepeat delay again.
                        drive.release()
                        turn.release()  # A/D no longer repeating: centre the steering
                    drive.press(ch, now)
                elif ch in TURN:
                    if drive.key and turn.key is None:
                        # A/D pressed while driving stops W/S repeating: keep driving
                        # for as long as A/D is held.
                        drive.keep_alive(now)
                    turn.press(ch, now)
                    if drive.key:
                        drive.last_seen = now
                elif ch in (' ', 'x'):
                    drive.release(); turn.release()
                elif ch in ('+', '='):
                    node.linear = min(node.linear + 0.1, node.max_linear)
                    node.angular = min(node.angular + 0.2, node.max_angular)
                elif ch in ('-', '_'):
                    node.linear = max(node.linear - 0.1, 0.1)
                    node.angular = max(node.angular - 0.2, 0.2)
            if turn.expire(now) and drive.key:
                drive.release()  # W/S's repeats ended when A/D was pressed
            drive.expire(now)
            node.status(drive.key, turn.key)
            node.publish(drive.key, turn.key)
    except KeyboardInterrupt:
        pass
    finally:
        # Terminal first: nothing below may leave it in cbreak mode.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            for _ in range(3):
                node.publish(None, None)
        except Exception:  # context already gone; base_driver's cmd_timeout stops it
            pass
        print('\nstopped')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
