"""Calibrate base_driver so /cmd_vel matches how the robot actually moves.

Drives a short scripted sequence on /cmd_vel_teleop (priority over Nav2, no Ackermann
clamp) while recording RKO-LIO's /odom as ground truth, then fits:

  max_wheel_speed  speed scale: measured vs commanded speed on straight runs
  track_width      turn scale (skid-steer slips, so the effective width is larger
                   than the physical one): measured vs commanded yaw rate on arcs
  deadband         slowest commanded speed that actually moves the robot
  accel / decel    from a 0 -> v -> 0 step

and prints the base_driver.yaml values (relative to base_driver's *running*
parameters) and Nav2 limits to match.

Every move is followed by its exact reverse, so the robot ends near where it started.
It stops on any key, if /odom goes quiet, or if the robot gets more than --max-radius
from the start. Needs base_driver and RKO-LIO running, ~2x max-radius of clear space,
the surface you'll drive on, and the e-stop in reach.

    ros2 run hound_base calibrate_base
    ros2 run hound_base calibrate_base -- --speeds 0.3 0.6 0.9 --max-radius 5
"""
import argparse
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class Abort(Exception):
    pass


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calibrator(Node):
    def __init__(self, args):
        super().__init__('calibrate_base')
        self.args = args
        self.pub = self.create_publisher(Twist, args.topic, 10)
        self.odom = []  # (monotonic time, x, y, yaw)
        self.lock = threading.Lock()
        self.create_subscription(Odometry, args.odom, self.on_odom, 50)
        self.segments = []  # dicts: label, kind, v, w, t0, t1
        self.start_xy = None

    def on_odom(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.odom.append((time.monotonic(), p.position.x, p.position.y, yaw_of(p.orientation)))

    # --- helpers -----------------------------------------------------------------
    def base_params(self):
        """base_driver's running values, so corrections are relative to what's in use."""
        names = ['max_wheel_speed', 'track_width', 'max_command', 'ramp_rate', 'min_turning_radius']
        cli = self.create_client(GetParameters, f'/{self.args.base_node}/get_parameters')
        if not cli.wait_for_service(timeout_sec=5.0):
            return None
        fut = cli.call_async(GetParameters.Request(names=names))
        t = time.monotonic()
        while not fut.done() and time.monotonic() - t < 3.0:
            time.sleep(0.05)
        if not fut.done():
            return None
        vals = [v.double_value if v.type == 3 else float(v.integer_value) for v in fut.result().values]
        return dict(zip(names, vals))

    def latest(self):
        with self.lock:
            return self.odom[-1] if self.odom else None

    def check_keys(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
            raise Abort('key pressed')

    def check_safety(self):
        last = self.latest()
        if last is None or time.monotonic() - last[0] > self.args.odom_timeout:
            raise Abort(f'no /odom for {self.args.odom_timeout} s')
        if math.hypot(last[1] - self.start_xy[0], last[2] - self.start_xy[1]) > self.args.max_radius:
            raise Abort(f'more than {self.args.max_radius} m from the start')

    def send(self, v, w, duration):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.check_keys()
            self.check_safety()
            self.pub.publish(msg)
            time.sleep(0.05)

    def stop(self, duration=0.0, interruptible=True):
        msg = Twist()
        end = time.monotonic() + duration
        while True:
            self.pub.publish(msg)
            if time.monotonic() >= end:
                break
            if interruptible:
                self.check_keys()
            time.sleep(0.05)

    def segment(self, label, kind, v, w, duration):
        t0 = time.monotonic()
        self.send(v, w, duration)
        self.segments.append(dict(label=label, kind=kind, v=v, w=w, t0=t0, t1=time.monotonic()))
        print(f'  {label:<34} done')
        self.stop(self.args.pause)

    # --- the sequence ---------------------------------------------------------------
    def run(self):
        a = self.args
        print('\nStraight runs (speed scale):')
        for v in a.speeds:
            self.segment(f'forward  {v:.2f} m/s', 'straight', v, 0.0, a.duration)
            self.segment(f'backward {v:.2f} m/s', 'straight', -v, 0.0, a.duration)
        print('Arcs (turn scale; each arc is retraced in reverse):')
        for w in a.yaw_rates:
            for sign in (1, -1):
                self.segment(f'arc {a.arc_speed:.2f} m/s, {sign * w:+.2f} rad/s', 'arc', a.arc_speed, sign * w, a.duration)
                self.segment(f'  back along it', 'arc', -a.arc_speed, -sign * w, a.duration)
        print('Step (acceleration / braking):')
        self.segment(f'step to {a.step_speed:.2f} m/s', 'step', a.step_speed, 0.0, a.duration)
        self.segment(f'step to {-a.step_speed:.2f} m/s', 'step', -a.step_speed, 0.0, a.duration)
        print('Slow runs (deadband):')
        for v in a.slow_speeds:
            self.segment(f'forward  {v:.2f} m/s', 'slow', v, 0.0, a.slow_duration)
            self.segment(f'backward {v:.2f} m/s', 'slow', -v, 0.0, a.slow_duration)

    # --- analysis ---------------------------------------------------------------------
    def samples(self, t0, t1):
        with self.lock:
            d = np.array([r for r in self.odom if t0 <= r[0] <= t1])
        return d

    def steady(self, seg):
        """Mean forward speed and yaw rate over the end of a segment (after the ramp)."""
        d = self.samples(seg['t1'] - self.args.steady_window, seg['t1'])
        if len(d) < 4:
            return None
        t, x, y, yaw = d.T
        yaw = np.unwrap(yaw)
        heading = (yaw[:-1] + yaw[1:]) / 2
        ds = np.diff(x) * np.cos(heading) + np.diff(y) * np.sin(heading)  # along the robot's x
        s = np.concatenate([[0.0], np.cumsum(ds)])
        return np.polyfit(t, s, 1)[0], np.polyfit(t, yaw, 1)[0]

    def along_track(self, t0, t1):
        """(t, distance along the robot's x since t0) from /odom."""
        d = self.samples(t0, t1)
        if len(d) < 4:
            return None, None
        t, x, y, yaw = d.T
        yaw = np.unwrap(yaw)
        heading = (yaw[:-1] + yaw[1:]) / 2
        ds = np.diff(x) * np.cos(heading) + np.diff(y) * np.sin(heading)
        return t - t0, np.concatenate([[0.0], np.cumsum(ds)])

    @staticmethod
    def fit_ramp(t, s, v0, v1):
        """Constant acceleration from v0 to v1 after a delay: grid-search the acceleration
        and delay that best reproduce the measured positions. Works at RKO-LIO's 10 Hz,
        where a fast ramp spans only a couple of samples (differencing can't resolve it)."""
        dv = v1 - v0
        if abs(dv) < 1e-3 or len(t) < 6:
            return None
        best = None
        for delay in np.arange(0.0, 0.5, 0.02):
            for accel in np.linspace(0.1, 8.0, 200):
                tr = abs(dv) / accel  # ramp duration
                tt = np.clip(t - delay, 0.0, None)
                ramp = np.minimum(tt, tr)
                # v0 * t: moving at v0 throughout; the ramp adds the change in speed
                model = v0 * t + (np.sign(dv) * 0.5 * accel * ramp ** 2 + dv * np.clip(tt - tr, 0.0, None))
                off = np.mean(s - model)
                err = float(np.sum((s - model - off) ** 2))
                if best is None or err < best[0]:
                    best = (err, accel, delay)
        return best[1], best[2]

    def report(self, base):
        a = self.args
        rows = []
        print('\n=== Measurements ===')
        print(f'{"segment":<36} {"cmd v":>6} {"meas v":>7} {"cmd w":>6} {"meas w":>7}')
        for seg in self.segments:
            m = self.steady(seg)
            if m is None:
                continue
            seg['vm'], seg['wm'] = m
            rows.append(seg)
            print(f'{seg["label"]:<36} {seg["v"]:6.2f} {m[0]:7.3f} {seg["w"]:6.2f} {m[1]:7.3f}')

        straight = [s for s in rows if s['kind'] == 'straight']
        arcs = [s for s in rows if s['kind'] == 'arc' and abs(s['w']) > 0]
        slow = [s for s in rows if s['kind'] == 'slow']
        out = {}

        if straight:
            vc = np.array([abs(s['v']) for s in straight])
            vm = np.array([s['vm'] * np.sign(s['v']) for s in straight])
            gain, offset = np.polyfit(vc, vm, 1) if len(set(vc)) > 1 else (np.mean(vm / vc), 0.0)
            through_zero = float(np.sum(vc * vm) / np.sum(vc * vc))  # fit without offset
            out['gain'] = through_zero
            fwd = [s['vm'] / s['v'] for s in straight if s['v'] > 0]
            rev = [s['vm'] / s['v'] for s in straight if s['v'] < 0]
            print(f'\nspeed: measured = {through_zero:.3f} x commanded (fit with offset: '
                  f'{gain:.3f} x {offset:+.3f} m/s); forward {np.mean(fwd):.3f}, reverse {np.mean(rev):.3f}')
            if abs(np.mean(fwd) - np.mean(rev)) > 0.1 * through_zero:
                print('  forward and reverse differ by >10%: check motor wiring / tyre pressure')

        if arcs and 'gain' in out:
            # base_driver: differential = w_cmd * track_param (commanded m/s); true m/s = gain x that;
            # true yaw rate = true differential / effective track.
            ratios = [s['w'] / s['wm'] for s in arcs if abs(s['wm']) > 1e-3]
            if ratios:
                out['turn_ratio'] = float(np.median(ratios))
                print(f'turn: commanded / measured yaw rate = {out["turn_ratio"]:.3f} '
                      f'(per arc: {", ".join(f"{r:.2f}" for r in ratios)})')
            for w in sorted({abs(s['w']) for s in arcs}):
                meas = [abs(s['vm'] / s['wm']) for s in arcs if abs(s['w']) == w and abs(s['wm']) > 1e-3]
                if meas:
                    print(f'  arc radius at {w:.2f} rad/s: commanded {a.arc_speed / w:.2f} m, measured {np.mean(meas):.2f} m')

        if slow:
            moving = [abs(s['v']) for s in slow if abs(s['vm']) >= 0.5 * abs(s['v'])]
            stalled = [abs(s['v']) for s in slow if abs(s['vm']) < 0.5 * abs(s['v'])]
            out['deadband'] = (max(stalled) if stalled else 0.0, min(moving) if moving else None)
            print(f'deadband: stalls at {sorted(set(stalled)) or "none"} m/s, moves at {sorted(set(moving)) or "none"} m/s')

        acc, dec = [], []
        for seg in (s for s in rows if s['kind'] == 'step'):
            v_ss = seg['vm']
            t, d = self.along_track(seg['t0'], min(seg['t0'] + 1.5, seg['t1']))  # speeding up
            r = self.fit_ramp(t, d, 0.0, v_ss) if t is not None else None
            t, d = self.along_track(seg['t1'], seg['t1'] + min(1.5, a.pause))       # braking
            f = self.fit_ramp(t, d, v_ss, 0.0) if t is not None else None
            if r:
                acc.append(r[0])
            if f:
                dec.append(f[0])
        # base_driver ramps its motor command at ramp_rate, so the robot can't change speed
        # faster than ramp_rate / max_command * (true) max_wheel_speed. At RKO-LIO's 10 Hz a
        # ramp much shorter than ~0.3 s is barely visible in the positions, so fits above
        # that limit are noise: cap them.
        limit = None
        if base and base.get('ramp_rate') and base.get('max_command') and 'gain' in out:
            limit = base['ramp_rate'] / base['max_command'] * base['max_wheel_speed'] * out['gain']
        for name, vals in (('accel', acc), ('decel', dec)):
            if not vals:
                continue
            est = min(vals)
            capped = limit is not None and est > limit
            out[name] = min(est, limit) if limit is not None else est
            print(f'{"accel" if name == "accel" else "braking"}: fits {", ".join(f"{x:.2f}" for x in vals)} m/s^2'
                  + (f' -> {out[name]:.2f} (capped at base_driver ramp limit {limit:.2f})' if capped else
                     f' (base_driver ramp limit {limit:.2f})' if limit else ''))

        print('\n=== Recommended settings ===')
        if base is None:
            print('(base_driver parameters unknown: pass --max-wheel-speed/--track-width to get values)')
            base = {'max_wheel_speed': a.max_wheel_speed, 'track_width': a.track_width}
        if 'gain' in out and base.get('max_wheel_speed'):
            mws = base['max_wheel_speed'] * out['gain']
            print(f'hound_base/config/base_driver.yaml:\n  max_wheel_speed: {mws:.3f}    # was {base["max_wheel_speed"]}')
            if 'turn_ratio' in out and base.get('track_width'):
                tw = base['track_width'] * out['gain'] * out['turn_ratio']
                print(f'  track_width: {tw:.3f}        # was {base["track_width"]} (effective, skid-steer)')
        top = max(a.speeds)
        print('zeta_navigation/config/nav2_params.yaml (tested up to '
              f'{top} m/s -- re-run with higher --speeds before allowing more):')
        print(f'  FollowPath.desired_linear_vel: {0.8 * top:.2f}')
        print(f'  velocity_smoother.max_velocity: [{top:.2f}, 0.0, ...]')
        if out.get('deadband') and out['deadband'][1]:
            print(f'  FollowPath.min_approach_linear_velocity / regulated_linear_scaling_min_speed: '
                  f'>= {out["deadband"][1]:.2f}   # below this the robot stalls')
        if out.get('accel'):
            print(f'  velocity_smoother.max_accel: [{0.8 * out["accel"]:.2f}, 0.0, ...]   # 80% of measured')
        if out.get('decel'):
            print(f'  velocity_smoother.max_decel: [{-0.8 * out["decel"]:.2f}, 0.0, ...]')
        print('Restart base_driver after editing base_driver.yaml (it reads parameters at '
              'startup), then run this again: measured should match commanded within a few %.')

    def save(self, path):
        with self.lock:
            odom = list(self.odom)
        with open(path, 'w') as f:
            f.write('# segments: label,kind,v,w,t0,t1\n')
            for s in self.segments:
                f.write(f'# {s["label"]},{s["kind"]},{s["v"]},{s["w"]},{s["t0"]:.3f},{s["t1"]:.3f}\n')
            f.write('t,x,y,yaw\n')
            for r in odom:
                f.write(','.join(f'{x:.4f}' for x in r) + '\n')
        print(f'raw log: {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speeds', type=float, nargs='+', default=[0.2, 0.4, 0.6], help='straight-run speeds, m/s')
    ap.add_argument('--arc-speed', type=float, default=0.3, help='forward speed on arcs, m/s')
    ap.add_argument('--yaw-rates', type=float, nargs='+', default=[0.2, 0.4], help='arc yaw rates, rad/s')
    ap.add_argument('--step-speed', type=float, default=0.5, help='speed for the accel/brake step, m/s')
    ap.add_argument('--slow-speeds', type=float, nargs='+', default=[0.05, 0.1, 0.15], help='deadband test, m/s')
    ap.add_argument('--duration', type=float, default=3.0, help='seconds per move')
    ap.add_argument('--slow-duration', type=float, default=2.5)
    ap.add_argument('--pause', type=float, default=1.5, help='seconds stopped between moves')
    ap.add_argument('--steady-window', type=float, default=1.5, help='end of each move used for the fit, s')
    ap.add_argument('--max-radius', type=float, default=6.0, help='abort beyond this distance from the start, m')
    ap.add_argument('--odom-timeout', type=float, default=0.5)
    ap.add_argument('--topic', default='/cmd_vel_teleop')
    ap.add_argument('--odom', default='/odom')
    ap.add_argument('--base-node', default='base_driver')
    ap.add_argument('--max-wheel-speed', type=float, default=None, help='if base_driver is not reachable')
    ap.add_argument('--track-width', type=float, default=None, help='if base_driver is not reachable')
    ap.add_argument('--log', default=os.path.expanduser(f'~/hound_calibration_{time.strftime("%Y%m%d_%H%M%S")}.csv'))
    args = ap.parse_args([a for a in sys.argv[1:] if a != '--'])
    if not sys.stdin.isatty():
        sys.exit('calibrate_base needs an interactive terminal (any key aborts): use `ros2 run`.')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Calibrator(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    # DDS discovery takes a moment: wait for base_driver before checking for it.
    t = time.monotonic()
    while node.pub.get_subscription_count() == 0 and time.monotonic() - t < 5.0:
        time.sleep(0.1)
    base = node.base_params()
    print(f'base_driver parameters: {base or "NOT reachable (is base_driver running?)"}')
    if node.pub.get_subscription_count() == 0:
        print(f'WARNING: nothing subscribes to {args.topic} -- base_driver is not running, the robot will not move.')
    t = time.monotonic()
    while node.latest() is None and time.monotonic() - t < 3.0:
        time.sleep(0.1)
    if node.latest() is None:
        executor.shutdown()
        sys.exit(f'No {args.odom} -- start RKO-LIO (sensors_bringup camera_lidar.launch.py) first.')
    n = len(args.speeds) * 2 + len(args.yaw_rates) * 4 + 2 + len(args.slow_speeds) * 2
    total = n * args.pause + (n - 2 * len(args.slow_speeds)) * args.duration + 2 * len(args.slow_speeds) * args.slow_duration
    print(f'\n{n} moves, ~{total:.0f} s, up to {max(args.speeds + [args.step_speed]):.2f} m/s, '
          f'each followed by its reverse. Aborts beyond {args.max_radius} m from the start.')
    print('Clear space, the surface you will drive on, e-stop in reach.')
    input('Press Enter to start (then ANY key stops the robot) ... ')

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        last = node.latest()
        node.start_xy = (last[1], last[2])
        node.stop(1.0)
        node.run()
        print('\nSequence complete.')
    except Abort as e:
        print(f'\nABORTED: {e}')
    except KeyboardInterrupt:
        print('\nABORTED: Ctrl-C')
    finally:
        node.stop(0.5, interruptible=False)  # always send zeros, whatever ended the run
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    node.save(args.log)
    node.report(base)
    executor.shutdown()
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
