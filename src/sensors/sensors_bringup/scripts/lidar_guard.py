#!/usr/bin/env python3
"""Republish the Ouster cloud for RKO-LIO, dropping scans with broken timing.

In TIME_FROM_ROS_TIME mode the Ouster driver stamps packets on arrival, and while its
stream settles (first ~0.5 s, or after packet loss) it publishes clouds whose stamps
repeat or go backwards, or whose points span several scan periods. RKO-LIO divides by
the time between scans, so two scans with the same stamp give an infinite velocity and
it aborts ("SO3::exp failed! omega: inf"). This drops such clouds:

  - stamp not at least min_step_ratio * scan_period after the last forwarded cloud
  - per-point time field ("t", ns from the cloud stamp) spanning more than
    max_span_ratio * scan_period
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

_NP = {PointField.UINT32: np.uint32, PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64,
       PointField.INT32: np.int32, PointField.UINT16: np.uint16}


class LidarGuard(Node):
    def __init__(self):
        super().__init__('lidar_guard')
        self.period = 1.0 / float(self.declare_parameter('scan_rate', 10.0).value)
        self.min_step = float(self.declare_parameter('min_step_ratio', 0.5).value) * self.period
        self.max_span = float(self.declare_parameter('max_span_ratio', 1.5).value) * self.period
        self.time_field = self.declare_parameter('time_field', 't').value
        self.time_scale = float(self.declare_parameter('time_field_to_seconds', 1e-9).value)
        self.last_stamp = None
        self.dropped = 0
        self.pub = self.create_publisher(PointCloud2, self.declare_parameter('output', '/lidar/points_lio').value,
                                         qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.declare_parameter('input', '/lidar/points').value,
                                 self.callback, qos_profile_sensor_data)

    def span(self, msg):
        field = next((f for f in msg.fields if f.name == self.time_field), None)
        if field is None or field.datatype not in _NP:
            return 0.0
        dtype = np.dtype(_NP[field.datatype]).newbyteorder('>' if msg.is_bigendian else '<')
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        t = rows[:, field.offset:field.offset + dtype.itemsize].copy().view(dtype).ravel()
        return float(t.max() - t.min()) * self.time_scale if t.size else 0.0

    def callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        reason = None
        if self.last_stamp is not None and stamp - self.last_stamp < self.min_step:
            reason = f'stamp step {(stamp - self.last_stamp) * 1000:.1f} ms'
        else:
            span = self.span(msg)
            if span > self.max_span:
                reason = f'points span {span * 1000:.0f} ms'
        if reason:
            self.dropped += 1
            self.get_logger().warn(f'dropped cloud for RKO-LIO ({reason}); {self.dropped} so far',
                                   throttle_duration_sec=1.0)
            return
        self.last_stamp = stamp
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LidarGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
