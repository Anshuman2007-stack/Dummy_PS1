"""Stage 1 - simulated GPS/GNSS receiver with noise and dropout.

Sub  /true_odom     nav_msgs/Odometry      Gazebo ground-truth pose (world = ENU at the datum)
     /gnss/enable   std_msgs/Bool          false = simulate signal loss, true = restore
Pub  /gnss/fix      sensor_msgs/NavSatFix  ``rate`` Hz, N(0, sigma^2) horizontal noise

During a dropout the node keeps publishing at the same rate but with
``status = STATUS_NO_FIX`` and NaN coordinates, as a real receiver does;
consumers must check the status.

A scheduled dropout (``dropout_start`` / ``dropout_duration``, seconds after
the first fix) makes dropout experiments reproducible; ``/gnss/enable`` is
for manual tests:

    ros2 topic pub --once /gnss/enable std_msgs/msg/Bool "{data: false}"
"""
import math
import random

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool
from ugv_core.geo import en_to_ll


class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        for n, v in (('rate', 5.0), ('sigma', 0.3), ('datum_lat', 22.62), ('datum_lon', 75.69),
                     ('dropout_start', 0.0), ('dropout_duration', 0.0), ('seed', -1)):
            self.declare_parameter(n, v)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.sigma = g('sigma')
        self.lat0, self.lon0 = g('datum_lat'), g('datum_lon')
        self.drop_start, self.drop_dur = g('dropout_start'), g('dropout_duration')
        self.rng = random.Random(None if g('seed') < 0 else g('seed'))

        self.xy = None
        self.enabled = True
        self.t_first = None
        self.was_ok = True
        self.create_subscription(Odometry, '/true_odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/gnss/enable', self.enable_cb, 10)
        self.pub = self.create_publisher(NavSatFix, '/gnss/fix', qos_profile_sensor_data)
        self.create_timer(1.0 / g('rate'), self.tick)
        msg = f'GPS sim: {g("rate")} Hz, sigma {self.sigma} m'
        if self.drop_dur > 0:
            msg += f', dropout {self.drop_start:.0f}-{self.drop_start + self.drop_dur:.0f} s'
        self.get_logger().info(msg)

    def odom_cb(self, msg):
        self.xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def enable_cb(self, msg):
        self.enabled = bool(msg.data)
        self.get_logger().warn(f'GPS manually {"enabled" if self.enabled else "DISABLED"}')

    def tick(self):
        if self.xy is None:
            return
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self.t_first is None:
            self.t_first = t
        rel = t - self.t_first
        scheduled_drop = self.drop_dur > 0 and \
            self.drop_start <= rel < self.drop_start + self.drop_dur
        ok = self.enabled and not scheduled_drop
        if ok != self.was_ok:
            self.get_logger().warn('GPS signal restored' if ok else 'GPS DROPOUT started')
            self.was_ok = ok

        fix = NavSatFix()
        fix.header.stamp = now.to_msg()
        fix.header.frame_id = 'gnss_link'
        fix.status.service = NavSatStatus.SERVICE_GPS
        if not ok:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            fix.latitude = fix.longitude = fix.altitude = float('nan')
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub.publish(fix)
            return
        east = self.xy[0] + self.rng.gauss(0.0, self.sigma)     # world +X = east
        north = self.xy[1] + self.rng.gauss(0.0, self.sigma)    # world +Y = north
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude, fix.longitude = en_to_ll(east, north, self.lat0, self.lon0)
        fix.altitude = 0.0
        v = self.sigma ** 2
        fix.position_covariance = [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, 4.0 * v]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        if math.isfinite(fix.latitude):
            self.pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
