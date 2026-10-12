"""Stage 2 - EKF sensor fusion (x, y, theta, v, omega).

Sub  /odom/wheels  nav_msgs/Odometry      encoder twist (v, omega)           50 Hz
     /imu/data     sensor_msgs/Imu        gyro z (+ optional orientation)    100 Hz
     /gnss/fix     sensor_msgs/NavSatFix  position, chi-square gated           5 Hz
Pub  /ekf/odom     nav_msgs/Odometry      fused state + covariance  (frame odom -> base_link)
     TF            odom -> base_link

Each measurement is fused at its own header time stamp (the filter is
propagated to that time first), so the different, jittery sensor rates are
handled exactly. GPS fixes with STATUS_NO_FIX are ignored; while GPS is
missing the filter dead-reckons on wheels + gyro and its position covariance
grows, then the next valid fix pulls it back (see ugv_core/ekf.py).

GPS -> odom: fixes are converted to east/north metres about the datum
(datum_lat, datum_lon) = the Gazebo world origin = where the robot spawns
with yaw 0, so the odom frame coincides with the world frame.
"""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster
from ugv_core.ekf import EKF, wrap
from ugv_core.geo import ll_to_en


def get_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        defaults = {'publish_rate': 50.0, 'publish_tf': True, 'q_pos': 1e-4, 'q_theta': 1e-5,
                    'q_v': 0.5, 'q_omega': 0.5, 'gyro_var': 1e-4, 'use_imu_yaw': False,
                    'imu_yaw_var': 4e-4, 'gps_gate': 9.21, 'gps_max_rejects': 10,
                    'gps_timeout': 1.0, 'datum_lat': 22.62, 'datum_lon': 75.69,
                    'odom_var_v': 0.0025, 'odom_var_omega': 0.04}
        for n, v in defaults.items():
            self.declare_parameter(n, v)
        self.p = {n: self.get_parameter(n).value for n in defaults}
        p = self.p
        self.ekf = EKF(p['q_pos'], p['q_theta'], p['q_v'], p['q_omega'],
                       p['gps_gate'], int(p['gps_max_rejects']))
        self.imu_yaw_offset = None
        self.gps_lost = False
        self.n_gps = 0

        self.create_subscription(Odometry, '/odom/wheels', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gnss/fix', self.gps_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Odometry, '/ekf/odom', 10)
        self.tf = TransformBroadcaster(self) if p['publish_tf'] else None
        self.create_timer(1.0 / p['publish_rate'], self.publish)
        self.get_logger().info('EKF started, waiting for /odom/wheels ...')

    # ------------------------------------------------------------------ utils
    def stamp_s(self, header):
        t = Time.from_msg(header.stamp).nanoseconds
        return (t if t > 0 else self.get_clock().now().nanoseconds) * 1e-9

    # ---------------------------------------------------------- measurements
    def odom_cb(self, msg):
        t = self.stamp_s(msg.header)
        if not self.ekf.initialized:
            # Robot spawns at the odom/world origin with yaw 0.
            self.ekf.initialize(t, 0.0, 0.0, 0.0)
            self.get_logger().info('EKF initialised at (0, 0, 0). Ready for goals.')
            return
        cv = msg.twist.covariance
        var_v = cv[0] if cv[0] > 0 else self.p['odom_var_v']
        var_w = cv[35] if cv[35] > 0 else self.p['odom_var_omega']
        self.ekf.update_wheel_odom(t, msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                                   var_v, var_w)

    def imu_cb(self, msg):
        if not self.ekf.initialized:
            return
        t = self.stamp_s(msg.header)
        self.ekf.update_gyro(t, msg.angular_velocity.z, self.p['gyro_var'])
        q = msg.orientation
        if not self.p['use_imu_yaw'] or (q.x == q.y == q.z == q.w == 0.0):
            return
        yaw = get_yaw(q)
        if self.imu_yaw_offset is None:
            self.imu_yaw_offset = wrap(self.ekf.state[2] - yaw)
            return
        self.ekf.update_yaw(t, wrap(yaw + self.imu_yaw_offset), self.p['imu_yaw_var'])

    def gps_cb(self, msg):
        if not self.ekf.initialized:
            return
        if msg.status.status < NavSatStatus.STATUS_FIX or \
                not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        t = self.stamp_s(msg.header)
        east, north = ll_to_en(msg.latitude, msg.longitude,
                               self.p['datum_lat'], self.p['datum_lon'])
        var = msg.position_covariance[0] \
            if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN \
            and msg.position_covariance[0] > 0 else 0.09
        if self.gps_lost:
            self.gps_lost = False
            x, y = self.ekf.state[:2]
            self.get_logger().info(
                f'GPS back. Jump to fix: {math.hypot(east - x, north - y):.2f} m')
        res = self.ekf.update_gps(t, east, north, var)
        self.n_gps += 1
        if res == 'resync':
            self.get_logger().warn('GPS rejected repeatedly - covariance inflated, resynced.')
        elif res == 'rejected':
            self.get_logger().debug(f'GPS outlier rejected (NIS {self.ekf.last_nis:.1f})')

    # ---------------------------------------------------------------- output
    def publish(self):
        if not self.ekf.initialized:
            return
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if not self.gps_lost and self.n_gps > 0 and self.ekf.gps_age(t) > self.p['gps_timeout']:
            self.gps_lost = True
            self.get_logger().warn('GPS lost - dead-reckoning on wheel odometry + gyro.')
        x, y, th, v, w = self.ekf.state_at(t)
        P = self.ekf.P
        m = Odometry()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(th / 2.0)
        m.pose.pose.orientation.w = math.cos(th / 2.0)
        m.twist.twist.linear.x = v
        m.twist.twist.angular.z = w
        # 6x6 row-major (x, y, z, roll, pitch, yaw)
        idx = {0: 0, 1: 1, 2: 5}
        for a, ia in idx.items():
            for b, ib in idx.items():
                m.pose.covariance[ia * 6 + ib] = float(P[a, b])
        m.twist.covariance[0] = float(P[3, 3])
        m.twist.covariance[5] = m.twist.covariance[30] = float(P[3, 4])
        m.twist.covariance[35] = float(P[4, 4])
        self.pub.publish(m)

        if self.tf:
            tf = TransformStamped()
            tf.header = m.header
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation = m.pose.pose.orientation
            self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
