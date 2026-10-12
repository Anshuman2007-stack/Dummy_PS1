"""Stage 1 - wheel odometry computed from encoder readings.

Sub  /joint_states   sensor_msgs/JointState   wheel joint angles from Gazebo (raw encoder source)
Pub  /wheel_states   sensor_msgs/JointState   per-wheel angle + speed from encoder ticks, 100 Hz
                                              (feedback for the wheel PIDs)
     /odom/wheels    nav_msgs/Odometry        pose + twist from the encoders, 50 Hz
                                              (the EKF uses the twist)

Each joint angle is first quantised to an integer tick count
(``ticks_per_rev``), exactly what a quadrature encoder delivers; speeds and
odometry are then computed from tick differences only (ugv_core.odometry).
"""
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from ugv_core import WHEEL_JOINTS
from ugv_core.kinematics import SkidSteerKinematics
from ugv_core.odometry import angle_to_ticks, EncoderOdometry


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        dp = self.declare_parameter
        dp('wheel_radius', 0.1)
        dp('track_width', 0.4)
        dp('slip_factor', 1.0)
        dp('ticks_per_rev', 8192)
        dp('odom_rate', 50.0)
        dp('wheel_state_rate', 100.0)
        dp('velocity_filter_tau', 0.0)
        dp('odom_var_v', 0.0025)
        dp('odom_var_omega', 0.04)
        dp('joint_names', list(WHEEL_JOINTS))
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        kin = SkidSteerKinematics(g('wheel_radius'), g('track_width'), g('slip_factor'))
        self.tpr = int(g('ticks_per_rev'))
        self.odom = EncoderOdometry(kin, self.tpr, g('velocity_filter_tau'))
        self.joints = list(g('joint_names'))
        self.ws_period = 1.0 / g('wheel_state_rate')
        self.odom_period = 1.0 / g('odom_rate')
        self.var_v, self.var_w = g('odom_var_v'), g('odom_var_omega')
        self.t_ws = self.t_odom = None
        self.warned = False

        self.ws_pub = self.create_publisher(JointState, '/wheel_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom/wheels', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 50)
        self.get_logger().info(
            f'Encoder odometry: {self.tpr} ticks/rev, r={kin.r} m, B={kin.B} m, chi={kin.chi}')

    def joint_cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in self.joints) or len(msg.position) < len(msg.name):
            if not self.warned:
                self.get_logger().warn(f'/joint_states lacks some of {self.joints}: {msg.name}')
                self.warned = True
            return
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        t = stamp.nanoseconds * 1e-9

        if self.t_ws is not None and t - self.t_ws < self.ws_period - 1e-4:
            return
        self.t_ws = t
        ticks = [angle_to_ticks(msg.position[idx[j]], self.tpr) for j in self.joints]
        if not self.odom.update(ticks, t):
            return

        ws = JointState()
        ws.header.stamp = stamp.to_msg()
        ws.name = list(self.joints)
        ws.position = list(self.odom.wheel_angles)
        ws.velocity = list(self.odom.wheel_speeds)
        self.ws_pub.publish(ws)

        if self.t_odom is not None and t - self.t_odom < self.odom_period - 1e-4:
            return
        self.t_odom = t
        self.publish_odom(stamp)

    def publish_odom(self, stamp):
        o = self.odom
        m = Odometry()
        m.header.stamp = stamp.to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = o.x
        m.pose.pose.position.y = o.y
        m.pose.pose.orientation.z = math.sin(o.theta / 2.0)
        m.pose.pose.orientation.w = math.cos(o.theta / 2.0)
        # pose covariance grows without bound (dead reckoning) -> mark as unknown/large
        for i in (0, 7, 14, 21, 28, 35):
            m.pose.covariance[i] = 1e3
        m.twist.twist.linear.x = o.v
        m.twist.twist.angular.z = o.omega
        m.twist.covariance[0] = self.var_v
        m.twist.covariance[35] = self.var_w
        for i in (7, 14, 21, 28):
            m.twist.covariance[i] = 1e-6   # non-holonomic: vy, vz, wx, wy ~ 0
        self.odom_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
