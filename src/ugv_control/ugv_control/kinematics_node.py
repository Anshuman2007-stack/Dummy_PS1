"""Stage 4 - four-wheel differential-drive kinematics.

Sub  /cmd_vel    geometry_msgs/Twist          (v = linear.x, omega = angular.z)
Pub  /wheel_cmd  std_msgs/Float64MultiArray   4 wheel speeds [rad/s], order FL, RL, FR, RR

    w_left  = (v - omega * chi * B / 2) / r      (front_left, rear_left)
    w_right = (v + omega * chi * B / 2) / r      (front_right, rear_right)

If a wheel would exceed max_wheel_speed both sides are scaled equally, which
keeps the commanded path curvature. No /cmd_vel for cmd_timeout -> zeros.
"""
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from ugv_core import WHEEL_NAMES
from ugv_core.kinematics import SkidSteerKinematics


def wheel_array(values):
    m = Float64MultiArray()
    m.layout.dim = [MultiArrayDimension(label=','.join(WHEEL_NAMES), size=4, stride=4)]
    m.data = [float(v) for v in values]
    return m


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        for n, v in (('wheel_radius', 0.1), ('track_width', 0.4), ('slip_factor', 1.0),
                     ('max_wheel_speed', 8.0), ('rate', 50.0), ('cmd_timeout', 0.5)):
            self.declare_parameter(n, v)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.kin = SkidSteerKinematics(g('wheel_radius'), g('track_width'),
                                       g('slip_factor'), g('max_wheel_speed'))
        self.timeout = g('cmd_timeout')
        self.cmd = (0.0, 0.0)
        self.t_cmd = None
        self.pub = self.create_publisher(Float64MultiArray, '/wheel_cmd', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(1.0 / g('rate'), self.tick)
        self.get_logger().info(
            f'Kinematics: r={self.kin.r} B={self.kin.B} chi={self.kin.chi} '
            f'(effective track {self.kin.effective_track:.3f} m), '
            f'max wheel speed {self.kin.max_w} rad/s')

    def cmd_cb(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.t_cmd = self.get_clock().now()

    def tick(self):
        fresh = self.t_cmd is not None and \
            (self.get_clock().now() - self.t_cmd).nanoseconds * 1e-9 < self.timeout
        v, w = self.cmd if fresh else (0.0, 0.0)
        self.pub.publish(wheel_array(self.kin.inverse(v, w)))


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
