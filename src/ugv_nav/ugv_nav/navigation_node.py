"""Stage 3 - target-based navigation.

Sub  /goal_pose        geometry_msgs/PoseStamped   target (x, y) in the odom frame
     /ekf/odom         nav_msgs/Odometry           estimated pose
Pub  /cmd_vel          geometry_msgs/Twist         (v, omega) -> kinematics_node
     /nav/state        std_msgs/String             IDLE | ROTATE | DRIVE | ALIGN | DONE
     /nav/goal_reached std_msgs/Bool               true once per reached goal

Control law: ugv_core/navigator.py (rotate-then-drive, PID on heading and
distance, cos(e) speed scaling, slew-rate limits). /cmd_vel is only
published while a goal is active (plus 1 s of zeros after it), so teleop and
the evaluation scripts can drive the robot when navigation is idle.
"""
import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ugv_core.navigator import DEFAULTS, Navigator


def get_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        for n, v in DEFAULTS.items():
            self.declare_parameter(n, v)
        self.declare_parameter('rate', 20.0)
        self.nav = Navigator(**{n: self.get_parameter(n).value for n in DEFAULTS})
        self.period = 1.0 / self.get_parameter('rate').value

        self.pose = None
        self.t_pose = None
        self.t_last = None
        self.stop_ticks = 0
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(Odometry, '/ekf/odom', self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/nav/state', 10)
        self.reached_pub = self.create_publisher(Bool, '/nav/goal_reached', 10)
        self.create_timer(self.period, self.loop)
        self.get_logger().info('Navigation ready. Send a goal on /goal_pose.')

    def goal_cb(self, msg):
        if msg.header.frame_id not in ('', 'odom'):
            self.get_logger().warn(f"Goal frame '{msg.header.frame_id}' treated as 'odom'.")
        q = msg.pose.orientation
        yaw = get_yaw(q) if (q.x, q.y, q.z, q.w) != (0.0, 0.0, 0.0, 0.0) else None
        self.nav.set_goal(msg.pose.position.x, msg.pose.position.y, yaw)
        self.get_logger().info(
            f'New goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def pose_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, get_yaw(p.orientation))
        self.t_pose = self.get_clock().now()

    def loop(self):
        now = self.get_clock().now()
        dt = self.period if self.t_last is None else (now - self.t_last).nanoseconds * 1e-9
        self.t_last = now
        self.state_pub.publish(String(data=self.nav.state))
        if self.pose is None:
            return
        if (now - self.t_pose).nanoseconds * 1e-9 > 0.5 and self.nav.active:
            self.get_logger().warn('/ekf/odom is stale - stopping', throttle_duration_sec=2.0)
            self.cmd_pub.publish(Twist())
            return

        was_active = self.nav.active
        v, w = self.nav.step(self.pose, min(dt, 0.2))
        if self.nav.just_finished:
            x, y, _ = self.pose
            self.get_logger().info(
                f'Goal reached. Estimated position ({x:.2f}, {y:.2f}), '
                f'distance {self.nav.dist:.3f} m')
            self.reached_pub.publish(Bool(data=True))
            self.stop_ticks = int(1.0 / self.period)

        if was_active or self.nav.active or self.stop_ticks > 0 or v or w:
            self.stop_ticks = max(0, self.stop_ticks - 1)
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = v, w
            self.cmd_pub.publish(cmd)
        if self.nav.active:
            self.get_logger().info(
                f'{self.nav.state} dist={self.nav.dist:.2f} m '
                f'err={math.degrees(self.nav.err):.0f} deg cmd=({v:.2f}, {w:.2f})',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
