import math
import threading

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class GoalInput(Node):
    def __init__(self):
        super().__init__('goal_input')
        self.pose = None
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Odometry, '/ekf/odom', self.pose_cb, 10)

    def pose_cb(self, msg):
        p = msg.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z),
                         1.0 - 2.0 * p.orientation.z ** 2)
        self.pose = (p.position.x, p.position.y, math.degrees(yaw))

    def send(self, x, y):
        m = PoseStamped()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GoalInput()

    # spin in the background so the pose keeps updating while we wait for input
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print("\nEnter a destination as:  x y   (metres in the odom/world frame, e.g. 4 5)")
    print("Commands: 'pos' = show current position, 'q' = quit\n")

    try:
        while rclpy.ok():
            try:
                text = input("destination> ").strip().replace(',', ' ')
            except EOFError:
                break
            if not text:
                continue
            if text.lower() in ('q', 'quit', 'exit'):
                break
            if text.lower() == 'pos':
                if node.pose is None:
                    print("  No pose yet. Is ekf_node running?")
                else:
                    print(f"  x={node.pose[0]:.2f}  y={node.pose[1]:.2f}  "
                          f"yaw={node.pose[2]:.0f} deg")
                continue

            parts = text.split()
            try:
                if len(parts) != 2:
                    raise ValueError
                x, y = float(parts[0]), float(parts[1])
            except ValueError:
                print("  Please type two numbers, like:  4 5")
                continue

            if node.pose is None:
                print("  Warning: no EKF pose received yet. Is ekf_node running?")
            if node.pub.get_subscription_count() == 0:
                print("  Warning: no controller is listening on /goal_pose.")

            node.send(x, y)
            print(f"  Sent goal ({x:.2f}, {y:.2f}). "
                  "You can type a new one at any time to replace it.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
