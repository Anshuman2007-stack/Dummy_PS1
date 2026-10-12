"""Calibrate the skid-steer slip factor chi (robot.yaml: slip_factor).

Spins the robot on the spot and compares the yaw rate the wheels imply
(assuming chi = 1) with the yaw rate the gyro measures:

    chi = r * (w_right - w_left) / (B * omega_gyro)

Run with Gazebo + sensors + kinematics + wheel PID up and NOTHING else
publishing /cmd_vel:

    ros2 launch ugv_control control.launch.py
    ros2 run ugv_control slip_calibration --ros-args -p use_sim_time:=true
"""
import statistics

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from ugv_core import WHEEL_JOINTS


class SlipCalibration(Node):
    def __init__(self):
        super().__init__('slip_calibration')
        for n, v in (('wheel_radius', 0.1), ('track_width', 0.4), ('omega', 0.8),
                     ('duration', 10.0), ('settle', 3.0)):
            self.declare_parameter(n, v)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.r, self.B = g('wheel_radius'), g('track_width')
        self.omega, self.dur, self.settle = g('omega'), g('duration'), g('settle')
        self.gyro, self.diff = [], []
        self.t0 = None
        self.done = False
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)
        self.create_subscription(JointState, '/wheel_states', self.ws_cb, 50)
        self.create_timer(0.05, self.tick)

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9 - (self.t0 or 0.0)

    def recording(self):
        return self.t0 is not None and self.settle <= self.t() < self.dur

    def imu_cb(self, msg):
        if self.recording():
            self.gyro.append(msg.angular_velocity.z)

    def ws_cb(self, msg):
        if not self.recording():
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        w = [msg.velocity[idx[j]] for j in WHEEL_JOINTS]
        self.diff.append((w[2] + w[3]) / 2 - (w[0] + w[1]) / 2)

    def tick(self):
        if self.done:
            return
        if self.t0 is None:
            if self.get_clock().now().nanoseconds > 0:
                self.t0 = self.get_clock().now().nanoseconds * 1e-9
            return
        cmd = Twist()
        if self.t() < self.dur:
            cmd.angular.z = self.omega
            self.pub.publish(cmd)
            return
        self.pub.publish(cmd)
        self.done = True
        if not self.gyro or not self.diff:
            self.get_logger().error('No /imu/data or /wheel_states received.')
            return
        wg = statistics.mean(self.gyro)
        wwheels = self.r * statistics.mean(self.diff) / self.B
        chi = wwheels / wg if abs(wg) > 1e-3 else float('nan')
        self.get_logger().info(
            f'gyro yaw rate {wg:.3f} rad/s, wheel-implied (chi=1) {wwheels:.3f} rad/s '
            f'-> slip_factor chi = {chi:.2f}. Put it in ugv_core/config/robot.yaml.')


def main(args=None):
    rclpy.init(args=args)
    node = SlipCalibration()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
