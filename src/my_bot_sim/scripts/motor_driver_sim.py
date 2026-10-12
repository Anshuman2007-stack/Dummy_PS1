#!/usr/bin/env python3
"""Simulated motor driver: PWM duty -> wheel joint torque in Gazebo.

Sub  /wheel_pwm   std_msgs/Float64MultiArray  4 duties in [-1, 1], order FL, RL, FR, RR
Pub  /model/my_bot/joint/<joint>/cmd_force   std_msgs/Float64   (bridged to Gazebo)

DC motor model: torque = stall_torque * duty - b * w. Only the first term is
sent; the back-EMF term b * w is the wheel joint <damping> in model.sdf, so
Gazebo applies it at every physics step. On real hardware this node is
replaced by the motor-driver firmware that turns the same duty into PWM.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from ugv_core import WHEEL_JOINTS


class MotorDriverSim(Node):
    def __init__(self):
        super().__init__('motor_driver_sim')
        self.declare_parameter('stall_torque', 5.0)
        self.declare_parameter('rate', 200.0)
        self.declare_parameter('timeout', 0.3)
        self.declare_parameter('model_name', 'my_bot')
        self.stall = self.get_parameter('stall_torque').value
        self.timeout = self.get_parameter('timeout').value
        model = self.get_parameter('model_name').value

        self.duty = [0.0] * 4
        self.last = None
        self.pubs = [self.create_publisher(Float64, f'/model/{model}/joint/{j}/cmd_force', 10)
                     for j in WHEEL_JOINTS]
        self.create_subscription(Float64MultiArray, '/wheel_pwm', self.pwm_cb, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.get_logger().info(f'Motor driver sim ready (stall torque {self.stall} N m).')

    def pwm_cb(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn('/wheel_pwm needs 4 values', throttle_duration_sec=2.0)
            return
        self.duty = [max(-1.0, min(1.0, d)) for d in msg.data]
        self.last = self.get_clock().now()

    def tick(self):
        stale = self.last is None or \
            (self.get_clock().now() - self.last).nanoseconds * 1e-9 > self.timeout
        for pub, d in zip(self.pubs, self.duty):
            pub.publish(Float64(data=0.0 if stale else self.stall * d))


def main():
    rclpy.init()
    node = MotorDriverSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
