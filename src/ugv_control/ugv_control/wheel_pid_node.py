"""Stage 5 - per-wheel velocity PID with saturation and anti-windup.

Sub  /wheel_cmd        std_msgs/Float64MultiArray  desired wheel speeds [rad/s] (FL, RL, FR, RR)
     /wheel_states     sensor_msgs/JointState      measured wheel speeds from the encoders
Pub  /wheel_pwm        std_msgs/Float64MultiArray  PWM duty in [-1, 1] per wheel
     /wheel_pid/state  std_msgs/Float64MultiArray  [ref x4, measured x4, duty x4] for plotting

duty = clip(w_ref/w_noload + accel_ff * dw_ref/dt + PID(w_ref - w_meas), -1, 1)
Identical gains on all four wheels. See ugv_core/wheel_control.py and docs/pid_tuning.md.

Safety: no /wheel_cmd for cmd_timeout -> target 0; no /wheel_states for
0.25 s -> duty 0 and integrators reset (no feedback = no closed loop).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from ugv_core import WHEEL_JOINTS, WHEEL_NAMES
from ugv_core.wheel_control import WheelSpeedController


class WheelPidNode(Node):
    def __init__(self):
        super().__init__('wheel_pid_node')
        params = {'kp': 0.12, 'ki': 2.0, 'kd': 0.0, 'd_tau': 0.02, 'duty_limit': 1.0,
                  'no_load_speed': 10.0, 'use_feedforward': True, 'anti_windup': 'clamp',
                  'max_wheel_accel': 20.0, 'accel_ff': 0.002, 'hold_i_while_ramping': True}
        for n, v in params.items():
            self.declare_parameter(n, v)
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('cmd_timeout', 0.5)
        cfg = {n: self.get_parameter(n).value for n in params}
        self.ctl = WheelSpeedController(**cfg)
        self.timeout = self.get_parameter('cmd_timeout').value
        self.period = 1.0 / self.get_parameter('rate').value

        self.target = [0.0] * 4
        self.meas = [0.0] * 4
        self.t_cmd = self.t_meas = self.t_last = None
        self.pwm_pub = self.create_publisher(Float64MultiArray, '/wheel_pwm', 10)
        self.dbg_pub = self.create_publisher(Float64MultiArray, '/wheel_pid/state', 10)
        self.create_subscription(Float64MultiArray, '/wheel_cmd', self.cmd_cb, 10)
        self.create_subscription(JointState, '/wheel_states', self.state_cb, 20)
        self.create_timer(self.period, self.loop)
        self.get_logger().info(
            f"Wheel PID: kp={cfg['kp']} ki={cfg['ki']} kd={cfg['kd']} "
            f"anti-windup={cfg['anti_windup']} ff={cfg['use_feedforward']}")

    def cmd_cb(self, msg):
        if len(msg.data) == 4:
            self.target = list(msg.data)
            self.t_cmd = self.get_clock().now()

    def state_cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(j in idx for j in WHEEL_JOINTS) and len(msg.velocity) >= len(msg.name):
            self.meas = [msg.velocity[idx[j]] for j in WHEEL_JOINTS]
            self.t_meas = self.get_clock().now()

    def _age(self, t, now):
        return float('inf') if t is None else (now - t).nanoseconds * 1e-9

    def loop(self):
        now = self.get_clock().now()
        dt = self.period if self.t_last is None else (now - self.t_last).nanoseconds * 1e-9
        self.t_last = now
        if dt <= 0.0:
            return
        if self._age(self.t_meas, now) > 0.25:
            self.ctl.reset()
            duty = [0.0] * 4
            self.get_logger().warn('No /wheel_states - motors off', throttle_duration_sec=5.0)
        else:
            target = self.target if self._age(self.t_cmd, now) < self.timeout else [0.0] * 4
            duty = self.ctl.step(target, self.meas, min(dt, 0.1))
        out = Float64MultiArray()
        out.layout.dim = [MultiArrayDimension(label=','.join(WHEEL_NAMES), size=4, stride=4)]
        out.data = [float(d) for d in duty]
        self.pwm_pub.publish(out)
        dbg = Float64MultiArray()
        dbg.data = [float(v) for v in list(self.ctl.ref) + list(self.meas) + list(duty)]
        self.dbg_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pwm_pub.publish(Float64MultiArray(data=[0.0] * 4))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
