"""Wheel PID step-response test (run with kinematics + wheel PID, WITHOUT navigation).

    ros2 launch ugv_control pid_step_test.launch.py      # starts everything and this node

Sequence on /cmd_vel: 2 s rest | v = 0.3 m/s for 4 s | 3 s rest | spin for 4 s | stop.
Measures every wheel on the true joint speed (/joint_states) and prints
rise time, 2 % settling time, overshoot, steady-state error and left/right
mismatch. A CSV of the whole run is written to ``output_dir``.
"""
import csv
import os
import statistics

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ugv_core import WHEEL_JOINTS, WHEEL_NAMES
from ugv_core.kinematics import SkidSteerKinematics
from ugv_core.metrics import step_metrics


class PidStepTest(Node):
    def __init__(self):
        super().__init__('pid_step_test')
        for n, v in (('wheel_radius', 0.1), ('track_width', 0.4), ('slip_factor', 1.0),
                     ('v_step', 0.3), ('w_step', 0.97), ('output_dir', '')):
            self.declare_parameter(n, v)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.kin = SkidSteerKinematics(g('wheel_radius'), g('track_width'), g('slip_factor'))
        v, w = g('v_step'), g('w_step')
        #             start  end   v    w    label
        self.phases = [(0.0, 2.0, 0.0, 0.0, None),
                       (2.0, 6.0, v, 0.0, f'forward v={v}'),
                       (6.0, 9.0, 0.0, 0.0, None),
                       (9.0, 13.0, 0.0, w, f'spin w={w}'),
                       (13.0, 15.0, 0.0, 0.0, None)]
        self.out_dir = g('output_dir') or os.getcwd()
        self.t0 = None
        self.rows = []          # t, true w x4
        self.pid = None         # latest [ref x4, enc x4, duty x4]
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self.js_cb, 100)
        self.create_subscription(Float64MultiArray, '/wheel_pid/state', self.pid_cb, 20)
        self.create_timer(0.02, self.tick)
        self.done = False
        self.get_logger().info('PID step test: waiting for /joint_states ...')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pid_cb(self, msg):
        if len(msg.data) == 12:
            self.pid = list(msg.data)

    def js_cb(self, msg):
        if self.t0 is None:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in WHEEL_JOINTS) or len(msg.velocity) < len(msg.name):
            return
        t = self.now() - self.t0
        true = [msg.velocity[idx[j]] for j in WHEEL_JOINTS]
        self.rows.append([t] + true + (self.pid if self.pid else [float('nan')] * 12))

    def tick(self):
        if self.done:
            return
        if self.t0 is None:
            if self.count_publishers('/joint_states') and self.now() > 0:
                self.t0 = self.now()
                self.get_logger().info('Starting step sequence (15 s)')
            return
        t = self.now() - self.t0
        cmd = Twist()
        for a, b, v, w, _ in self.phases:
            if a <= t < b:
                cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)
        if t >= self.phases[-1][1]:
            self.done = True
            self.report()

    def report(self):
        path = os.path.join(self.out_dir, 'pid_step_test.csv')
        hdr = ['t'] + [f'true_{n}' for n in WHEEL_NAMES] + [f'ref_{n}' for n in WHEEL_NAMES] + \
            [f'enc_{n}' for n in WHEEL_NAMES] + [f'duty_{n}' for n in WHEEL_NAMES]
        with open(path, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(hdr)
            wr.writerows(self.rows)
        lines = ['', 'Wheel PID step test (true wheel speed, 2 % band)',
                 f"{'step':<16}{'wheel':<13}{'rise[s]':>9}{'settle[s]':>11}"
                 f"{'OS[%]':>8}{'SSE[rad/s]':>12}"]
        for a, b, v, w, label in self.phases:
            if label is None:
                continue
            target = self.kin.inverse(v, w)
            seg = [r for r in self.rows if a <= r[0] < b]
            if len(seg) < 10:
                lines.append(f'{label}: not enough data')
                continue
            ts = [r[0] for r in seg]
            for i, name in enumerate(WHEEL_NAMES):
                m = step_metrics(ts, [r[1 + i] for r in seg], 0.0, target[i])
                lines.append(f"{label if i == 0 else '':<16}{name:<13}{m['rise']:>9.3f}"
                             f"{m['settling']:>11.3f}{m['overshoot']:>8.1f}{m['sse']:>+12.4f}")
            lr = [abs(abs(r[1] + r[2]) / 2 - abs(r[3] + r[4]) / 2) for r in seg]
            steady = [d for r, d in zip(seg, lr) if r[0] > a + 1.0]
            lines.append(f"{'':<16}left/right mismatch: max {max(lr):.3f} rad/s, "
                         f"mean after 1 s {statistics.mean(steady):.4f} rad/s")
        lines.append(f'CSV: {path}')
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = PidStepTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.pub.publish(Twist())
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
