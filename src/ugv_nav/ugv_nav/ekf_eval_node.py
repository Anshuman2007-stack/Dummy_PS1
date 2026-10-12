"""State-estimation evaluation: scripted motion, EKF vs ground truth vs encoder odometry.

    ros2 launch ugv_bringup eval_ekf.launch.py

Segments (each preceded by 2 s of rest):
  stationary 20 s | straight v=0.3 15 s | rotation w=0.8 15 s |
  combined v=0.3 w=0.3 20 s | combined + GPS dropout (10 s, via /gnss/enable) 20 s
Per segment: RMSE of position, heading, speed for the EKF and for the
encoder-only odometry. Writes ekf_eval_summary.txt and ekf_eval.csv.
"""
import csv
import math
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ugv_core.ekf import wrap
from ugv_core.metrics import rmse

SEGMENTS = [  # name, duration, v, w, gps_dropout(start, dur) within the segment
    ('Stationary', 20.0, 0.0, 0.0, None),
    ('Straight line', 15.0, 0.3, 0.0, None),
    ('Rotation', 15.0, 0.0, 0.8, None),
    ('Combined', 20.0, 0.3, 0.3, None),
    ('Combined + GPS dropout', 20.0, 0.3, 0.3, (5.0, 10.0)),
]
REST = 2.0


def get_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_twist(m):
    p = m.pose.pose
    return (p.position.x, p.position.y, get_yaw(p.orientation), m.twist.twist.linear.x)


class EkfEval(Node):
    def __init__(self):
        super().__init__('ekf_eval_node')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('start_delay', 3.0)
        self.out_dir = self.get_parameter('output_dir').value or os.getcwd()
        self.start_delay = self.get_parameter('start_delay').value
        self.ekf = self.true = self.odom = None
        self.t0 = None
        self.rows = []
        self.done = False
        self.gps_off = False
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gps_pub = self.create_publisher(Bool, '/gnss/enable', 10)
        self.create_subscription(Odometry, '/ekf/odom', lambda m: self.store('ekf', m), 10)
        self.create_subscription(Odometry, '/true_odom', lambda m: self.store('true', m), 10)
        self.create_subscription(Odometry, '/odom/wheels', lambda m: self.store('odom', m), 10)
        self.create_timer(0.02, self.tick)
        total = sum(REST + s[1] for s in SEGMENTS)
        self.get_logger().info(f'EKF evaluation: {len(SEGMENTS)} segments, {total:.0f} s')

    def store(self, k, m):
        setattr(self, k, pose_twist(m))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.done or None in (self.ekf, self.true, self.odom):
            return
        t = self.now()
        if self.t0 is None:
            self.t0 = t + self.start_delay
        rel = t - self.t0
        if rel < 0:
            return
        start = 0.0
        seg = None
        for s in SEGMENTS:
            if start + REST <= rel < start + REST + s[1]:
                seg = s
                srel = rel - start - REST
                break
            start += REST + s[1]
        if rel >= start and seg is None and start == sum(REST + s[1] for s in SEGMENTS):
            self.cmd_pub.publish(Twist())
            self.finish()
            return
        cmd = Twist()
        drop = False
        if seg:
            cmd.linear.x, cmd.angular.z = seg[2], seg[3]
            if seg[4]:
                drop = seg[4][0] <= srel < seg[4][0] + seg[4][1]
        if drop != self.gps_off:
            self.gps_off = drop
            self.gps_pub.publish(Bool(data=not drop))
        self.cmd_pub.publish(cmd)
        self.rows.append((rel, seg[0] if seg else 'rest', int(drop)) + self.true + self.ekf +
                         self.odom)

    def finish(self):
        self.done = True
        L = ['', 'EKF evaluation (RMSE against /true_odom)',
             f"{'segment':<25}{'EKF pos':>9}{'EKF hdg':>9}{'EKF spd':>9}"
             f"{'odom pos':>10}{'odom hdg':>10}"]
        for name, *_ in SEGMENTS:
            r = [x for x in self.rows if x[1] == name]
            if not r:
                continue
            # odometry error measured relative to the segment start (it is dead reckoning)
            ep = [math.hypot(x[3] - x[7], x[4] - x[8]) for x in r]
            eh = [wrap(x[5] - x[9]) for x in r]
            es = [x[10] - x[6] for x in r]
            op = [math.hypot(x[3] - x[11], x[4] - x[12]) for x in r]
            oh = [wrap(x[5] - x[13]) for x in r]
            L.append(f'{name:<25}{rmse(ep):>9.3f}{rmse(eh):>9.3f}{rmse(es):>9.3f}'
                     f'{rmse(op):>10.3f}{rmse(oh):>10.3f}')
            if name.endswith('dropout'):
                d = [math.hypot(x[3] - x[7], x[4] - x[8]) for x in r if x[2]]
                if d:
                    L.append(f"{'':<25}max EKF position error during dropout: {max(d):.3f} m")
        L.append('(odom columns: accumulated encoder dead reckoning since start, no GPS/IMU)')
        text = '\n'.join(L)
        os.makedirs(self.out_dir, exist_ok=True)
        with open(os.path.join(self.out_dir, 'ekf_eval_summary.txt'), 'w') as f:
            f.write(text + '\n')
        with open(os.path.join(self.out_dir, 'ekf_eval.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'segment', 'gps_dropout', 'true_x', 'true_y', 'true_yaw', 'true_v',
                        'ekf_x', 'ekf_y', 'ekf_yaw', 'ekf_v', 'odom_x', 'odom_y', 'odom_yaw',
                        'odom_v'])
            w.writerows(self.rows)
        self.get_logger().info(text + f'\nFiles written to {self.out_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = EkfEval()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.gps_pub.publish(Bool(data=True))
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
