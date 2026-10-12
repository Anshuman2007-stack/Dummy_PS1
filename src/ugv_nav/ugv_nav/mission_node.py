"""Navigation evaluation: drive to a list of goals and score each one on ground truth.

    ros2 launch ugv_bringup eval_navigation.launch.py                 # default 4 goals
    ros2 launch ugv_bringup eval_navigation.launch.py goals:="[4.0, 0.0, 0.0, 4.0]"

For every goal: time to reach, final position error on /true_odom (what the
robot really achieved), final error of the EKF estimate, path length ratio and
DRIVE->ROTATE re-rotations. Over the whole run: EKF position / heading RMSE
against ground truth. Writes mission_summary.txt and mission_trajectory.csv.
"""
import csv
import math
import os

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ugv_core.ekf import wrap
from ugv_core.metrics import rmse


def get_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.declare_parameter('goals', [4.0, 0.0, 0.0, 4.0, -3.0, -3.0, 5.0, -2.0])
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('settle_time', 1.0)
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('gps_dropout', [0.0, 0.0])   # [start, duration] s after start
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        flat = list(g('goals'))
        self.goals = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        self.start_delay, self.settle = g('start_delay'), g('settle_time')
        self.timeout = g('goal_timeout')
        self.out_dir = g('output_dir') or os.getcwd()
        self.drop = list(g('gps_dropout'))

        self.ekf = self.true = None
        self.state = 'IDLE'
        self.phase = 'wait'
        self.i = -1
        self.t_phase = None
        self.t_start = None
        self.reached = False
        self.results = []
        self.traj = []
        self.seg_len = 0.0
        self.last_xy = None
        self.rotations = 0
        self.prev_state = None
        self.drop_active = False
        self.done = False

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.gps_pub = self.create_publisher(Bool, '/gnss/enable', 10)
        self.create_subscription(Odometry, '/ekf/odom', self.ekf_cb, 10)
        self.create_subscription(Odometry, '/true_odom', self.true_cb, 10)
        self.create_subscription(Bool, '/nav/goal_reached', self.reached_cb, 10)
        self.create_subscription(String, '/nav/state', self.state_cb, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Mission with {len(self.goals)} goals: {self.goals}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def ekf_cb(self, m):
        p = m.pose.pose
        self.ekf = (p.position.x, p.position.y, get_yaw(p.orientation))

    def true_cb(self, m):
        p = m.pose.pose
        self.true = (p.position.x, p.position.y, get_yaw(p.orientation))

    def reached_cb(self, m):
        if m.data:
            self.reached = True

    def state_cb(self, m):
        if self.phase == 'drive' and m.data == 'ROTATE' and self.prev_state == 'DRIVE':
            self.rotations += 1
        self.prev_state = m.data
        self.state = m.data

    def send_goal(self, x, y):
        m = PoseStamped()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        self.goal_pub.publish(m)

    def tick(self):
        if self.done or self.ekf is None or self.true is None:
            return
        t = self.now()
        if self.t_start is None:
            self.t_start = t
            self.t_phase = t
        rel = t - self.t_start
        self.traj.append((rel,) + self.true + self.ekf)

        # scheduled GPS dropout (for the robustness experiment)
        if self.drop[1] > 0:
            want_drop = self.drop[0] <= rel < self.drop[0] + self.drop[1]
            if want_drop != self.drop_active:
                self.drop_active = want_drop
                self.gps_pub.publish(Bool(data=not want_drop))

        if self.last_xy is not None and self.phase == 'drive':
            self.seg_len += math.hypot(self.true[0] - self.last_xy[0],
                                       self.true[1] - self.last_xy[1])
        self.last_xy = self.true[:2]

        if self.phase == 'wait' and t - self.t_phase >= self.start_delay:
            self.next_goal(t)
        elif self.phase == 'drive':
            if self.reached or t - self.t_phase > self.timeout:
                self.phase = 'settle'
                self.t_reach = t
        elif self.phase == 'settle' and t - self.t_reach >= self.settle:
            gx, gy = self.goals[self.i]
            straight = max(1e-6, math.hypot(gx - self.p0[0], gy - self.p0[1]))
            self.results.append({
                'goal': (gx, gy), 'reached': self.reached, 'time': self.t_reach - self.t_phase,
                'err_true': math.hypot(self.true[0] - gx, self.true[1] - gy),
                'err_ekf': math.hypot(self.ekf[0] - gx, self.ekf[1] - gy),
                'path_ratio': self.seg_len / straight,
                'rotations': self.rotations})
            self.next_goal(t)

    def next_goal(self, t):
        self.i += 1
        if self.i >= len(self.goals):
            self.finish()
            return
        gx, gy = self.goals[self.i]
        self.p0 = self.true[:2]
        self.seg_len, self.rotations, self.reached = 0.0, 0, False
        self.send_goal(gx, gy)
        self.phase, self.t_phase = 'drive', t
        self.get_logger().info(f'Goal {self.i + 1}/{len(self.goals)}: ({gx:g}, {gy:g})')

    def finish(self):
        self.done = True
        if self.drop_active:
            self.gps_pub.publish(Bool(data=True))
        pos = [math.hypot(r[1] - r[4], r[2] - r[5]) for r in self.traj]
        hd = [wrap(r[3] - r[6]) for r in self.traj]
        L = ['', 'Navigation mission results (errors on ground truth /true_odom)',
             f"{'goal':<14}{'ok':<5}{'time[s]':>8}{'err_true[m]':>13}{'err_ekf[m]':>12}"
             f"{'path_ratio':>12}{'re-rot':>8}"]
        for r in self.results:
            gs = f"({r['goal'][0]:g}, {r['goal'][1]:g})"
            L.append(f"{gs:<14}{'yes' if r['reached'] else 'NO':<5}{r['time']:>8.1f}"
                     f"{r['err_true']:>13.3f}{r['err_ekf']:>12.3f}{r['path_ratio']:>12.3f}"
                     f"{r['rotations']:>8d}")
        L.append(f'EKF vs truth over the run: position RMSE {rmse(pos):.3f} m, '
                 f'heading RMSE {rmse(hd):.3f} rad, max position error {max(pos):.3f} m')
        text = '\n'.join(L)
        os.makedirs(self.out_dir, exist_ok=True)
        with open(os.path.join(self.out_dir, 'mission_summary.txt'), 'w') as f:
            f.write(text + '\n')
        with open(os.path.join(self.out_dir, 'mission_trajectory.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'true_x', 'true_y', 'true_yaw', 'ekf_x', 'ekf_y', 'ekf_yaw'])
            w.writerows(self.traj)
        self.get_logger().info(text + f'\nFiles written to {self.out_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
