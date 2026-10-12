"""Target-based navigation control law - Stage 3.

Inputs: goal (x_g, y_g) and the EKF estimate (x, y, theta).

    d = || (x_g - x, y_g - y) ||                    distance to goal
    e = wrap(atan2(y_g - y, x_g - x) - theta)       heading error

State machine
    ROTATE  v = 0,                         omega = PID_h(e)
            -> DRIVE when |e| < rotate_done
    DRIVE   v = min(PID_d(d), v_max)*max(0, cos e),  omega = PID_h(e)
            steering frozen inside steer_freeze (heading to a point is
            ill-defined there and GPS noise would make it chatter)
            -> ROTATE when |e| > rotate_again
    ALIGN   (optional) turn to the requested final yaw
    DONE    d < goal_tol

Why this law
    * Turning on the spot first means the robot never drives away from a goal
      behind it, which a pure "v = k d, omega = k e" law does.
    * The cos(e) factor makes v vanish as |e| -> 90 deg, so the robot cannot
      spiral around the goal; with v = k_d d cos e and omega = k_h e the
      Lyapunov function V = d^2/2 + e^2/2 decreases for k_h > k_d (standard
      polar-coordinate result), giving convergence to the goal.
    * v and omega are slew-rate limited (max_lin_acc, max_ang_acc), which
      gives a smooth trajectory and keeps the wheel PIDs out of saturation.
"""
import math

from .pid import PID


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


DEFAULTS = {
    'max_linear': 0.4, 'max_angular': 1.2, 'max_lin_acc': 0.6, 'max_ang_acc': 2.5,
    'goal_tol': 0.05, 'steer_freeze': 0.15, 'rotate_done': 0.10, 'rotate_again': 0.70,
    'align_final': False, 'align_tol': 0.08,
    'kp_lin': 0.8, 'ki_lin': 0.0, 'kd_lin': 0.0,
    'kp_ang': 2.0, 'ki_ang': 0.0, 'kd_ang': 0.1,
}


class Navigator:
    ROTATE, DRIVE, ALIGN, DONE, IDLE = 'ROTATE', 'DRIVE', 'ALIGN', 'DONE', 'IDLE'

    def __init__(self, **params):
        p = dict(DEFAULTS)
        unknown = set(params) - set(p)
        if unknown:
            raise ValueError(f'unknown navigator params: {sorted(unknown)}')
        p.update(params)
        self.p = p
        self.lin_pid = PID(p['kp_lin'], p['ki_lin'], p['kd_lin'],
                           0.0, p['max_linear'], anti_windup='clamp')
        self.ang_pid = PID(p['kp_ang'], p['ki_ang'], p['kd_ang'],
                           -p['max_angular'], p['max_angular'], anti_windup='clamp', d_tau=0.05)
        self.goal = None
        self.goal_yaw = None
        self.state = self.IDLE
        self.v = self.w = 0.0          # last (rate-limited) command
        self.dist = self.err = 0.0
        self.just_finished = False

    # ------------------------------------------------------------------ goals
    def set_goal(self, x, y, yaw=None):
        self.goal = (float(x), float(y))
        self.goal_yaw = yaw
        self._set_state(self.ROTATE)

    def cancel(self):
        self.goal = None
        self._set_state(self.IDLE)

    def _set_state(self, s):
        self.state = s
        self.lin_pid.reset()
        self.ang_pid.reset()

    @property
    def active(self):
        return self.state in (self.ROTATE, self.DRIVE, self.ALIGN)

    # ------------------------------------------------------------------- step
    def step(self, pose, dt):
        """Advance one control step for pose (x, y, theta); return (v, omega)."""
        self.just_finished = False
        v_des = w_des = 0.0
        if self.active and dt > 0.0:
            v_des, w_des = self._law(pose, dt)
        # slew-rate limit (also when stopping -> smooth deceleration)
        if dt > 0.0:
            dv = self.p['max_lin_acc'] * dt
            dw = self.p['max_ang_acc'] * dt
            self.v += max(-dv, min(dv, v_des - self.v))
            self.w += max(-dw, min(dw, w_des - self.w))
        if not self.active and abs(self.v) < 1e-3 and abs(self.w) < 1e-3:
            self.v = self.w = 0.0
        return self.v, self.w

    def _finish(self):
        self._set_state(self.DONE)
        self.just_finished = True

    def _law(self, pose, dt):
        p = self.p
        x, y, th = pose
        dx, dy = self.goal[0] - x, self.goal[1] - y
        d = math.hypot(dx, dy)
        e = wrap(math.atan2(dy, dx) - th)
        self.dist, self.err = d, e

        if self.state in (self.ROTATE, self.DRIVE) and d < p['goal_tol']:
            if p['align_final'] and self.goal_yaw is not None:
                self._set_state(self.ALIGN)
            else:
                self._finish()
                return 0.0, 0.0

        if self.state == self.ROTATE:
            if abs(e) < p['rotate_done']:
                self._set_state(self.DRIVE)
            else:
                return 0.0, self.ang_pid.step(e, dt)

        if self.state == self.DRIVE:
            if d > p['steer_freeze'] and abs(e) > p['rotate_again']:
                self._set_state(self.ROTATE)
                return 0.0, self.ang_pid.step(e, dt)
            if d <= p['steer_freeze'] and abs(e) > math.pi / 2:
                # passed the goal inside the freeze radius
                if d < 2.0 * p['goal_tol']:
                    self._finish()
                    return 0.0, 0.0
                self._set_state(self.ROTATE)
                return 0.0, 0.0
            v = self.lin_pid.step(d, dt) * max(0.0, math.cos(e))
            w = self.ang_pid.step(e, dt) if d > p['steer_freeze'] else 0.0
            return v, w

        if self.state == self.ALIGN:
            ea = wrap(self.goal_yaw - th)
            if abs(ea) < p['align_tol']:
                self._finish()
                return 0.0, 0.0
            return 0.0, self.ang_pid.step(ea, dt)
        return 0.0, 0.0
