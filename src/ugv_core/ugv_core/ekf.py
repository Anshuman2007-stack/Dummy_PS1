"""Extended Kalman Filter, state (x, y, theta, v, omega) - Stage 2.

Motion model (unicycle, v and omega modelled as random walks)::

    x'     = x + v cos(theta) dt
    y'     = y + v sin(theta) dt
    theta' = theta + omega dt
    v'     = v
    omega' = omega

Asynchronous / multi-rate fusion: every measurement carries its own time
stamp. Before an update the filter is propagated from the time of the last
update to the stamp of the new measurement (``predict(t)``), so sensors at
50 Hz, 100 Hz and 5 Hz can be fused in any order without assuming a fixed
rate. A measurement older than the filter time is applied at the current
time (a few ms of latency at most).

Measurements
    wheel odometry  z = [v, omega]     (encoder twist; the drifting pose is NOT used)
    gyro            z = omega          (IMU angular_velocity.z)
    yaw (optional)  z = theta          (IMU orientation; off by default, see README)
    GPS             z = [x, y]         (local ENU metres), chi-square gated

Covariance update uses the Joseph form, which stays symmetric positive
definite despite rounding.
"""
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class EKF:
    N = 5
    IX, IY, ITH, IV, IW = range(5)

    def __init__(self, q_pos=1e-4, q_theta=1e-5, q_v=0.5, q_omega=0.5,
                 gps_gate=9.21, gps_max_rejects=10):
        # Q is a continuous-time spectral density: Q_k = diag(q) * dt
        self.q = np.diag([q_pos, q_pos, q_theta, q_v, q_omega])
        self.gps_gate = gps_gate
        self.gps_max_rejects = gps_max_rejects
        self.x = np.zeros((5, 1))
        self.P = np.eye(5)
        self.t = None
        self.initialized = False
        self.gps_rejects = 0
        self.last_gps_t = None
        self.last_nis = 0.0
        self.stats = {'gps_accepted': 0, 'gps_rejected': 0, 'gps_resync': 0}

    # ------------------------------------------------------------------ setup
    def initialize(self, t, x=0.0, y=0.0, theta=0.0, P0=(0.01, 0.01, 0.01, 0.01, 0.01)):
        self.x = np.array([[x], [y], [theta], [0.0], [0.0]], dtype=float)
        self.P = np.diag(P0).astype(float)
        self.t = t
        self.initialized = True

    # ------------------------------------------------------------- prediction
    def predict(self, t):
        if not self.initialized or t <= self.t:
            return
        dt = t - self.t
        self.t = t
        th, v, w = self.x[2, 0], self.x[3, 0], self.x[4, 0]
        c, s = math.cos(th), math.sin(th)
        self.x[0, 0] += v * c * dt
        self.x[1, 0] += v * s * dt
        self.x[2, 0] = wrap(th + w * dt)

        F = np.eye(5)
        F[0, 2] = -v * s * dt
        F[0, 3] = c * dt
        F[1, 2] = v * c * dt
        F[1, 3] = s * dt
        F[2, 4] = dt
        self.P = F @ self.P @ F.T + self.q * dt

    # ---------------------------------------------------------- generic update
    def _update(self, z, H, R, angle_row=None, gate=None):
        y = z - H @ self.x
        if angle_row is not None:
            y[angle_row, 0] = wrap(y[angle_row, 0])
        S = H @ self.P @ H.T + R
        S_inv = np.linalg.inv(S)
        nis = (y.T @ S_inv @ y).item()
        self.last_nis = nis
        if gate is not None and nis > gate:
            return False
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        self.x[2, 0] = wrap(self.x[2, 0])
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return True

    # ------------------------------------------------------------ measurements
    def update_wheel_odom(self, t, v, omega, var_v, var_omega):
        self.predict(t)
        H = np.zeros((2, 5))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        return self._update(np.array([[v], [omega]]), H, np.diag([var_v, var_omega]))

    def update_gyro(self, t, omega, var):
        self.predict(t)
        H = np.zeros((1, 5))
        H[0, 4] = 1.0
        return self._update(np.array([[omega]]), H, np.array([[var]]))

    def update_yaw(self, t, yaw, var):
        self.predict(t)
        H = np.zeros((1, 5))
        H[0, 2] = 1.0
        return self._update(np.array([[yaw]]), H, np.array([[var]]), angle_row=0)

    def update_gps(self, t, x, y, var):
        """Fuse a GPS fix; return 'accepted', 'rejected' or 'resync'."""
        self.predict(t)
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        z = np.array([[x], [y]])
        R = np.eye(2) * var
        self.last_gps_t = t
        if self._update(z, H, R, gate=self.gps_gate):
            self.gps_rejects = 0
            self.stats['gps_accepted'] += 1
            return 'accepted'
        self.gps_rejects += 1
        self.stats['gps_rejected'] += 1
        if self.gps_rejects >= self.gps_max_rejects:
            # Many consecutive rejections: the estimate, not the GPS, is wrong
            # (e.g. long dropout + slip). Inflate P and accept.
            self.P[0, 0] += 4.0 * var * self.gps_rejects
            self.P[1, 1] += 4.0 * var * self.gps_rejects
            self._update(z, H, R)
            self.gps_rejects = 0
            self.stats['gps_resync'] += 1
            return 'resync'
        return 'rejected'

    # ---------------------------------------------------------------- helpers
    def gps_age(self, t):
        return math.inf if self.last_gps_t is None else t - self.last_gps_t

    @property
    def state(self):
        return tuple(float(v) for v in self.x[:, 0])

    def state_at(self, t):
        """Extrapolate the state to time t without changing the filter."""
        x, y, th, v, w = self.state
        if self.t is None or t <= self.t:
            return x, y, th, v, w
        dt = t - self.t
        return (x + v * math.cos(th) * dt, y + v * math.sin(th) * dt,
                wrap(th + w * dt), v, w)
