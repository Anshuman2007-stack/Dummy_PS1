"""Wheel odometry computed from quantised encoder ticks - Stage 1.

The simulator gives the continuous wheel joint angle. A real incremental
encoder only reports an integer tick count, so :func:`angle_to_ticks`
quantises the angle first; everything after that works on ticks exactly as it
would on hardware.

For each update interval dt:

    w_i      = 2*pi * (ticks_i - ticks_i_prev) / (ticks_per_rev * dt)   per wheel
    w_left   = mean(front_left, rear_left),  w_right = mean(front_right, rear_right)
    v, omega = forward kinematics (see kinematics.py)
    pose    += exact arc integration of (v, omega) over dt
"""
import math

from .kinematics import SkidSteerKinematics


def angle_to_ticks(angle_rad, ticks_per_rev):
    return int(math.floor(angle_rad / (2.0 * math.pi) * ticks_per_rev))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class EncoderOdometry:
    def __init__(self, kinematics: SkidSteerKinematics, ticks_per_rev=8192,
                 velocity_filter_tau=0.0):
        self.kin = kinematics
        self.tpr = int(ticks_per_rev)
        self.tau = float(velocity_filter_tau)
        self.reset()

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta
        self.v = self.omega = 0.0
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]
        self._ticks = None
        self._t = None

    def update(self, ticks, t):
        """Feed 4 tick counts (FL, RL, FR, RR) at time t [s].

        Returns True when a new odometry estimate was produced.
        """
        ticks = list(ticks)
        self.wheel_angles = [2.0 * math.pi * k / self.tpr for k in ticks]
        if self._ticks is None:
            self._ticks, self._t = ticks, t
            return False
        dt = t - self._t
        if dt <= 0.0:
            return False

        raw = [2.0 * math.pi * (k - k0) / (self.tpr * dt)
               for k, k0 in zip(ticks, self._ticks)]
        a = 1.0 if self.tau <= 0.0 else dt / (self.tau + dt)
        self.wheel_speeds = [w + a * (r - w) for w, r in zip(self.wheel_speeds, raw)]
        self._ticks, self._t = ticks, t

        # Pose from the *unfiltered* displacement over dt (filtering is only
        # for the velocity outputs, it must not lose distance).
        v_raw, w_raw = self.kin.forward4(raw)
        ds, dth = v_raw * dt, w_raw * dt
        if abs(dth) < 1e-9:
            self.x += ds * math.cos(self.theta)
            self.y += ds * math.sin(self.theta)
        else:
            rad = ds / dth
            self.x += rad * (math.sin(self.theta + dth) - math.sin(self.theta))
            self.y -= rad * (math.cos(self.theta + dth) - math.cos(self.theta))
        self.theta = wrap(self.theta + dth)
        self.v, self.omega = self.kin.forward4(self.wheel_speeds)
        return True
