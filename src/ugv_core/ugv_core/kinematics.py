"""Four-wheel differential-drive (skid-steer) kinematics - Stage 4.

Wheel order everywhere: (front_left, rear_left, front_right, rear_right).
Positive wheel speed drives the robot forward.

Inverse kinematics (body twist -> wheel speeds)::

    w_left  = (v - omega * chi * B / 2) / r      -> front_left, rear_left
    w_right = (v + omega * chi * B / 2) / r      -> front_right, rear_right

Forward kinematics (wheel speeds -> body twist)::

    v     = r * (w_right + w_left) / 2
    omega = r * (w_right - w_left) / (chi * B)

r = wheel radius, B = track width (distance between left and right wheel
centres), chi >= 1 = skid-steer slip factor. On a four-wheel skid-steer
vehicle the wheels must slide sideways to turn, so the instantaneous centres
of rotation of the two wheel sides lie further apart than B; chi models that
"effective track" (chi = 1 is the ideal two-wheel differential drive).

Both wheels on one side get the same command because they are on the same
side of a rigid chassis and must roll at the same ground speed.
"""
import math


class SkidSteerKinematics:
    def __init__(self, wheel_radius=0.1, track_width=0.4, slip_factor=1.0,
                 max_wheel_speed=float('inf')):
        if wheel_radius <= 0 or track_width <= 0 or slip_factor <= 0:
            raise ValueError('wheel_radius, track_width and slip_factor must be > 0')
        self.r = float(wheel_radius)
        self.B = float(track_width)
        self.chi = float(slip_factor)
        self.max_w = float(max_wheel_speed)

    @property
    def effective_track(self):
        return self.chi * self.B

    def side_speeds(self, v, omega):
        """Unsaturated (w_left, w_right) in rad/s."""
        half = omega * self.effective_track / 2.0
        return (v - half) / self.r, (v + half) / self.r

    def inverse(self, v, omega):
        """(v, omega) -> 4 wheel speeds [rad/s], order FL, RL, FR, RR.

        If a wheel would exceed ``max_wheel_speed`` both sides are scaled by
        the same factor. This keeps the ratio w_left / w_right, i.e. the
        path curvature omega / v, so a saturated command still drives along
        the intended arc, only slower.
        """
        wl, wr = self.side_speeds(v, omega)
        peak = max(abs(wl), abs(wr))
        if peak > self.max_w and math.isfinite(self.max_w):
            s = self.max_w / peak
            wl, wr = wl * s, wr * s
        return [wl, wl, wr, wr]

    def forward(self, w_left, w_right):
        """Side wheel speeds [rad/s] -> (v [m/s], omega [rad/s])."""
        v = self.r * (w_right + w_left) / 2.0
        omega = self.r * (w_right - w_left) / self.effective_track
        return v, omega

    def forward4(self, wheel_speeds):
        """Four wheel speeds (FL, RL, FR, RR) -> (v, omega); sides are averaged."""
        fl, rl, fr, rr = wheel_speeds
        return self.forward((fl + rl) / 2.0, (fr + rr) / 2.0)
