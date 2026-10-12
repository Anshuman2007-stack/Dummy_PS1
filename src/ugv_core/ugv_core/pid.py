"""Discrete PID controller with feed-forward, filtered derivative and anti-windup.

    u = ff + Kp*e + I + D,         u_sat = clip(u, out_min, out_max)

Integrator ``I`` stores the *integral term* (already multiplied by Ki), so
changing Ki at run time does not make the output jump.

Anti-windup modes
-----------------
``'backcalc'`` (default) - back-calculation: I += (Ki*e + (u_sat - u)/Tt) * dt.
    While the output is saturated the integrator is driven back so that u
    stays at the limit instead of growing without bound. Tt defaults to
    Kp/Ki (= Ti), the usual choice.
``'clamp'`` - conditional integration: the integrator is frozen while the
    output is saturated *and* the error would push it further into saturation.
``'none'`` - plain integrator, kept only to demonstrate the windup it causes.

The derivative can act on the measurement (no "derivative kick" on set-point
steps) and is low-pass filtered with time constant ``d_tau``.
"""
import math


def clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class PID:
    MODES = ('backcalc', 'clamp', 'none')

    def __init__(self, kp, ki=0.0, kd=0.0, out_min=-math.inf, out_max=math.inf,
                 anti_windup='backcalc', tt=None, d_tau=0.0, i_limit=math.inf):
        if anti_windup not in self.MODES:
            raise ValueError(f'anti_windup must be one of {self.MODES}')
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.out_min, self.out_max = float(out_min), float(out_max)
        self.mode = anti_windup
        if tt is None:
            tt = (self.kp / self.ki) if (self.ki > 0 and self.kp > 0) else 1.0
        self.tt = float(tt)
        self.d_tau = float(d_tau)
        self.i_limit = float(i_limit)
        self.reset()

    def reset(self):
        self.i_term = 0.0
        self.d_term = 0.0
        self._prev = None
        self.saturated = False
        self.last = (0.0, 0.0, 0.0, 0.0)  # p, i, d, u_sat (for logging)

    def step(self, error, dt, feedforward=0.0, measurement=None, hold_integrator=False):
        """Advance one sample. Returns the saturated output.

        hold_integrator=True freezes the integral (used while the set-point is
        being ramped: the feed-forward drives that transient, and integrating
        the measurement lag during the ramp would only cause overshoot).
        """
        if dt <= 0.0:
            return clip(feedforward + self.kp * error + self.i_term,
                        self.out_min, self.out_max)

        # derivative: of -measurement if given (no kick), else of the error
        sig = -measurement if measurement is not None else error
        if self._prev is None:
            self._prev = sig
        raw_d = self.kd * (sig - self._prev) / dt
        self._prev = sig
        a = dt / (self.d_tau + dt)
        self.d_term += a * (raw_d - self.d_term)

        p = self.kp * error
        u = feedforward + p + self.i_term + self.d_term
        u_sat = clip(u, self.out_min, self.out_max)
        self.saturated = (u_sat != u)

        if self.ki > 0.0 and hold_integrator and self.mode == 'backcalc':
            self.i_term += (u_sat - u) / self.tt * dt          # still unwind if saturated
        elif self.ki > 0.0 and not hold_integrator:
            if self.mode == 'backcalc':
                self.i_term += (self.ki * error + (u_sat - u) / self.tt) * dt
            elif self.mode == 'clamp':
                pushing_out = (u > self.out_max and error > 0) or \
                              (u < self.out_min and error < 0)
                if not pushing_out:
                    self.i_term += self.ki * error * dt
            else:
                self.i_term += self.ki * error * dt
            self.i_term = clip(self.i_term, -self.i_limit, self.i_limit)

        self.last = (p, self.i_term, self.d_term, u_sat)
        return u_sat
