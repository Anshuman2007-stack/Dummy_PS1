r"""Per-wheel velocity control - Stage 5.

One PID per wheel, all four with the SAME gains (identical motors; different
gains are the most common cause of left/right desynchronisation). The output
is a normalised PWM duty cycle in [-1, 1], the same quantity a real motor
driver takes, so the optional hardware extension only has to forward it.

    duty_i = clip( w_ref_i / w_noload  +  PID(w_ref_i - w_meas_i) , -1, 1 )
             \_______ feed-forward ___/   \____ feedback _______/

The feed-forward term is the steady-state duty of an unloaded DC motor
(back-EMF balances the supply at w = duty * w_noload), so the PID only has
to supply the load torque (rolling resistance, skid-steer scrubbing) and
correct model error. Saturation of the duty is handled by the anti-windup in
:class:`ugv_core.pid.PID`.

``max_wheel_accel`` [rad/s^2] (0 = off) rate-limits the reference. A step
the tyres cannot follow (traction limit mu*g/r) only fills the integrator and
causes overshoot, so the reference is ramped at an achievable rate instead.

``accel_ff`` [duty per rad/s^2] adds the torque needed to accelerate the
wheel plus its share of the vehicle mass along the ramp:
accel_ff = (J_wheel + m/4 * r^2) / stall_torque.

``hold_i_while_ramping`` freezes each integrator while its reference is still
ramping; the integrator then only removes the steady load error.
"""
from .pid import PID

DEFAULTS = {'kp': 0.12, 'ki': 2.0, 'kd': 0.0, 'd_tau': 0.02, 'duty_limit': 1.0,
            'no_load_speed': 10.0, 'use_feedforward': True, 'anti_windup': 'clamp',
            'max_wheel_accel': 20.0, 'accel_ff': 0.002, 'hold_i_while_ramping': True}


class WheelSpeedController:
    def __init__(self, n=4, **params):
        p = dict(DEFAULTS)
        unknown = set(params) - set(p)
        if unknown:
            raise ValueError(f'unknown wheel controller params: {sorted(unknown)}')
        p.update(params)
        self.p = p
        lim = p['duty_limit']
        self.pids = [PID(p['kp'], p['ki'], p['kd'], -lim, lim,
                         anti_windup=p['anti_windup'], d_tau=p['d_tau'])
                     for _ in range(n)]
        self.duty = [0.0] * n
        self.ref = [0.0] * n

    def reset(self):
        for pid in self.pids:
            pid.reset()
        self.duty = [0.0] * len(self.pids)
        self.ref = [0.0] * len(self.pids)

    def _ramp(self, targets, dt):
        a = self.p['max_wheel_accel']
        if a <= 0.0:
            self.ref = list(targets)
        else:
            step = a * dt
            self.ref = [r + max(-step, min(step, t - r)) for r, t in zip(self.ref, targets)]
        return self.ref

    def step(self, targets, measured, dt):
        if dt <= 0.0:
            return self.duty
        prev = list(self.ref)
        ref = self._ramp(targets, dt)
        ff_gain = 1.0 / self.p['no_load_speed'] if self.p['use_feedforward'] else 0.0
        acc_gain = self.p['accel_ff'] if self.p['use_feedforward'] else 0.0
        out = []
        hold = self.p['hold_i_while_ramping']
        for pid, r, r0, tgt, meas in zip(self.pids, ref, prev, targets, measured):
            ff = ff_gain * r + acc_gain * (r - r0) / dt
            ramping = hold and abs(tgt - r) > 1e-9
            out.append(pid.step(r - meas, dt, feedforward=ff, measurement=meas,
                                hold_integrator=ramping))
        self.duty = out
        return out


class DCMotor:
    """Simple DC gear-motor: torque = stall_torque * duty - b * w.

    In Gazebo the viscous back-EMF term ``b * w`` (b = stall_torque /
    no_load_speed) is the wheel joint's <damping>, applied by the physics
    engine every step, and only ``stall_torque * duty`` is commanded through
    the ApplyJointForce plugin. The offline simulator uses both terms.
    """

    def __init__(self, stall_torque=5.0, no_load_speed=10.0):
        self.stall = float(stall_torque)
        self.w0 = float(no_load_speed)

    @property
    def damping(self):
        return self.stall / self.w0

    def drive_torque(self, duty):
        return self.stall * max(-1.0, min(1.0, duty))

    def torque(self, duty, w):
        return self.drive_torque(duty) - self.damping * w
