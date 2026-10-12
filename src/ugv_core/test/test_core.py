"""Unit tests for ugv_core (pure Python: run with `python3 -m pytest src/ugv_core/test`)."""
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ugv_core.ekf import EKF  # noqa: E402
from ugv_core.geo import en_to_ll, ll_to_en  # noqa: E402
from ugv_core.kinematics import SkidSteerKinematics  # noqa: E402
from ugv_core.metrics import step_metrics  # noqa: E402
from ugv_core.navigator import Navigator  # noqa: E402
from ugv_core.odometry import angle_to_ticks, EncoderOdometry  # noqa: E402
from ugv_core.pid import PID  # noqa: E402
from ugv_core.wheel_control import WheelSpeedController  # noqa: E402


# ------------------------------------------------------------- kinematics
@pytest.mark.parametrize('v,w', [(0.3, 0.0), (0.0, 1.0), (0.2, -0.5), (-0.3, 0.4), (0, 0)])
def test_kinematics_roundtrip(v, w):
    k = SkidSteerKinematics(0.1, 0.4, 1.55)
    fl, rl, fr, rr = k.inverse(v, w)
    assert fl == rl and fr == rr                     # same side, same speed
    v2, w2 = k.forward4([fl, rl, fr, rr])
    assert v2 == pytest.approx(v) and w2 == pytest.approx(w)


def test_kinematics_known_values():
    k = SkidSteerKinematics(0.1, 0.4, 1.0)
    assert k.inverse(0.3, 0.0) == pytest.approx([3, 3, 3, 3])
    assert k.inverse(0.0, 1.0) == pytest.approx([-2, -2, 2, 2])   # (w * B/2) / r = 0.2/0.1


def test_kinematics_saturation_keeps_curvature():
    k = SkidSteerKinematics(0.1, 0.4, 1.0, max_wheel_speed=5.0)
    w = k.inverse(1.0, 2.0)                          # wants 6 and 14 rad/s
    assert max(abs(x) for x in w) == pytest.approx(5.0)
    v, om = k.forward4(w)
    assert om / v == pytest.approx(2.0 / 1.0)


# ------------------------------------------------------------------ geo
def test_geo_roundtrip():
    lat, lon = en_to_ll(123.4, -56.7, 22.62, 75.69)
    e, n = ll_to_en(lat, lon, 22.62, 75.69)
    assert e == pytest.approx(123.4, abs=1e-6) and n == pytest.approx(-56.7, abs=1e-6)


# -------------------------------------------------------------- odometry
def _drive(odom, w_left, w_right, T=10.0, dt=0.01, tpr=8192):
    ang = [0.0] * 4
    t = 0.0
    odom.update([0] * 4, t)
    while t < T - 1e-9:
        t += dt
        for i, w in enumerate((w_left, w_left, w_right, w_right)):
            ang[i] += w * dt
        odom.update([angle_to_ticks(a, tpr) for a in ang], t)


def test_odometry_straight():
    o = EncoderOdometry(SkidSteerKinematics(0.1, 0.4, 1.0))
    _drive(o, 3.0, 3.0)                              # 0.3 m/s for 10 s
    assert o.x == pytest.approx(3.0, abs=2e-3) and abs(o.y) < 1e-3
    assert o.v == pytest.approx(0.3, abs=0.01)


def test_odometry_circle():
    k = SkidSteerKinematics(0.1, 0.4, 1.5)
    o = EncoderOdometry(k)
    wl, _, wr, _ = k.inverse(0.2, 0.2)               # radius 1 m
    _drive(o, wl, wr, T=2 * math.pi / 0.2)           # one full circle
    assert math.hypot(o.x, o.y) < 0.02
    assert o.omega == pytest.approx(0.2, abs=0.01)


# ------------------------------------------------------------------- PID
def _first_order(pid, ref, T=3.0, dt=0.01, tau=0.2, gain=1.0, umax=1.0):
    y, ys = 0.0, []
    for _ in range(int(T / dt)):
        u = pid.step(ref - y, dt)
        y += dt / tau * (gain * u - y)
        ys.append(y)
    return ys


def test_pid_tracks_without_steady_state_error():
    ys = _first_order(PID(1.0, 5.0, 0.0, -1, 1), 0.5)
    assert ys[-1] == pytest.approx(0.5, abs=1e-3)


@pytest.mark.parametrize('mode', ['backcalc', 'clamp'])
def test_anti_windup_limits_integrator(mode):
    aw = PID(1.0, 5.0, 0.0, -1, 1, anti_windup=mode)
    raw = PID(1.0, 5.0, 0.0, -1, 1, anti_windup='none')
    for _ in range(300):                             # 3 s with an unreachable error
        aw.step(5.0, 0.01)
        raw.step(5.0, 0.01)
    assert aw.saturated and raw.saturated
    assert abs(aw.i_term) < 2.0                      # stays near the limit
    assert raw.i_term > 50.0                         # wound up


def test_wheel_controller_same_gains_and_ramp():
    c = WheelSpeedController(max_wheel_accel=20.0)
    duty = c.step([3, 3, 3, 3], [0, 0, 0, 0], 0.01)
    assert len({round(d, 12) for d in duty}) == 1
    assert c.ref[0] == pytest.approx(0.2)            # 20 rad/s^2 * 0.01 s
    assert all(-1 <= d <= 1 for d in c.step([50] * 4, [0] * 4, 0.01))


def test_step_metrics():
    ts = [i * 0.01 for i in range(200)]
    ys = [1 - math.exp(-t / 0.1) for t in ts]
    m = step_metrics(ts, ys, 0.0, 1.0)
    assert m['rise'] == pytest.approx(0.1 * math.log(9), abs=0.02)
    assert m['overshoot'] == 0.0


# ------------------------------------------------------------------- EKF
def test_ekf_fuses_gps_and_rejects_outlier():
    rng = random.Random(0)
    f = EKF()
    f.initialize(0.0)
    t = 0.0
    for k in range(1, 1001):                         # 20 s straight at 0.5 m/s
        t = k * 0.02
        f.update_wheel_odom(t, 0.5, 0.0, 0.0025, 0.04)
        f.update_gyro(t, rng.gauss(0, 0.002), 1e-4)
        if k % 10 == 0:
            f.update_gps(t, 0.5 * t + rng.gauss(0, 0.3), rng.gauss(0, 0.3), 0.09)
    x, y, th, v, _ = f.state
    assert x == pytest.approx(10.0, abs=0.25) and abs(y) < 0.25
    assert v == pytest.approx(0.5, abs=0.01)
    assert f.update_gps(t, x + 20.0, y, 0.09) == 'rejected'


def test_ekf_heading_from_gps_motion():
    """Heading error is observable through GPS when the robot moves."""
    f = EKF()
    f.initialize(0.0, theta=0.3)                    # 17 deg wrong start heading
    f.P[2, 2] = 0.2
    for k in range(1, 1501):
        t = k * 0.02
        f.update_wheel_odom(t, 0.5, 0.0, 0.0025, 0.04)
        f.update_gyro(t, 0.0, 1e-4)
        if k % 10 == 0:
            f.update_gps(t, 0.5 * t, 0.0, 0.09)
    assert abs(f.state[2]) < 0.05


def test_ekf_covariance_grows_during_dropout():
    f = EKF()
    f.initialize(0.0)
    p0 = f.P[0, 0]
    for k in range(1, 501):
        f.update_wheel_odom(k * 0.02, 0.5, 0.0, 0.0025, 0.04)
    assert f.P[0, 0] > p0


# ------------------------------------------------------------- navigator
@pytest.mark.parametrize('goal', [(4, 0), (0, 4), (-3, -3), (5, -2), (-1, 0.2)])
def test_navigator_converges_on_unicycle(goal):
    nav = Navigator()
    nav.set_goal(*goal)
    x = y = th = 0.0
    dt = 0.05
    for _ in range(2000):
        v, w = nav.step((x, y, th), dt)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        if nav.state == Navigator.DONE and abs(v) < 1e-3:
            break
    assert nav.state == Navigator.DONE
    assert math.hypot(x - goal[0], y - goal[1]) < 0.15
