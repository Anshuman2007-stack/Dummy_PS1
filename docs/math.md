# Mathematical formulation

Symbols: wheel radius `r = 0.1 m`, track width (left–right wheel centres) `B = 0.4 m`,
wheelbase `L = 0.3 m`, skid-steer slip factor `χ = 1.55`. Wheel order everywhere:
front-left (FL), rear-left (RL), front-right (FR), rear-right (RR). A positive wheel speed drives
the robot forward. Code: `src/ugv_core/ugv_core/`.

## 1. Four-wheel differential-drive kinematics (Stage 4, `kinematics.py`)

The robot is a skid-steer vehicle. Both wheels on one side are on the same rigid chassis side
and must roll at the same ground speed, so they get the same command.

**Inverse kinematics** (body twist → wheel speeds):

```
ω_L = (v − ω·χB/2) / r          → FL, RL
ω_R = (v + ω·χB/2) / r          → FR, RR
```

**Forward kinematics** (wheel speeds → body twist, sides averaged):

```
v = r·(ω_R + ω_L)/2             ω = r·(ω_R − ω_L)/(χB)
```

**Why χ.** A two-wheel differential drive turns about a point on its axle, and the wheels roll
without sliding sideways. With four wheels the front and rear pairs must *slide sideways* to turn.
The scrubbing friction moves the instantaneous centres of rotation of the two wheel tracks
outwards, so the robot behaves like a differential drive with an *effective* track `χB` (χ ≥ 1).
χ depends on the wheelbase/track ratio and friction. It is measured by spinning on the spot and
comparing the gyro with the wheels (`ros2 run ugv_control slip_calibration`):
`χ = r(ω_R − ω_L) / (B·ω_gyro)`. The config uses 1.55 (a model of this robot
on isotropic Coulomb friction gives 1.55–1.58); recalibrate it in Gazebo.

**Saturation.** If a wheel would exceed `max_wheel_speed` (8 rad/s), both sides are multiplied
by the same factor `s = ω_max / max(|ω_L|, |ω_R|)`. The ratio `ω_L/ω_R`, and therefore the path
curvature `ω/v`, is preserved: a saturated command still follows the intended arc, just slower.

## 2. Wheel odometry from encoders (Stage 1, `odometry.py`)

Gazebo publishes continuous joint angles. They are quantised first, as a 2048-line
quadrature encoder with ×4 decoding would report them: `ticks = floor(θ / 2π · 8192)`.
Every 10 ms:

```
ω_i  = 2π·(ticks_i − ticks_i,prev) / (8192·Δt)                       per wheel
v, ω = forward kinematics of (mean(ω_FL, ω_RL), mean(ω_FR, ω_RR))
Δs = v·Δt,  Δθ = ω·Δt
x += (Δs/Δθ)·(sin(θ+Δθ) − sin θ),   y −= (Δs/Δθ)·(cos(θ+Δθ) − cos θ),   θ += Δθ
```

The last line is exact arc integration (straight-line formula when `Δθ ≈ 0`). `/wheel_states`
(100 Hz) carries the per-wheel speeds for the PIDs. `/odom/wheels` (50 Hz) carries pose and twist.
The EKF only uses the twist. The integrated pose drifts with wheel-radius errors and slip
(0.29 m after 9 m in the evaluation), which is exactly why it is fused rather than trusted.

## 3. EKF (Stage 2, `ekf.py`)

State `x = [x, y, θ, v, ω]ᵀ` in the `odom` frame (= Gazebo world = ENU at the GPS datum).

### Prediction (unicycle, constant v and ω over Δt)

```
x ← x + v·cosθ·Δt     y ← y + v·sinθ·Δt     θ ← wrap(θ + ω·Δt)     v ← v     ω ← ω

F = ∂f/∂x = I₅ +  [ 0 0 −v·sinθ·Δt  cosθ·Δt  0  ]
                  [ 0 0  v·cosθ·Δt  sinθ·Δt  0  ]
                  [ 0 0  0          0        Δt ]
                  [ 0 0  0          0        0  ]
                  [ 0 0  0          0        0  ]

P ← F P Fᵀ + Q·Δt,     Q = diag(1e-4, 1e-4, 1e-5, 0.5, 0.5)   (per second)
```

`v` and `ω` are random walks. `q_v` and `q_ω` set how fast they may change (acceleration). The
small `q_pos` and `q_θ` allow for slip that is not explained by `v` and `ω`. These values were
tuned on a model of the robot: lowering `q_pos` from 1e-3 to 1e-4 cut the stationary position
error from 5.4 cm to 3.9 cm, because less GPS noise leaks into the estimate.

### Measurement updates

| Sensor | Rate | z | H selects | R |
|---|---|---|---|---|
| Wheel odometry | 50 Hz | `[v, ω]` | v, ω | `diag(0.0025, 0.04)` (ω large: skid-steer slip) |
| Gyro | 100 Hz | `ω_z` | ω | `1e-4` (σ = 0.002 rad/s, inflated for timing) |
| IMU yaw (off by default) | 100 Hz | `θ` | θ | `4e-4` |
| GPS | 5 Hz | `[x, y]` (ENU of the fix) | x, y | `σ²·I`, σ = 0.3 m (from the message covariance) |

```
y = z − H x  (angle residuals wrapped)       S = H P Hᵀ + R       K = P Hᵀ S⁻¹
x ← x + K y                                   P ← (I−KH) P (I−KH)ᵀ + K R Kᵀ   (Joseph form)
```

**Asynchronous / multi-rate fusion.** Every measurement is applied at its own header stamp.
The filter is first propagated from its last time to that stamp (`predict(t)`), then updated. No
fixed rate is assumed, so 50/100/5 Hz streams with jitter fuse correctly in any order. The
published estimate is extrapolated to "now" at 50 Hz without changing the filter.

**GPS → metres.** `east = R_E·Δlon·cos(lat₀)`, `north = R_E·Δlat`, about the fixed datum
(22.62° N, 75.69° E), which is the Gazebo world origin.

**Outlier gating.** A fix is rejected if its normalised innovation squared `yᵀS⁻¹y > 9.21`
(χ² with 2 DOF, 99 %). After 10 consecutive rejections the filter itself is assumed wrong (for
example after a long dropout with slip). P is inflated and the fix is accepted.

**GPS dropout.** The receiver publishes `STATUS_NO_FIX`, which the EKF ignores. Prediction plus
wheel and gyro updates continue, so the position is dead-reckoned and `P_xx, P_yy` grow. The first fix afterwards pulls the estimate back.

**Why IMU yaw is off by default.** Gazebo's IMU orientation is the noise-free simulator
orientation, so fusing it would hide the estimator's real behaviour. Heading comes from
integrating the gyro, and it is observable through GPS once the robot moves: the x–θ / y–θ
cross-covariance in F lets position innovations correct θ (tested in `test_ekf_heading_from_gps_motion`).

## 4. Target navigation (Stage 3, `navigator.py`)

```
d = ‖(x_g − x, y_g − y)‖              e = wrap(atan2(y_g − y, x_g − x) − θ)
```

| State | v | ω | Transition |
|---|---|---|---|
| ROTATE | 0 | `PID_h(e)` | → DRIVE when `\|e\| < 0.10 rad` |
| DRIVE | `min(PID_d(d), v_max)·max(0, cos e)` | `PID_h(e)` (0 inside `d < 0.15 m`) | → ROTATE when `\|e\| > 0.70 rad` |
| DONE | 0 | 0 | when `d < goal_tol = 0.05 m` |

Gains: `PID_d` with Kp 0.8; `PID_h` with Kp 2.0, Kd 0.1. Limits `v_max = 0.4 m/s`,
`ω_max = 1.2 rad/s`. Slew-rate limits are 0.6 m/s² and 2.5 rad/s².

**Justification.** In polar coordinates the error dynamics are `ḋ = −v cos e` and
`ė = (v/d)·sin e − ω`. Take the law `v = k_d·d·cos e`, `ω = k_h·e` and the Lyapunov
function `V = ½d² + ½e²`:

```
V̇ = d·ḋ + e·ė = −k_d d² cos²e + k_d·e·sin e·cos e − k_h e²
   ≤ −k_d d² cos²e − (k_h − k_d)·e²            (because |sin e cos e| ≤ |e|)
```

For `k_h > k_d` (here 2.0 > 0.8), `V̇ < 0` everywhere except at `d = 0, e = 0`, so the robot
converges to the goal. Saturating `v` at `v_max` only makes `v/d` smaller, so the bound still
holds.
The extra pieces handle the parts of the plane where this law misbehaves:
- **Turn first:** with a goal behind the robot (`|e| > π/2`) the law would back away or spiral.
- **`cos e` factor:** forward speed vanishes when the robot is not facing the goal.
- **Steering freeze inside 0.15 m:** `atan2` becomes ill-conditioned near the goal, and the
  GPS-induced estimate noise would make it chatter.
- **Slew limits:** give a smooth trajectory and keep the wheel loops out of saturation.

## 5. Wheel velocity control (Stage 5, `wheel_control.py`, `pid.py`)

**Motor/plant model.** DC gear-motor: `τ = τ_s·u − b·ω_w`, with duty `u ∈ [−1, 1]`,
stall torque `τ_s = 5 N·m` and back-EMF damping `b = τ_s/ω₀ = 0.5 N·m·s/rad` (no-load speed
`ω₀ = 10 rad/s`). In Gazebo `b` is the wheel joint `<damping>`, and `τ_s·u` is applied through
ApplyJointForce by `motor_driver_sim`.

**Controller** (same gains on all four wheels):

```
ω_ref  = ramp(ω_target, 20 rad/s²)                                     reference shaping
u      = clip( ω_ref/ω₀ + k_a·dω_ref/dt + Kp·e + I − Kd·dω_meas/dt , −1, 1 ),   e = ω_ref − ω_meas
           \___ static FF ___/  \_ accel. FF _/
Kp = 0.12 /(rad/s),  Ki = 2.0 /rad,  Kd = 0,  k_a = 0.002 s²/rad
```

- *Static feed-forward* `ω_ref/ω₀` is the duty at which back-EMF balances the supply. The PID
  only has to supply the load torque (rolling resistance, skid-steer scrubbing).
- *Acceleration feed-forward* supplies the torque `(J_w + m r²/4)·ω̇` during the ramp.
- *Reference ramp:* a step faster than the tyres can follow (`μg/r ≈ 98 rad/s²`, less with the
  vehicle mass) only fills the integrator.
- *Integrator hold while ramping:* the integrator is frozen until the reference has reached its
  target. The encoder speed lags the reference by about 15 ms, and integrating that lag
  caused 10–11 % overshoot. With the hold the overshoot is 3 %.
- *Derivative* (unused, Kd = 0) would act on the measurement, filtered with τ = 20 ms.

**Saturation and anti-windup.** The duty is clipped to ±1. With `clamp` (conditional
integration, the default) the integrator is frozen whenever the output is saturated *and* the
error pushes further into saturation. `backcalc` (`İ = Ki·e + (u_sat − u)/T_t`) is also
implemented. It performs worse here because the static feed-forward alone can exceed the limit
(a 12 rad/s request → `ω/ω₀ = 1.2`). Back-calculation then drives the integrator negative
to cancel it, and that causes a 27 % undershoot when the request drops.
