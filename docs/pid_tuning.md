# PID tuning methodology

Two sets of PID loops exist. The **wheel velocity PIDs** (Stage 5, one per wheel, identical
gains) track wheel speed. The **navigation PIDs** (Stage 3) turn heading and distance errors
into `(v, ω)`. Tuning ran inside out: wheels first, then navigation.

## 1. Wheel velocity PID

### Plant
Per wheel `J_eff·ω̇ = τ_s·u − b·ω − τ_load`, with `J_eff = J_w + m r²/4 ≈ 0.0225 kg·m²`,
`τ_s = 5 N·m`, `b = 0.5 N·m·s/rad`. That is a first-order system with gain `τ_s/b = 10 rad/s`
per unit duty and time constant `J_eff/b ≈ 45 ms`. The feedback is the encoder speed
(8192 ticks/rev, differenced at 100 Hz). It is quantised to 0.077 rad/s and lags by about
5–15 ms.

### Procedure
1. **Feed-forward first.** Static FF `u = ω_ref/ω₀` comes from the motor model, so with
   `Kp = Ki = 0` the wheel already reaches about 90 % of the target unloaded. Acceleration FF
   `k_a ≈ J_eff/τ_s = 0.0045` was the model value. Sweeping it gave the lowest overshoot at
   0.002, because the ramp and the measurement lag already account for part of it.
2. **Reference ramp.** Pure steps made the wheel slip ahead of the chassis, and the
   integrator wound up (18–26 % overshoot for every gain set tried). The reference is ramped
   at 20 rad/s², which is achievable without wheelspin.
3. **Kp/Ki sweep** on a forward step 0 → 3 rad/s (light load) and a spin step ±3 rad/s
   (heavy skid-steer scrubbing load). For each pair we recorded rise time, 2 % settling,
   overshoot and steady-state error. Results of the sweep:

   | Kp | Ki | Extra | Fwd OS | Fwd settle | Spin settle | Note |
   |---|---|---|---|---|---|---|
   | 0.05 | 0.8 | pure step | 18 % | 0.43 s | 0.43 s | integrator windup during wheelspin |
   | 0.10 | 1.0 | ramp + FF | 10.7 % | 0.38 s | 0.60 s | encoder lag integrated during ramp |
   | 0.20 | 1.0 | ramp + FF | 12.1 % | 0.34 s | 1.44 s | too much Kp: noise on quantised speed |
   | 0.10 | 1.0 | + I-hold | 4.5 % | 0.19 s | 0.70 s | spin load now integrated too late |
   | **0.12** | **2.0** | **+ I-hold, k_a 0.002** | **2.9 %** | **0.20 s** | **0.42 s** | **chosen** |

4. **Ziegler–Nichols as a cross-check.** With feed-forward on, the loop has no sustained
   oscillation for any Kp below where encoder quantisation noise dominates (Kp ≳ 0.3). The
   ultimate gain is not a meaningful operating point here, so Z-N was used only to confirm
   that the chosen Kp has a large margin. The final gains came from the sweep above.
5. **Same gains on all four wheels.** This avoids left/right desynchronisation. Verified:
   steady left–right mismatch is 0.004 rad/s (forward) and 0.012 rad/s (spin).
6. **Anti-windup.** With a 12 rad/s request (impossible, since no-load speed is 10 rad/s) for 2 s
   and then 4 rad/s:
   - without anti-windup the duty stays saturated 0.76 s after the drop;
   - back-calculation undershoots 27 %;
   - conditional integration (`clamp`) recovers in 0.39 s with 1 % undershoot, so it is the default.

### Re-tuning in Gazebo
```bash
ros2 launch ugv_control pid_step_test.launch.py headless:=true output_dir:=$HOME/ugv_results
```
This prints the same metrics table from the true joint speeds and writes `pid_step_test.csv`.
To change the gains, edit `src/ugv_control/config/control.yaml` (`kp`, `ki`, `accel_ff`,
`max_wheel_accel`, `anti_windup`) and relaunch. With `--symlink-install` no rebuild is needed.
Plot the run with `ros2 run plotjuggler plotjuggler` on `/wheel_pid/state` (ref ×4, measured ×4,
duty ×4).

## 2. Navigation PIDs

Tuned after the wheel loops, on goals in all four quadrants:

1. Heading loop alone (ROTATE state): raise `kp_ang` until 90° turns finish fast without
   overshooting `rotate_done`. Kp 2.0 plus Kd 0.1 (filtered) damps the remaining overshoot.
   `ki = 0`, because the wheel PIDs already remove steady-state error.
2. Distance loop: `kp_lin = 0.8` saturates at `max_linear` beyond 0.5 m, and the robot
   decelerates linearly inside that. Slew limits (0.6 m/s², 2.5 rad/s²) keep the motion
   smooth.
3. `k_h > k_d` (2.0 > 0.8) is required for convergence (see `math.md` §4).
4. Goal tolerance: lowering `goal_tol` from 0.10 to 0.05 m (and `steer_freeze` from 0.20 to
   0.15 m) cut the mean final error from 0.116 to 0.070 m at a cost of about 1 s per goal.
   Going to 0.04 m caused re-rotations (oscillation), because the tolerance then approaches the
   EKF noise.

Watch it live:
```bash
ros2 topic echo /nav/state
ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z
```
