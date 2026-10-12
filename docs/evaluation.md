# Evaluation

Three levels of evidence, from fastest to most realistic:

| Level | What runs | How | Output |
|---|---|---|---|
| A. Offline simulator | `ugv_core` algorithms + YAML params on a 4-wheel skid-steer physics model, 5 seeds | `python3 tools/offline_sim/run_evaluation.py` (about 1 min, no ROS) | [`results/results.md`](results/results.md) + plots |
| B. Node harness | the **actual ROS node classes** (all 7 + test nodes), wired by topic, simulated clock, same physics model in place of Gazebo | `python3 tools/offline_sim/ros_harness.py` (about 10 s, no ROS) | [`results/node_harness/`](results/node_harness/) |
| C. Gazebo | the full launch files on ROS 2 Humble + Gazebo Fortress | commands in §5 | your `output_dir` |

A and B were produced for this report. **C must be run on a ROS 2 Humble machine.** The
contact model, the bridge and the timing in Gazebo differ from the offline model, so expect
the same behaviour with somewhat different numbers. Re-tune with the procedures in
`pid_tuning.md` if needed.

Sensor models (all levels): IMU gyro σ = 0.002 rad/s at 100 Hz; GPS 5 Hz with σ = 0.3 m;
encoders 8192 ticks/rev. In A and B the real wheels are also 0.6 % (left) and 0.4 % (right)
off nominal size, and skid-steer slip is present, so encoder odometry drifts as it would on
a real robot.

## 1. State estimation

Offline simulator, RMSE over 30 s, mean of 5 seeds:

| Scenario | EKF pos [m] | EKF heading [rad] | EKF speed [m/s] | Encoder-only pos [m] | Encoder-only heading [rad] |
|---|---|---|---|---|---|
| Stationary | 0.043 | 0.001 | 0.000 | 0.000 | 0.000 |
| Straight line (v = 0.3 m/s) | 0.058 | 0.021 | 0.001 | 0.288 | 0.083 |
| Rotation (ω = 0.8 rad/s) | 0.043 | 0.004 | 0.002 | 0.002 | 0.264 |
| Combined (v = 0.3, ω = 0.3) | 0.057 | 0.026 | 0.002 | 0.124 | 0.120 |
| Combined + 10 s GPS dropout | 0.060 | 0.029 | 0.002 | 0.124 | 0.120 |

- **Fusion beats every single sensor.** Raw GPS has 0.42 m RMSE, and encoder odometry drifts
  0.29 m and 0.26 rad. The EKF stays at about 5 cm and 0.02 rad.
- **GPS dropout (10 s):** the worst error during the dropout is 0.064 m. The 2σ bound grows
  and then contracts within 2 s after the signal returns (`results/ekf_dropout.png`). The
  15 s dropout in the navigation test still reaches all goals.
- **Noise sensitivity:** GPS σ 0.15 → 1.0 m raises the position RMSE from 0.039 to 0.077 m.
  Ten times the gyro noise has no visible effect. An unmodelled 0.005 rad/s gyro bias raises
  heading RMSE from 0.026 to 0.043 rad. Using Gazebo's noise-free IMU yaw would make heading
  perfect, which is why it is off.

Node harness (real `ekf_node` + `ekf_eval_node`, scripted motion; encoder-only errors accumulate
over the whole sequence):

```
segment                    EKF pos  EKF hdg  EKF spd  odom pos  odom hdg
Stationary                   0.032    0.000    0.000     0.000     0.000
Straight line                0.082    0.025    0.002     0.072     0.041
Rotation                     0.051    0.005    0.002     0.164     0.206
Combined                     0.038    0.012    0.002     0.587     0.375
Combined + GPS dropout       0.051    0.006    0.002     0.697     0.515
                         max EKF position error during dropout: 0.062 m
```

## 2. Navigation

Goals visited in sequence from (0, 0). Final error is measured on **ground truth** 1 s after
"goal reached". Offline simulator, 5 seeds:

| Goal (x, y) | Reached | Final error mean / max [m] | Time [s] | Path ratio | Re-rotations |
|---|---|---|---|---|---|
| (4, 0) | 5/5 | 0.040 / 0.058 | 13.3 | 0.998 | 0.2 |
| (0, 4) | 5/5 | 0.102 / 0.124 | 18.7 | 0.988 | 0.0 |
| (-3, -3) | 5/5 | 0.060 / 0.135 | 23.5 | 0.995 | 0.0 |
| (5, -2) | 5/5 | 0.073 / 0.116 | 24.8 | 0.995 | 0.0 |

Path ratio is driven length divided by straight-line distance. Values close to 1 mean straight,
smooth paths (the robot turns on the spot first). Re-rotations count DRIVE→ROTATE switches; a
value near 0 means no oscillation. Results with a 15 s GPS dropout are in `results/results.md`
(all goals reached, final error 0.07–0.10 m). The final error is set by the EKF accuracy
(about 5 cm) plus the 5 cm goal tolerance.

Node harness (real `navigation_node` + `mission_node`):

```
goal          ok    time[s]  err_true[m]  err_ekf[m]  path_ratio  re-rot
(4, 0)        yes      15.4        0.045       0.084       1.007       0
(0, 4)        yes      19.2        0.019       0.050       0.998       0
(-3, -3)      yes      23.7        0.072       0.035       0.991       0
(5, -2)       yes      25.0        0.077       0.040       0.993       0
EKF vs truth over the run: position RMSE 0.052 m, heading RMSE 0.016 rad
```

![mission](results/nav_mission.png)

## 3. Wheel PID / motion control

Node harness (real `kinematics_node` → `wheel_pid_node` → `motor_driver_sim`, measured on the
true joint speed, 2 % band). The reference is ramped at 20 rad/s², so the ramp alone takes
0.15 s for 3 rad/s:

| Step | Rise 10–90 % [s] | Settling 2 % [s] | Overshoot [%] | Steady-state error [rad/s] | L/R mismatch steady [rad/s] |
|---|---|---|---|---|---|
| Forward 0 → 3 rad/s (v = 0.3 m/s) | 0.118 | 0.20 | 2.9–3.1 | ≤ 0.0003 | 0.004 |
| Spin 0 → ±3 rad/s (ω = 0.97 rad/s) | 0.23–0.24 | 0.41–0.42 | 0.4–1.2 | ≤ 0.0003 | 0.012 |

The offline simulator (less latency) gives 1.3–1.8 % overshoot and 0.14 s settling forward.
Wheel tracking RMSE over a whole navigation mission is 0.087 rad/s (about 3 % of cruise speed).
Saturation and anti-windup are covered in `pid_tuning.md` §1.6 and `results/anti_windup.png`.

![steps](results/pid_steps.png)

## 4. Real-world effects covered

| Effect (PS §1.6) | Where |
|---|---|
| Sensor noise | Gazebo IMU noise, `gps_node` σ, noise sweep above |
| Multi-rate / asynchronous fusion | EKF propagates to each message stamp (`math.md` §3) |
| Temporary GPS dropout | `gps_node` schedule or `/gnss/enable`, EKF dead-reckons, tests above |
| Wheel slip / odometry drift | skid-steer slip (χ), wheel-radius error, gyro-dominated ω in the EKF |
| Actuator saturation | duty clip ±1, anti-windup, curvature-preserving wheel-speed scaling |
| TF consistency | single `odom → base_link` publisher (EKF) + `robot_state_publisher` |

## 5. Gazebo procedure (level C)

```bash
source install/setup.bash
mkdir -p ~/ugv_results
ros2 launch ugv_control  pid_step_test.launch.py   headless:=true output_dir:=$HOME/ugv_results
ros2 launch ugv_bringup  eval_ekf.launch.py        headless:=true output_dir:=$HOME/ugv_results
ros2 launch ugv_bringup  eval_navigation.launch.py headless:=true output_dir:=$HOME/ugv_results
ros2 launch ugv_bringup  eval_navigation.launch.py headless:=true output_dir:=$HOME/ugv_results/dropout \
    gps_dropout:="[10.0, 15.0]"
```

Each prints its table when finished, writes `*_summary.txt` and a CSV, and can then be
stopped with Ctrl-C. Run `./scripts/kill_sim.sh` between runs. Record a bag at the same time
for plots:

```bash
ros2 bag record -o ~/ugv_results/bag /ekf/odom /true_odom /odom/wheels /imu/data /gnss/fix \
    /cmd_vel /wheel_cmd /wheel_pid/state /wheel_pwm /nav/state /tf
```

Paste the Gazebo tables here when you have them:

| Test | Gazebo result |
|---|---|
| PID step (pid_step_test_summary in console) | _to be measured_ |
| EKF segments (ekf_eval_summary.txt) | _to be measured_ |
| Navigation (mission_summary.txt) | _to be measured_ |
| Navigation with dropout | _to be measured_ |
| `slip_calibration` χ | _to be measured_ |
