# Evaluation

All experiments run in Gazebo with the launch files in this repository. Each one prints a
results table when it finishes, writes a `*_summary.txt` and a CSV to `output_dir`, and can then
be stopped with Ctrl-C. Run `./scripts/kill_sim.sh` between runs.

Sensor models: IMU gyro σ = 0.002 rad/s at 100 Hz; GPS 5 Hz with σ = 0.3 m (`sensors.yaml`);
encoders 8192 ticks/rev. Ground truth is `/true_odom`. Because the robot spawns at the world
origin with yaw 0, `/true_odom` and `/ekf/odom` are in the same frame.

```bash
source install/setup.bash
mkdir -p ~/ugv_results
```

## 0. Calibration (run first)

```bash
ros2 launch ugv_control control.launch.py headless:=true
ros2 run ugv_control slip_calibration --ros-args -p use_sim_time:=true     # second terminal
```

Put the printed `slip_factor` into `src/ugv_core/config/robot.yaml`.

| Quantity | Value |
|---|---|
| Skid-steer slip factor χ | |

## 1. Wheel PID / motion control

```bash
ros2 launch ugv_control pid_step_test.launch.py headless:=true output_dir:=$HOME/ugv_results
```

The test drives a 0 → 3 rad/s forward step (v = 0.3 m/s) and a ±3 rad/s spin step
(ω = 0.97 rad/s). It reports rise time (10–90 %), settling time (2 % band), overshoot, steady-state
error per wheel, and left/right mismatch, all measured on the true joint speed. The reference
is ramped at 20 rad/s², so a 3 rad/s step takes at least 0.15 s.

| Step | Wheel | Rise [s] | Settling [s] | Overshoot [%] | SSE [rad/s] |
|---|---|---|---|---|---|
| Forward | front_left | | | | |
| | rear_left | | | | |
| | front_right | | | | |
| | rear_right | | | | |
| Spin | front_left | | | | |
| | rear_left | | | | |
| | front_right | | | | |
| | rear_right | | | | |

| Step | Left/right mismatch max [rad/s] | Left/right mismatch steady [rad/s] |
|---|---|---|
| Forward | | |
| Spin | | |

Plot: `pid_step_test.csv` (true, reference, encoder and duty for each wheel), or PlotJuggler on
`/wheel_pid/state`. Saturation and anti-windup design: `pid_tuning.md` §1.

## 2. State estimation

```bash
ros2 launch ugv_bringup eval_ekf.launch.py headless:=true output_dir:=$HOME/ugv_results
```

Scripted `/cmd_vel` segments (each preceded by 2 s of rest): stationary 20 s, straight
v = 0.3 for 15 s, rotation ω = 0.8 for 15 s, combined v = 0.3 / ω = 0.3 for 20 s, and the
combined motion again with a 10 s GPS dropout. Columns are RMSE against ground truth.
The "odom" columns are encoder dead reckoning accumulated since the start, with no GPS or IMU.

| Segment | EKF pos [m] | EKF heading [rad] | EKF speed [m/s] | Odom pos [m] | Odom heading [rad] |
|---|---|---|---|---|---|
| Stationary | | | | | |
| Straight line | | | | | |
| Rotation | | | | | |
| Combined | | | | | |
| Combined + GPS dropout | | | | | |

Max EKF position error during the dropout: ___ m

Noise sensitivity: change `sigma` in `ugv_sensors/config/sensors.yaml` (and the EKF reads the
new value from the fix covariance), then re-run:

| GPS σ [m] | EKF pos RMSE (combined) [m] |
|---|---|
| 0.15 | |
| 0.3 | |
| 0.6 | |
| 1.0 | |

## 3. Navigation

```bash
ros2 launch ugv_bringup eval_navigation.launch.py headless:=true output_dir:=$HOME/ugv_results
ros2 launch ugv_bringup eval_navigation.launch.py headless:=true output_dir:=$HOME/ugv_results/dropout \
    gps_dropout:="[10.0, 15.0]"
```

Goals (4, 0), (0, 4), (−3, −3), (5, −2) are visited in sequence; the world has a coloured pad at
each one. Final error is measured on ground truth 1 s after "goal reached". Path ratio is the
driven length divided by the straight-line distance (1.00 = straight). Re-rotations count
DRIVE→ROTATE switches, an oscillation indicator.

| Goal | Reached | Time [s] | Final error true [m] | Final error EKF [m] | Path ratio | Re-rotations |
|---|---|---|---|---|---|---|
| (4, 0) | | | | | | |
| (0, 4) | | | | | | |
| (−3, −3) | | | | | | |
| (5, −2) | | | | | | |

With a 15 s GPS dropout:

| Goal | Reached | Final error true [m] |
|---|---|---|
| (4, 0) | | |
| (0, 4) | | |
| (−3, −3) | | |
| (5, −2) | | |

Trajectory plot: `mission_trajectory.csv` (true and EKF x, y, yaw over time).

## 4. Recording for plots

```bash
ros2 bag record -o ~/ugv_results/bag /ekf/odom /true_odom /odom/wheels /imu/data /gnss/fix \
    /cmd_vel /wheel_cmd /wheel_pid/state /wheel_pwm /nav/state /tf
```

## 5. Real-world effects covered

| Effect (PS §1.6) | Where |
|---|---|
| Sensor noise | Gazebo IMU noise, `gps_node` σ, noise sweep in §2 |
| Multi-rate / asynchronous fusion | EKF propagates to each message stamp (`math.md` §3) |
| Temporary GPS dropout | `gps_node` schedule or `/gnss/enable`; EKF dead-reckons (§2, §3) |
| Wheel slip / odometry drift | skid-steer slip factor χ, gyro-dominated ω in the EKF, odom columns in §2 |
| Actuator saturation | duty clip ±1, anti-windup, curvature-preserving wheel-speed scaling |
| TF consistency | single `odom → base_link` publisher (EKF) + `robot_state_publisher` |
