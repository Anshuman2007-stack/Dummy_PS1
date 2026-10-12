# UGV EKF Localisation, Navigation and Wheel Control (ROS 2 + Gazebo)

A complete navigation and control stack for a **four-wheel differential-drive (skid-steer) UGV**
in simulation. Given only a target `(x, y)`, the robot:

1. estimates its state `(x, y, θ, v, ω)` by fusing **IMU + wheel-encoder odometry + GPS** in an EKF;
2. computes `(v, ω)` towards the target with a navigation control law;
3. converts `(v, ω)` into **four individual wheel speeds**;
4. tracks those speeds with **one PID per wheel** (feed-forward, saturation, anti-windup) that
   outputs PWM duty to a motor model driving each Gazebo wheel joint.

```
          /goal_pose
              │
  ┌──────────▼─────────┐ /cmd_vel ┌───────────────┐ /wheel_cmd ┌──────────────┐ /wheel_pwm ┌────────────────┐
  │ navigation_node    ├────────►│kinematics_node├──────────►│wheel_pid_node├──────────►│motor_driver_sim├─► Gazebo
  └──────────▲─────────┘          └───────────────┘            └──────▲───────┘            └────────────────┘  4 wheel
             │ /ekf/odom + TF odom→base_link                          │ /wheel_states                         torques
  ┌──────────┴─────────┐ /odom/wheels ┌─────────────────────┐         │                                     │
  │ ekf_node           │◄─────────────┤ wheel_odometry_node ├─────────┘◄──────── /joint_states ◄───────────┤
  │                    │◄── /imu/data ◄──────────────────────────────────────────────────────────────────── ┤
  │                    │◄── /gnss/fix ◄── gps_node ◄── /true_odom ◄────────────────────────────────────────┘
  └────────────────────┘
```

## Problem-statement coverage

| # | Stage | Implementation | Run on its own |
|---|---|---|---|
| 1 | Sensor interfaces | Gazebo IMU (100 Hz) and wheel joint encoders → `wheel_odometry_node` quantises angles into **encoder ticks** (8192/rev) and **computes odometry** from them; `gps_node` = 5 Hz GNSS with noise + dropout | `ros2 launch ugv_sensors sensors.launch.py` |
| 2 | EKF fusion | `ekf_node`: state `(x, y, θ, v, ω)`, wheel twist + gyro + GPS, each fused at its own time stamp (multi-rate), χ² gating, dropout handling, publishes **TF `odom→base_link`** | `ros2 launch ugv_nav ekf.launch.py` |
| 3 | Target navigation | `navigation_node`: rotate-then-drive, PID on heading/distance, `cos e` speed scaling, slew limits; convergence proof in `docs/math.md` | `ros2 launch ugv_nav navigation.launch.py` |
| 4 | Diff-drive kinematics (4 wheels) | `kinematics_node`: `(v, ω)` → FL, RL, FR, RR wheel speeds with skid-steer slip factor and curvature-preserving saturation | `ros2 launch ugv_control control.launch.py` |
| 5 | Per-wheel PID | `wheel_pid_node`: 4 identical PIDs, static + acceleration feed-forward, duty saturation, **anti-windup** (clamp / back-calculation) | same, plus `pid_step_test.launch.py` |
| 6 | End-to-end | `ros2 launch ugv_bringup full_system.launch.py`: one closed loop, consistent TF tree, nested per-stage launch files | ← |
| – | Optional hardware | PID outputs PWM duty already; see [`docs/hardware.md`](docs/hardware.md) | |

Real-world effects modelled: sensor noise, multi-rate fusion, GPS dropout, wheel slip and odometry
drift, actuator saturation, TF consistency. Details are in [`docs/evaluation.md`](docs/evaluation.md) §4.

## 1. Repository layout

```
ugv-ekf-navigation/
├── src/
│   ├── ugv_core/      algorithms without ROS (ekf, kinematics, odometry, navigator, pid,
│   │                  wheel_control, geo, metrics) + config/robot.yaml + unit tests
│   ├── my_bot_sim/    Stage 0  world, 4-wheel SDF model, URDF (TF), bridge, motor_driver_sim
│   ├── ugv_sensors/   Stage 1  wheel_odometry_node, gps_node
│   ├── ugv_nav/       Stage 2-3  ekf_node, navigation_node, goal_pose_node, ekf_eval_node, mission_node
│   ├── ugv_control/   Stage 4-5  kinematics_node, wheel_pid_node, pid_step_test, slip_calibration
│   └── ugv_bringup/   Stage 6  full_system, eval_ekf, eval_navigation launch files
├── tools/offline_sim/ physics model + evaluation runner + ROS-node harness (no ROS needed)
├── docs/              architecture · math · pid_tuning · evaluation · hardware · results/
└── scripts/           setup.sh · check_system.sh · kill_sim.sh
```

## 2. Requirements

* Ubuntu 22.04, **ROS 2 Humble**, **Gazebo Fortress** (`ros-humble-ros-gz`). The SDF uses the
  Fortress plugin names (`ignition-gazebo-*`) and the bridge uses `ignition.msgs.*`.
* `ros-humble-robot-state-publisher`, `ros-humble-tf2-ros`, `python3-numpy`, `xterm` (goal window).
* Offline tools only: `python3-yaml`, `python3-matplotlib`.

## 3. Build

```bash
sudo apt install ros-humble-desktop ros-humble-ros-gz xterm
git clone https://github.com/<your-user>/ugv-ekf-navigation.git && cd ugv-ekf-navigation
source /opt/ros/humble/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash                       # in EVERY new terminal
```

Or run `./scripts/setup.sh`. Check the build with `ros2 pkg list | grep -E "ugv_|my_bot_sim"`,
which should list 6 packages.

## 4. Run the whole system

```bash
ros2 launch ugv_bringup full_system.launch.py            # headless:=true  goal_input:=none
```

Wait for `EKF initialised at (0, 0, 0). Ready for goals.`, then type a goal in the xterm
window, for example `4 5` (metres in the odom/world frame; `pos` shows the estimate, `q` quits).
Without the window:

```bash
ros2 run ugv_nav goal_pose_node --ros-args -p use_sim_time:=true
# or
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 4.0, y: 5.0}}}"
```

The world has coloured pads at the evaluation goals (4, 0), (0, 4), (−3, −3) and (5, −2).
Check health with `./scripts/check_system.sh`.

## 5. Run stage by stage

Launch files nest (sim ⊂ sensors ⊂ control), so each command below starts everything it needs.

```bash
# Stage 1 – sensors: Gazebo + encoders/odometry + GPS
ros2 launch ugv_sensors sensors.launch.py
ros2 topic hz /wheel_states /odom/wheels /imu/data /gnss/fix

# Stages 4+5 – kinematics + wheel PIDs; drive by hand
ros2 launch ugv_control control.launch.py
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"      # straight
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.8}}"     # spin
ros2 topic echo /wheel_cmd ; ros2 topic echo /wheel_pid/state                  # ref|meas|duty x4
ros2 launch ugv_control pid_step_test.launch.py                                 # step metrics
ros2 run ugv_control slip_calibration --ros-args -p use_sim_time:=true          # measure chi

# Stage 2 – add the EKF (in a second terminal, with control.launch.py running)
ros2 launch ugv_nav ekf.launch.py
ros2 topic echo /ekf/odom --field pose.pose.position
ros2 run tf2_ros tf2_echo odom base_link

# Stage 3 – add navigation
ros2 launch ugv_nav navigation.launch.py
ros2 run ugv_nav goal_pose_node --ros-args -p use_sim_time:=true
```

GPS dropout by hand: `ros2 topic pub --once /gnss/enable std_msgs/msg/Bool "{data: false}"`
(and `true` to restore), or set `dropout_start`/`dropout_duration` in `ugv_sensors/config/sensors.yaml`.

## 6. Evaluation

Results and plots: [`docs/evaluation.md`](docs/evaluation.md), generated tables in
[`docs/results/results.md`](docs/results/results.md).

In Gazebo (each prints a results table and writes CSV/summary files):

```bash
ros2 launch ugv_control pid_step_test.launch.py   headless:=true output_dir:=$HOME/ugv_results
ros2 launch ugv_bringup eval_ekf.launch.py        headless:=true output_dir:=$HOME/ugv_results
ros2 launch ugv_bringup eval_navigation.launch.py headless:=true output_dir:=$HOME/ugv_results
ros2 launch ugv_bringup eval_navigation.launch.py goals:="[3.0, 3.0, -2.0, 1.0]" gps_dropout:="[10.0, 15.0]"
```

`goals` must be floats (`3.0`, not `3`). Without ROS, using the identical algorithm code and the
real node classes:

```bash
python3 -m pytest src/ugv_core/test/test_core.py      # 23 unit tests
python3 tools/offline_sim/run_evaluation.py           # regenerates docs/results/
python3 tools/offline_sim/ros_harness.py              # runs all ROS nodes closed-loop
```

Headline numbers (offline model, 5 seeds): EKF position RMSE **4–6 cm** (raw GPS 42 cm; encoder
odometry drifts 29 cm on a 9 m line), worst error during a 10 s GPS dropout **6.4 cm**. All
navigation goals reached with **0.04–0.10 m** final error and no oscillation. Wheel step response
**3 %** overshoot, **0.2 s** settling, zero steady-state error, left/right mismatch < 0.02 rad/s.

## 7. Topics, frames, parameters

| Topic | Type | From → To |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → wheel_odometry_node, robot_state_publisher |
| `/wheel_states` | `sensor_msgs/JointState` | wheel_odometry_node → wheel_pid_node (encoder speeds) |
| `/odom/wheels` | `nav_msgs/Odometry` | wheel_odometry_node → ekf_node |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo → ekf_node |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | gps_node → ekf_node |
| `/gnss/enable` | `std_msgs/Bool` | user / eval nodes → gps_node (dropout) |
| `/true_odom` | `nav_msgs/Odometry` | Gazebo → gps_node, evaluation only |
| `/ekf/odom` + TF | `nav_msgs/Odometry` | ekf_node → navigation_node |
| `/goal_pose` | `geometry_msgs/PoseStamped` | goal_pose_node / mission_node → navigation_node |
| `/cmd_vel` | `geometry_msgs/Twist` | navigation_node → kinematics_node |
| `/wheel_cmd` | `std_msgs/Float64MultiArray` | kinematics_node → wheel_pid_node (FL, RL, FR, RR rad/s) |
| `/wheel_pwm` | `std_msgs/Float64MultiArray` | wheel_pid_node → motor_driver_sim (duty −1…1) |
| `/model/my_bot/joint/*/cmd_force` | `std_msgs/Float64` | motor_driver_sim → Gazebo (N·m) |

TF: `odom → base_link` (EKF) → `chassis_link`, `imu_link`, `gnss_link`, 4 wheels
(robot_state_publisher). The robot spawns at the world origin with yaw 0, so `odom` coincides
with the Gazebo world frame and `/true_odom` can be compared directly.

Parameters: `ugv_core/config/robot.yaml` (geometry, motor, slip factor, shared by all nodes),
`ugv_sensors/config/sensors.yaml`, `ugv_control/config/control.yaml`, `ugv_nav/config/ugv_nav.yaml`.
Edit and relaunch; with `--symlink-install` no rebuild is needed.

Robot: 4 wheels, r = 0.1 m, track 0.4 m, wheelbase 0.3 m, 7 kg; motors 5 N·m stall,
10 rad/s no-load; limits 0.4 m/s and 1.2 rad/s (navigation), 8 rad/s per wheel.

## 8. Debugging

| Symptom | Likely cause / fix |
|---|---|
| `Package ... not found` | `source install/setup.bash` in this terminal. |
| Robot does not move, `/wheel_pwm` is non-zero | Check `ros2 topic echo /model/my_bot/joint/front_left_wheel_joint/cmd_force`; check the bridge log for the `cmd_force` lines; the ApplyJointForce plugin name must be `ignition-gazebo-apply-joint-force-system` (Fortress). |
| `No /wheel_states - motors off` keeps repeating | `/joint_states` is missing: the JointStatePublisher topic is `/world/my_world/model/my_bot/joint_state`; check with `ign topic -l`. One warning at start-up is normal. |
| Nodes stuck at time 0 | `/clock` not bridged or Gazebo paused (the world starts with `-r`). |
| Robot turns too slowly or too fast for the command | Recalibrate `slip_factor` with `slip_calibration`. |
| Wheels oscillate | Lower `kp` in `control.yaml`; check `/wheel_pid/state` in PlotJuggler. |
| Estimate jumps | `gps_node` `sigma` must match reality; inspect `/ekf/odom` covariance. |
| Robot circles the goal | Lower `kp_lin` or raise `goal_tol` in `ugv_nav.yaml`. |
| `Failed to load system plugin [gz-sim-...]` | Garden/Harmonic names; this repo targets Fortress. |
| Stale processes | `./scripts/kill_sim.sh` |

## 9. Status

All six stages are implemented. They have been verified with 23 unit tests, a 5-seed offline
evaluation, and a closed-loop run of every ROS node against a physics model (`tools/offline_sim`).
The Gazebo runs (level C in `docs/evaluation.md`) still have to be performed on a Humble
machine and their tables filled in. If Gazebo's contact model behaves differently from the
offline model, re-run `slip_calibration` and `pid_step_test` first.

## 10. License

MIT, see [LICENSE](LICENSE).
