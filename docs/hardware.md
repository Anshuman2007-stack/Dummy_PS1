# Optional extension: real-hardware deployment

Not required for the baseline. The stack is built so that only one node changes.

```
simulation:  wheel_pid_node --/wheel_pwm--> motor_driver_sim --> Gazebo joint torques
hardware:    wheel_pid_node --/wheel_pwm--> motor_bridge     --> microcontroller --> H-bridge PWM
                                  ▲
             encoder counts --> microcontroller --> /joint_states (angles from ticks)
```

1. **Microcontroller firmware** (e.g. ESP32 or STM32 with micro-ROS, or a serial link):
   - Read the 4 quadrature encoders with hardware counters and publish the angles as
     `sensor_msgs/JointState` on `/joint_states` (names `front_left_wheel_joint`, …) at ≥ 100 Hz.
     `wheel_odometry_node` then works unchanged; set `ticks_per_rev` to the real counts per
     wheel revolution (encoder CPR × 4 × gear ratio).
   - Take the 4 duties from `/wheel_pwm` (`Float64MultiArray`, −1…1) and set
     direction pin + PWM compare value = `|duty| × PWM_TOP` per motor driver channel.
   - Watchdog: zero all PWM if no `/wheel_pwm` arrives for 300 ms (same as `motor_driver_sim`).
2. **Robot constants:** measure `wheel_radius`, `track_width`, and the motor `no_load_speed`
   (run at duty 1.0 with the wheels in the air) in `ugv_core/config/robot.yaml`. Calibrate
   `slip_factor` on the floor with `ros2 run ugv_control slip_calibration`.
3. **Re-tune** the wheel PIDs on the real motors with `pid_step_test` (wheels on the ground),
   starting from the simulation gains with `kp` halved.
4. **Real sensors:** IMU driver → `/imu/data`, GNSS driver → `/gnss/fix`. Set the EKF
   `datum_lat`/`datum_lon` to the start position and set `use_sim_time: false` in the
   launch files.
