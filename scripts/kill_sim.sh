#!/usr/bin/env bash
# Kill leftover Gazebo / bridge / node processes after a crashed or Ctrl-C'd run.
for p in "ign gazebo" "ruby.*ign" parameter_bridge robot_state_publisher motor_driver_sim \
         wheel_odometry_node gps_node kinematics_node wheel_pid_node ekf_node navigation_node \
         goal_pose_node mission_node ekf_eval_node pid_step_test slip_calibration; do
  pkill -f "$p" 2>/dev/null
done
sleep 1
echo "Remaining Gazebo/ROS processes (should be empty):"
pgrep -af "ign gazebo|parameter_bridge|_node|motor_driver_sim" || echo "  none"
