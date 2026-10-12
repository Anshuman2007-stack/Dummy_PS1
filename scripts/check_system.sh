#!/usr/bin/env bash
# Quick health check of a RUNNING system: nodes, topics, rates and the TF odom -> base_link.
# Usage:  source install/setup.bash && ./scripts/check_system.sh
set -u

if ! command -v ros2 >/dev/null; then
  echo "ros2 not found - source /opt/ros/humble/setup.bash and install/setup.bash first." >&2
  exit 1
fi

pass=0; fail=0
ok()   { echo "  [ OK ] $*"; pass=$((pass+1)); }
bad()  { echo "  [FAIL] $*"; fail=$((fail+1)); }

echo "== Nodes =="
nodes="$(ros2 node list 2>/dev/null)"
for n in /ros_gz_bridge /robot_state_publisher /motor_driver_sim /wheel_odometry_node /gps_node \
         /kinematics_node /wheel_pid_node /ekf_node /navigation_node; do
  if echo "$nodes" | grep -q "^${n}$"; then ok "$n"; else bad "$n not running"; fi
done

echo "== Topics (one message within 3 s) =="
for t in /clock /joint_states /wheel_states /odom/wheels /imu/data /gnss/fix /true_odom \
         /wheel_cmd /wheel_pwm /ekf/odom; do
  if timeout 3 ros2 topic echo --once "$t" >/dev/null 2>&1; then ok "$t"; else bad "$t no data"; fi
done

echo "== Rates (4 s sample, sim time) =="
for t in /wheel_states /odom/wheels /imu/data /gnss/fix /wheel_cmd /wheel_pwm /ekf/odom; do
  r="$(timeout 4 ros2 topic hz "$t" 2>/dev/null | grep 'average rate' | tail -1 | awk '{print $3}')"
  echo "  $t : ${r:-no data} Hz"
done
echo "  expected: wheel_states 100, odom/wheels 50, imu 100, gnss 5, wheel_cmd 50, wheel_pwm 100, ekf 50"

echo "== TF odom -> base_link =="
if timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -q Translation; then
  ok "odom -> base_link"; else bad "no odom -> base_link transform"; fi

echo
echo "Result: ${pass} ok, ${fail} failed"
[ "$fail" -eq 0 ]
