"""Pure-Python algorithm library for the UGV stack (no ROS dependency).

Every ROS node in this repository is a thin wrapper around one of these
classes, so the algorithms can be unit-tested without ROS
(``python3 -m pytest src/ugv_core/test``).

Modules
-------
geo         GPS latitude/longitude <-> local east/north metres
kinematics  Four-wheel differential (skid-steer) drive kinematics   (Stage 4)
odometry    Wheel odometry from quantised encoder ticks             (Stage 1)
ekf         5-state EKF: x, y, theta, v, omega                     (Stage 2)
navigator   Target-based navigation control law                    (Stage 3)
pid         PID with feed-forward, derivative filter, anti-windup  (Stage 5)
"""

WHEEL_NAMES = ('front_left', 'rear_left', 'front_right', 'rear_right')
WHEEL_JOINTS = tuple(n + '_wheel_joint' for n in WHEEL_NAMES)
LEFT_WHEELS = (0, 1)    # indices into WHEEL_NAMES
RIGHT_WHEELS = (2, 3)
