"""Stage 2 - EKF only.

Needs /odom/wheels, /imu/data, /gnss/fix
(e.g. `ros2 launch ugv_control control.launch.py` in another terminal).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ugv_nav'), 'config', 'ugv_nav.yaml')
    return LaunchDescription([
        Node(package='ugv_nav', executable='ekf_node', name='ekf_node', output='screen',
             parameters=[params, {'use_sim_time': True}]),
    ])
