"""Stage 3 - navigation controller only. Needs /ekf/odom, publishes /cmd_vel."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ugv_nav'), 'config', 'ugv_nav.yaml')
    return LaunchDescription([
        Node(package='ugv_nav', executable='navigation_node', name='navigation_node',
             output='screen', parameters=[params, {'use_sim_time': True}]),
    ])
