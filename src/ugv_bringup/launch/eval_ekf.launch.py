"""Evaluation of state estimation (scripted /cmd_vel, no navigation).

Stationary / straight / rotation / combined motion and a 10 s GPS dropout.

    ros2 launch ugv_bringup eval_ekf.launch.py headless:=true output_dir:=$HOME/ugv_results
Prints a table when done (~100 s sim time) and writes ekf_eval_summary.txt + ekf_eval.csv.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('ugv_control'), 'launch', 'control.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items())
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('ugv_nav'), 'launch', 'ekf.launch.py')))
    test = Node(package='ugv_nav', executable='ekf_eval_node', name='ekf_eval_node',
                output='screen',
                parameters=[{'use_sim_time': True,
                             'output_dir': LaunchConfiguration('output_dir')}])
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=''),
        control, ekf,
        TimerAction(period=5.0, actions=[test]),
    ])
