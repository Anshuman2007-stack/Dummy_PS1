"""Evaluation: drive to a list of goals and score them on ground truth.

Usage:
    ros2 launch ugv_bringup eval_navigation.launch.py headless:=true output_dir:=$HOME/ugv_results
    ros2 launch ugv_bringup eval_navigation.launch.py goals:="[3.0, 3.0, -2.0, 1.0]"
    ros2 launch ugv_bringup eval_navigation.launch.py gps_dropout:="[10.0, 15.0]"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('ugv_control'), 'launch', 'control.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items())
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('ugv_nav'), 'launch', 'nav_stack.launch.py')),
        launch_arguments={'goal_input': 'none'}.items())
    mission = Node(
        package='ugv_nav', executable='mission_node', name='mission_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_dir': LaunchConfiguration('output_dir'),
            'goals': ParameterValue(LaunchConfiguration('goals')),        # YAML-parsed
            'gps_dropout': ParameterValue(LaunchConfiguration('gps_dropout')),
        }])
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('goals', default_value='[4.0, 0.0, 0.0, 4.0, -3.0, -3.0, 5.0, -2.0]',
                              description='flat list x1, y1, x2, y2, ... (floats)'),
        DeclareLaunchArgument('gps_dropout', default_value='[0.0, 0.0]',
                              description='[start, duration] s after mission start'),
        control, nav,
        TimerAction(period=5.0, actions=[mission]),
    ])
