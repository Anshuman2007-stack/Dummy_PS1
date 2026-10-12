"""Stage 6 - end-to-end system.

  Gazebo + 4-wheel robot + bridge + TF + motor driver   (my_bot_sim/sim.launch.py)
  encoder odometry + GPS                                (ugv_sensors/sensors.launch.py)
  kinematics + wheel PID                                (ugv_control/control.launch.py)
  EKF + navigation + goal input                         (ugv_nav/nav_stack.launch.py)

Usage:
  ros2 launch ugv_bringup full_system.launch.py
  ros2 launch ugv_bringup full_system.launch.py headless:=true goal_input:=none
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('ugv_control'), 'launch', 'control.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items())
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('ugv_nav'), 'launch', 'nav_stack.launch.py')),
        launch_arguments={'goal_input': LaunchConfiguration('goal_input')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = Gazebo server only (no GUI)'),
        DeclareLaunchArgument('goal_input', default_value='auto',
                              description='auto | xterm | gnome-terminal | none'),
        control,
        nav,
    ])
