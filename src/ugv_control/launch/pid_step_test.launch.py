"""Stage 5 evaluation: Gazebo + sensors + kinematics + wheel PID + step test.

Usage:
    ros2 launch ugv_control pid_step_test.launch.py headless:=true output_dir:=$HOME
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
    robot_yaml = os.path.join(share('ugv_core'), 'config', 'robot.yaml')
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('ugv_control'), 'launch', 'control.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items())
    test = Node(package='ugv_control', executable='pid_step_test', name='pid_step_test',
                output='screen',
                parameters=[robot_yaml, {'use_sim_time': True,
                                         'output_dir': LaunchConfiguration('output_dir')}])
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value='',
                              description='where to write pid_step_test.csv (default: cwd)'),
        control,
        TimerAction(period=5.0, actions=[test]),   # let Gazebo come up first
    ])
