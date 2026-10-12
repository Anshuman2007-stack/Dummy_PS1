"""Stages 4 + 5 - kinematics and wheel PID (plus Stage 1 sensors and Gazebo by default).

    ros2 launch ugv_control control.launch.py                  # Gazebo + sensors + control
    ros2 launch ugv_control control.launch.py with_sim:=false  # control nodes only
Then drive it:  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory
    robot_yaml = os.path.join(share('ugv_core'), 'config', 'robot.yaml')
    ctrl_yaml = os.path.join(share('ugv_control'), 'config', 'control.yaml')
    sim_time = {'use_sim_time': True}

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('ugv_sensors'), 'launch', 'sensors.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
        condition=IfCondition(LaunchConfiguration('with_sim')))

    return LaunchDescription([
        DeclareLaunchArgument('with_sim', default_value='true',
                              description='also start Gazebo + Stage 1 sensor nodes'),
        DeclareLaunchArgument('headless', default_value='false'),
        sensors,
        Node(package='ugv_control', executable='kinematics_node', name='kinematics_node',
             output='screen', parameters=[robot_yaml, ctrl_yaml, sim_time]),
        Node(package='ugv_control', executable='wheel_pid_node', name='wheel_pid_node',
             output='screen', parameters=[robot_yaml, ctrl_yaml, sim_time]),
    ])
