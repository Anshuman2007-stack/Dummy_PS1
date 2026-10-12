"""Stage 1 - sensor bring-up: Gazebo + robot + encoder odometry + GPS.

    ros2 launch ugv_sensors sensors.launch.py                 # starts Gazebo too
    ros2 launch ugv_sensors sensors.launch.py with_sim:=false # sensors nodes only

Topics to check: /joint_states /wheel_states /odom/wheels /imu/data /gnss/fix /true_odom
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
    sens_yaml = os.path.join(share('ugv_sensors'), 'config', 'sensors.yaml')
    sim_time = {'use_sim_time': True}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('my_bot_sim'), 'launch', 'sim.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
        condition=IfCondition(LaunchConfiguration('with_sim')))

    return LaunchDescription([
        DeclareLaunchArgument('with_sim', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        sim,
        Node(package='ugv_sensors', executable='wheel_odometry_node', name='wheel_odometry_node',
             output='screen', parameters=[robot_yaml, sens_yaml, sim_time]),
        Node(package='ugv_sensors', executable='gps_node', name='gps_node',
             output='screen', parameters=[sens_yaml, sim_time]),
    ])
