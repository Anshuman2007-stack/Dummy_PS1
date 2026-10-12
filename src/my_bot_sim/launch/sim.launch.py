"""Gazebo world + four-wheel robot + ROS<->Gazebo bridge + TF + motor driver.

Gazebo -> ROS
  /clock          rosgraph_msgs/Clock      (all nodes run on sim time)
  /joint_states   sensor_msgs/JointState   wheel angles/velocities (raw encoder source)
  /imu/data       sensor_msgs/Imu          100 Hz
  /true_odom      nav_msgs/Odometry        ground truth (evaluation + GPS source only)
ROS -> Gazebo
  /model/my_bot/joint/<wheel>_wheel_joint/cmd_force   std_msgs/Float64 torque [N m]
ROS nodes
  robot_state_publisher  TF base_link -> chassis/imu/gnss/wheels (urdf/my_bot.urdf)
  motor_driver_sim       /wheel_pwm duty -> wheel torques
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

WHEELS = ('front_left', 'rear_left', 'front_right', 'rear_right')


def generate_launch_description():
    pkg = get_package_share_directory('my_bot_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_yaml = os.path.join(get_package_share_directory('ugv_core'), 'config', 'robot.yaml')

    world = os.path.join(pkg, 'worlds', 'my_world.sdf')
    model = os.path.join(pkg, 'models', 'my_bot', 'model.sdf')
    with open(os.path.join(pkg, 'urdf', 'my_bot.urdf')) as f:
        urdf = f.read()

    headless = LaunchConfiguration('headless')
    sim_time = {'use_sim_time': True}

    # '-r' = start running, '-s' = server only (no GUI)
    gz_flags = PythonExpression(
        ["'-r -s ' if '", headless, "'.lower() == 'true' else '-r '"])
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [gz_flags, world]}.items())

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-file', model, '-name', 'my_bot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.02', '-Y', '0.0'])
    # Spawned at the world origin with yaw 0: the EKF odom frame, the GPS datum
    # and the Gazebo world frame then coincide, so /ekf/odom and /true_odom are
    # directly comparable.

    # '[' = Gazebo -> ROS, ']' = ROS -> Gazebo   (Fortress: ignition.msgs.*)
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/world/my_world/model/my_bot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/true_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
    ] + [f'/model/my_bot/joint/{w}_wheel_joint/cmd_force@std_msgs/msg/Float64]ignition.msgs.Double'
         for w in WHEELS]
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        output='screen', arguments=bridge_args, parameters=[sim_time],
        remappings=[
            ('/world/my_world/model/my_bot/joint_state', '/joint_states'),
            ('/imu', '/imu/data'),
        ])

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}])

    motor = Node(
        package='my_bot_sim', executable='motor_driver_sim.py', name='motor_driver_sim',
        output='screen', parameters=[robot_yaml, sim_time])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = Gazebo server only (no GUI)'),
        gz, spawn, bridge, rsp, motor,
    ])
