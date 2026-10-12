"""Stages 2 + 3: EKF + navigation controller + interactive goal input.

goal_pose_node reads goals from the keyboard (input()), so it needs its own
terminal window. `goal_input` selects how it is started:

  auto           xterm if installed, else gnome-terminal, else print a hint (default)
  xterm          force xterm
  gnome-terminal force gnome-terminal
  none           do not start it; run `ros2 run ugv_nav goal_pose_node` yourself
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MANUAL_HINT = ("goal_pose_node NOT started in a terminal. In a new terminal run:\n"
               "    source install/setup.bash && ros2 run ugv_nav goal_pose_node")


def _goal_input(context):
    choice = LaunchConfiguration('goal_input').perform(context).strip().lower()

    if choice == 'auto':
        choice = next((t for t in ('xterm', 'gnome-terminal') if shutil.which(t)), 'none')
    if choice != 'none' and not shutil.which(choice):
        return [LogInfo(msg=f"'{choice}' not found. " + MANUAL_HINT)]
    if choice == 'none':
        return [LogInfo(msg=MANUAL_HINT)]

    prefix = {'xterm': 'xterm -e', 'gnome-terminal': 'gnome-terminal --'}.get(choice)
    if prefix is None:
        return [LogInfo(msg=f"Unknown goal_input '{choice}'. " + MANUAL_HINT)]

    return [Node(package='ugv_nav', executable='goal_pose_node', name='goal_input',
                 output='screen', prefix=prefix, parameters=[{'use_sim_time': True}])]


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ugv_nav'), 'config', 'ugv_nav.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_input', default_value='auto',
            description='How to start goal_pose_node: auto | xterm | gnome-terminal | none'),
        Node(package='ugv_nav', executable='ekf_node', name='ekf_node',
             output='screen', parameters=[params, {'use_sim_time': True}]),
        Node(package='ugv_nav', executable='navigation_node',
             name='navigation_node', output='screen', parameters=[params, {'use_sim_time': True}]),
        OpaqueFunction(function=_goal_input),
    ])
