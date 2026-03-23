"""
Single-command launch: Gazebo world + TidyBot spawn + tidying task node.

Usage:
  ros2 launch tidybot_gazebo tidybot_full.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('tidybot_gazebo')
    nav_pkg    = get_package_share_directory('tidybot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless     = LaunchConfiguration('headless',     default='false')

    # ── World + robot (re-uses tidybot_world.launch.py) ───────
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'tidybot_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── Navigation / task node (delayed 8 s for Gazebo to settle) ──
    navigator_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='tidybot_navigation',
                executable='navigator',
                name='tidybot_navigator',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    # ── Arm controller node (delayed 8 s) ─────────────────────
    arm_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='tidybot_navigation',
                executable='arm_controller',
                name='tidybot_arm_controller',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    # ── Gripper node (delayed 8 s) ──────────────────────────
    gripper_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='tidybot_navigation',
                executable='gripper',
                name='tidybot_gripper',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless',     default_value='false'),
        world_launch,
        navigator_node,
        arm_node,
        gripper_node,
    ])
