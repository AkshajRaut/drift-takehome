"""Launch Gazebo with the two-room home world and spawn TidyBot."""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_rsp_and_spawn(context, *args, **kwargs):
    """Resolve xacro at launch time and return RSP + spawn nodes."""
    desc_pkg   = get_package_share_directory('tidybot_description')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'tidybot.urdf.xacro')

    # Run xacro to get the URDF string
    result = subprocess.run(
        ['xacro', xacro_file],
        capture_output=True, text=True, check=True
    )
    robot_description = result.stdout

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time == 'true',
        }],
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tidybot',
            '-topic',  'robot_description',
            '-x', '1.0', '-y', '0.0', '-z', '0.05',
        ],
        output='screen',
    )

    return [rsp, TimerAction(period=3.0, actions=[spawn])]


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('tidybot_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(gazebo_pkg, 'worlds', 'home.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock'),
        gzserver,
        gzclient,
        # Delay RSP + spawn by 4 s so gzserver is ready
        TimerAction(period=4.0, actions=[OpaqueFunction(function=_make_rsp_and_spawn)]),
    ])
