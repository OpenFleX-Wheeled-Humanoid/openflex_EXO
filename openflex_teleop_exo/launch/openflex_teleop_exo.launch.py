#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Safe default: do not start the WebSocket server here. Start it separately
    # with websocket_publisher.launch.py, then start this control chain.
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('openflex_teleop_exo'),
                'launch',
                'exo_retargeting.launch.py',
            ])),
        ),
    ])
