#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('websocket_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('websocket_port', default_value='19091'),
        DeclareLaunchArgument('joint_command_topic', default_value='/exo/joint_command'),
        DeclareLaunchArgument('gamepad_keys_topic', default_value='/exo/gamepad_keys'),

        Node(
            package='openflex_teleop_exo',
            executable='websocket_teleoperator',
            name='websocket_teleoperator',
            output='screen',
            parameters=[{
                'websocket_host': LaunchConfiguration('websocket_host'),
                'websocket_port': LaunchConfiguration('websocket_port'),
                'joint_command_topic': LaunchConfiguration('joint_command_topic'),
                'gamepad_keys_topic': LaunchConfiguration('gamepad_keys_topic'),
                'enable_left_arm': True,
                'enable_right_arm': True,
                'enable_left_joystick': True,
                'enable_right_joystick': True,
                'enable_vehicle_control': True,
            }],
            emulate_tty=True,
        ),
    ])
