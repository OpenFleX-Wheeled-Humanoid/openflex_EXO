#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('openflex_teleop_exo'),
        'config',
        'openflex_teleop_exo.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='OpenArmX'),
        DeclareLaunchArgument('start_arm_retargeting', default_value='true'),
        DeclareLaunchArgument('start_arm_bridge', default_value='true'),
        DeclareLaunchArgument('start_whole_body_control', default_value='true'),
        DeclareLaunchArgument('enable_safety_check', default_value='true'),
        DeclareLaunchArgument('max_joint_diff_rad', default_value='0.873'),
        DeclareLaunchArgument('interpolation_duration', default_value='3.0'),
        DeclareLaunchArgument('interpolation_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('arm_config_file', default_value=PathJoinSubstitution([
            FindPackageShare('openflex_teleop_exo'),
            'config',
            'retargeting_OpenArmX.yaml',
        ])),

        Node(
            package='openflex_teleop_exo',
            executable='openflex_arm_retargeting_node',
            name='openflex_arm_retargeting_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_arm_retargeting')),
            parameters=[{
                'config_file': LaunchConfiguration('arm_config_file'),
                'enable_left_arm_retargeting': True,
                'enable_right_arm_retargeting': True,
            }],
        ),
        Node(
            package='openflex_teleop_exo',
            executable='openflex_arm_bridge_node',
            name='openflex_arm_bridge_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_arm_bridge')),
            parameters=[{
                'max_joint_diff_rad': LaunchConfiguration('max_joint_diff_rad'),
                'interpolation_duration': LaunchConfiguration('interpolation_duration'),
                'interpolation_rate_hz': LaunchConfiguration('interpolation_rate_hz'),
                'enable_safety_check': LaunchConfiguration('enable_safety_check'),
            }],
        ),
        Node(
            package='openflex_teleop_exo',
            executable='openflex_exo_teleop_node',
            name='openflex_exo_teleop_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_whole_body_control')),
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
