#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 使用 FindPackageShare 定位到 amr_launch package 的 share 目錄
    ctrl_yaml = PathJoinSubstitution([
        FindPackageShare('amr_launch'),
        'params',
        'bicycle_drive_controller.yaml',
    ])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ctrl_yaml],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    bicycle_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'bicycle_steering_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    relay_node = Node(
        package='topic_tools',
        executable='relay',
        arguments=[
            '/cmd_vel_final',
            '/bicycle_steering_controller/reference_unstamped'
        ],
        output='screen',
    )

    return LaunchDescription([
        controller_manager,
        joint_state_spawner,
        bicycle_spawner,
        relay_node,
    ])

