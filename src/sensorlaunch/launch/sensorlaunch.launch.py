#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    mode = LaunchConfiguration('mode')

    # 1. 機器人描述
    amr_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('amr_description'),
                'launch',
                'amr.launch.py'
            )
        ])
    )

    # 2. RPLIDAR
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        ])
    )

    # 3. Livox
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'launch_ROS2',
                'rviz_MID360_launch.py'
            )
        ])
    )

    # 4. pointcloud_to_laserscan
    pcl_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('pointcloud_to_laserscan'),
                'launch',
                'sample_pointcloud_to_laserscan_launch.py'
            )
        ])
    )

    # 5. ros2_control
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('amr_control'),
                'launch',
                'amr_control.launch.py'
            )
        ])
    )

    # 6. robot_localization
    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('robot_localization'),
                'launch',
                'ekf.launch.py'
            )
        ])
    )

    # 7. SLAM 或 Localization
    slam_launch_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam'),
                'launch',
                'mapping_3d.launch.py'
            )
        ])
    )

    slam_launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam'),
                'launch',
                'localization_3d.launch.py'
            )
        ]),
        launch_arguments={'database_path': '/home/ace428/map/4f.db'}.items()
    )

    # 組合所有動作
    actions = [
        DeclareLaunchArgument(
            'mode',
            default_value='localization',
            description='Mode: "mapping" or "localization"'
        ),
        amr_description_launch,
        rplidar_launch,
        livox_driver_launch,
        pcl_to_laserscan_launch,
        ros2_control_launch,
        robot_localization_launch,
    ]

    actions.append(
        slam_launch_mapping if mode == 'mapping' else slam_launch_localization
    )

    return LaunchDescription(actions)

