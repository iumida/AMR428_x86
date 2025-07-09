#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

def generate_launch_description():

    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        
        # 參數
        DeclareLaunchArgument(
            'mode',
            default_value='localization',
            description='Mode: "mapping" or "localization"'
        ),

        # 機器人描述
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('amr_description'), 'launch', 'amr.launch.py')
            ])
        ),

        # RPLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')
            ])
        ),

        # Livox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'rviz_MID360_launch.py')
            ])
        ),

        # Pointcloud to Laserscan
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'launch', 'sample_pointcloud_to_laserscan_launch.py')
            ])
        ),

        # ros2_control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('amr_control'), 'launch', 'amr_control.launch.py')
            ])
        ),

        # robot_localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('robot_localization'), 'launch', 'ekf.launch.py')
            ])
        ),

        # USB 相機
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('usb_cam'), 'launch', 'camera.launch.py')
            ])
        ),

        # Dock Pose Converter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('dock_pose_converter'), 'launch', 'dock_pose_converter.launch.py')
            ]),
            launch_arguments={
                'tag_family': 'tag36h11',
                'use_first_detection': 'true'
            }.items()
        ),


        # Mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam'), 'launch', 'mapping_3d.launch.py')
            ]),
            condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
        ),

        # Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam'), 'launch', 'localization_3d.launch.py')
            ]),
            launch_arguments={'database_path': '/home/ace428/map/4f.db'}.items(),
            condition=UnlessCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
        ),

        # Foxglove Bridge
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
            )
        )
    ])

