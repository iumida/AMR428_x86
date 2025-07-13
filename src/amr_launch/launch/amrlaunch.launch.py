#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------------------------------------------
    # 參數設定
    # ---------------------------------------------------
    mode = LaunchConfiguration('mode')
    launch_args = [
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description='Mode: "mapping" or "localization"'
        )
    ]

    # ---------------------------------------------------
    # Robot State Publisher (URDF)
    # ---------------------------------------------------
    urdf_file = os.path.join(
        get_package_share_directory('amr_launch'),
        'urdf', 'amr.urdf'
    )
    robot_description_cmd = Command(['xacro ', urdf_file])
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_cmd}]
    )

    launch_entities = []
    launch_entities.extend(launch_args)
    launch_entities.append(rsp_node)

    # ---------------------------------------------------
    # Include other launch files
    # ---------------------------------------------------
    pkg = get_package_share_directory('amr_launch')
    base_launch = os.path.join(pkg, 'launch')

    # RPLIDAR
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(base_launch, 'rplidar_c1.launch.py')
            ])
        )
    )
    # Livox
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('livox_ros_driver2'),
                    'launch_ROS2', 'rviz_MID360_launch.py'
                )
            ])
        )
    )
    # Pointcloud to Laserscan
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('pointcloud_to_laserscan'),
                    'launch', 'sample_pointcloud_to_laserscan_launch.py'
                )
            ])
        )
    )
    # ros2_control (now from amr_launch)
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(base_launch, 'amr_control.launch.py')
            ])
        )
    )

    # ---------------------------------------
    # 【改动】直接以 Node 启动 ekf_filter_node
    # ---------------------------------------
    ekf_params = os.path.join(
        get_package_share_directory('amr_launch'),
        'params', 'ekf.yaml'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params]
    )
    launch_entities.append(ekf_node)

    # USB 相機
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(base_launch, 'camera.launch.py')
            ])
        )
    )
    # Dock Pose Converter
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('dock_pose_converter'),
                    'launch', 'dock_pose_converter.launch.py'
                )
            ]),
            launch_arguments={
                'tag_family': 'tag36h11',
                'use_first_detection': 'true'
            }.items()
        )
    )
    # Mapping
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(base_launch, 'mapping_3d.launch.py')
            ]),
            condition=IfCondition(
                PythonExpression(["'", mode, "' == 'mapping'"])
            )
        )
    )
    # Localization
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(base_launch, 'localization_3d.launch.py')
            ]),
            launch_arguments={'database_path': '/home/ace428/map/4f.db'}.items(),
            condition=UnlessCondition(
                PythonExpression(["'", mode, "' == 'mapping'"])
            )
        )
    )
    # Foxglove Bridge
    launch_entities.append(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('foxglove_bridge'),
                    'launch', 'foxglove_bridge_launch.xml'
                )
            ]),
            launch_arguments={'send_buffer_limit': '100000000'}.items()
        )
    )

    return LaunchDescription(launch_entities)

