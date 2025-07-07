#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time    = LaunchConfiguration('use_sim_time')
    deskewing       = LaunchConfiguration('deskewing')
    database_path   = LaunchConfiguration('database_path')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'deskewing', default_value='false',
            description='Enable lidar deskewing'),
        DeclareLaunchArgument(
            'database_path', default_value=os.path.join(os.environ['HOME'], 'map/4f.db'),
            description='RTAB-Map database path'),

        # ICP Odometry Node（已關閉 INFO 輸出，只保留 WARN 以上）
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],  # <-- 加在這裡
            parameters=[{
              'frame_id':                     'base_link',
              'odom_frame_id':                'odom',
              'wait_for_transform':           0.2,
              'wait_imu_to_init':             False,
              'expected_update_rate':         15.0,
              'deskewing':                    deskewing,
              'use_sim_time':                 use_sim_time,
              'Icp/PointToPlane':             'true',
              'Icp/Iterations':               '10',
              'Icp/VoxelSize':                '0.1',
              'Icp/Epsilon':                  '0.001',
              'Icp/PointToPlaneK':            '20',
              'Icp/PointToPlaneRadius':       '0',
              'Icp/MaxTranslation':           '2',
              'Icp/MaxCorrespondenceDistance':'1',
              'Icp/Strategy':                 '1',
              'Icp/OutlierRatio':             '0.7',
              'Icp/CorrespondenceRatio':      '0.01',
              'Odom/ScanKeyFrameThr':         '0.4',
              'OdomF2M/ScanSubtractRadius':   '0.1',
              'OdomF2M/ScanMaxSize':          '15000',
              'OdomF2M/BundleAdjustment':     'false',
              'Odom/Strategy':                '0'
            }],
            remappings=[
                ('scan_cloud', 'livox/lidar')
            ]
        ),

        # 其餘節點不變...
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
                'max_clouds':        10,
                'fixed_frame_id':    '',
                'use_sim_time':      use_sim_time,
            }],
            remappings=[
                ('cloud', 'odom_filtered_input_scan')
            ]
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            arguments=['--ros-args', '--log-level', 'warn'], 
            parameters=[{
                'frame_id':                  'base_link',
                'subscribe_depth':           False,
                'subscribe_rgb':             False,
                'subscribe_scan_cloud':      True,
                'subscribe_scan':            False,
                'approx_sync':               False,
                'wait_for_transform':        0.2,
                'use_sim_time':              use_sim_time,
                'database_path':             database_path,
                'Mem/IncrementalMemory':     'false',
                'Mem/InitWMWithAllNodes':    'true',
                'RGBD/ProximityMaxGraphDepth':'0',
                'RGBD/ProximityPathMaxNeighbors':'1',
                'RGBD/CreateOccupancyGrid':  'false',
                'Mem/NotLinkedNodesKept':    'false',
                'Mem/STMSize':               '30',
                'Mem/LaserScanNormalK':      '20',
                'Reg/Strategy':              '1',
                'Icp/VoxelSize':             '0.1',
                'Icp/PointToPlaneK':         '20',
                'Icp/PointToPlaneRadius':    '0',
                'Icp/PointToPlane':          'true',
                'Icp/Iterations':            '10',
                'Icp/Epsilon':               '0.001',
                'Icp/MaxTranslation':        '3',
                'Icp/MaxCorrespondenceDistance':'1',
                'Icp/Strategy':              '1',
                'Icp/OutlierRatio':          '0.7',
                'Grid/Sensor':               '0',
                'Kp/MaxFeatures':            '-1',
                'Icp/CorrespondenceRatio':   '0.2',
            }],
            remappings=[
                ('scan_cloud', 'assembled_cloud'),
                ('tag_detections', '/none')
            ]
        ),
    ])

