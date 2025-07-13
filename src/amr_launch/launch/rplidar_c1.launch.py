#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 第一支 LiDAR 的參數
    channel_type1    = LaunchConfiguration('channel_type1',    default='serial')
    serial_port1     = LaunchConfiguration('serial_port1',     default='/dev/ttyUSB0')
    serial_baudrate1 = LaunchConfiguration('serial_baudrate1', default='460800')
    frame_id1        = LaunchConfiguration('frame_id1',        default='lidar_front')
    inverted1        = LaunchConfiguration('inverted1',        default='false')
    angle_compensate1= LaunchConfiguration('angle_compensate1',default='true')
    scan_mode1       = LaunchConfiguration('scan_mode1',       default='Standard')

    # 第二支 LiDAR 的參數
    channel_type2    = LaunchConfiguration('channel_type2',    default='serial')
    serial_port2     = LaunchConfiguration('serial_port2',     default='/dev/ttyUSB1')
    serial_baudrate2 = LaunchConfiguration('serial_baudrate2', default='460800')
    frame_id2        = LaunchConfiguration('frame_id2',        default='lidar_rear')
    inverted2        = LaunchConfiguration('inverted2',        default='false')
    angle_compensate2= LaunchConfiguration('angle_compensate2',default='true')
    scan_mode2       = LaunchConfiguration('scan_mode2',       default='Standard')

    return LaunchDescription([
        # 前 LiDAR 的 launch args
        DeclareLaunchArgument('channel_type1',    default_value=channel_type1),
        DeclareLaunchArgument('serial_port1',     default_value=serial_port1),
        DeclareLaunchArgument('serial_baudrate1', default_value=serial_baudrate1),
        DeclareLaunchArgument('frame_id1',        default_value=frame_id1),
        DeclareLaunchArgument('inverted1',        default_value=inverted1),
        DeclareLaunchArgument('angle_compensate1',default_value=angle_compensate1),
        DeclareLaunchArgument('scan_mode1',       default_value=scan_mode1),

        # 後 LiDAR 的 launch args
        DeclareLaunchArgument('channel_type2',    default_value=channel_type2),
        DeclareLaunchArgument('serial_port2',     default_value=serial_port2),
        DeclareLaunchArgument('serial_baudrate2', default_value=serial_baudrate2),
        DeclareLaunchArgument('frame_id2',        default_value=frame_id2),
        DeclareLaunchArgument('inverted2',        default_value=inverted2),
        DeclareLaunchArgument('angle_compensate2',default_value=angle_compensate2),
        DeclareLaunchArgument('scan_mode2',       default_value=scan_mode2),

        # 啟動前 LiDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_front',
            parameters=[{
                'channel_type':    channel_type1,
                'serial_port':     serial_port1,
                'serial_baudrate': serial_baudrate1,
                'frame_id':        frame_id1,
                'inverted':        inverted1,
                'angle_compensate':angle_compensate1,
                'scan_mode':       scan_mode1,
            }],
            remappings=[('scan', 'scan_front')],
            output='screen'
        ),

        # 啟動後 LiDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_rear',
            parameters=[{
                'channel_type':    channel_type2,
                'serial_port':     serial_port2,
                'serial_baudrate': serial_baudrate2,
                'frame_id':        frame_id2,
                'inverted':        inverted2,
                'angle_compensate':angle_compensate2,
                'scan_mode':       scan_mode2,
            }],
            remappings=[('scan', 'scan_rear')],
            output='screen'
        ),
    ])

