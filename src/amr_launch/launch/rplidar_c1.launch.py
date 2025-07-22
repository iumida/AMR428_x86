#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ===== Front LiDAR (ttyUSB0) =====
    serial_port1      = LaunchConfiguration('serial_port1',      default='/dev/ttyUSB0')
    serial_baudrate1  = LaunchConfiguration('serial_baudrate1',  default='460800')
    frame_id1         = LaunchConfiguration('frame_id1',         default='lidar_front')
    angle_compensate1 = LaunchConfiguration('angle_compensate1', default='false')
    inverted1         = LaunchConfiguration('inverted1',         default='false')
    scan_mode1      = LaunchConfiguration('scan_mode1',        default='Standard')  # 先不用

    # ===== Rear LiDAR (ttyUSB1) =====
    serial_port2      = LaunchConfiguration('serial_port2',      default='/dev/ttyUSB1')
    serial_baudrate2  = LaunchConfiguration('serial_baudrate2',  default='460800')
    frame_id2         = LaunchConfiguration('frame_id2',         default='lidar_rear')
    angle_compensate2 = LaunchConfiguration('angle_compensate2', default='false')
    inverted2         = LaunchConfiguration('inverted2',         default='false')
    scan_mode2      = LaunchConfiguration('scan_mode2',        default='Standard')

    def sllidar_node(name, port, baud, frame_id, angle_comp, inverted, scan_topic):
        params = {
            'channel_type': 'serial',
            'serial_port': port,
            'serial_baudrate': baud,
            'frame_id': frame_id,
            'angle_compensate': angle_comp,
            'inverted': inverted,
        }
        # 如果你想再加 scan_mode，就取消註解並放進 params
        # params['scan_mode'] = scan_mode

        return Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name=name,
            output='screen',
            parameters=[params],
            remappings=[('scan', scan_topic)]
        )

    return LaunchDescription([
        # ---- Declare Args (可用 ros2 launch 覆蓋) ----
        DeclareLaunchArgument('serial_port1',      default_value=serial_port1),
        DeclareLaunchArgument('serial_baudrate1',  default_value=serial_baudrate1),
        DeclareLaunchArgument('frame_id1',         default_value=frame_id1),
        DeclareLaunchArgument('angle_compensate1', default_value=angle_compensate1),
        DeclareLaunchArgument('inverted1',         default_value=inverted1),
        DeclareLaunchArgument('scan_mode1',        default_value=scan_mode1),

        DeclareLaunchArgument('serial_port2',      default_value=serial_port2),
        DeclareLaunchArgument('serial_baudrate2',  default_value=serial_baudrate2),
        DeclareLaunchArgument('frame_id2',         default_value=frame_id2),
        DeclareLaunchArgument('angle_compensate2', default_value=angle_compensate2),
        DeclareLaunchArgument('inverted2',         default_value=inverted2),
        DeclareLaunchArgument('scan_mode2',        default_value=scan_mode2),

        # ---- Nodes ----
        sllidar_node('sllidar_front', serial_port1, serial_baudrate1,
                     frame_id1, angle_compensate1, inverted1, 'scan_front'),

        sllidar_node('sllidar_rear',  serial_port2, serial_baudrate2,
                     frame_id2, angle_compensate2, inverted2, 'scan_rear'),
    ])

