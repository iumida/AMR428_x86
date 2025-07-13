#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---------------------------------------------------
    # 1. 相機清單與 apriltag camera
    # ---------------------------------------------------
    camera_configs = [
        {'name': 'camera_fork', 'suffix': 'fork'},
    ]
    apriltag_cameras = ['camera_fork']

    # ---------------------------------------------------
    # 2. 找到參數資料夾
    # ---------------------------------------------------
    pkg_share = get_package_share_directory('amr_launch')
    params_folder = os.path.join(pkg_share, 'params')
    tag_param_file = os.path.join(params_folder, 'tags_36h11.yaml')

    composables = []

    # ---------------------------------------------------
    # 3. 加入 usb_cam、image_proc 與 apriltag_ros 到 container
    # ---------------------------------------------------
    for cam in camera_configs:
        name = cam['name']
        suffix = cam['suffix']
        cam_param_file = os.path.join(params_folder, f"params_{suffix}.yaml")
        if not os.path.exists(cam_param_file):
            raise FileNotFoundError(f"Missing camera config: {cam_param_file}")

        # USB 攝影機
        composables.append(
            ComposableNode(
                package='usb_cam',
                plugin='usb_cam::UsbCamNode',
                name=f'usb_cam_{suffix}',
                namespace=name,
                remappings=[
                    ('image_raw', f'/{name}/image_raw'),
                    ('camera_info', f'/{name}/camera_info'),
                ],
                parameters=[cam_param_file],
            )
        )

        # Image Rectify
        composables.append(
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name=f'rectify_{suffix}',
                namespace=name,
                remappings=[
                    ('image', 'image_raw'),
                    ('camera_info', 'camera_info'),
                    ('image_rect', 'image_rect'),  # 必要時顯式加上
                ],
            )
        )

        # AprilTag（如果在清單內）
        if name in apriltag_cameras:
            composables.append(
                ComposableNode(
                    package='apriltag_ros',
                    plugin='AprilTagNode',
                    name=f'apriltag_{suffix}',
                    namespace=name,
                    remappings=[
                        ('image', f'/{name}/image_rect'),         # 對應 image_proc 輸出
                        ('camera_info', f'/{name}/camera_info'),  # camera info 不變
                    ],
                    parameters=[tag_param_file],
                )
            )

    # ---------------------------------------------------
    # 4. Composable container
    # ---------------------------------------------------
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=composables,
    )

    return LaunchDescription([container])

