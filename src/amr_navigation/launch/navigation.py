#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    pkg_share = get_package_share_directory('amr_navigation')

    nav2_params        = os.path.join(pkg_share, 'params', 'nav2.yaml')
    docking_params     = os.path.join(pkg_share, 'params', 'docking_server.yaml')
    scan_filter_chain  = os.path.join(pkg_share, 'params', 'scan_filter_chain.yaml')
    keepout_params     = os.path.join(pkg_share, 'params', 'keepout_params.yaml')
    rviz_config        = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')

    nav2_common_params = [nav2_params]

    lifecycle_nodes = [
        'controller_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower', 'collision_monitor',
        'docking_server', 'global_costmap', 'local_costmap',
        'filter_mask_server', 'costmap_filter_info_server', 'velocity_smoother'
    ]

    return LaunchDescription([
        # RViz
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen', arguments=['-d', rviz_config]
        ),

        # Scan filter chains
        Node(
            package='laser_filters', executable='scan_to_scan_filter_chain',
            name='scan_front_filter', output='screen',
            remappings=[('scan', '/scan_front'), ('scan_filtered', '/scan_front_filtered')],
            parameters=[scan_filter_chain]
        ),
        Node(
            package='laser_filters', executable='scan_to_scan_filter_chain',
            name='scan_rear_filter', output='screen',
            remappings=[('scan', '/scan_rear'), ('scan_filtered', '/scan_rear_filtered')],
            parameters=[scan_filter_chain]
        ),

        # Keepout / costmap filter servers
        LifecycleNode(
            package='nav2_map_server', executable='map_server',
            name='filter_mask_server', namespace='', output='screen',
            emulate_tty=True, parameters=[keepout_params]
        ),
        LifecycleNode(
            package='nav2_map_server', executable='costmap_filter_info_server',
            name='costmap_filter_info_server', namespace='', output='screen',
            emulate_tty=True, parameters=[keepout_params]
        ),

        # Global & Local Costmaps
        LifecycleNode(
            package='nav2_costmap_2d', executable='nav2_costmap_2d',
            name='global_costmap', namespace='', output='screen',
            parameters=nav2_common_params
        ),
        LifecycleNode(
            package='nav2_costmap_2d', executable='nav2_costmap_2d',
            name='local_costmap', namespace='', output='screen',
            parameters=nav2_common_params
        ),

        # Planner Server
        LifecycleNode(
            package='nav2_planner', executable='planner_server',
            name='planner_server', namespace='', output='screen',
            parameters=nav2_common_params
        ),

        # Controller Server
        LifecycleNode(
            package='nav2_controller', executable='controller_server',
            name='controller_server', namespace='', output='screen',
            parameters=nav2_common_params
        ),

        # Velocity Smoother (LifecycleNode)
        LifecycleNode(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', namespace='', output='screen',
            emulate_tty=True,
            parameters=nav2_common_params
        ),

        # Behavior Server
        LifecycleNode(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', namespace='', output='screen',
            parameters=nav2_common_params
        ),

        # BT Navigator
        LifecycleNode(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', namespace='', output='screen',
            parameters=nav2_common_params
        ),

        # Waypoint Follower
        LifecycleNode(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', namespace='', output='screen',
            parameters=nav2_common_params
        ),

        # Collision Monitor
        LifecycleNode(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', namespace='', output='screen',
            parameters=nav2_common_params
        ),

        # Docking Server
        LifecycleNode(
            package='opennav_docking', executable='opennav_docking',
            name='docking_server', namespace='', output='screen',
            parameters=[docking_params]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace='',
            output='screen',
            parameters=nav2_common_params
        ),
    ])
