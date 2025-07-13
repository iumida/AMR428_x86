#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_navigation')
    # Unified Nav2 configuration in params directory
    nav2_params = os.path.join(pkg_share, 'params', 'nav2.yaml')
    # Separate docking and scan filter configs in params directory
    docking_params = os.path.join(pkg_share, 'params', 'docking_server.yaml')
    scan_filter_chain = os.path.join(pkg_share, 'params', 'scan_filter_chain.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')

    return LaunchDescription([
        # RViz2
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        ),

        # Scan filters (use separate config)
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

        # Global costmap server (from unified nav2_params)
        LifecycleNode(
            package='nav2_costmap_2d', executable='nav2_costmap_2d',
            name='global_costmap', namespace='', output='screen',
            parameters=[nav2_params]
        ),
        # Local costmap server (from unified nav2_params)
        LifecycleNode(
            package='nav2_costmap_2d', executable='nav2_costmap_2d',
            name='local_costmap', namespace='', output='screen',
            parameters=[nav2_params]
        ),

        # Planner server
        LifecycleNode(
            package='nav2_planner', executable='planner_server',
            name='planner_server', namespace='', output='screen',
            parameters=[nav2_params]
        ),
        # Controller server
        LifecycleNode(
            package='nav2_controller', executable='controller_server',
            name='controller_server', namespace='', output='screen',
            parameters=[nav2_params]
        ),
        # Behavior server
        LifecycleNode(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', namespace='', output='screen',
            parameters=[nav2_params]
        ),
        # BT Navigator
        LifecycleNode(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', namespace='', output='screen',
            parameters=[nav2_params]
        ),
        # Waypoint follower
        LifecycleNode(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', namespace='', output='screen',
            parameters=[nav2_params]
        ),
        # Collision monitor
        LifecycleNode(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', namespace='', output='screen',
            parameters=[nav2_params]
        ),

        # Docking server (separate config)
        LifecycleNode(
            package='opennav_docking', executable='opennav_docking',
            name='docking_server', namespace='', output='screen',
            parameters=[docking_params]
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', namespace='', output='screen',
            parameters=[
                {'use_sim_time': False, 'autostart': True,
                 'node_names': [
                     'controller_server', 'planner_server', 'behavior_server',
                     'bt_navigator', 'waypoint_follower', 'collision_monitor',
                     'docking_server'
                 ]},
                nav2_params
            ]
        ),
    ])

