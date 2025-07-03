from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    nav2_params = '/home/ace428/amr/src/amr_navigation/config/bao.yaml'
    rviz_config = '/home/ace428/amr/src/amr_navigation/rviz/nav2_config.rviz'

    use_sim_time    = False
    autostart       = True
    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'collision_monitor'
    ]

    return LaunchDescription([

        # RViz2
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        ),

        # planner_server
        LifecycleNode(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                nav2_params,
            ]
        ),

        # controller_server
        LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                nav2_params,
            ]
        ),

        # behavior_server
        LifecycleNode(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                nav2_params,
            ]
        ),

        # bt_navigator
        LifecycleNode(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                nav2_params,
            ]
        ),

        # waypoint_follower
        LifecycleNode(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                nav2_params,
            ]
        ),

        # collision_monitor
        LifecycleNode(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                nav2_params,
            ]
        ),

        # lifecycle_manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace='',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 10000.0
            }]
        )
    ])

