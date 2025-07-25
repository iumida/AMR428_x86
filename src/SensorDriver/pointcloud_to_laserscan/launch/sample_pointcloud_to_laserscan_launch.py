from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner',
            default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            parameters=[{
                'target_frame':        'livox_frame',
                'transform_tolerance': 0.05,
                'min_height':          -1.0,
                'max_height':          0.5,
                'angle_min':          -3.141592653589793,
                'angle_max':           3.141592653589793,
                'angle_increment':     0.003141592,
                'scan_time':           0.1,
                'range_min':           0.1,
                'range_max':          70.0,
                'use_inf':            True,
                'inf_epsilon':         1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])

