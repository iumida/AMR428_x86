from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'tag_id',
            default_value='0',
            description='AprilTag ID'
        ),

        DeclareLaunchArgument(
            'use_first_detection',
            default_value='true',
            description='Use first detection or filter by tag ID'
        ),

        Node(
            package='dock_pose_converter',
            executable='dock_pose_converter_node',
            name='dock_pose_converter_node',
            output='screen',
            parameters=[{
                'dock_tag_id': LaunchConfiguration('tag_id'),
                'use_first_detection': LaunchConfiguration('use_first_detection'),
            }]
        )
    ])

