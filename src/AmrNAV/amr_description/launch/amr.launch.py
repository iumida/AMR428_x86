import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    package_share_directory = get_package_share_directory("amr_description")

    # Define the path to the URDF
    urdf_file = os.path.join(package_share_directory, "urdf", "amr.urdf")

    # Define robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf_file).read()}],
    )

    # Define marker_publisher node from amr_visualization package
    marker_publisher_node = Node(
        package="amr_visualization",
        executable="marker_publisher",
        name="marker_publisher",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        robot_state_publisher_node,
        marker_publisher_node,  # 👈 加進來了
    ])

