from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ctrl_yaml = PathJoinSubstitution([
        FindPackageShare('amr_control'),
        'config', 'bicycle_drive_controller.yaml',  # 更新成新參數檔案
    ])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ctrl_yaml],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # 將 Ackermann 轉向控制器替換為 Bicycle 控制器
    bicycle_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'bicycle_steering_controller',  # 更新為新控制器
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    relay_node = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/cmd_vel_final', '/bicycle_steering_controller/reference_unstamped'],  # 根據新控制器調整topic名稱
        output='screen',
    )

    return LaunchDescription([
        controller_manager,
        joint_state_spawner,
        bicycle_spawner,  # 新的自行車控制器啟動
        relay_node,
    ])

