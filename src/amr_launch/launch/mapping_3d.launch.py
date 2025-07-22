from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing    = LaunchConfiguration('deskewing')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'deskewing', default_value='false',
            description='Enable lidar deskewing'),
          
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
              'frame_id':'base_link',
              'odom_frame_id':'odom',
              'wait_for_transform':0.2,
              'expected_update_rate':15.0,
              'deskewing':deskewing,
              'use_sim_time':use_sim_time,
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '10',
              'Icp/VoxelSize': '0.1',
              'Icp/Epsilon': '0.001',
              'Icp/PointToPlaneK': '20',
              'Icp/PointToPlaneRadius': '0',
              'Icp/MaxTranslation': '2',
              'Icp/MaxCorrespondenceDistance': '1',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.7',
              'Icp/CorrespondenceRatio': '0.01',
              'Odom/ScanKeyFrameThr': '0.4',
              'OdomF2M/ScanSubtractRadius': '0.1',
              'OdomF2M/ScanMaxSize': '15000',
              'OdomF2M/BundleAdjustment': 'false'
            }],
            remappings=[
              ('scan_cloud', 'livox/lidar')
            ]),

        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
              'max_clouds':10,
              'fixed_frame_id':'',
              'use_sim_time':use_sim_time,
            }],
            remappings=[
              ('cloud', 'odom_filtered_input_scan')
            ]),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
              'frame_id':'base_link',
              'subscribe_depth':False,
              'subscribe_rgb':True,
              'subscribe_scan_cloud':True,
              'approx_sync':True,
              'map_always_update':False,
              'wait_for_transform':0.2,
              'use_sim_time':use_sim_time,
              'database_path':'/home/ace428/map/rtabmap.db',
              'RGBD/ProximityMaxGraphDepth': '0',
              'RGBD/ProximityPathMaxNeighbors': '1',
              'RGBD/AngularUpdate': '0.05',
              'RGBD/LinearUpdate': '0.05',
              'RGBD/CreateOccupancyGrid': 'true',
              'Mem/NotLinkedNodesKept': 'false',
              'Mem/STMSize': '30',
              'Mem/LaserScanNormalK': '20',
              'Reg/Strategy': '2',
              'Icp/VoxelSize': '0.1',
              'Icp/PointToPlaneK': '20',
              'Icp/PointToPlaneRadius': '0',
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '10',
              'Icp/Epsilon': '0.001',
              'Icp/MaxTranslation': '3',
              'Icp/MaxCorrespondenceDistance': '1',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.7',
              'Icp/CorrespondenceRatio': '0.2',
              'Grid/MaxObstacleHeight': '1.6',
              'Grid/RayTracing': 'true',
              'Grid/RangeMax': '5.0'
            }],
            remappings=[
              ('scan_cloud', 'assembled_cloud'),
              ('rgb/image',  '/camera_fork/image_rect'),
              ('rgb/camera_info',  '/camera_fork/camera_info_rect')
            ],
            arguments=['-d']
        ),

        # 4) RTAB-Map Viz (GUI) 節點
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'subscribe_rgb':        True,
                'subscribe_scan_cloud': True,
                'approx_sync':          True,
                'use_sim_time':         use_sim_time,
            }],
            remappings=[
                ('rgb/image',        '/camera_fork/image_rect'),
                ('rgb/camera_info',  '/camera_fork/camera_info'),
                ('scan_cloud',       'assembled_cloud'),
                ('odom',             '/odom'),
            ]
        ),

    ])


