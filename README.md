導航
ros2 launch amr_navigation navigation.py

開啟所有感應器2d and 3d lidar and odom，與控制底盤程式 slam
ros2 launch amr_launch amrlaunch.launch.py mode:=localization


建圖
ros2 launch amr_launch amrlaunch.launch.py mode:=mapping

編譯
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

插棧板
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{use_dock_id: true, dock_id: 'pallet', dock_type: 'pallet_dock', max_staging_time: 1000.0, navigate_to_staging_pose: true}"

