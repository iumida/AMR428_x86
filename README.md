導航
ros2 launch amr_navigation navigation.py

 開啟所有感應器2d and 3d lidar and odom，與控制底盤程式 slam
ros2 launch sensorlaunch sensorlaunch.launch.py

建圖
ros2 launch sensorlaunch sensorlaunch.launch.py mode:=mapping

編譯
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble


