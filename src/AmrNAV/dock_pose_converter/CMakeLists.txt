cmake_minimum_required(VERSION 3.8)
project(dock_pose_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 建置執行檔
add_executable(dock_pose_converter_node src/dock_pose_converter_node.cpp)

ament_target_dependencies(dock_pose_converter_node
  rclcpp
  geometry_msgs
  isaac_ros_apriltag_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 安裝執行檔
install(TARGETS dock_pose_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝 launch 檔
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

