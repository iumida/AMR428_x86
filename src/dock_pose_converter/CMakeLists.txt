cmake_minimum_required(VERSION 3.8)
project(dock_pose_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)

# build executable
add_executable(dock_pose_converter_node src/dock_pose_converter_node.cpp)

ament_target_dependencies(dock_pose_converter_node
  rclcpp
  geometry_msgs
  apriltag_msgs
)

# install executable
install(TARGETS dock_pose_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
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

