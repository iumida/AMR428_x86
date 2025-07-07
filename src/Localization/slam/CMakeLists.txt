cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rtabmap_slam REQUIRED)
find_package(rtabmap_sync REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检测
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装 config/ 与 launch/ 目录到 install space
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

