cmake_minimum_required(VERSION 3.8)
project(amr_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 建立 library
add_library(${PROJECT_NAME} SHARED
  src/amr_system.cpp
)

# 指定依賴
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 關鍵：正確指定 include 目錄
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 導出 plugin 描述檔
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# 安裝 include 檔
install(
  DIRECTORY include/
  DESTINATION include
)

# 安裝 library
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# 測試相關設定（如有）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

