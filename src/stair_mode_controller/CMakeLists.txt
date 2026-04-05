cmake_minimum_required(VERSION 3.8)
project(stair_mode_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(msg_process REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(stair_mode_controller_node
  src/stair_mode_controller.cpp
)

ament_target_dependencies(stair_mode_controller_node
  geometry_msgs
  msg_process
  rclcpp
  tf2
  tf2_ros
)

target_compile_features(stair_mode_controller_node PUBLIC cxx_std_17)

install(TARGETS
  stair_mode_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
