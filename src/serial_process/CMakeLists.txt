cmake_minimum_required(VERSION 3.8)
project(serial_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(msg_process REQUIRED) # 自定义消息包


add_subdirectory(serial_data)


# 接收进程节点
add_executable(receive_process
  src/receive_process.cpp
)

# 发送进程节点
add_executable(send_process
  src/send_process.cpp
)

# 车辆控制中心节点
add_executable(car_control
  src/car_control.cpp
)


target_include_directories(receive_process PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial_data/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial_data/serial_port/include>
)

target_include_directories(send_process PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial_data/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial_data/serial_port/include>
)

target_include_directories(car_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)


set_target_properties(receive_process send_process car_control PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

ament_target_dependencies(receive_process
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "msg_process"
)
target_link_libraries(receive_process serial_data)

ament_target_dependencies(send_process
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "msg_process"
)
target_link_libraries(send_process serial_data)

ament_target_dependencies(car_control
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "msg_process"
)

install(TARGETS 
  receive_process 
  send_process 
  car_control
  DESTINATION lib/${PROJECT_NAME}
)

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