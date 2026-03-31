find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(serial_port src/serial_port.cpp)

target_include_directories(serial_port PUBLIC include)

ament_target_dependencies(serial_port rclcpp)