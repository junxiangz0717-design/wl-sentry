find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_subdirectory(serial_port)

set(CMAKE_BUILD_TYPE Debug)

add_library(serial_data src/serial_data.cpp)

target_include_directories(serial_data PUBLIC include)

target_link_libraries(serial_data  serial_port)

ament_target_dependencies(serial_data rclcpp)