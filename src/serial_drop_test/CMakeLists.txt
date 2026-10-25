cmake_minimum_required(VERSION 3.10)
project(serial_drop_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(serial_drop_test_node src/serial_drop_test_node.cpp)
ament_target_dependencies(serial_drop_test_node rclcpp)

install(TARGETS serial_drop_test_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
