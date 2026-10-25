cmake_minimum_required(VERSION 3.10)
project(px4_exploration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(SDL2 QUIET)
if(NOT SDL2_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(SDL2 REQUIRED sdl2)
endif()

include_directories(include ${SDL2_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(px4_exploration_node
  src/exploration_main.cpp
  src/Arduino.cpp)

target_include_directories(px4_exploration_node PRIVATE
  ${pcl_conversions_INCLUDE_DIRS})

target_link_libraries(px4_exploration_node
  ${SDL2_LIBRARIES} ${PCL_LIBRARIES})

ament_target_dependencies(px4_exploration_node
  rclcpp geometry_msgs nav_msgs sensor_msgs mavros_msgs pcl_conversions)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS px4_exploration_node DESTINATION lib/${PROJECT_NAME})

ament_package()
