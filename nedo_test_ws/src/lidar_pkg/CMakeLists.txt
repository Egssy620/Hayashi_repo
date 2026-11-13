cmake_minimum_required(VERSION 3.8)
project(lidar_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(scan_shadow_filter_node
  src/scan_shadow_filter_node.cpp
)
ament_target_dependencies(scan_shadow_filter_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  scan_shadow_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
