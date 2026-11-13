cmake_minimum_required(VERSION 3.8)
project(points_projection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(points_projection_node src/points_projection_node.cpp)
target_include_directories(points_projection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(points_projection_node PUBLIC cxx_std_17)
ament_target_dependencies(points_projection_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS points_projection_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
