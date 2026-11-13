cmake_minimum_required(VERSION 3.8)
project(rover_odom_to_pose2d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(rover_odom_to_pose2d_node src/rover_odom_to_pose2d_node.cpp)
target_include_directories(rover_odom_to_pose2d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rover_odom_to_pose2d_node PUBLIC cxx_std_17)
ament_target_dependencies(rover_odom_to_pose2d_node
  rclcpp geometry_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS rover_odom_to_pose2d_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
