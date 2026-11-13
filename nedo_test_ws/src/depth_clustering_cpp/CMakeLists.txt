cmake_minimum_required(VERSION 3.8)
project(depth_clustering_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nlohmann_json REQUIRED)

# Main: Iterative Seed Convergence Wavefront Ground Removal (range image + metadata)
add_executable(depth_clustering_node src/depth_clustering_node.cpp)
ament_target_dependencies(depth_clustering_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2
)
target_link_libraries(depth_clustering_node nlohmann_json::nlohmann_json)

# Alternative: simple Bogoslavskyi-only (PointCloud2 input, no metadata)
add_executable(depth_clustering_node_simple src/backup.cpp)
ament_target_dependencies(depth_clustering_node_simple
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  depth_clustering_node
  depth_clustering_node_simple
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
