cmake_minimum_required(VERSION 3.8)
project(manage_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# --- ROS2 基本依存 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(vehicle_interface REQUIRED)

# --- OpenCV 追加 ---
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}           # ← /usr/include/opencv4 を自動追加
)

# --- ノード追加 ---
add_executable(manage_node src/manage_node.cpp)
ament_target_dependencies(manage_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  vehicle_interface
)

# --- OpenCV ライブラリリンク ---
target_link_libraries(manage_node
  ${OpenCV_LIBRARIES}
)

# --- マスター接続版 (/goal_pose で目的地を受け取り、/navigation/result で結果を返す) ---
add_executable(manage_node_master src/manage_node_master.cpp)
ament_target_dependencies(manage_node_master
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  vehicle_interface
)
target_link_libraries(manage_node_master
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  manage_node
  manage_node_master
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
