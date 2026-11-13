cmake_minimum_required(VERSION 3.8)
project(drive_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(drive_node src/drive_node_wall.cpp)
ament_target_dependencies(drive_node rclcpp std_msgs geometry_msgs nav_msgs)

target_link_libraries(drive_node ${OpenCV_LIBS})

install(TARGETS
  drive_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

