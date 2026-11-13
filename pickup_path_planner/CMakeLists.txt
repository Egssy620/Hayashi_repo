cmake_minimum_required(VERSION 3.8)
project(pickup_path_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(pickup_path_planner_core
  src/planner_core.cpp
)
target_include_directories(pickup_path_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(pickup_path_planner_node
  src/pickup_path_planner_node.cpp
)
target_link_libraries(pickup_path_planner_node pickup_path_planner_core)
ament_target_dependencies(pickup_path_planner_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_ros
)

install(TARGETS
  pickup_path_planner_core
  pickup_path_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner_core test/test_planner_core.cpp)
  if(TARGET test_planner_core)
    target_link_libraries(test_planner_core pickup_path_planner_core)
    target_include_directories(test_planner_core PRIVATE include)
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(pickup_path_planner_core)
ament_package()
