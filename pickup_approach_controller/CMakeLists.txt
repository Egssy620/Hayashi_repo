cmake_minimum_required(VERSION 3.8)
project(pickup_approach_controller)

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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(pickup_approach_core
  src/controller_core.cpp
)
target_include_directories(pickup_approach_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(pickup_approach_controller_node
  src/pickup_approach_controller_node.cpp
)
target_link_libraries(pickup_approach_controller_node pickup_approach_core)
ament_target_dependencies(pickup_approach_controller_node
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs
)

install(TARGETS
  pickup_approach_core
  pickup_approach_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller_core test/test_controller_core.cpp)
  if(TARGET test_controller_core)
    target_link_libraries(test_controller_core pickup_approach_core)
    target_include_directories(test_controller_core PRIVATE include)
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(pickup_approach_core)
ament_package()
