cmake_minimum_required(VERSION 3.10)
project(heading_test_node)

# 1) コンパイラ設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# 2) 依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 3) ソース
add_executable(heading_test_node
  src/heading_test_node.cpp
)

# 4) 依存のリンク
ament_target_dependencies(heading_test_node
  rclcpp
  geometry_msgs
)

# OpenCVは通常のCMakeライブラリとしてリンク
target_link_libraries(heading_test_node
  ${OpenCV_LIBS}
)

# 5) 警告オプション(任意)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(heading_test_node PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
  target_compile_options(heading_test_node PRIVATE /W4)
endif()

# 6) インストール
install(TARGETS heading_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 7) ament
ament_package()

