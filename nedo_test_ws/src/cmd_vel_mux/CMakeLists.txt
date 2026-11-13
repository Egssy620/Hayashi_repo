cmake_minimum_required(VERSION 3.8)
project(cmd_vel_mux)

# Warnings (GCC/Clang)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++標準
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 実行ファイル
add_executable(cmd_vel_mux_node
  src/cmd_vel_mux_node.cpp
)

# 依存リンク
ament_target_dependencies(cmd_vel_mux_node
  rclcpp
  geometry_msgs
  std_msgs
)

# インストール
install(TARGETS
  cmd_vel_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

# （将来launchやconfigを入れる場合の雛形）
# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}
# )

# エクスポート
ament_package()

