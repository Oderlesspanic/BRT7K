cmake_minimum_required(VERSION 3.8)
project(frontier_explorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(frontier_explorer_node
  src/main.cpp
  src/frontier_explorer.cpp
  src/frontier_explorer_node.cpp
)

target_include_directories(frontier_explorer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(frontier_explorer_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  nav2_msgs
)

install(TARGETS
  frontier_explorer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
