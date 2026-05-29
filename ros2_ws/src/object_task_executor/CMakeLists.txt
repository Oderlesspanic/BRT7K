cmake_minimum_required(VERSION 3.8)
project(object_task_executor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(include)

add_executable(object_task_executor_node
  src/object_task_executor_main.cpp
  src/object_task_executor_node.cpp
)

target_include_directories(object_task_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(object_task_executor_node
  geometry_msgs
  interfaces
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  vision_msgs
)

install(TARGETS
  object_task_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
