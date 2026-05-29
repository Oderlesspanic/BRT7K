cmake_minimum_required(VERSION 3.8)
project(object_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(object_manager_node
  src/object_manager_main.cpp
  src/object_manager_node.cpp
)

ament_target_dependencies(object_manager_node
  rclcpp
  geometry_msgs
  interfaces
  sensor_msgs
  visualization_msgs
  vision_msgs
  std_msgs
)

install(TARGETS
  object_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
