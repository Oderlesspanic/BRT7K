cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_toolbox REQUIRED)

install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

add_executable(map_saver_node
  src/map_saver_main.cpp
  src/map_saver_node.cpp
)

ament_target_dependencies(map_saver_node
  rclcpp
  slam_toolbox
)


install(TARGETS
  map_saver_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()