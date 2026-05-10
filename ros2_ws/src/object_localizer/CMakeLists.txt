cmake_minimum_required(VERSION 3.8)
project(object_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(object_localizer_node
  src/object_localizer_main.cpp
  src/object_localizer_node.cpp
  src/ground_contact_localizer.cpp
)

ament_target_dependencies(object_localizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

target_link_libraries(object_localizer_node
  ${OpenCV_LIBS}
)

install(TARGETS
  object_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()