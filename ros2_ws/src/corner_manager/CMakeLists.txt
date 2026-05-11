cmake_minimum_required(VERSION 3.8)
project(corner_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_library(corner_manager_lib
  src/corner_manager.cpp
)

target_include_directories(corner_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(corner_manager_lib
  rclcpp
  interfaces
  geometry_msgs
)

add_executable(corner_manager_node
  src/corner_manager_node.cpp
  src/corner_manager_main.cpp
)

target_include_directories(corner_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(corner_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  interfaces
)

target_link_libraries(corner_manager_node
  corner_manager_lib
)

install(
  TARGETS
    corner_manager_lib
    corner_manager_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()