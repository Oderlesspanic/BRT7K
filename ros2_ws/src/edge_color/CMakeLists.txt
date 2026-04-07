cmake_minimum_required(VERSION 3.8)
project(edge_color)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_library(color_patch_detector_lib
  src/color_patch_detector.cpp
)

target_include_directories(color_patch_detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(color_patch_detector_lib
  ${OpenCV_LIBS}
)

add_executable(color_edge_detection_node
  src/color_edge_detection_node.cpp
  src/color_edge_detection_main.cpp
)

target_include_directories(color_edge_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(color_edge_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
  interfaces
)

target_link_libraries(color_edge_detection_node
  color_patch_detector_lib
  ${OpenCV_LIBS}
)

add_executable(color_edge_mapper_node
  src/color_edge_mapper_node.cpp
  src/color_edge_mapper_main.cpp
  src/color_edge_transform.cpp
)

target_include_directories(color_edge_mapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(color_edge_mapper_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
  interfaces
  tf2_geometry_msgs
)

target_link_libraries(color_edge_mapper_node
  ${OpenCV_LIBS}
)

add_executable(semantic_color_edge_map_manager_node
  src/semantic_color_edge_map_manager_node.cpp
  src/semantic_color_edge_map_manager_main.cpp
  src/semantic_color_edge_map_manager.cpp
)

target_include_directories(semantic_color_edge_map_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(semantic_color_edge_map_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  interfaces
)

target_link_libraries(semantic_color_edge_map_manager_node
  nlohmann_json::nlohmann_json
)

install(
  TARGETS
    color_patch_detector_lib
    color_edge_detection_node
    color_edge_mapper_node
    semantic_color_edge_map_manager_node
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

if(BUILD_TESTING)
  ament_add_gtest(test_color_patch_detector
    test/test_color_patch_detector.cpp
  )

  target_include_directories(test_color_patch_detector PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_color_patch_detector
    color_patch_detector_lib
    ${OpenCV_LIBS}
  )
endif()

ament_package()