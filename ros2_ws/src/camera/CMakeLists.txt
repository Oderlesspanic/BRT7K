cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(
  include
  ${LIBCAMERA_INCLUDE_DIRS}
)

link_directories(${LIBCAMERA_LIBRARY_DIRS})

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
  src/libcamera_driver.cpp
  src/image_converter.cpp
)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
)

target_link_libraries(camera_node
  ${LIBCAMERA_LIBRARIES}
)

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()