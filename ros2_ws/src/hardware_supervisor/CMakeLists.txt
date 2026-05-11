cmake_minimum_required(VERSION 3.8)
project(hardware_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(hardware_supervisor_node
  src/hardware_supervisor_main.cpp
  src/hardware_supervisor_node.cpp
  src/hardware_supervisor.cpp
  src/device_status_builder.cpp
  src/composite_device_monitor.cpp
  src/usb_monitor.cpp
  src/i2c_monitor.cpp
  src/topic_monitor.cpp
  src/esp32_heartbeat_monitor.cpp
)

ament_target_dependencies(hardware_supervisor_node
  rclcpp
  diagnostic_msgs
  std_msgs
)

install(TARGETS
  hardware_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()