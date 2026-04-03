cmake_minimum_required(VERSION 3.8)
project(mmc5603)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(mmc5603_node
  src/main.cpp
  src/i2c_bus.cpp
  src/mmc5603_driver.cpp
  src/mmc5603_node.cpp
  src/mmc5603_convert.cpp
)

ament_target_dependencies(mmc5603_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  mmc5603_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()