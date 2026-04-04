cmake_minimum_required(VERSION 3.8)
project(imu_mag_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(imu_filter_madgwick REQUIRED)

add_executable(imu_covariance_fixer src/imu_covariance_fixer.cpp)
ament_target_dependencies(imu_covariance_fixer rclcpp sensor_msgs)

install(TARGETS
  imu_covariance_fixer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
