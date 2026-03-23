cmake_minimum_required(VERSION 3.8)
project(imu)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)


add_library(${PROJECT_NAME}_lib
  src/imu_converter.cpp
  src/mpu6050_driver.cpp
  src/mpu6050_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

install(
  TARGETS
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_node
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

  ament_add_gtest(test_imu_converter
    test/test_imu_converter.cpp
  )
  if(TARGET test_imu_converter)
    target_link_libraries(test_imu_converter
      ${PROJECT_NAME}_lib
    )
    ament_target_dependencies(test_imu_converter
      rclcpp
      sensor_msgs
      std_msgs
      geometry_msgs
    )
  endif()

  ament_add_gtest(test_mpu6050_driver
    test/test_mpu6050_driver.cpp
  )
  if(TARGET test_mpu6050_driver)
    target_link_libraries(test_mpu6050_driver
      ${PROJECT_NAME}_lib
    )
    ament_target_dependencies(test_mpu6050_driver
      rclcpp
      sensor_msgs
      std_msgs
      geometry_msgs
    )
  endif()

  ament_add_gtest(test_mpu6050_node
    test/test_mpu6050_node.cpp
  )
  if(TARGET test_mpu6050_node)
    target_link_libraries(test_mpu6050_node
      ${PROJECT_NAME}_lib
    )
    ament_target_dependencies(test_mpu6050_node
      rclcpp
      sensor_msgs
      std_msgs
      geometry_msgs
    )
  endif()

endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

ament_package()