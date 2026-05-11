cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedColorPatch.msg"
  "msg/DetectedColorPatchArray.msg"

  "msg/ProjectedColorPatch.msg"
  "msg/ProjectedColorPatchArray.msg"

  "msg/ValidColorCorner.msg"
  "msg/ValidColorCornerArray.msg"

  "msg/ManagedCorner.msg"
  "msg/ManagedCornerArray.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
  "msg/MappedColorPatch.msg"
  "msg/MappedColorPatchArray.msg"
  "msg/CameraDetection.msg"
  "msg/CameraDetectionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()