#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "interfaces/msg/detected_color_patch.hpp"
#include "interfaces/msg/detected_color_patch_array.hpp"
#include "interfaces/msg/mapped_color_patch.hpp"
#include "interfaces/msg/mapped_color_patch_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "edge_color/color_edge_transform.hpp"

class ColorEdgeMapperNode : public rclcpp::Node
{
public:
  ColorEdgeMapperNode();

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void detected_patches_callback(
    const interfaces::msg::DetectedColorPatchArray::SharedPtr msg);

  interfaces::msg::MappedColorPatch create_mapped_patch(
    const interfaces::msg::DetectedColorPatch & detected_patch,
    const std_msgs::msg::Header & header) const;

  bool is_projection_distance_valid(const geometry_msgs::msg::Point & point) const;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<interfaces::msg::DetectedColorPatchArray>::SharedPtr detected_patches_sub_;
  rclcpp::Publisher<interfaces::msg::MappedColorPatchArray>::SharedPtr mapped_patches_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string input_topic_;
  std::string output_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_;
  std::string map_frame_;

  double ground_z_;
  double min_projection_distance_;
  double max_projection_distance_;
  bool publish_debug_markers_;
  bool has_camera_info_;

  CameraIntrinsics intrinsics_;
};