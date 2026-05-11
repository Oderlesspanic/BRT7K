#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "interfaces/msg/detected_color_patch.hpp"
#include "interfaces/msg/detected_color_patch_array.hpp"
#include "interfaces/msg/projected_color_patch.hpp"
#include "interfaces/msg/projected_color_patch_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "edge_color/color_wall_projection.hpp"

class ColorWallProjectorNode : public rclcpp::Node
{
public:
  ColorWallProjectorNode();

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void detected_patches_callback(
    const interfaces::msg::DetectedColorPatchArray::SharedPtr msg);

  std::optional<interfaces::msg::ProjectedColorPatch> create_projected_patch(
    const interfaces::msg::DetectedColorPatch & detected_patch,
    const std_msgs::msg::Header & header) const;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<interfaces::msg::DetectedColorPatchArray>::SharedPtr detected_patches_sub_;
  rclcpp::Publisher<interfaces::msg::ProjectedColorPatchArray>::SharedPtr projected_patches_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string input_topic_;
  std::string output_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_;
  std::string map_frame_;

  double arena_x_min_;
  double arena_x_max_;
  double arena_y_min_;
  double arena_y_max_;

  double wall_z_min_;
  double wall_z_max_;

  double min_projection_distance_;
  double max_projection_distance_;

  bool has_camera_info_;

  CameraIntrinsics intrinsics_;
};