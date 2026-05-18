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
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <opencv2/opencv.hpp>

#include "edge_color/color_wall_projection.hpp"

class ColorWallProjectorNode : public rclcpp::Node
{
public:
  ColorWallProjectorNode();

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void detected_patches_callback(
    const interfaces::msg::DetectedColorPatchArray::SharedPtr msg);

  std::optional<interfaces::msg::ProjectedColorPatch> create_projected_patch(
    const interfaces::msg::DetectedColorPatch & detected_patch,
    const std_msgs::msg::Header & header) const;

  cv::Mat draw_projected_patches(
    const cv::Mat & base_image,
    const interfaces::msg::DetectedColorPatchArray & detected,
    const std::vector<bool> & projection_ok,
    const std::vector<std::string> & wall_ids,
    const std::vector<double> & map_widths,
    const std::vector<double> & map_heights) const;

  void publish_debug_image(
    const cv::Mat & image,
    const std_msgs::msg::Header & header);

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<interfaces::msg::DetectedColorPatchArray>::SharedPtr detected_patches_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<interfaces::msg::ProjectedColorPatchArray>::SharedPtr projected_patches_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string input_topic_;
  std::string output_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_;
  std::string map_frame_;
  std::string image_topic_;

  double arena_x_min_;
  double arena_x_max_;
  double arena_y_min_;
  double arena_y_max_;

  double wall_z_min_;
  double wall_z_max_;

  double min_projection_distance_;
  double max_projection_distance_;

  bool has_camera_info_;
  bool publish_debug_image_;
  cv::Mat last_image_;

  CameraIntrinsics intrinsics_;
};