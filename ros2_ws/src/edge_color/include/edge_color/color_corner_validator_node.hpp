#pragma once

#include <string>

#include "edge_color/color_corner_validator.hpp"

#include "interfaces/msg/detected_color_patch_array.hpp"
#include "interfaces/msg/projected_color_patch_array.hpp"
#include "interfaces/msg/valid_color_corner_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <opencv2/opencv.hpp>

class ColorCornerValidatorNode : public rclcpp::Node
{
public:
  ColorCornerValidatorNode();

private:
  void projected_patches_callback(
    const interfaces::msg::ProjectedColorPatchArray::SharedPtr msg);

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void detected_patches_callback(
    const interfaces::msg::DetectedColorPatchArray::SharedPtr msg);

  void publish_json(
    const interfaces::msg::ValidColorCornerArray & corners_msg);

  void publish_markers(
    const interfaces::msg::ValidColorCornerArray & corners_msg);

  cv::Mat draw_validation_results(
    const cv::Mat & base_image,
    const interfaces::msg::DetectedColorPatchArray & detected,
    const interfaces::msg::ProjectedColorPatchArray & projected,
    const std::vector<bool> & is_valid,
    const std::vector<std::string> & corner_ids,
    const std::vector<double> & scores) const;

  void publish_debug_image(
    const cv::Mat & image,
    const std_msgs::msg::Header & header);

  rclcpp::Subscription<interfaces::msg::ProjectedColorPatchArray>::SharedPtr input_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<interfaces::msg::DetectedColorPatchArray>::SharedPtr detected_patches_sub_;

  rclcpp::Publisher<interfaces::msg::ValidColorCornerArray>::SharedPtr valid_corners_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string json_output_topic_;
  std::string marker_topic_;
  std::string map_frame_;
  std::string image_topic_;
  std::string detected_topic_;

  bool publish_json_;
  bool publish_markers_;
  bool publish_debug_image_;

  cv::Mat last_image_;
  interfaces::msg::DetectedColorPatchArray::SharedPtr last_detected_patches_;

  ColorCornerValidator validator_;
};