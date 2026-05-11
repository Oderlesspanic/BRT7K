#pragma once

#include <string>

#include "edge_color/color_corner_validator.hpp"

#include "interfaces/msg/projected_color_patch_array.hpp"
#include "interfaces/msg/valid_color_corner_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ColorCornerValidatorNode : public rclcpp::Node
{
public:
  ColorCornerValidatorNode();

private:
  void projected_patches_callback(
    const interfaces::msg::ProjectedColorPatchArray::SharedPtr msg);

  void publish_json(
    const interfaces::msg::ValidColorCornerArray & corners_msg);

  void publish_markers(
    const interfaces::msg::ValidColorCornerArray & corners_msg);

  rclcpp::Subscription<interfaces::msg::ProjectedColorPatchArray>::SharedPtr input_sub_;

  rclcpp::Publisher<interfaces::msg::ValidColorCornerArray>::SharedPtr valid_corners_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string json_output_topic_;
  std::string marker_topic_;
  std::string map_frame_;

  bool publish_json_;
  bool publish_markers_;

  ColorCornerValidator validator_;
};