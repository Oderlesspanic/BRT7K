#pragma once

#include <string>

#include "corner_manager/corner_manager.hpp"

#include "interfaces/msg/managed_corner_array.hpp"
#include "interfaces/msg/valid_color_corner_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class CornerManagerNode : public rclcpp::Node
{
public:
  CornerManagerNode();

private:
  void valid_corners_callback(
    const interfaces::msg::ValidColorCornerArray::SharedPtr msg);

  void publish_corners();
  void publish_json();
  void publish_markers();

  std::string to_json_string() const;

  rclcpp::Subscription<interfaces::msg::ValidColorCornerArray>::SharedPtr input_sub_;

  rclcpp::Publisher<interfaces::msg::ManagedCornerArray>::SharedPtr corners_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string json_output_topic_;
  std::string marker_topic_;
  std::string map_frame_;

  bool publish_json_;
  bool publish_markers_;

  CornerManager manager_;
};