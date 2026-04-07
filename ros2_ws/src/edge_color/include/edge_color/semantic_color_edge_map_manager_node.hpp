#pragma once

#include <string>

#include "edge_color/semantic_color_edge_map_manager.hpp"
#include "interfaces/msg/mapped_color_patch_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SemanticColorEdgeMapManagerNode : public rclcpp::Node
{
public:
  SemanticColorEdgeMapManagerNode();

private:
  void input_callback(const interfaces::msg::MappedColorPatchArray::SharedPtr msg);
  void publish_state();

  rclcpp::Subscription<interfaces::msg::MappedColorPatchArray>::SharedPtr input_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_;

  SemanticColorEdgeMapManager manager_;

  std::string input_topic_;
  std::string output_topic_;
};

