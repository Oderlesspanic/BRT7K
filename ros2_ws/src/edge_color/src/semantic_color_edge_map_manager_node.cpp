#include "edge_color/semantic_color_edge_map_manager_node.hpp"

#include <utility>

SemanticColorEdgeMapManagerNode::SemanticColorEdgeMapManagerNode()
: Node("semantic_color_edge_map_manager_node")
{
  input_topic_ = this->declare_parameter<std::string>(
    "input_topic",
    "/color_patches/mapped");

  output_topic_ = this->declare_parameter<std::string>(
    "output_topic",
    "/color_edge/managed_json");

  const double merge_distance =
    this->declare_parameter<double>("merge_distance", 0.20);

  const double max_reacquire_distance_factor =
    this->declare_parameter<double>("max_reacquire_distance_factor", 3.0);

  const bool save_to_file =
    this->declare_parameter<bool>("save_to_file", true);

  const std::string save_path =
    this->declare_parameter<std::string>("save_path", "/tmp/color_edge_map.json");

  const int max_objects =
    this->declare_parameter<int>("max_objects", 100);

  const bool forget_after_time =
    this->declare_parameter<bool>("forget_after_time", false);

  const double forget_after_seconds =
    this->declare_parameter<double>("forget_after_seconds", 300.0);

  const bool print_updates =
    this->declare_parameter<bool>("print_updates", true);

  const bool use_hue_for_merge =
    this->declare_parameter<bool>("use_hue_for_merge", true);

  const double max_hue_distance =
    this->declare_parameter<double>("max_hue_distance", 12.0);

  const double min_s_for_hue_match =
    this->declare_parameter<double>("min_s_for_hue_match", 40.0);

  const bool use_map_area_for_merge =
    this->declare_parameter<bool>("use_map_area_for_merge", true);

  const double max_area_relative_difference =
    this->declare_parameter<double>("max_area_relative_difference", 0.40);

  manager_.configure(
    merge_distance,
    max_reacquire_distance_factor,
    save_to_file,
    save_path,
    static_cast<std::size_t>(max_objects),
    forget_after_time,
    forget_after_seconds,
    print_updates,
    use_hue_for_merge,
    max_hue_distance,
    min_s_for_hue_match,
    use_map_area_for_merge,
    max_area_relative_difference
  );

  const bool loaded = manager_.load_from_json();
  if (loaded) {
    RCLCPP_INFO(
      this->get_logger(),
      "Bestehende JSON-Datei geladen: %zu surfaces",
      manager_.get_surfaces().size());
  } else {
    RCLCPP_INFO(this->get_logger(), "Keine bestehende JSON-Datei geladen");
  }

  input_sub_ = this->create_subscription<interfaces::msg::MappedColorPatchArray>(
    input_topic_,
    10,
    std::bind(&SemanticColorEdgeMapManagerNode::input_callback, this, std::placeholders::_1));

  json_pub_ = this->create_publisher<std_msgs::msg::String>(output_topic_, 10);

  publish_state();

  RCLCPP_INFO(
    this->get_logger(),
    "semantic_color_edge_map_manager_node gestartet, input_topic=%s",
    input_topic_.c_str());
}

void SemanticColorEdgeMapManagerNode::input_callback(
  const interfaces::msg::MappedColorPatchArray::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);

  manager_.process_patches(msg->patches, stamp);
  manager_.remove_expired(this->now());
  manager_.enforce_max_objects();

  if (!manager_.save_to_json()) {
    RCLCPP_WARN(this->get_logger(), "JSON-Datei konnte nicht gespeichert werden");
  }

  publish_state();

  RCLCPP_INFO(
    this->get_logger(),
    "Verarbeitet: %zu patches, verwaltet: %zu surfaces",
    msg->patches.size(),
    manager_.get_surfaces().size());
}

void SemanticColorEdgeMapManagerNode::publish_state()
{
  std_msgs::msg::String msg;
  msg.data = manager_.to_json().dump(2);
  json_pub_->publish(msg);
}