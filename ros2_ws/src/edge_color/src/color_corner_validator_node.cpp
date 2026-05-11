#include "edge_color/color_corner_validator_node.hpp"

#include <sstream>

ColorCornerValidatorNode::ColorCornerValidatorNode()
: Node("color_corner_validator_node"),
  validator_(
    ColorCornerValidatorConfig{
      this->declare_parameter<double>("arena_x_min", -1.5),
      this->declare_parameter<double>("arena_x_max", 1.5),
      this->declare_parameter<double>("arena_y_min", -1.0),
      this->declare_parameter<double>("arena_y_max", 1.0),
      this->declare_parameter<double>("wall_distance_tolerance", 0.08),
      this->declare_parameter<double>("corner_distance_tolerance", 0.35),
      this->declare_parameter<double>("a3_width", 0.297),
      this->declare_parameter<double>("a3_height", 0.420),
      this->declare_parameter<double>("a3_size_tolerance", 0.35),
      this->declare_parameter<double>("min_s_for_color", 40.0),
      this->declare_parameter<double>("min_v_for_color", 50.0)
    })
{
  input_topic_ =
    this->declare_parameter<std::string>(
      "input_topic",
      "/color_patches/projected");

  output_topic_ =
    this->declare_parameter<std::string>(
      "output_topic",
      "/color_corners/valid");

  json_output_topic_ =
    this->declare_parameter<std::string>(
      "json_output_topic",
      "/color_corners/valid_json");

  marker_topic_ =
    this->declare_parameter<std::string>(
      "marker_topic",
      "/color_corners/markers");

  map_frame_ =
    this->declare_parameter<std::string>(
      "map_frame",
      "map");

  publish_json_ =
    this->declare_parameter<bool>(
      "publish_json",
      true);

  publish_markers_ =
    this->declare_parameter<bool>(
      "publish_markers",
      true);

  input_sub_ =
    this->create_subscription<interfaces::msg::ProjectedColorPatchArray>(
      input_topic_,
      10,
      std::bind(
        &ColorCornerValidatorNode::projected_patches_callback,
        this,
        std::placeholders::_1));

  valid_corners_pub_ =
    this->create_publisher<interfaces::msg::ValidColorCornerArray>(
      output_topic_,
      10);

  json_pub_ =
    this->create_publisher<std_msgs::msg::String>(
      json_output_topic_,
      10);

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_,
      10);

  RCLCPP_INFO(this->get_logger(), "color_corner_validator_node gestartet");
}

void ColorCornerValidatorNode::projected_patches_callback(
  const interfaces::msg::ProjectedColorPatchArray::SharedPtr msg)
{
  interfaces::msg::ValidColorCornerArray valid_msg;
  valid_msg.header = msg->header;
  valid_msg.header.frame_id = map_frame_;

  for (const auto & patch : msg->patches) {
    auto corner = validator_.validate(patch);

    if (corner.has_value()) {
      valid_msg.corners.push_back(corner.value());
    }
  }

  valid_corners_pub_->publish(valid_msg);

  if (publish_json_) {
    publish_json(valid_msg);
  }

  if (publish_markers_) {
    publish_markers(valid_msg);
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Projected patches: %zu | valid corners: %zu",
    msg->patches.size(),
    valid_msg.corners.size());
}

void ColorCornerValidatorNode::publish_json(
  const interfaces::msg::ValidColorCornerArray & corners_msg)
{
  std::ostringstream oss;

  oss << "{\n";
  oss << "  \"corners\": [\n";

  for (std::size_t i = 0; i < corners_msg.corners.size(); ++i) {
    const auto & c = corners_msg.corners[i];

    oss << "    {\n";
    oss << "      \"corner_id\": \"" << c.corner_id << "\",\n";
    oss << "      \"wall_id\": \"" << c.wall_id << "\",\n";
    oss << "      \"color_name\": \"" << c.color_name << "\",\n";
    oss << "      \"center_map\": {\n";
    oss << "        \"x\": " << c.center_map.x << ",\n";
    oss << "        \"y\": " << c.center_map.y << ",\n";
    oss << "        \"z\": " << c.center_map.z << "\n";
    oss << "      },\n";
    oss << "      \"map_width\": " << c.map_width << ",\n";
    oss << "      \"map_height\": " << c.map_height << ",\n";
    oss << "      \"map_area\": " << c.map_area << ",\n";
    oss << "      \"median_h\": " << c.median_h << ",\n";
    oss << "      \"median_s\": " << c.median_s << ",\n";
    oss << "      \"median_v\": " << c.median_v << ",\n";
    oss << "      \"validation_score\": " << c.validation_score << "\n";
    oss << "    }";

    if (i + 1 < corners_msg.corners.size()) {
      oss << ",";
    }

    oss << "\n";
  }

  oss << "  ]\n";
  oss << "}";

  std_msgs::msg::String json_msg;
  json_msg.data = oss.str();

  json_pub_->publish(json_msg);
}

void ColorCornerValidatorNode::publish_markers(
  const interfaces::msg::ValidColorCornerArray & corners_msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (std::size_t i = 0; i < corners_msg.corners.size(); ++i) {
    const auto & c = corners_msg.corners[i];

    visualization_msgs::msg::Marker marker;

    marker.header = corners_msg.header;
    marker.header.frame_id = map_frame_;

    marker.ns = "valid_color_corners";
    marker.id = static_cast<int>(i);

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = c.center_map;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = c.map_width;
    marker.scale.z = c.map_height;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;

    marker_array.markers.push_back(marker);

    visualization_msgs::msg::Marker text;

    text.header = marker.header;
    text.ns = "valid_color_corner_labels";
    text.id = static_cast<int>(i + 1000);

    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;

    text.pose.position = c.center_map;
    text.pose.position.z += 0.15;
    text.pose.orientation.w = 1.0;

    text.scale.z = 0.10;

    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;

    text.text = c.corner_id + " / " + c.wall_id;

    marker_array.markers.push_back(text);
  }

  marker_pub_->publish(marker_array);
}