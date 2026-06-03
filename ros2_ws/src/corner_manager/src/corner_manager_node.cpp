#include "corner_manager/corner_manager_node.hpp"

#include <sstream>

CornerManagerNode::CornerManagerNode()
: Node("corner_manager_node"),
  manager_(
    CornerManagerConfig{
      this->declare_parameter<double>("merge_distance", 0.20),
      this->declare_parameter<bool>("use_hsv_for_merge", true),
      this->declare_parameter<double>("max_hue_distance", 12.0),
      this->declare_parameter<double>("max_s_difference", 80.0),
      this->declare_parameter<double>("max_v_difference", 100.0),
      this->declare_parameter<double>("min_s_for_hue_match", 40.0),
      static_cast<std::size_t>(this->declare_parameter<int>("max_corners", 20)),
      this->declare_parameter<bool>("forget_after_time", false),
      this->declare_parameter<double>("forget_after_seconds", 300.0)
    })
{
  input_topic_ =
    this->declare_parameter<std::string>(
      "input_topic",
      "/color_corners/valid");

  output_topic_ =
    this->declare_parameter<std::string>(
      "output_topic",
      "/corners");

  json_output_topic_ =
    this->declare_parameter<std::string>(
      "json_output_topic",
      "/corners/json");

  marker_topic_ =
    this->declare_parameter<std::string>(
      "marker_topic",
      "/corners/markers");

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
    this->create_subscription<interfaces::msg::ValidColorCornerArray>(
      input_topic_,
      10,
      std::bind(
        &CornerManagerNode::valid_corners_callback,
        this,
        std::placeholders::_1));

  corners_pub_ =
    this->create_publisher<interfaces::msg::ManagedCornerArray>(
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

  RCLCPP_INFO(this->get_logger(), "corner_manager_node gestartet");
}

void CornerManagerNode::valid_corners_callback(
  const interfaces::msg::ValidColorCornerArray::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);

  manager_.process_corners(msg->corners, stamp);
  manager_.remove_expired(this->now());
  manager_.enforce_max_corners();

  publish_corners();

  if (publish_json_) {
    publish_json();
  }

  if (publish_markers_) {
    publish_markers();
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Valid corners input: %zu | managed corners: %zu",
    msg->corners.size(),
    manager_.get_corners().size());
}

void CornerManagerNode::publish_corners()
{
  interfaces::msg::ManagedCornerArray msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = map_frame_;
  msg.corners = manager_.get_corners();

  corners_pub_->publish(msg);
}

void CornerManagerNode::publish_json()
{
  std_msgs::msg::String msg;
  msg.data = to_json_string();
  json_pub_->publish(msg);
}

void CornerManagerNode::publish_markers()
{
  visualization_msgs::msg::MarkerArray marker_array;

  const auto & corners = manager_.get_corners();

  for (std::size_t i = 0; i < corners.size(); ++i) {
    const auto & c = corners[i];

    visualization_msgs::msg::Marker marker;

    marker.header.stamp = this->now();
    marker.header.frame_id = map_frame_;

    marker.ns = "managed_corners";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = c.center_map;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.06;
    marker.scale.y = c.map_width;
    marker.scale.z = c.map_height;

    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 1.0;
    marker.color.a = 0.75;

    marker_array.markers.push_back(marker);

    visualization_msgs::msg::Marker text;

    text.header = marker.header;
    text.ns = "managed_corner_labels";
    text.id = static_cast<int>(i + 1000);
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;

    text.pose.position = c.center_map;
    text.pose.position.z += 0.18;
    text.pose.orientation.w = 1.0;

    text.scale.z = 0.10;

    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;

    text.text =
      "Ecke " + std::to_string(i + 1) + "\n" +
      c.corner_uid + "\n" +
      c.corner_id + " / " + c.wall_id +
      "\nHSV: " +
      std::to_string(static_cast<int>(c.median_h)) + ", " +
      std::to_string(static_cast<int>(c.median_s)) + ", " +
      std::to_string(static_cast<int>(c.median_v));

    marker_array.markers.push_back(text);
  }

  marker_pub_->publish(marker_array);
}

std::string CornerManagerNode::to_json_string() const
{
  std::ostringstream oss;

  const auto & corners = manager_.get_corners();

  oss << "{\n";
  oss << "  \"corners\": [\n";

  for (std::size_t i = 0; i < corners.size(); ++i) {
    const auto & c = corners[i];

    oss << "    {\n";
    oss << "      \"number\": " << (i + 1) << ",\n";
    oss << "      \"corner_uid\": \"" << c.corner_uid << "\",\n";
    oss << "      \"corner_id\": \"" << c.corner_id << "\",\n";
    oss << "      \"wall_id\": \"" << c.wall_id << "\",\n";
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
    oss << "      \"seen_count\": " << c.seen_count << ",\n";
    oss << "      \"validation_score\": " << c.validation_score << "\n";
    oss << "    }";

    if (i + 1 < corners.size()) {
      oss << ",";
    }

    oss << "\n";
  }

  oss << "  ]\n";
  oss << "}";

  return oss.str();
}
