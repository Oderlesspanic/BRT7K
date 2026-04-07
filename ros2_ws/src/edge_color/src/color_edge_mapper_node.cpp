#include "edge_color/color_edge_mapper_node.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

ColorEdgeMapperNode::ColorEdgeMapperNode()
: Node("color_edge_mapper_node"),
  input_topic_(this->declare_parameter<std::string>("input_topic", "/color_patches/detected")),
  output_topic_(this->declare_parameter<std::string>("output_topic", "/color_patches/mapped")),
  camera_info_topic_(this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info")),
  camera_frame_(this->declare_parameter<std::string>("camera_frame", "camera_optical_link")),
  map_frame_(this->declare_parameter<std::string>("map_frame", "map")),
  ground_z_(this->declare_parameter<double>("ground_z", 0.0)),
  min_projection_distance_(this->declare_parameter<double>("min_projection_distance", 0.1)),
  max_projection_distance_(this->declare_parameter<double>("max_projection_distance", 5.0)),
  publish_debug_markers_(this->declare_parameter<bool>("publish_debug_markers", true)),
  has_camera_info_(false)
{
  intrinsics_.fx = 0.0;
  intrinsics_.fy = 0.0;
  intrinsics_.cx = 0.0;
  intrinsics_.cy = 0.0;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    10,
    std::bind(&ColorEdgeMapperNode::camera_info_callback, this, std::placeholders::_1)
  );

  detected_patches_sub_ = this->create_subscription<interfaces::msg::DetectedColorPatchArray>(
    input_topic_,
    10,
    std::bind(&ColorEdgeMapperNode::detected_patches_callback, this, std::placeholders::_1)
  );

  mapped_patches_pub_ = this->create_publisher<interfaces::msg::MappedColorPatchArray>(
    output_topic_,
    10
  );

  RCLCPP_INFO(this->get_logger(), "color_edge_mapper_node gestartet");
}

void ColorEdgeMapperNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  intrinsics_.fx = msg->k[0];
  intrinsics_.fy = msg->k[4];
  intrinsics_.cx = msg->k[2];
  intrinsics_.cy = msg->k[5];

  has_camera_info_ = true;
}

void ColorEdgeMapperNode::detected_patches_callback(
  const interfaces::msg::DetectedColorPatchArray::SharedPtr msg)
{
  if (!has_camera_info_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Noch keine CameraInfo empfangen"
    );
    return;
  }

  interfaces::msg::MappedColorPatchArray mapped_msg;
  mapped_msg.header = msg->header;
  mapped_msg.header.frame_id = map_frame_;

  for (const auto & detected_patch : msg->patches) {
    try {
      auto mapped_patch = create_mapped_patch(detected_patch, msg->header);

      if (!is_projection_distance_valid(mapped_patch.center_map)) {
        continue;
      }

      mapped_msg.patches.push_back(mapped_patch);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Patch konnte nicht transformiert werden: %s", e.what());
    }
  }

  mapped_patches_pub_->publish(mapped_msg);
}

interfaces::msg::MappedColorPatch ColorEdgeMapperNode::create_mapped_patch(
  const interfaces::msg::DetectedColorPatch & detected_patch,
  const std_msgs::msg::Header & header) const
{
  const rclcpp::Time stamp(header.stamp);

  auto center_map_opt = ColorEdgeTransform::image_point_to_map(
    detected_patch.center_image,
    intrinsics_,
    camera_frame_,
    map_frame_,
    stamp,
    *tf_buffer_,
    ground_z_
  );

  if (!center_map_opt.has_value()) {
    throw std::runtime_error("center_image konnte nicht nach map transformiert werden");
  }

  auto corners_map_opt = ColorEdgeTransform::image_corners_to_map(
    detected_patch.corners_image,
    intrinsics_,
    camera_frame_,
    map_frame_,
    stamp,
    *tf_buffer_,
    ground_z_
  );

  if (!corners_map_opt.has_value()) {
    throw std::runtime_error("corners_image konnten nicht nach map transformiert werden");
  }

  interfaces::msg::MappedColorPatch mapped_patch;
  mapped_patch.center_map = center_map_opt.value();
  mapped_patch.corners_map = corners_map_opt.value();

  mapped_patch.pixel_area = detected_patch.pixel_area;
  mapped_patch.mean_h = detected_patch.mean_h;
  mapped_patch.mean_s = detected_patch.mean_s;
  mapped_patch.mean_v = detected_patch.mean_v;
  mapped_patch.median_h = detected_patch.median_h;
  mapped_patch.median_s = detected_patch.median_s;
  mapped_patch.median_v = detected_patch.median_v;
  mapped_patch.std_h = detected_patch.std_h;
  mapped_patch.std_s = detected_patch.std_s;
  mapped_patch.std_v = detected_patch.std_v;

  return mapped_patch;
}

bool ColorEdgeMapperNode::is_projection_distance_valid(
  const geometry_msgs::msg::Point & point) const
{
  const double distance = std::sqrt(point.x * point.x + point.y * point.y);
  return distance >= min_projection_distance_ && distance <= max_projection_distance_;
}