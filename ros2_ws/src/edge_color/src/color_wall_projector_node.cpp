#include "edge_color/color_wall_projector_node.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>

ColorWallProjectorNode::ColorWallProjectorNode()
: Node("color_wall_projector_node"),
  input_topic_(this->declare_parameter<std::string>("input_topic", "/color_patches/detected")),
  output_topic_(this->declare_parameter<std::string>("output_topic", "/color_patches/projected")),
  camera_info_topic_(this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info")),
  camera_frame_(this->declare_parameter<std::string>("camera_frame", "camera_optical_link")),
  map_frame_(this->declare_parameter<std::string>("map_frame", "map")),
  arena_x_min_(this->declare_parameter<double>("arena_x_min", -1.5)),
  arena_x_max_(this->declare_parameter<double>("arena_x_max", 1.5)),
  arena_y_min_(this->declare_parameter<double>("arena_y_min", -1.0)),
  arena_y_max_(this->declare_parameter<double>("arena_y_max", 1.0)),
  wall_z_min_(this->declare_parameter<double>("wall_z_min", 0.05)),
  wall_z_max_(this->declare_parameter<double>("wall_z_max", 1.20)),
  min_projection_distance_(this->declare_parameter<double>("min_projection_distance", 0.1)),
  max_projection_distance_(this->declare_parameter<double>("max_projection_distance", 5.0)),
  has_camera_info_(false)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    10,
    std::bind(&ColorWallProjectorNode::camera_info_callback, this, std::placeholders::_1)
  );

  detected_patches_sub_ = this->create_subscription<interfaces::msg::DetectedColorPatchArray>(
    input_topic_,
    10,
    std::bind(&ColorWallProjectorNode::detected_patches_callback, this, std::placeholders::_1)
  );

  projected_patches_pub_ =
    this->create_publisher<interfaces::msg::ProjectedColorPatchArray>(
      output_topic_,
      10
    );

  RCLCPP_INFO(this->get_logger(), "color_wall_projector_node gestartet");
}

void ColorWallProjectorNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  intrinsics_.fx = msg->k[0];
  intrinsics_.fy = msg->k[4];
  intrinsics_.cx = msg->k[2];
  intrinsics_.cy = msg->k[5];

  has_camera_info_ = true;
}

void ColorWallProjectorNode::detected_patches_callback(
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

  interfaces::msg::ProjectedColorPatchArray projected_msg;
  projected_msg.header = msg->header;
  projected_msg.header.frame_id = map_frame_;

  for (const auto & detected_patch : msg->patches) {
    auto projected_patch = create_projected_patch(detected_patch, msg->header);

    if (!projected_patch.has_value()) {
      continue;
    }

    projected_msg.patches.push_back(projected_patch.value());
  }

  projected_patches_pub_->publish(projected_msg);
}

std::optional<interfaces::msg::ProjectedColorPatch>
ColorWallProjectorNode::create_projected_patch(
  const interfaces::msg::DetectedColorPatch & detected_patch,
  const std_msgs::msg::Header & header) const
{
  const rclcpp::Time stamp(header.stamp);

  ArenaWalls walls;
  walls.x_min = arena_x_min_;
  walls.x_max = arena_x_max_;
  walls.y_min = arena_y_min_;
  walls.y_max = arena_y_max_;
  walls.z_min = wall_z_min_;
  walls.z_max = wall_z_max_;

  auto center_projection = ColorWallProjection::image_point_to_wall(
    detected_patch.center_image,
    intrinsics_,
    camera_frame_,
    map_frame_,
    stamp,
    *tf_buffer_,
    walls
  );

  if (!center_projection.has_value()) {
    return std::nullopt;
  }

  if (
    center_projection->distance < min_projection_distance_ ||
    center_projection->distance > max_projection_distance_)
  {
    return std::nullopt;
  }

  auto corners_projection = ColorWallProjection::image_corners_to_same_wall(
    detected_patch.corners_image,
    intrinsics_,
    camera_frame_,
    map_frame_,
    stamp,
    *tf_buffer_,
    walls,
    center_projection->wall_id
  );

  if (!corners_projection.has_value()) {
    return std::nullopt;
  }

  interfaces::msg::ProjectedColorPatch projected_patch;

  projected_patch.center_map = center_projection->point;
  projected_patch.corners_map = corners_projection.value();
  projected_patch.wall_id = center_projection->wall_id;

  projected_patch.pixel_area = detected_patch.pixel_area;

  projected_patch.mean_h = detected_patch.mean_h;
  projected_patch.mean_s = detected_patch.mean_s;
  projected_patch.mean_v = detected_patch.mean_v;

  projected_patch.median_h = detected_patch.median_h;
  projected_patch.median_s = detected_patch.median_s;
  projected_patch.median_v = detected_patch.median_v;

  projected_patch.std_h = detected_patch.std_h;
  projected_patch.std_s = detected_patch.std_s;
  projected_patch.std_v = detected_patch.std_v;

  double min_u = 0.0;
  double max_u = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
  bool first = true;

  for (const auto & p : projected_patch.corners_map) {
    double u = 0.0;

    if (
      projected_patch.wall_id == "x_min" ||
      projected_patch.wall_id == "x_max")
    {
      u = p.y;
    } else {
      u = p.x;
    }

    if (first) {
      min_u = max_u = u;
      min_z = max_z = p.z;
      first = false;
    } else {
      min_u = std::min(min_u, u);
      max_u = std::max(max_u, u);
      min_z = std::min(min_z, p.z);
      max_z = std::max(max_z, p.z);
    }
  }

  projected_patch.map_width = std::abs(max_u - min_u);
  projected_patch.map_height = std::abs(max_z - min_z);
  projected_patch.map_area = projected_patch.map_width * projected_patch.map_height;

  return projected_patch;
}