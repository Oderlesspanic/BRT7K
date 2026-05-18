#include "edge_color/color_wall_projector_node.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>

#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"

ColorWallProjectorNode::ColorWallProjectorNode()
: Node("color_wall_projector_node"),
  input_topic_(this->declare_parameter<std::string>("input_topic", "/color_patches/detected")),
  output_topic_(this->declare_parameter<std::string>("output_topic", "/color_patches/projected")),
  camera_info_topic_(this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info")),
  camera_frame_(this->declare_parameter<std::string>("camera_frame", "camera_optical_link")),
  map_frame_(this->declare_parameter<std::string>("map_frame", "map")),
  image_topic_(this->declare_parameter<std::string>("image_topic", "/camera/image_raw")),
  arena_x_min_(this->declare_parameter<double>("arena_x_min", -1.5)),
  arena_x_max_(this->declare_parameter<double>("arena_x_max", 1.5)),
  arena_y_min_(this->declare_parameter<double>("arena_y_min", -1.0)),
  arena_y_max_(this->declare_parameter<double>("arena_y_max", 1.0)),
  wall_z_min_(this->declare_parameter<double>("wall_z_min", 0.05)),
  wall_z_max_(this->declare_parameter<double>("wall_z_max", 1.20)),
  min_projection_distance_(this->declare_parameter<double>("min_projection_distance", 0.1)),
  max_projection_distance_(this->declare_parameter<double>("max_projection_distance", 5.0)),
  has_camera_info_(false),
  publish_debug_image_(this->declare_parameter<bool>("publish_debug_image", true))
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

  if (publish_debug_image_) {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      10,
      std::bind(&ColorWallProjectorNode::image_callback, this, std::placeholders::_1)
    );
    debug_image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>(
        "/color_patches/projected_debug_image", 10);
  }

  RCLCPP_INFO(this->get_logger(), "color_wall_projector_node gestartet");
  RCLCPP_INFO(this->get_logger(), "publish_debug_image: %s", publish_debug_image_ ? "true" : "false");
}

void ColorWallProjectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    last_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in image_callback: %s", e.what());
  }
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

  std::vector<bool> projection_ok(msg->patches.size(), false);
  std::vector<std::string> wall_ids(msg->patches.size());
  std::vector<double> map_widths(msg->patches.size(), 0.0);
  std::vector<double> map_heights(msg->patches.size(), 0.0);

  for (std::size_t i = 0; i < msg->patches.size(); ++i) {
    auto projected_patch = create_projected_patch(msg->patches[i], msg->header);
    if (!projected_patch.has_value()) {
      continue;
    }
    projected_msg.patches.push_back(projected_patch.value());
    projection_ok[i] = true;
    wall_ids[i] = projected_patch->wall_id;
    map_widths[i] = projected_patch->map_width;
    map_heights[i] = projected_patch->map_height;
  }

  projected_patches_pub_->publish(projected_msg);

  if (publish_debug_image_ && !last_image_.empty()) {
    const cv::Mat debug_img = draw_projected_patches(
      last_image_, *msg, projection_ok, wall_ids, map_widths, map_heights);
    publish_debug_image(debug_img, msg->header);
  }
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

cv::Mat ColorWallProjectorNode::draw_projected_patches(
  const cv::Mat & base_image,
  const interfaces::msg::DetectedColorPatchArray & detected,
  const std::vector<bool> & projection_ok,
  const std::vector<std::string> & wall_ids,
  const std::vector<double> & map_widths,
  const std::vector<double> & map_heights) const
{
  cv::Mat debug_image = base_image.clone();

  for (std::size_t i = 0; i < detected.patches.size(); ++i) {
    const auto & patch = detected.patches[i];
    const bool ok = projection_ok[i];
    const cv::Scalar color = ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

    std::vector<cv::Point> polygon;
    for (const auto & corner : patch.corners_image) {
      const cv::Point p(static_cast<int>(corner.x), static_cast<int>(corner.y));
      polygon.push_back(p);
      cv::circle(debug_image, p, 5, cv::Scalar(0, 255, 255), -1);
    }

    if (polygon.size() == 4) {
      cv::polylines(debug_image, polygon, true, color, 2);
    }

    const cv::Point center(
      static_cast<int>(patch.center_image.x),
      static_cast<int>(patch.center_image.y));
    cv::circle(debug_image, center, 6, color, -1);

    std::string label;
    if (ok) {
      label = wall_ids[i] +
        " " + std::to_string(static_cast<int>(std::round(map_widths[i] * 100))) +
        "x" + std::to_string(static_cast<int>(std::round(map_heights[i] * 100))) + "cm";
    } else {
      label = "no proj";
    }

    cv::Point text_pos = polygon.empty() ? center : polygon[0];
    text_pos.y = std::max(text_pos.y - 5, 15);

    cv::putText(debug_image, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
  }

  return debug_image;
}

void ColorWallProjectorNode::publish_debug_image(
  const cv::Mat & image,
  const std_msgs::msg::Header & header)
{
  auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
  debug_image_pub_->publish(*msg);
}