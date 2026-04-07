#include "edge_color/color_edge_transform.hpp"

#include <cmath>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

std::optional<geometry_msgs::msg::Point> ColorEdgeTransform::image_point_to_map(
  const geometry_msgs::msg::Point32 & image_point,
  const CameraIntrinsics & intrinsics,
  const std::string & camera_frame,
  const std::string & target_frame,
  const rclcpp::Time & stamp,
  tf2_ros::Buffer & tf_buffer,
  double ground_z)
{
  if (std::abs(intrinsics.fx) < 1e-9 || std::abs(intrinsics.fy) < 1e-9) {
    return std::nullopt;
  }

  geometry_msgs::msg::PointStamped origin_cam;
  origin_cam.header.frame_id = camera_frame;
  origin_cam.header.stamp = stamp;
  origin_cam.point.x = 0.0;
  origin_cam.point.y = 0.0;
  origin_cam.point.z = 0.0;

  geometry_msgs::msg::PointStamped ray_cam;
  ray_cam.header.frame_id = camera_frame;
  ray_cam.header.stamp = stamp;
  ray_cam.point.x = (static_cast<double>(image_point.x) - intrinsics.cx) / intrinsics.fx;
  ray_cam.point.y = (static_cast<double>(image_point.y) - intrinsics.cy) / intrinsics.fy;
  ray_cam.point.z = 1.0;

  geometry_msgs::msg::PointStamped origin_map;
  geometry_msgs::msg::PointStamped ray_map;

  try {
    origin_map = tf_buffer.transform(origin_cam, target_frame);
    ray_map = tf_buffer.transform(ray_cam, target_frame);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }

  const double dx = ray_map.point.x - origin_map.point.x;
  const double dy = ray_map.point.y - origin_map.point.y;
  const double dz = ray_map.point.z - origin_map.point.z;

  if (std::abs(dz) < 1e-9) {
    return std::nullopt;
  }

  const double t = (ground_z - origin_map.point.z) / dz;

  if (t < 0.0) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point result;
  result.x = origin_map.point.x + t * dx;
  result.y = origin_map.point.y + t * dy;
  result.z = ground_z;

  return result;
}

std::optional<std::vector<geometry_msgs::msg::Point>> ColorEdgeTransform::image_corners_to_map(
  const std::vector<geometry_msgs::msg::Point32> & image_corners,
  const CameraIntrinsics & intrinsics,
  const std::string & camera_frame,
  const std::string & target_frame,
  const rclcpp::Time & stamp,
  tf2_ros::Buffer & tf_buffer,
  double ground_z)
{
  std::vector<geometry_msgs::msg::Point> map_corners;
  map_corners.reserve(image_corners.size());

  for (const auto & corner : image_corners) {
    auto map_point = image_point_to_map(
      corner,
      intrinsics,
      camera_frame,
      target_frame,
      stamp,
      tf_buffer,
      ground_z);

    if (!map_point.has_value()) {
      return std::nullopt;
    }

    map_corners.push_back(map_point.value());
  }

  return map_corners;
}