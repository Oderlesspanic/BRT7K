#include "edge_color/color_wall_projection.hpp"

#include <cmath>
#include <limits>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

std::optional<WallProjectionResult> ColorWallProjection::image_point_to_wall(
  const geometry_msgs::msg::Point32 & image_point,
  const CameraIntrinsics & intrinsics,
  const std::string & camera_frame,
  const std::string & target_frame,
  const rclcpp::Time & stamp,
  tf2_ros::Buffer & tf_buffer,
  const ArenaWalls & walls)
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

  std::vector<std::string> wall_ids = {
    "x_min",
    "x_max",
    "y_min",
    "y_max"
  };

  bool found = false;
  WallProjectionResult best_result;
  best_result.distance = std::numeric_limits<double>::max();

  for (const auto & wall_id : wall_ids) {
    WallProjectionResult result;

    if (!intersect_with_wall(
        origin_map.point,
        ray_map.point,
        walls,
        wall_id,
        result))
    {
      continue;
    }

    if (!is_inside_wall_limits(result.point, walls, wall_id)) {
      continue;
    }

    if (result.distance < best_result.distance) {
      best_result = result;
      found = true;
    }
  }

  if (!found) {
    return std::nullopt;
  }

  return best_result;
}

std::optional<std::vector<geometry_msgs::msg::Point>>
ColorWallProjection::image_corners_to_same_wall(
  const std::vector<geometry_msgs::msg::Point32> & image_corners,
  const CameraIntrinsics & intrinsics,
  const std::string & camera_frame,
  const std::string & target_frame,
  const rclcpp::Time & stamp,
  tf2_ros::Buffer & tf_buffer,
  const ArenaWalls & walls,
  const std::string & wall_id)
{
  std::vector<geometry_msgs::msg::Point> projected_corners;
  projected_corners.reserve(image_corners.size());

  if (std::abs(intrinsics.fx) < 1e-9 || std::abs(intrinsics.fy) < 1e-9) {
    return std::nullopt;
  }

  geometry_msgs::msg::PointStamped origin_cam;
  origin_cam.header.frame_id = camera_frame;
  origin_cam.header.stamp = stamp;
  origin_cam.point.x = 0.0;
  origin_cam.point.y = 0.0;
  origin_cam.point.z = 0.0;

  geometry_msgs::msg::PointStamped origin_map;

  try {
    origin_map = tf_buffer.transform(origin_cam, target_frame);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }

  for (const auto & image_corner : image_corners) {
    geometry_msgs::msg::PointStamped ray_cam;
    ray_cam.header.frame_id = camera_frame;
    ray_cam.header.stamp = stamp;
    ray_cam.point.x = (static_cast<double>(image_corner.x) - intrinsics.cx) / intrinsics.fx;
    ray_cam.point.y = (static_cast<double>(image_corner.y) - intrinsics.cy) / intrinsics.fy;
    ray_cam.point.z = 1.0;

    geometry_msgs::msg::PointStamped ray_map;

    try {
      ray_map = tf_buffer.transform(ray_cam, target_frame);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }

    WallProjectionResult result;

    if (!intersect_with_wall(
        origin_map.point,
        ray_map.point,
        walls,
        wall_id,
        result))
    {
      return std::nullopt;
    }

    projected_corners.push_back(result.point);
  }

  return projected_corners;
}

bool ColorWallProjection::intersect_with_wall(
  const geometry_msgs::msg::Point & origin,
  const geometry_msgs::msg::Point & ray_point,
  const ArenaWalls & walls,
  const std::string & wall_id,
  WallProjectionResult & result)
{
  const double dx = ray_point.x - origin.x;
  const double dy = ray_point.y - origin.y;
  const double dz = ray_point.z - origin.z;

  double t = -1.0;

  if (wall_id == "x_min") {
    if (std::abs(dx) < 1e-9) {
      return false;
    }
    t = (walls.x_min - origin.x) / dx;
  } else if (wall_id == "x_max") {
    if (std::abs(dx) < 1e-9) {
      return false;
    }
    t = (walls.x_max - origin.x) / dx;
  } else if (wall_id == "y_min") {
    if (std::abs(dy) < 1e-9) {
      return false;
    }
    t = (walls.y_min - origin.y) / dy;
  } else if (wall_id == "y_max") {
    if (std::abs(dy) < 1e-9) {
      return false;
    }
    t = (walls.y_max - origin.y) / dy;
  } else {
    return false;
  }

  if (t <= 0.0) {
    return false;
  }

  result.point.x = origin.x + t * dx;
  result.point.y = origin.y + t * dy;
  result.point.z = origin.z + t * dz;
  result.wall_id = wall_id;
  result.distance = distance_3d(origin, result.point);

  return true;
}

bool ColorWallProjection::is_inside_wall_limits(
  const geometry_msgs::msg::Point & point,
  const ArenaWalls & walls,
  const std::string & wall_id)
{
  if (point.z < walls.z_min || point.z > walls.z_max) {
    return false;
  }

  if (wall_id == "x_min" || wall_id == "x_max") {
    return point.y >= walls.y_min && point.y <= walls.y_max;
  }

  if (wall_id == "y_min" || wall_id == "y_max") {
    return point.x >= walls.x_min && point.x <= walls.x_max;
  }

  return false;
}

double ColorWallProjection::distance_3d(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}