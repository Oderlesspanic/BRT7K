#pragma once

#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"

struct CameraIntrinsics
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

struct WallProjectionResult
{
  geometry_msgs::msg::Point point;
  std::string wall_id;
  double distance = 0.0;
};

struct ArenaWalls
{
  double x_min = -1.0;
  double x_max = 1.0;
  double y_min = -1.0;
  double y_max = 1.0;
  double z_min = 0.0;
  double z_max = 1.2;
};

class ColorWallProjection
{
public:
  static std::optional<WallProjectionResult> image_point_to_wall(
    const geometry_msgs::msg::Point32 & image_point,
    const CameraIntrinsics & intrinsics,
    const std::string & camera_frame,
    const std::string & target_frame,
    const rclcpp::Time & stamp,
    tf2_ros::Buffer & tf_buffer,
    const ArenaWalls & walls);

  static std::optional<std::vector<geometry_msgs::msg::Point>> image_corners_to_same_wall(
    const std::vector<geometry_msgs::msg::Point32> & image_corners,
    const CameraIntrinsics & intrinsics,
    const std::string & camera_frame,
    const std::string & target_frame,
    const rclcpp::Time & stamp,
    tf2_ros::Buffer & tf_buffer,
    const ArenaWalls & walls,
    const std::string & wall_id);

private:
  static bool intersect_with_wall(
    const geometry_msgs::msg::Point & origin,
    const geometry_msgs::msg::Point & ray_point,
    const ArenaWalls & walls,
    const std::string & wall_id,
    WallProjectionResult & result);

  static bool is_inside_wall_limits(
    const geometry_msgs::msg::Point & point,
    const ArenaWalls & walls,
    const std::string & wall_id);

  static double distance_3d(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);
};