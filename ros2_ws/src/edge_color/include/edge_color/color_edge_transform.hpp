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
  double fx;
  double fy;
  double cx;
  double cy;
};

class ColorEdgeTransform
{
public:
  static std::optional<geometry_msgs::msg::Point> image_point_to_map(
    const geometry_msgs::msg::Point32 & image_point,
    const CameraIntrinsics & intrinsics,
    const std::string & camera_frame,
    const std::string & target_frame,
    const rclcpp::Time & stamp,
    tf2_ros::Buffer & tf_buffer,
    double ground_z = 0.0);

  static std::optional<std::vector<geometry_msgs::msg::Point>> image_corners_to_map(
    const std::vector<geometry_msgs::msg::Point32> & image_corners,
    const CameraIntrinsics & intrinsics,
    const std::string & camera_frame,
    const std::string & target_frame,
    const rclcpp::Time & stamp,
    tf2_ros::Buffer & tf_buffer,
    double ground_z = 0.0);
};