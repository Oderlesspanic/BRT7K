#pragma once

#include <map>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "object_localizer/ground_contact_localizer.hpp"

struct ObjectConfig
{
  double size_x = 0.10;
  double size_y = 0.10;
  double size_z = 0.10;

  double offset_x = 0.0;
  double offset_y = 0.0;
};

class ObjectLocalizerNode : public rclcpp::Node
{
public:
  ObjectLocalizerNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  bool projectPixelToGround(
    const cv::Point2d & pixel,
    geometry_msgs::msg::PoseStamped & pose_target);

  std::string getClassName(
    const vision_msgs::msg::Detection2D & detection) const;

  ObjectConfig getObjectConfig(const std::string & class_name) const;

  void loadObjectConfigs();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr localized_objects_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  sensor_msgs::msg::Image::SharedPtr last_image_;
  sensor_msgs::msg::CameraInfo::SharedPtr last_camera_info_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  GroundContactLocalizer ground_contact_localizer_;

  std::string target_frame_;
  std::string camera_frame_;

  double ground_z_;

  std::map<std::string, ObjectConfig> object_configs_;
};