#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>

#include "edge_color/color_patch_detector.hpp"
#include "interfaces/msg/detected_color_patch.hpp"
#include "interfaces/msg/detected_color_patch_array.hpp"

class ColorEdgeDetectionNode : public rclcpp::Node
{
public:
  ColorEdgeDetectionNode();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void publish_detected_patches(
    const std::vector<DetectedColorPatch> & patches,
    const std_msgs::msg::Header & header);

  cv::Mat draw_detected_patches(
    const cv::Mat & input_image,
    const std::vector<DetectedColorPatch> & patches) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<interfaces::msg::DetectedColorPatchArray>::SharedPtr detected_patches_pub_;

  std::string image_topic_;
  std::string detected_patches_topic_;
  bool debug_view_;

  ColorPatchDetector detector_;
};