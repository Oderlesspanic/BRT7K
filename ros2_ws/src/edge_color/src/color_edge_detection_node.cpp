#include "edge_color/color_edge_detection_node.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/point32.hpp"

ColorEdgeDetectionNode::ColorEdgeDetectionNode()
: Node("color_edge_detection_node"),
  image_topic_(this->declare_parameter<std::string>("image_topic", "/camera/image_raw")),
  detected_patches_topic_(this->declare_parameter<std::string>("detected_patches_topic", "/color_patches/detected")),
  debug_view_(this->declare_parameter<bool>("debug_view", true)),
  detector_(
    this->declare_parameter<double>("min_contour_area", 800.0),
    this->declare_parameter<int>("saturation_min", 80),
    this->declare_parameter<int>("value_min", 60))
{
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    10,
    std::bind(&ColorEdgeDetectionNode::image_callback, this, std::placeholders::_1)
  );

  detected_patches_pub_ =
    this->create_publisher<interfaces::msg::DetectedColorPatchArray>(
      detected_patches_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "color_edge_detection_node gestartet");
  RCLCPP_INFO(this->get_logger(), "image_topic: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "detected_patches_topic: %s", detected_patches_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "debug_view: %s", debug_view_ ? "true" : "false");
}

void ColorEdgeDetectionNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat & bgr_image = cv_ptr->image;

  if (bgr_image.empty()) {
    RCLCPP_WARN(this->get_logger(), "Leeres Bild empfangen");
    return;
  }

  const std::vector<DetectedColorPatch> patches = detector_.detect(bgr_image);

  publish_detected_patches(patches, msg->header);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Erkannte Patches: %zu",
    patches.size()
  );

  for (const auto & patch : patches) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "PixelArea: %.2f | Center: (%.1f, %.1f) | mean HSV: (%.2f, %.2f, %.2f) | median HSV: (%.2f, %.2f, %.2f) | std HSV: (%.2f, %.2f, %.2f)",
      patch.pixel_area,
      patch.center.x,
      patch.center.y,
      patch.hsv_stats.mean_h,
      patch.hsv_stats.mean_s,
      patch.hsv_stats.mean_v,
      patch.hsv_stats.median_h,
      patch.hsv_stats.median_s,
      patch.hsv_stats.median_v,
      patch.hsv_stats.std_h,
      patch.hsv_stats.std_s,
      patch.hsv_stats.std_v
    );
  }

  if (debug_view_) {
    const cv::Mat debug_image = draw_detected_patches(bgr_image, patches);
    cv::imshow("color_edge_detection", debug_image);
    cv::waitKey(1);
  }
}

void ColorEdgeDetectionNode::publish_detected_patches(
  const std::vector<DetectedColorPatch> & patches,
  const std_msgs::msg::Header & header)
{
  interfaces::msg::DetectedColorPatchArray msg;
  msg.header = header;

  for (const auto & patch : patches) {
    interfaces::msg::DetectedColorPatch patch_msg;

    patch_msg.pixel_area = patch.pixel_area;

    patch_msg.center_image.x = static_cast<float>(patch.center.x);
    patch_msg.center_image.y = static_cast<float>(patch.center.y);
    patch_msg.center_image.z = 0.0f;

    for (const auto & corner : patch.corners) {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(corner.x);
      p.y = static_cast<float>(corner.y);
      p.z = 0.0f;
      patch_msg.corners_image.push_back(p);
    }

    patch_msg.mean_h = patch.hsv_stats.mean_h;
    patch_msg.mean_s = patch.hsv_stats.mean_s;
    patch_msg.mean_v = patch.hsv_stats.mean_v;

    patch_msg.median_h = patch.hsv_stats.median_h;
    patch_msg.median_s = patch.hsv_stats.median_s;
    patch_msg.median_v = patch.hsv_stats.median_v;

    patch_msg.std_h = patch.hsv_stats.std_h;
    patch_msg.std_s = patch.hsv_stats.std_s;
    patch_msg.std_v = patch.hsv_stats.std_v;

    msg.patches.push_back(patch_msg);
  }

  detected_patches_pub_->publish(msg);
}

cv::Mat ColorEdgeDetectionNode::draw_detected_patches(
  const cv::Mat & input_image,
  const std::vector<DetectedColorPatch> & patches) const
{
  cv::Mat debug_image = input_image.clone();

  for (const auto & patch : patches) {
    std::vector<cv::Point> polygon;
    polygon.reserve(patch.corners.size());

    for (const auto & corner : patch.corners) {
      const cv::Point point(
        static_cast<int>(std::round(corner.x)),
        static_cast<int>(std::round(corner.y))
      );

      polygon.push_back(point);

      cv::circle(
        debug_image,
        point,
        5,
        cv::Scalar(0, 255, 255),
        -1
      );
    }

    if (polygon.size() == 4) {
      cv::polylines(
        debug_image,
        polygon,
        true,
        cv::Scalar(0, 255, 0),
        2
      );
    }

    const cv::Point center_point(
      static_cast<int>(std::round(patch.center.x)),
      static_cast<int>(std::round(patch.center.y))
    );

    cv::circle(
      debug_image,
      center_point,
      6,
      cv::Scalar(255, 0, 255),
      -1
    );

    cv::Point text_position(20, 30);
    if (!polygon.empty()) {
      text_position = polygon[0];
    }

    const std::string label =
      "H=" + std::to_string(static_cast<int>(std::round(patch.hsv_stats.mean_h))) +
      " S=" + std::to_string(static_cast<int>(std::round(patch.hsv_stats.mean_s))) +
      " V=" + std::to_string(static_cast<int>(std::round(patch.hsv_stats.mean_v))) +
      " A=" + std::to_string(static_cast<int>(std::round(patch.pixel_area)));

    cv::putText(
      debug_image,
      label,
      text_position,
      cv::FONT_HERSHEY_SIMPLEX,
      0.6,
      cv::Scalar(255, 255, 255),
      2
    );
  }

  return debug_image;
}