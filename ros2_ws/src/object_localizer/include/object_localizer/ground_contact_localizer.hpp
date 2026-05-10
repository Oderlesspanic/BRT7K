#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "vision_msgs/msg/detection2_d.hpp"

class GroundContactLocalizer
{
public:
  bool findGroundContactPoint(
    const cv::Mat & image,
    const vision_msgs::msg::Detection2D & detection,
    cv::Point2d & contact_pixel);

private:
  bool findCubeGroundContact(const cv::Mat & roi, cv::Point2d & roi_contact);
  bool findCanGroundContact(const cv::Mat & roi, cv::Point2d & roi_contact);
  bool findBallGroundContact(const cv::Mat & roi, cv::Point2d & roi_contact);

  bool fallbackGroundContact(
    const vision_msgs::msg::Detection2D & detection,
    cv::Point2d & contact_pixel);

  std::string getClassName(
    const vision_msgs::msg::Detection2D & detection) const;
};
