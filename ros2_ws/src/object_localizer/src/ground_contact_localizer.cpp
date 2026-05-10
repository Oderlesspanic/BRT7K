#include "object_localizer/ground_contact_localizer.hpp"

#include <algorithm>
#include <cmath>

bool GroundContactLocalizer::findGroundContactPoint(
  const cv::Mat & image,
  const vision_msgs::msg::Detection2D & detection,
  cv::Point2d & contact_pixel)
{
  const double cx = detection.bbox.center.position.x;
  const double cy = detection.bbox.center.position.y;
  const double w = detection.bbox.size_x;
  const double h = detection.bbox.size_y;

  int x = static_cast<int>(cx - w / 2.0);
  int y = static_cast<int>(cy - h / 2.0);
  int width = static_cast<int>(w);
  int height = static_cast<int>(h);

  x = std::clamp(x, 0, image.cols - 1);
  y = std::clamp(y, 0, image.rows - 1);
  width = std::clamp(width, 1, image.cols - x);
  height = std::clamp(height, 1, image.rows - y);

  cv::Rect roi_rect(x, y, width, height);
  cv::Mat roi = image(roi_rect);

  cv::Point2d roi_contact;
  const std::string class_name = getClassName(detection);

  bool success = false;

  if (class_name == "rubixcube" || class_name == "fhgr_logo") {
    success = findCubeGroundContact(roi, roi_contact);
  } else if (class_name == "mate") {
    success = findCanGroundContact(roi, roi_contact);
  } else if (class_name == "ball" || class_name == "wurfel") {
    success = findBallGroundContact(roi, roi_contact);
  }

  if (!success) {
    return fallbackGroundContact(detection, contact_pixel);
  }

  contact_pixel.x = static_cast<double>(x) + roi_contact.x;
  contact_pixel.y = static_cast<double>(y) + roi_contact.y;

  return true;
}

bool GroundContactLocalizer::findCubeGroundContact(
  const cv::Mat & roi,
  cv::Point2d & roi_contact)
{
  cv::Mat gray;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

  cv::Mat edges;
  cv::Canny(gray, edges, 60, 160);

  const int min_y = static_cast<int>(roi.rows * 0.55);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edges, lines, 1, CV_PI / 180.0, 25, 20, 8);

  double best_score = -1.0;
  cv::Vec4i best_line;

  for (const auto & line : lines) {
    const int x1 = line[0];
    const int y1 = line[1];
    const int x2 = line[2];
    const int y2 = line[3];

    const double dx = std::abs(x2 - x1);
    const double dy = std::abs(y2 - y1);

    if (dx < 10.0) {
      continue;
    }

    if (dy / dx > 0.25) {
      continue;
    }

    const double y_avg = 0.5 * (y1 + y2);

    if (y_avg < min_y) {
      continue;
    }

    const double length = std::sqrt(dx * dx + dy * dy);
    const double score = length + y_avg;

    if (score > best_score) {
      best_score = score;
      best_line = line;
    }
  }

  if (best_score < 0.0) {
    return false;
  }

  roi_contact.x = 0.5 * (best_line[0] + best_line[2]);
  roi_contact.y = 0.5 * (best_line[1] + best_line[3]);

  return true;
}

bool GroundContactLocalizer::findCanGroundContact(
  const cv::Mat & roi,
  cv::Point2d & roi_contact)
{
  cv::Mat gray;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);

  cv::Mat edges;
  cv::Canny(blurred, edges, 50, 140);

  const int start_y = static_cast<int>(roi.rows * 0.55);

  std::vector<cv::Point> points;

  for (int y = start_y; y < edges.rows; ++y) {
    for (int x = 0; x < edges.cols; ++x) {
      if (edges.at<uint8_t>(y, x) > 0) {
        points.emplace_back(x, y);
      }
    }
  }

  if (points.empty()) {
    return false;
  }

  auto bottom_it = std::max_element(
    points.begin(),
    points.end(),
    [](const cv::Point & a, const cv::Point & b) {
      return a.y < b.y;
    }
  );

  const int bottom_y = bottom_it->y;

  std::vector<cv::Point> bottom_points;

  for (const auto & p : points) {
    if (std::abs(p.y - bottom_y) <= 4) {
      bottom_points.push_back(p);
    }
  }

  if (bottom_points.empty()) {
    return false;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;

  for (const auto & p : bottom_points) {
    sum_x += p.x;
    sum_y += p.y;
  }

  roi_contact.x = sum_x / bottom_points.size();
  roi_contact.y = sum_y / bottom_points.size();

  return true;
}

bool GroundContactLocalizer::findBallGroundContact(
  const cv::Mat & roi,
  cv::Point2d & roi_contact)
{
  cv::Mat gray;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(7, 7), 2.0);

  cv::Mat edges;
  cv::Canny(blurred, edges, 50, 130);

  std::vector<std::vector<cv::Point>> contours;

  cv::findContours(
    edges,
    contours,
    cv::RETR_EXTERNAL,
    cv::CHAIN_APPROX_SIMPLE
  );

  if (contours.empty()) {
    return false;
  }

  double best_area = 0.0;
  int best_index = -1;

  for (size_t i = 0; i < contours.size(); ++i) {
    const double area = cv::contourArea(contours[i]);

    if (area > best_area) {
      best_area = area;
      best_index = static_cast<int>(i);
    }
  }

  if (best_index < 0) {
    return false;
  }

  const auto & contour = contours[best_index];

  auto bottom_it = std::max_element(
    contour.begin(),
    contour.end(),
    [](const cv::Point & a, const cv::Point & b) {
      return a.y < b.y;
    }
  );

  roi_contact.x = bottom_it->x;
  roi_contact.y = bottom_it->y;

  return true;
}

bool GroundContactLocalizer::fallbackGroundContact(
  const vision_msgs::msg::Detection2D & detection,
  cv::Point2d & contact_pixel)
{
  contact_pixel.x = detection.bbox.center.position.x;
  contact_pixel.y =
    detection.bbox.center.position.y +
    detection.bbox.size_y / 2.0;

  return true;
}

std::string GroundContactLocalizer::getClassName(
  const vision_msgs::msg::Detection2D & detection) const
{
  if (detection.results.empty()) {
    return "unknown";
  }

  return detection.results[0].hypothesis.class_id;
}