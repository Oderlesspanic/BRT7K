#include "edge_color/color_patch_detector.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

ColorPatchDetector::ColorPatchDetector(
  double min_contour_area,
  int saturation_min,
  int value_min)
: min_contour_area_(min_contour_area),
  saturation_min_(saturation_min),
  value_min_(value_min)
{
}

std::vector<DetectedColorPatch> ColorPatchDetector::detect(const cv::Mat & bgr_image) const
{
  std::vector<DetectedColorPatch> results;

  if (bgr_image.empty()) {
    return results;
  }

  cv::Mat hsv_image;
  cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(
    hsv_image,
    cv::Scalar(0, saturation_min_, value_min_),
    cv::Scalar(180, 255, 255),
    mask
  );

  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

  cv::erode(mask, mask, kernel);
  cv::dilate(mask, mask, kernel);
  cv::dilate(mask, mask, kernel);
  cv::erode(mask, mask, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < min_contour_area_) {
      continue;
    }

    const cv::RotatedRect rect = cv::minAreaRect(contour);

    cv::Point2f rect_points[4];
    rect.points(rect_points);

    DetectedColorPatch patch;
    patch.pixel_area = area;
    patch.corners.assign(rect_points, rect_points + 4);
    patch.center = rect.center;
    patch.hsv_stats = compute_hsv_stats(hsv_image, contour);

    results.push_back(patch);
  }

  return results;
}

HSVStats ColorPatchDetector::compute_hsv_stats(
  const cv::Mat & hsv_image,
  const std::vector<cv::Point> & contour) const
{
  HSVStats stats{};

  cv::Mat contour_mask = cv::Mat::zeros(hsv_image.rows, hsv_image.cols, CV_8UC1);
  std::vector<std::vector<cv::Point>> contours = {contour};
  cv::drawContours(contour_mask, contours, 0, cv::Scalar(255), cv::FILLED);

  std::vector<int> h_values;
  std::vector<int> s_values;
  std::vector<int> v_values;

  for (int y = 0; y < hsv_image.rows; ++y) {
    for (int x = 0; x < hsv_image.cols; ++x) {
      if (contour_mask.at<unsigned char>(y, x) == 0) {
        continue;
      }

      const cv::Vec3b hsv = hsv_image.at<cv::Vec3b>(y, x);
      h_values.push_back(static_cast<int>(hsv[0]));
      s_values.push_back(static_cast<int>(hsv[1]));
      v_values.push_back(static_cast<int>(hsv[2]));
    }
  }

  if (h_values.empty()) {
    return stats;
  }

  stats.mean_h = compute_mean(h_values);
  stats.mean_s = compute_mean(s_values);
  stats.mean_v = compute_mean(v_values);

  stats.median_h = compute_median(h_values);
  stats.median_s = compute_median(s_values);
  stats.median_v = compute_median(v_values);

  stats.std_h = compute_stddev(h_values, stats.mean_h);
  stats.std_s = compute_stddev(s_values, stats.mean_s);
  stats.std_v = compute_stddev(v_values, stats.mean_v);

  return stats;
}

double ColorPatchDetector::compute_mean(const std::vector<int> & values)
{
  if (values.empty()) {
    return 0.0;
  }

  const double sum = std::accumulate(values.begin(), values.end(), 0.0);
  return sum / static_cast<double>(values.size());
}

double ColorPatchDetector::compute_median(std::vector<int> values)
{
  if (values.empty()) {
    return 0.0;
  }

  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();

  if (n % 2 == 0) {
    return (static_cast<double>(values[n / 2 - 1]) +
            static_cast<double>(values[n / 2])) / 2.0;
  }

  return static_cast<double>(values[n / 2]);
}

double ColorPatchDetector::compute_stddev(const std::vector<int> & values, double mean)
{
  if (values.empty()) {
    return 0.0;
  }

  double sum_sq = 0.0;

  for (const int value : values) {
    const double diff = static_cast<double>(value) - mean;
    sum_sq += diff * diff;
  }

  return std::sqrt(sum_sq / static_cast<double>(values.size()));
}