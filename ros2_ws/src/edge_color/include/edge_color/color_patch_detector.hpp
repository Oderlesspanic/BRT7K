#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

struct HSVStats
{
  double mean_h;
  double mean_s;
  double mean_v;

  double median_h;
  double median_s;
  double median_v;

  double std_h;
  double std_s;
  double std_v;
};

struct DetectedColorPatch
{
  std::vector<cv::Point2f> corners;
  cv::Point2f center;
  double pixel_area;
  HSVStats hsv_stats;
};

class ColorPatchDetector
{
public:
  ColorPatchDetector(
    double min_contour_area = 800.0,
    int saturation_min = 80,
    int value_min = 60);

  std::vector<DetectedColorPatch> detect(const cv::Mat & bgr_image) const;

private:
  HSVStats compute_hsv_stats(
    const cv::Mat & hsv_image,
    const std::vector<cv::Point> & contour) const;

  static double compute_mean(const std::vector<int> & values);
  static double compute_median(std::vector<int> values);
  static double compute_stddev(const std::vector<int> & values, double mean);

  double min_contour_area_;
  int saturation_min_;
  int value_min_;
};