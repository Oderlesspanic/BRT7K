#include <gtest/gtest.h>

#include "edge_color/color_patch_detector.hpp"

#include <cmath>
#include <opencv2/opencv.hpp>

namespace
{

bool approx_equal(double a, double b, double tolerance)
{
  return std::abs(a - b) <= tolerance;
}

}  // namespace

// BGR

TEST(ColorPatchDetectorTest, DetectsBlueRectangle)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(200, 150),
    cv::Point(400, 300),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 1u);
  EXPECT_EQ(patches[0].corners.size(), 4u);
  EXPECT_GT(patches[0].pixel_area, 1000.0);

  EXPECT_TRUE(approx_equal(patches[0].center.x, 300.0, 10.0));
  EXPECT_TRUE(approx_equal(patches[0].center.y, 225.0, 10.0));

  EXPECT_TRUE(approx_equal(patches[0].hsv_stats.mean_h, 120.0, 5.0));
  EXPECT_GT(patches[0].hsv_stats.mean_s, 200.0);
  EXPECT_GT(patches[0].hsv_stats.mean_v, 200.0);

  EXPECT_TRUE(approx_equal(patches[0].hsv_stats.median_h, 120.0, 5.0));
}

TEST(ColorPatchDetectorTest, DetectsRedRectangle)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(100, 100),
    cv::Point(250, 250),
    cv::Scalar(0, 0, 255),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 1u);
  EXPECT_EQ(patches[0].corners.size(), 4u);
  EXPECT_GT(patches[0].pixel_area, 1000.0);

  EXPECT_TRUE(approx_equal(patches[0].center.x, 175.0, 10.0));
  EXPECT_TRUE(approx_equal(patches[0].center.y, 175.0, 10.0));

  // Rot liegt in OpenCV-HSV typischerweise bei H nahe 0
  EXPECT_TRUE(
    approx_equal(patches[0].hsv_stats.mean_h, 0.0, 5.0) ||
    approx_equal(patches[0].hsv_stats.mean_h, 179.0, 5.0)
  );

  EXPECT_GT(patches[0].hsv_stats.mean_s, 200.0);
  EXPECT_GT(patches[0].hsv_stats.mean_v, 200.0);
}

TEST(ColorPatchDetectorTest, IgnoresSmallNoise)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(10, 10),
    cv::Point(20, 20),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  EXPECT_TRUE(patches.empty());
}

TEST(ColorPatchDetectorTest, DetectsMultipleColoredPatches)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(50, 50),
    cv::Point(180, 180),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  cv::rectangle(
    image,
    cv::Point(300, 200),
    cv::Point(500, 350),
    cv::Scalar(0, 255, 255),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 2u);

  bool found_blue_like = false;
  bool found_yellow_like = false;

  for (const auto & patch : patches) {
    EXPECT_EQ(patch.corners.size(), 4u);
    EXPECT_GT(patch.pixel_area, 1000.0);

    if (approx_equal(patch.hsv_stats.mean_h, 120.0, 8.0)) {
      found_blue_like = true;
    }

    if (approx_equal(patch.hsv_stats.mean_h, 30.0, 8.0)) {
      found_yellow_like = true;
    }
  }

  EXPECT_TRUE(found_blue_like);
  EXPECT_TRUE(found_yellow_like);
}

TEST(ColorPatchDetectorTest, DetectsRotatedBlueRectangle)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::RotatedRect rotated_rect(
    cv::Point2f(320.0f, 240.0f),
    cv::Size2f(180.0f, 100.0f),
    30.0f
  );

  cv::Point2f vertices[4];
  rotated_rect.points(vertices);

  std::vector<cv::Point> polygon;
  for (int i = 0; i < 4; ++i) {
    polygon.emplace_back(
      static_cast<int>(std::round(vertices[i].x)),
      static_cast<int>(std::round(vertices[i].y))
    );
  }

  cv::fillConvexPoly(image, polygon, cv::Scalar(255, 0, 0));

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 1u);
  EXPECT_EQ(patches[0].corners.size(), 4u);
  EXPECT_GT(patches[0].pixel_area, 1000.0);

  EXPECT_TRUE(approx_equal(patches[0].center.x, 320.0, 10.0));
  EXPECT_TRUE(approx_equal(patches[0].center.y, 240.0, 10.0));
  EXPECT_TRUE(approx_equal(patches[0].hsv_stats.mean_h, 120.0, 8.0));
}

TEST(ColorPatchDetectorTest, DetectsTwoSeparateBlueRectangles)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(60, 80),
    cv::Point(180, 200),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  cv::rectangle(
    image,
    cv::Point(350, 220),
    cv::Point(520, 360),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 2u);

  for (const auto & patch : patches) {
    EXPECT_EQ(patch.corners.size(), 4u);
    EXPECT_GT(patch.pixel_area, 1000.0);
    EXPECT_TRUE(approx_equal(patch.hsv_stats.mean_h, 120.0, 8.0));
  }
}

TEST(ColorPatchDetectorTest, ReturnsNoDetectionsForEmptyBlackImage)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  EXPECT_TRUE(patches.empty());
}

TEST(ColorPatchDetectorTest, DetectsBlueRectangleAtImageBorder)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(0, 120),
    cv::Point(120, 300),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 1u);
  EXPECT_EQ(patches[0].corners.size(), 4u);
  EXPECT_GT(patches[0].pixel_area, 1000.0);
  EXPECT_TRUE(approx_equal(patches[0].hsv_stats.mean_h, 120.0, 8.0));
}

TEST(ColorPatchDetectorTest, RejectsRectangleBelowMinContourArea)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(10, 10),
    cv::Point(25, 25),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  EXPECT_TRUE(patches.empty());
}

TEST(ColorPatchDetectorTest, HSVStdDevIsSmallForUniformPatch)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(150, 100),
    cv::Point(350, 280),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 1u);

  EXPECT_LT(patches[0].hsv_stats.std_h, 5.0);
  EXPECT_LT(patches[0].hsv_stats.std_s, 5.0);
  EXPECT_LT(patches[0].hsv_stats.std_v, 5.0);
}

TEST(ColorPatchDetectorTest, DetectsBluePatchWithBrightnessGradient)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  for (int y = 100; y < 300; ++y) {
    for (int x = 150; x < 350; ++x) {
      const int blue_value = 180 + ((x - 150) * 75) / 200;  // von 180 bis ca. 255
      image.at<cv::Vec3b>(y, x) = cv::Vec3b(
        static_cast<unsigned char>(blue_value), 0, 0
      );
    }
  }

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 1u);
  EXPECT_EQ(patches[0].corners.size(), 4u);
  EXPECT_GT(patches[0].pixel_area, 1000.0);

  EXPECT_TRUE(approx_equal(patches[0].hsv_stats.mean_h, 120.0, 10.0));
  EXPECT_GT(patches[0].hsv_stats.mean_s, 150.0);
  EXPECT_GT(patches[0].hsv_stats.mean_v, 150.0);

  // Durch den Helligkeitsverlauf sollte std_v merklich groesser als 0 sein
  EXPECT_GT(patches[0].hsv_stats.std_v, 5.0);
}

TEST(ColorPatchDetectorTest, DetectsBluePatchWithBlackHole)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(160, 120),
    cv::Point(360, 320),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  cv::circle(
    image,
    cv::Point(260, 220),
    25,
    cv::Scalar(0, 0, 0),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 1u);
  EXPECT_EQ(patches[0].corners.size(), 4u);
  EXPECT_GT(patches[0].pixel_area, 1000.0);

  EXPECT_TRUE(approx_equal(patches[0].hsv_stats.mean_h, 120.0, 10.0));
  EXPECT_GT(patches[0].hsv_stats.mean_s, 150.0);
}

TEST(ColorPatchDetectorTest, DetectsPastelBluePatchWithLowerSaturationThreshold)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  // Pastelliges Blau: B hoch, G und R etwas mit drin
  cv::rectangle(
    image,
    cv::Point(180, 140),
    cv::Point(380, 300),
    cv::Scalar(255, 180, 180),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0, 40, 60);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 1u);
  EXPECT_EQ(patches[0].corners.size(), 4u);
  EXPECT_GT(patches[0].pixel_area, 1000.0);

  EXPECT_GT(patches[0].hsv_stats.mean_s, 40.0);
  EXPECT_GT(patches[0].hsv_stats.mean_v, 150.0);
}

TEST(ColorPatchDetectorTest, DetectsTwoCloseButSeparateBluePatches)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(180, 150),
    cv::Point(260, 260),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  cv::rectangle(
    image,
    cv::Point(290, 150),
    cv::Point(370, 260),
    cv::Scalar(255, 0, 0),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0);
  const auto patches = detector.detect(image);

  ASSERT_EQ(patches.size(), 2u);

  for (const auto & patch : patches) {
    EXPECT_EQ(patch.corners.size(), 4u);
    EXPECT_GT(patch.pixel_area, 1000.0);
    EXPECT_TRUE(approx_equal(patch.hsv_stats.mean_h, 120.0, 10.0));
  }
}

TEST(ColorPatchDetectorTest, RejectsGrayPatchBecauseSaturationIsTooLow)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(150, 120),
    cv::Point(350, 300),
    cv::Scalar(180, 180, 180),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0, 80, 60);
  const auto patches = detector.detect(image);

  EXPECT_TRUE(patches.empty());
}

TEST(ColorPatchDetectorTest, RejectsWhitePatchBecauseSaturationIsTooLow)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  cv::rectangle(
    image,
    cv::Point(150, 120),
    cv::Point(350, 300),
    cv::Scalar(255, 255, 255),
    cv::FILLED
  );

  ColorPatchDetector detector(500.0, 80, 60);
  const auto patches = detector.detect(image);

  EXPECT_TRUE(patches.empty());
}
