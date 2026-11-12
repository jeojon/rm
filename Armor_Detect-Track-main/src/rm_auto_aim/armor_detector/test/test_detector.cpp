// Copyright 2026 rm_auto_aim contributors
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

#include "armor_detector/detector.hpp"

namespace
{
std::vector<rm_auto_aim::Light> detectEllipse(const double angle)
{
  const rm_auto_aim::Detector::LightParams light_params{0.1, 0.4, 35.0, 0.5};
  const rm_auto_aim::Detector::ArmorParams armor_params{0.7, 0.8, 3.2, 3.2, 5.5, 35.0};
  rm_auto_aim::Detector detector(160, rm_auto_aim::RED, light_params, armor_params);

  cv::Mat rgb = cv::Mat::zeros(100, 100, CV_8UC3);
  cv::Mat binary = cv::Mat::zeros(100, 100, CV_8UC1);
  cv::ellipse(
    rgb, cv::Point(50, 50), cv::Size(4, 20), angle, 0, 360, cv::Scalar(255, 0, 0),
    cv::FILLED);
  cv::ellipse(
    binary, cv::Point(50, 50), cv::Size(4, 20), angle, 0, 360, cv::Scalar(255), cv::FILLED);

  return detector.findLights(rgb, binary);
}
}  // namespace

TEST(DetectorTest, AcceptsVerticalLight)
{
  const auto lights = detectEllipse(0);
  ASSERT_EQ(lights.size(), 1U);
  EXPECT_NEAR(lights.front().tilt_angle, 0.0, 1.0);
}

TEST(DetectorTest, AcceptsBothTiltDirections)
{
  const auto clockwise_lights = detectEllipse(15);
  const auto counterclockwise_lights = detectEllipse(-15);

  ASSERT_EQ(clockwise_lights.size(), 1U);
  ASSERT_EQ(counterclockwise_lights.size(), 1U);
  EXPECT_NEAR(clockwise_lights.front().tilt_angle, 15.0, 1.0);
  EXPECT_NEAR(counterclockwise_lights.front().tilt_angle, 15.0, 1.0);
}

TEST(DetectorTest, RejectsHorizontalLight)
{
  EXPECT_TRUE(detectEllipse(90).empty());
}
