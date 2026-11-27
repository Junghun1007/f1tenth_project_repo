#include <gtest/gtest.h>

#include "manual_capture/bev_projector.hpp"

using namespace manual_capture;

namespace
{
LensCalibration lens()
{
  LensCalibration calibration;
  calibration.fx = 560.0;
  calibration.fy = 560.0;
  calibration.cx = 640.0;
  calibration.cy = 400.0;
  calibration.width = 1280;
  calibration.height = 800;
  return calibration;
}
}  // namespace

TEST(BevProjector, ProjectsColorAndGrayOntoTheSameGrid)
{
  BevGridConfig grid;
  BevProjector projector(grid);
  EXPECT_FALSE(projector.ready());
  projector.configure(lens(), MountPose{0.20, 0.0, 14.0});
  ASSERT_TRUE(projector.ready());

  const cv::Mat gray(800, 1280, CV_8UC1, cv::Scalar(200));
  const cv::Mat color(800, 1280, CV_8UC3, cv::Scalar(10, 20, 30));
  const cv::Mat gray_bev = projector.project(gray);
  const cv::Mat color_bev = projector.project(color);
  EXPECT_EQ(gray_bev.size(), cv::Size(grid.output_width, grid.output_height));
  EXPECT_EQ(color_bev.size(), gray_bev.size());
  EXPECT_EQ(color_bev.type(), CV_8UC3);
  // Ground straight ahead of a centered camera is visible on the center column.
  EXPECT_EQ(gray_bev.at<std::uint8_t>(grid.output_height / 2, grid.output_width / 2), 200);
}

TEST(BevProjector, SymmetricCenteredLensGivesSymmetricCoverage)
{
  BevProjector projector(BevGridConfig{});
  projector.configure(lens(), MountPose{0.20, 0.0, 14.0});
  const cv::Mat bev = projector.project(cv::Mat(800, 1280, CV_8UC1, cv::Scalar(255)));
  const int row = bev.rows / 3;
  for (int offset = 0; offset < bev.cols / 2; ++offset) {
    EXPECT_EQ(bev.at<std::uint8_t>(row, bev.cols / 2 - 1 - offset) > 0,
      bev.at<std::uint8_t>(row, bev.cols / 2 + offset) > 0) << offset;
  }
}

TEST(BevProjector, RejectsWrongImageSizeAndBadGrid)
{
  BevProjector projector(BevGridConfig{});
  projector.configure(lens(), MountPose{});
  EXPECT_THROW(projector.project(cv::Mat(400, 640, CV_8UC1)), std::invalid_argument);
  BevGridConfig bad;
  bad.output_width = 100;
  EXPECT_THROW(BevProjector{bad}, std::invalid_argument);
}

namespace
{
// Pixel coordinates encoded in the image: B = x, G = y (scaled to 8 bits).
cv::Mat coordinateImage()
{
  cv::Mat image(800, 1280, CV_8UC3);
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      image.at<cv::Vec3b>(y, x) = cv::Vec3b(
        static_cast<std::uint8_t>(x * 255 / 1279), static_cast<std::uint8_t>(y * 255 / 799), 255);
    }
  }
  return image;
}

cv::Matx33d smallTilt()
{
  return rotationX(degToRad(1.2)) * rotationZ(degToRad(-0.7)) * rotationY(degToRad(0.4));
}

// The tilted frame equals a camera whose attitude is the mount pose times the tilt,
// i.e. the static LUT of a lens rotated by R (same projection center).
void expectSameBev(const cv::Mat & a, const cv::Mat & b)
{
  ASSERT_EQ(a.size(), b.size());
  int compared = 0;
  for (int row = 0; row < a.rows; ++row) {
    for (int column = 0; column < a.cols; ++column) {
      const auto pa = a.at<cv::Vec3b>(row, column);
      const auto pb = b.at<cv::Vec3b>(row, column);
      if (pa[2] < 250 || pb[2] < 250) {
        continue;  // border of the image coverage
      }
      ++compared;
      ASSERT_LE(std::abs(pa[0] - pb[0]), 2) << row << ',' << column;
      ASSERT_LE(std::abs(pa[1] - pb[1]), 2) << row << ',' << column;
    }
  }
  EXPECT_GT(compared, a.rows * a.cols / 2);
}
}  // namespace

TEST(BevProjector, IdentityTiltMatchesFixedPose)
{
  BevProjector projector(BevGridConfig{});
  projector.configure(lens(), MountPose{0.20, 0.5, 14.0});
  const cv::Mat image = coordinateImage();
  const cv::Mat fixed = projector.project(image);
  const cv::Mat tilted = projector.project(image, cv::Matx33d::eye());
  EXPECT_EQ(cv::norm(fixed, tilted, cv::NORM_INF), 0.0);
}

TEST(BevProjector, ColorTiltMatchesRotatedCamera)
{
  const MountPose pose{0.20, 0.5, 14.0};
  const cv::Matx33d tilt = smallTilt();
  BevProjector corrected(BevGridConfig{});
  corrected.configure(lens(), pose);
  auto rotated_lens = lens();
  rotated_lens.rotation_rgb_from_view = tilt;
  BevProjector reference(BevGridConfig{});
  reference.configure(rotated_lens, pose);

  const cv::Mat image = coordinateImage();
  expectSameBev(corrected.project(image, tilt), reference.project(image));
  // ...and the correction is not negligible for a ~1.4 degree tilt.
  EXPECT_GT(cv::norm(corrected.project(image), corrected.project(image, tilt), cv::NORM_INF), 10.0);
}

TEST(BevProjector, OtherLensTiltUsesTheSameBodyRotation)
{
  const MountPose pose{0.20, 0.0, 14.0};
  const cv::Matx33d tilt = smallTilt();
  auto left = lens();
  left.rotation_rgb_from_view = rotationY(degToRad(2.0)) * rotationX(degToRad(-1.0));
  BevProjector corrected(BevGridConfig{});
  corrected.configure(left, pose);
  auto rotated = left;
  rotated.rotation_rgb_from_view = tilt * left.rotation_rgb_from_view;
  BevProjector reference(BevGridConfig{});
  reference.configure(rotated, pose);

  const cv::Mat image = coordinateImage();
  expectSameBev(corrected.project(image, tilt), reference.project(image));
}
