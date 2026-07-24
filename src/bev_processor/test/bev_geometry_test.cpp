#include <cmath>
#include <cstdlib>
#include <iostream>

#include <opencv2/core.hpp>

#include "bev_processor/bev_geometry.hpp"

namespace
{

bool require(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  bev_processor::RectifiedCameraModel camera{
    561.400939941,
    561.136352539,
    643.032653809,
    352.621124268,
    1280,
    720,
    cv::Vec3d(0.0, 0.0, 0.20),
    bev_processor::mountRotationVehicleFromCamera(
      0.0, bev_processor::degToRad(14.0), 0.0)};
  const bev_processor::BevConfig config{
    0.18, 5.0, -0.35, 0.35, 0.01, 70, 482};

  const auto lut = bev_processor::generateRemap(camera, config);
  bool passed = true;
  passed &= require(
    lut.map_x.rows == 482 && lut.map_x.cols == 70,
    "remap dimensions must be 70x482");
  passed &= require(lut.map_x.type() == CV_32FC1, "map_x must be float32");
  passed &= require(lut.map_y.type() == CV_32FC1, "map_y must be float32");
  passed &= require(
    lut.valid_mask.type() == CV_8UC1, "valid mask must be uint8");

  const int valid = cv::countNonZero(lut.valid_mask);
  const double valid_ratio = static_cast<double>(valid) / (70.0 * 482.0);
  passed &= require(
    valid_ratio > 0.99,
    "more than 99 percent of the configured BEV must project into the image");

  const int far_row = 0;
  const int near_row = config.output_height - 1;
  const int left_column = 0;
  const int right_column = config.output_width - 1;
  passed &= require(
    lut.map_x.at<float>(far_row, left_column) <
    lut.map_x.at<float>(far_row, right_column),
    "vehicle-left BEV pixels must map left of vehicle-right pixels");
  passed &= require(
    lut.map_y.at<float>(near_row, config.output_width / 2) >
    lut.map_y.at<float>(far_row, config.output_width / 2),
    "near ground must map lower in the camera image than far ground");

  cv::Mat input(720, 1280, CV_8UC3);
  for (int row = 0; row < input.rows; ++row) {
    for (int column = 0; column < input.cols; ++column) {
      input.at<cv::Vec3b>(row, column) = cv::Vec3b(
        static_cast<std::uint8_t>(column % 256),
        static_cast<std::uint8_t>(row % 256),
        127U);
    }
  }
  const cv::Mat output = bev_processor::convertToBev(input, lut);
  passed &= require(
    output.rows == 482 && output.cols == 70 && output.type() == CV_8UC3,
    "converted image must be 70x482 BGR8");
  passed &= require(
    cv::countNonZero(output.reshape(1)) > 0,
    "converted image must contain projected source pixels");

  if (!passed) {
    return EXIT_FAILURE;
  }
  std::cout << "BEV geometry test passed; valid LUT ratio="
            << valid_ratio * 100.0 << "%\n";
  return EXIT_SUCCESS;
}
