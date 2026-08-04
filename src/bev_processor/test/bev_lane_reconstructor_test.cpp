#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "bev_processor/bev_lane_reconstructor.hpp"

namespace
{

constexpr double kXMaxM = 3.0;
constexpr double kYMaxM = 0.6;
constexpr double kMeterPerPixel = 0.01;

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

cv::Point metricPoint(const double x_m, const double y_m)
{
  return cv::Point(
    static_cast<int>(std::lround(
      (kYMaxM - y_m) / kMeterPerPixel - 0.5)),
    static_cast<int>(std::lround(
      (kXMaxM - x_m) / kMeterPerPixel - 0.5)));
}

double expectedCenter(const double x_m)
{
  return 0.055 * x_m * x_m + 0.01 * x_m;
}

std::vector<cv::Point> makeBoundary(
  const double lateral_offset_m,
  const double maximum_x_m)
{
  std::vector<cv::Point> points;
  for (double x_m = 0.20; x_m <= maximum_x_m; x_m += 0.01) {
    points.push_back(metricPoint(
      x_m, expectedCenter(x_m) + lateral_offset_m));
  }
  return points;
}

bev_processor::BevLaneReconstructor makeReconstructor()
{
  bev_processor::BevLaneReconstructorConfig config;
  config.minimum_brightness = 150;
  config.maximum_saturation = 255;
  config.observation_minimum_x_m = 0.20;
  config.observation_maximum_x_m = 1.80;
  config.reconstruction_minimum_x_m = 0.20;
  config.reconstruction_maximum_x_m = 2.30;
  config.maximum_extrapolation_m = 0.50;
  config.expected_lane_width_m = 0.60;
  config.lane_width_tolerance_m = 0.12;
  config.minimum_points = 15;
  config.maximum_fit_residual_m = 0.04;
  config.temporal_smoothing_alpha = 1.0;
  return bev_processor::BevLaneReconstructor(config);
}

void testCurvesAreReconstructedPastTrustedRange()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  const std::vector<std::vector<cv::Point>> left{
    makeBoundary(0.30, 1.80)};
  const std::vector<std::vector<cv::Point>> right{
    makeBoundary(-0.30, 1.80)};
  cv::polylines(image, left, false, cv::Scalar(240, 240, 240), 5);
  cv::polylines(image, right, false, cv::Scalar(240, 240, 240), 5);

  // This wide far-distance smear must not influence the trusted fit.
  cv::line(
    image,
    metricPoint(1.90, -0.45),
    metricPoint(2.70, 0.45),
    cv::Scalar(255, 255, 255),
    24);

  const auto result = makeReconstructor().reconstruct(image);
  require(result.valid, "curved lane pair must produce a clean mask");
  require(
    result.center_curve.valid,
    "trusted near/mid-distance points must produce a center curve");
  require(
    result.center_curve.maximum_observed_x_m <= 1.81,
    "points beyond the 1.8m trust limit must not enter the fit");
  require(
    result.reconstructed_maximum_x_m > 2.20 &&
    result.reconstructed_maximum_x_m <= 2.31,
    "the fitted curve must be extrapolated by at most 0.5m");
  require(
    std::abs(result.center_curve.lateralAt(2.20) - expectedCenter(2.20)) <
    0.04,
    "limited extrapolation must follow the synthetic curve");

  const int reconstructed_row = metricPoint(2.20, 0.0).y;
  require(
    cv::countNonZero(result.reconstructed_mask.row(reconstructed_row)) > 0,
    "the clean mask must contain white lanes at 2.2m");
  const int outside_row = metricPoint(2.60, 0.0).y;
  require(
    cv::countNonZero(result.reconstructed_mask.row(outside_row)) == 0,
    "the clean mask must not extrapolate past the configured limit");
}

void testSingleBoundaryStillReconstructsBothLanes()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  const std::vector<std::vector<cv::Point>> left{
    makeBoundary(0.30, 1.70)};
  cv::polylines(image, left, false, cv::Scalar(245, 245, 245), 5);

  const auto result = makeReconstructor().reconstruct(image);
  require(result.valid, "one trusted boundary must produce a reconstruction");
  const int near_row = metricPoint(1.0, 0.0).y;

  int run_count = 0;
  bool inside_run = false;
  const auto * pixels = result.reconstructed_mask.ptr<std::uint8_t>(near_row);
  for (int column = 0; column < result.reconstructed_mask.cols; ++column) {
    if (pixels[column] != 0U && !inside_run) {
      ++run_count;
      inside_run = true;
    } else if (pixels[column] == 0U) {
      inside_run = false;
    }
  }
  require(run_count == 2, "one detected boundary must yield two clean lanes");
}

void testLowSaturationGateIsOptional()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  const auto left = makeBoundary(0.30, 1.70);
  const auto right = makeBoundary(-0.30, 1.70);
  cv::polylines(
    image, std::vector<std::vector<cv::Point>>{left}, false,
    cv::Scalar(0, 0, 255), 5);
  cv::polylines(
    image, std::vector<std::vector<cv::Point>>{right}, false,
    cv::Scalar(0, 0, 255), 5);

  bev_processor::BevLaneReconstructorConfig config;
  config.minimum_brightness = 70;
  config.maximum_saturation = 80;
  config.expected_lane_width_m = 0.60;
  config.lane_width_tolerance_m = 0.12;
  bev_processor::BevLaneReconstructor reconstructor(config);
  const auto result = reconstructor.reconstruct(image);
  require(
    !result.valid,
    "the optional saturation gate must reject strongly colored markings");
}

}  // namespace

int main()
{
  testCurvesAreReconstructedPastTrustedRange();
  testSingleBoundaryStillReconstructsBothLanes();
  testLowSaturationGateIsOptional();
  std::cout << "BEV lane reconstructor tests passed\n";
  return EXIT_SUCCESS;
}
