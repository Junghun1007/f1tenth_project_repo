#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "bev_processor/bev_geometry.hpp"
#include "bev_processor/ground_plane_estimator.hpp"

namespace
{

constexpr double kRadiansToDegrees =
  180.0 / 3.141592653589793238462643383279502884;

bool require(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return condition;
}

cv::Vec3d upVector(
  const double roll_rad,
  const double pitch_down_rad)
{
  return cv::Vec3d(
    -std::sin(roll_rad) * std::cos(pitch_down_rad),
    -std::cos(roll_rad) * std::cos(pitch_down_rad),
    -std::sin(pitch_down_rad));
}

}  // namespace

int main()
{
  constexpr double expected_height_m = 0.17;
  constexpr double expected_roll_deg = 2.0;
  constexpr double expected_pitch_deg = 13.0;
  const double expected_roll_rad =
    bev_processor::degToRad(expected_roll_deg);
  const double expected_pitch_rad =
    bev_processor::degToRad(expected_pitch_deg);

  const auto rotation_vehicle_from_camera =
    bev_processor::mountRotationVehicleFromCamera(
    expected_roll_rad, expected_pitch_rad, 0.0);
  const auto rotation_camera_from_vehicle =
    rotation_vehicle_from_camera.t();
  const cv::Vec3d camera_position(0.0, 0.0, expected_height_m);

  std::vector<cv::Vec3d> points;
  for (int x_index = 0; x_index < 50; ++x_index) {
    const double x_m = 0.30 + static_cast<double>(x_index) * 0.045;
    for (int y_index = 0; y_index < 25; ++y_index) {
      const double y_m = -0.50 + static_cast<double>(y_index) * 0.04;
      cv::Vec3d point_camera =
        rotation_camera_from_vehicle *
        (cv::Vec3d(x_m, y_m, 0.0) - camera_position);
      const double noise_m =
        0.0015 * std::sin(
        static_cast<double>(x_index * 17 + y_index * 13));
      point_camera +=
        upVector(expected_roll_rad, expected_pitch_rad) * noise_m;
      points.push_back(point_camera);
    }
  }
  for (int index = 0; index < 250; ++index) {
    points.emplace_back(
      -0.8 + 0.006 * static_cast<double>(index),
      -0.5 + 0.004 * static_cast<double>(index % 80),
      0.4 + 0.007 * static_cast<double>(index % 120));
  }

  const cv::Vec3d biased_imu_reference =
    upVector(
    bev_processor::degToRad(-1.0),
    bev_processor::degToRad(18.5));
  bev_processor::GroundPlaneFitConfig config;
  config.ransac_iterations = 300;
  config.inlier_threshold_m = 0.006;
  config.minimum_inliers = 900U;
  config.minimum_inlier_ratio = 0.70;
  config.maximum_residual_mad_m = 0.003;
  config.maximum_reference_angle_deg = 15.0;

  std::string rejection;
  const auto estimate = bev_processor::fitGroundPlane(
    points, biased_imu_reference, config, &rejection);
  bool passed = true;
  passed &= require(estimate.has_value(), rejection.c_str());
  if (estimate) {
    passed &= require(
      std::abs(estimate->height_m - expected_height_m) < 0.002,
      "ground-plane height must recover the synthetic camera height");
    passed &= require(
      std::abs(
        estimate->roll_rad * kRadiansToDegrees -
        expected_roll_deg) < 0.15,
      "ground-plane roll must ignore the biased IMU reference");
    passed &= require(
      std::abs(
        estimate->pitch_down_rad * kRadiansToDegrees -
        expected_pitch_deg) < 0.15,
      "ground-plane pitch must ignore the biased IMU reference");
    passed &= require(
      estimate->inlier_ratio > 0.80,
      "ground-plane fit must retain most synthetic road points");
    passed &= require(
      estimate->reference_angle_deg > 4.0,
      "test must exercise a meaningful IMU-to-plane disagreement");
  }

  auto strict_config = config;
  strict_config.maximum_reference_angle_deg = 2.0;
  const auto rejected = bev_processor::fitGroundPlane(
    points, biased_imu_reference, strict_config, &rejection);
  passed &= require(
    !rejected.has_value(),
    "plane must be rejected when it disagrees with the IMU safety bound");

  if (!passed) {
    return EXIT_FAILURE;
  }
  std::cout << "ground_plane_estimator_test passed\n";
  return EXIT_SUCCESS;
}
