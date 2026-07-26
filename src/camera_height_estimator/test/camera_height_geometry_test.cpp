#include "camera_height_estimator/camera_height_geometry.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace
{

constexpr double kDegreesToRadians =
  3.141592653589793238462643383279502884 / 180.0;

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
  using camera_height_estimator::CameraIntrinsics;
  using camera_height_estimator::DepthSample;
  using camera_height_estimator::HeightDiagnostics;
  using camera_height_estimator::HeightGeometryConfig;
  using camera_height_estimator::HeightRejection;

  const CameraIntrinsics intrinsics{400.0, 400.0, 319.5, 199.5};
  const double roll_rad = 3.0 * kDegreesToRadians;
  const double pitch_rad = 14.0 * kDegreesToRadians;
  const std::array<double, 3> specific_force{
    -std::sin(roll_rad) * std::cos(pitch_rad),
    -std::cos(roll_rad) * std::cos(pitch_rad),
    -std::sin(pitch_rad)};
  constexpr double expected_height_m = 0.20;

  std::vector<DepthSample> ground_samples;
  for (int v = 195; v < 205; ++v) {
    for (int u = 315; u < 325; ++u) {
      const double ray_x =
        (static_cast<double>(u) - intrinsics.cx) / intrinsics.fx;
      const double ray_y =
        (static_cast<double>(v) - intrinsics.cy) / intrinsics.fy;
      const double height_per_z = -(
        specific_force[0] * ray_x +
        specific_force[1] * ray_y +
        specific_force[2]);
      const double depth_noise_m =
        static_cast<double>((u + 3 * v) % 5 - 2) * 0.0005;
      ground_samples.push_back(DepthSample{
        static_cast<double>(u),
        static_cast<double>(v),
        expected_height_m / height_per_z + depth_noise_m});
    }
  }

  bool passed = true;
  const auto estimate = camera_height_estimator::estimateCameraHeight(
    ground_samples, intrinsics, specific_force, HeightGeometryConfig{});
  passed &= require(
    estimate.has_value(),
    "synthetic ground patch must be accepted");
  if (estimate) {
    passed &= require(
      std::abs(estimate->height_m - expected_height_m) < 0.001,
      "estimated height must match the synthetic geometry");
    passed &= require(
      estimate->valid_pixel_count == 100U,
      "all synthetic center pixels must be valid");
  }

  auto inconsistent_samples = ground_samples;
  for (std::size_t index = 0; index < inconsistent_samples.size(); ++index) {
    if ((index % 2U) == 0U) {
      inconsistent_samples[index].z_m += 0.50;
    }
  }
  HeightDiagnostics inconsistent_diagnostics;
  const auto inconsistent =
    camera_height_estimator::estimateCameraHeight(
    inconsistent_samples, intrinsics, specific_force,
    HeightGeometryConfig{}, &inconsistent_diagnostics);
  passed &= require(
    !inconsistent.has_value(),
    "an inconsistent center region must be rejected");
  passed &= require(
    inconsistent_diagnostics.rejection ==
    HeightRejection::INCONSISTENT_HEIGHTS,
    "spatial inconsistency must report the correct rejection");

  HeightDiagnostics horizontal_diagnostics;
  const auto horizontal = camera_height_estimator::estimateCameraHeight(
    ground_samples, intrinsics, {0.0, -1.0, 0.0},
    HeightGeometryConfig{}, &horizontal_diagnostics);
  passed &= require(
    !horizontal.has_value(),
    "an almost-horizontal center ray must be rejected");
  passed &= require(
    horizontal_diagnostics.rejection ==
    HeightRejection::CENTER_RAY_NOT_DOWNWARD,
    "horizontal-ray rejection must report the correct reason");

  std::vector<DepthSample> too_few_samples(
    ground_samples.begin(), ground_samples.begin() + 10);
  HeightDiagnostics count_diagnostics;
  const auto too_few = camera_height_estimator::estimateCameraHeight(
    too_few_samples, intrinsics, specific_force,
    HeightGeometryConfig{}, &count_diagnostics);
  passed &= require(
    !too_few.has_value(),
    "too few valid pixels must be rejected");
  passed &= require(
    count_diagnostics.rejection ==
    HeightRejection::INSUFFICIENT_VALID_PIXELS,
    "pixel-count rejection must report the correct reason");

  if (!passed) {
    return EXIT_FAILURE;
  }
  std::cout << "camera_height_geometry_test passed\n";
  return EXIT_SUCCESS;
}
