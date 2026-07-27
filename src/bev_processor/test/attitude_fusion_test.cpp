#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "bev_processor/attitude_fusion.hpp"

namespace
{

bool require(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return condition;
}

}  // namespace

int main()
{
  bool passed = true;
  std::string rejection;

  bev_processor::AttitudeFusionConfig config;
  const auto close_result = bev_processor::fuseAttitudeNormals(
    bev_processor::attitudeUpVector(2.2, 13.4),
    0.10,
    bev_processor::attitudeUpVector(2.0, 13.0),
    0.15,
    config,
    &rejection);
  passed &= require(close_result.has_value(), rejection.c_str());
  if (close_result) {
    passed &= require(
      close_result->source ==
      bev_processor::AttitudeFusionSource::kFused,
      "close observations must be fused");
    passed &= require(
      close_result->depth_weight > 0.95,
      "default systematic floor must make the road plane dominant");
    passed &= require(
      close_result->pitch_down_deg > 13.0 &&
      close_result->pitch_down_deg < 13.1,
      "fused pitch must remain near the more reliable depth plane");
  }

  const auto uncalibrated_result = bev_processor::fuseAttitudeNormals(
    bev_processor::attitudeUpVector(0.0, 18.8),
    0.10,
    bev_processor::attitudeUpVector(0.0, 13.0),
    0.10,
    config,
    &rejection);
  passed &= require(uncalibrated_result.has_value(), rejection.c_str());
  if (uncalibrated_result) {
    passed &= require(
      uncalibrated_result->depth_weight > 0.98,
      "uncalibrated IMU mounting uncertainty must favor the depth plane");
    passed &= require(
      std::abs(uncalibrated_result->pitch_down_deg - 13.0) < 0.15,
      "an uncalibrated IMU must not pull a confident plane far from truth");
  }

  auto bias_config = config;
  bias_config.imu_pitch_bias_deg = 5.8;
  const auto bias_result = bev_processor::fuseAttitudeNormals(
    bev_processor::attitudeUpVector(0.0, 18.8),
    0.10,
    bev_processor::attitudeUpVector(0.0, 13.0),
    0.10,
    bias_config,
    &rejection);
  passed &= require(bias_result.has_value(), rejection.c_str());
  if (bias_result) {
    passed &= require(
      std::abs(bias_result->corrected_imu_pitch_down_deg - 13.0) < 1.0e-6,
      "configured IMU pitch bias must be removed");
    passed &= require(
      std::abs(bias_result->pitch_down_deg - 13.0) < 1.0e-6,
      "matching corrected observations must preserve the true pitch");
  }

  const auto depth_selected = bev_processor::fuseAttitudeNormals(
    bev_processor::attitudeUpVector(0.0, 20.0),
    0.10,
    bev_processor::attitudeUpVector(0.0, 13.0),
    0.10,
    config,
    &rejection);
  passed &= require(depth_selected.has_value(), rejection.c_str());
  if (depth_selected) {
    passed &= require(
      depth_selected->source ==
      bev_processor::AttitudeFusionSource::kDepthSelected,
      "a confident depth plane must win a significant disagreement");
    passed &= require(
      std::abs(depth_selected->pitch_down_deg - 13.0) < 1.0e-6,
      "depth-selected result must use the depth attitude exactly");
  }

  auto imu_dominant_config = config;
  imu_dominant_config.imu_uncertainty_floor_deg = 0.10;
  imu_dominant_config.depth_uncertainty_floor_deg = 2.0;
  const auto imu_selected = bev_processor::fuseAttitudeNormals(
    bev_processor::attitudeUpVector(0.0, 13.0),
    0.05,
    bev_processor::attitudeUpVector(0.0, 23.0),
    0.20,
    imu_dominant_config,
    &rejection);
  passed &= require(imu_selected.has_value(), rejection.c_str());
  if (imu_selected) {
    passed &= require(
      imu_selected->source ==
      bev_processor::AttitudeFusionSource::kImuSelected,
      "a calibrated dominant IMU must win an unreliable depth conflict");
    passed &= require(
      std::abs(imu_selected->pitch_down_deg - 13.0) < 1.0e-6,
      "IMU-selected result must use the corrected IMU attitude exactly");
  }

  auto equal_config = config;
  equal_config.imu_uncertainty_floor_deg = 0.5;
  equal_config.depth_uncertainty_floor_deg = 0.5;
  const auto rejected = bev_processor::fuseAttitudeNormals(
    bev_processor::attitudeUpVector(0.0, 5.0),
    0.10,
    bev_processor::attitudeUpVector(0.0, 15.0),
    0.10,
    equal_config,
    &rejection);
  passed &= require(
    !rejected.has_value(),
    "large disagreement without a dominant sensor must be rejected");

  if (!passed) {
    return EXIT_FAILURE;
  }
  std::cout << "attitude_fusion_test passed\n";
  return EXIT_SUCCESS;
}
