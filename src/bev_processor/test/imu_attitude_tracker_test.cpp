#include <cmath>
#include <cstdlib>
#include <iostream>

#include "bev_processor/attitude_fusion.hpp"
#include "bev_processor/imu_attitude_tracker.hpp"

namespace
{

constexpr double kDegreesToRadians =
  3.141592653589793238462643383279502884 / 180.0;
constexpr double kGravityMps2 = 9.80665;

bool require(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return condition;
}

cv::Vec3d accelerationFor(const double roll_deg, const double pitch_deg)
{
  return
    kGravityMps2 *
    bev_processor::attitudeUpVector(roll_deg, pitch_deg);
}

}  // namespace

int main()
{
  bool passed = true;

  bev_processor::ImuAttitudeTracker pitch_tracker;
  auto estimate = pitch_tracker.update(
    accelerationFor(0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0), 0.0);
  passed &= require(estimate.has_value(), "level initialization must succeed");

  for (int index = 1; index <= 100; ++index) {
    const double timestamp_sec = static_cast<double>(index) * 0.01;
    const double pitch_deg = 10.0 * timestamp_sec;
    estimate = pitch_tracker.update(
      accelerationFor(0.0, pitch_deg),
      cv::Vec3d(-10.0 * kDegreesToRadians, 0.0, 0.0),
      timestamp_sec);
  }
  passed &= require(estimate.has_value(), "pitch tracking must produce output");
  if (estimate) {
    passed &= require(
      std::abs(estimate->pitch_down_deg - 10.0) < 0.05,
      "camera-X gyro integration must track downward pitch");
  }

  bev_processor::ImuAttitudeTracker roll_tracker;
  estimate = roll_tracker.update(
    accelerationFor(0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0), 0.0);
  for (int index = 1; index <= 100; ++index) {
    const double timestamp_sec = static_cast<double>(index) * 0.01;
    const double roll_deg = 5.0 * timestamp_sec;
    estimate = roll_tracker.update(
      accelerationFor(roll_deg, 0.0),
      cv::Vec3d(0.0, 0.0, 5.0 * kDegreesToRadians),
      timestamp_sec);
  }
  passed &= require(estimate.has_value(), "roll tracking must produce output");
  if (estimate) {
    passed &= require(
      std::abs(estimate->roll_deg - 5.0) < 0.05,
      "camera-Z gyro integration must track positive roll");
  }

  bev_processor::ImuAttitudeTracker outlier_tracker;
  estimate = outlier_tracker.update(
    accelerationFor(0.0, 13.0), cv::Vec3d(0.0, 0.0, 0.0), 1.0);
  estimate = outlier_tracker.update(
    accelerationFor(0.0, 25.0), cv::Vec3d(0.0, 0.0, 0.0), 1.01);
  passed &= require(estimate.has_value(), "outlier tracking must produce output");
  if (estimate) {
    passed &= require(
      std::abs(estimate->pitch_down_deg - 13.0) < 0.05,
      "acceleration direction outside the gate must not tilt the estimate");
    passed &= require(
      !estimate->acceleration_correction_used,
      "rejected acceleration must be reported");
  }

  bev_processor::ImuAttitudeTracker invalid_tracker;
  estimate = invalid_tracker.update(
    cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0), 0.0);
  passed &= require(
    !estimate.has_value(),
    "tracker must reject initialization without gravity magnitude");

  if (!passed) {
    return EXIT_FAILURE;
  }
  std::cout << "imu_attitude_tracker_test passed\n";
  return EXIT_SUCCESS;
}
