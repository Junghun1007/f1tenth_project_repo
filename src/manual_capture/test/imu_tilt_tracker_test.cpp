#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "manual_capture/imu_tilt_tracker.hpp"
#include "manual_capture/startup_attitude.hpp"

using namespace manual_capture;

namespace
{

constexpr double kG = 9.80665;
constexpr double kDt = 0.005;  // 200 Hz

ImuTiltTrackerConfig fastConfig()
{
  ImuTiltTrackerConfig config;
  config.stabilizer.startup_discard_duration_sec = 0.0;
  config.stabilizer.reference_calibration_duration_sec = 0.05;
  config.stabilizer.stationary_detection_window_sec = 0.05;
  config.stabilizer.maximum_frame_imu_wait_sec = 0.0001;
  return config;
}

ImuSample sample(const cv::Vec3d & up, const cv::Vec3d & gyro, double t)
{
  return ImuSample{up * kG, gyro, t, t};
}

// Stationary on flat ground at the measured pose; ERPM 0 so standstill is known.
double calibrate(ImuTiltTracker & tracker, const cv::Vec3d & up, double t = 1.0)
{
  for (int i = 0; i < 300; ++i, t += kDt) {
    tracker.onMeasuredErpm(0, t);
    tracker.processBatch({sample(up, cv::Vec3d(0, 0, 0), t)}, t);
  }
  return t;
}

}  // namespace

TEST(ImuTiltTracker, ErpmStandstillNeedsOneSecondAndExitsImmediately)
{
  ImuTiltTracker tracker(fastConfig());
  double t = 10.0;
  tracker.onMeasuredErpm(0, t);
  EXPECT_FALSE(tracker.vehicleStationary(t));
  for (; t < 10.9; t += 0.05) {tracker.onMeasuredErpm(20, t);}
  EXPECT_FALSE(tracker.vehicleStationary(t));
  for (; t < 11.2; t += 0.05) {tracker.onMeasuredErpm(20, t);}
  EXPECT_TRUE(tracker.vehicleStationary(t));
  tracker.onMeasuredErpm(800, t + 0.05);  // departure: raw ERPM, no filter delay
  EXPECT_FALSE(tracker.vehicleStationary(t + 0.05));
  // Telemetry timeout is never standstill.
  for (t += 0.1; t < 13.0; t += 0.05) {tracker.onMeasuredErpm(0, t);}
  EXPECT_TRUE(tracker.vehicleStationary(t));
  EXPECT_FALSE(tracker.vehicleStationary(t + 1.5));
}

TEST(ImuTiltTracker, RejectsInvalidConfiguration)
{
  auto config = fastConfig();
  config.lever_arm_m = 2.0;
  EXPECT_THROW(ImuTiltTracker{config}, std::invalid_argument);
  config = fastConfig();
  config.stationary_erpm_exit_threshold = config.stationary_erpm_enter_threshold;
  EXPECT_THROW(ImuTiltTracker{config}, std::invalid_argument);
  config = fastConfig();
  config.enabled = false;
  config.lever_arm_m = 2.0;  // ignored when disabled
  EXPECT_NO_THROW(ImuTiltTracker{config});
}

TEST(ImuTiltTracker, OffUntilStreamingAndCalibratingBeforeReference)
{
  ImuTiltTracker tracker(fastConfig());
  EXPECT_EQ(tracker.tiltAt(1.0).kind, FrameTiltKind::OFF);
  tracker.setImuStreaming(true);
  EXPECT_EQ(tracker.tiltAt(1.0).kind, FrameTiltKind::CALIBRATING);
  tracker.setApply(false);
  EXPECT_EQ(tracker.tiltAt(1.0).kind, FrameTiltKind::OFF);
}

TEST(ImuTiltTracker, MeasuredGroundIsTheReferenceAndGyroTiltIsReported)
{
  ImuTiltTracker tracker(fastConfig());
  tracker.setImuStreaming(true);
  const cv::Vec3d up = attitudeUpVector(0.5, 14.0);
  ASSERT_TRUE(tracker.setGroundReference(up));
  double t = calibrate(tracker, up);
  ASSERT_TRUE(tracker.status().calibrated);
  EXPECT_TRUE(tracker.status().reference_from_ground);

  auto still = tracker.tiltAt(t - kDt);
  ASSERT_TRUE(still.applied());
  EXPECT_LT(still.angle_deg, 0.05);

  // Driving (ERPM high, no CAN acceleration -> accelerometer ignored): the camera
  // pitches by ~1 degree about its optical X axis; only the gyro sees it.
  const double rate = 2.0 * M_PI / 180.0;
  for (int i = 0; i < 100; ++i, t += kDt) {
    tracker.onMeasuredErpm(3000, t);
    tracker.processBatch({sample(up, cv::Vec3d(rate, 0, 0), t)}, t);
  }
  const auto pitched = tracker.tiltAt(t - kDt);
  ASSERT_TRUE(pitched.applied());
  EXPECT_GT(std::abs(pitched.pitch_deg), 0.6);
  EXPECT_LT(std::abs(pitched.pitch_deg), 1.2);
  EXPECT_LT(std::abs(pitched.roll_deg), 0.2);
  EXPECT_GT(tracker.status().motion_misses, 0U);
}

TEST(ImuTiltTracker, RefitRestartsCalibrationWithTheNewGround)
{
  ImuTiltTracker tracker(fastConfig());
  tracker.setImuStreaming(true);
  const cv::Vec3d up = attitudeUpVector(0.0, 14.0);
  ASSERT_TRUE(tracker.setGroundReference(up));
  double t = calibrate(tracker, up);
  ASSERT_TRUE(tracker.status().calibrated);

  const cv::Vec3d refit_up = attitudeUpVector(1.0, 12.0);
  ASSERT_TRUE(tracker.setGroundReference(refit_up));
  EXPECT_FALSE(tracker.status().calibrated);
  EXPECT_EQ(tracker.tiltAt(t).kind, FrameTiltKind::CALIBRATING);
  t = calibrate(tracker, refit_up, t + 5.0);  // camera restarted after the measurement
  ASSERT_TRUE(tracker.status().calibrated);
  const auto tilt = tracker.tiltAt(t - kDt);
  ASSERT_TRUE(tilt.applied());
  EXPECT_LT(tilt.angle_deg, 0.05);
}

namespace
{
// Accelerating at 0.5 m/s^2 forward (inside the 4.3 degree accelerometer gate):
// the accelerometer reads gravity + forward acceleration.
// reported_mps2 is what vehicle_dynamics_monitor sends.
FrameTilt driveAccelerating(double reported_mps2, std::uint64_t * compensated = nullptr)
{
  auto config = fastConfig();
  config.lever_arm_m = 0.0;
  ImuTiltTracker tracker(config);
  tracker.setImuStreaming(true);
  const cv::Vec3d up = attitudeUpVector(0.0, 14.0);
  EXPECT_TRUE(tracker.setGroundReference(up));
  double t = calibrate(tracker, up);
  cv::Vec3d forward = cv::Vec3d(0, 0, 1) - up * up[2];
  forward /= cv::norm(forward);
  for (int i = 0; i < 400; ++i, t += kDt) {
    tracker.onMeasuredErpm(3000, t);
    tracker.onVehicleAcceleration(t, reported_mps2, 0.0);
    tracker.processBatch({ImuSample{up * kG + forward * 0.5, cv::Vec3d(0, 0, 0), t, t}}, t);
  }
  if (compensated) {*compensated = tracker.status().motion_compensated;}
  return tracker.tiltAt(t - kDt);
}
}  // namespace

TEST(ImuTiltTracker, CanAccelerationIsRemovedWhileDriving)
{
  std::uint64_t compensated = 0U;
  const auto tilt = driveAccelerating(0.5, &compensated);
  ASSERT_TRUE(tilt.applied());
  EXPECT_LT(tilt.angle_deg, 0.05);
  EXPECT_GT(compensated, 300U);
  // Negative control: without the CAN value the forward acceleration leaks in as pitch.
  const auto uncompensated = driveAccelerating(0.0);
  EXPECT_GT(uncompensated.angle_deg, 0.15) << uncompensated.angle_deg;
}
