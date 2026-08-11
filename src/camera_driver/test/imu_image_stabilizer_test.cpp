#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>

#include <opencv2/core.hpp>

#include "camera_driver/imu_image_stabilizer.hpp"

namespace
{

constexpr double kDegreesToRadians =
  3.141592653589793238462643383279502884 / 180.0;

void require(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

camera_driver::ImuImageStabilizerConfig fastConfig()
{
  camera_driver::ImuImageStabilizerConfig config;
  config.startup_discard_duration_sec = 0.0;
  config.reference_calibration_duration_sec = 0.01;
  config.stationary_detection_window_sec = 0.01;
  config.maximum_frame_imu_wait_sec = 0.0001;
  config.maximum_frame_imu_age_sec = 0.006;
  config.maximum_frame_imu_prediction_sec = 0.015;
  return config;
}

void calibrate(camera_driver::ImuImageStabilizer & stabilizer)
{
  const cv::Vec3d startup_acceleration(0.0, -9.80665, 0.0);
  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      0.0025 * static_cast<double>(index));
  }
  require(stabilizer.initialized(), "stationary calibration did not finish");
}

void calibrateAtTilt(
  camera_driver::ImuImageStabilizer & stabilizer,
  const cv::Vec3d & startup_acceleration)
{
  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      0.0025 * static_cast<double>(index));
  }
  require(stabilizer.initialized(), "tilted calibration did not finish");
}

void verifyStartupSamplesAreDiscarded()
{
  auto config = fastConfig();
  config.startup_discard_duration_sec = 0.01;
  camera_driver::ImuImageStabilizer stabilizer(config);

  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 1.0),
      0.0025 * static_cast<double>(index));
  }
  const auto discarded = stabilizer.calibrationProgress();
  require(
    !discarded.discarding_startup_samples,
    "startup discard did not finish at its configured boundary");
  require(
    discarded.accepted_samples == 0U,
    "a startup-discard gyro sample leaked into calibration");

  for (int index = 1; index <= 6; ++index) {
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      0.01 + 0.0025 * static_cast<double>(index));
  }
  require(stabilizer.initialized(), "post-discard calibration did not finish");
  require(
    cv::norm(stabilizer.gyroscopeBiasRadps()) < 1.0e-12,
    "discarded startup gyro contaminated the learned bias");
}

void verifyLateralAccelerationDoesNotCreateRoll()
{
  const auto config = fastConfig();
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    stabilizer.update(
      cv::Vec3d(1.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec);
  }
  const auto correction = stabilizer.correctionAt(timestamp_sec);
  require(correction.has_value(), "lateral-acceleration lookup failed");
  require(
    std::abs(correction->roll_error_deg) < 1.0e-9,
    "lateral acceleration was incorrectly accumulated as roll");
}

void verifyPitchedUturnDoesNotCreateTilt()
{
  const auto config = fastConfig();
  camera_driver::ImuImageStabilizer stabilizer(config);
  constexpr double pitch_rad = 13.0 * kDegreesToRadians;
  const cv::Vec3d startup_acceleration(
    0.0,
    -9.80665 * std::cos(pitch_rad),
    -9.80665 * std::sin(pitch_rad));
  calibrateAtTilt(stabilizer, startup_acceleration);

  const cv::Vec3d yaw_axis = startup_acceleration / 9.80665;
  double timestamp_sec = 0.01;
  for (int index = 1; index <= 1260; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    stabilizer.update(std::nullopt, yaw_axis, timestamp_sec);
  }
  const auto correction = stabilizer.correctionAt(timestamp_sec);
  require(correction.has_value(), "pitched U-turn lookup failed");
  require(
    correction->correction_angle_deg < 1.0e-6,
    "gravity-axis U-turn leaked yaw into tilt");
}

void verifyStationaryRecoveryAndThreeAxisBiasUpdate()
{
  auto config = fastConfig();
  config.stationary_tilt_recovery_time_constant_sec = 0.02;
  config.online_gyroscope_tilt_bias_time_constant_sec = 0.02;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 40; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 1.0),
      timestamp_sec);
  }
  const auto drifted = stabilizer.correctionAt(timestamp_sec);
  require(drifted.has_value(), "drifted tilt lookup failed");
  require(
    std::abs(drifted->roll_error_deg) > 3.0,
    "synthetic gyro drift did not create the expected tilt error");

  const cv::Vec3d stationary_gyro_bias(0.0, 0.001, 0.0);
  for (int index = 1; index <= 200; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      stationary_gyro_bias,
      timestamp_sec,
      true);
  }
  const auto recovered = stabilizer.correctionAt(timestamp_sec);
  require(recovered.has_value(), "stationary recovery lookup failed");
  require(
    std::abs(recovered->roll_error_deg) < 0.1,
    "stationary recovery did not restore the startup view");
  require(
    stabilizer.gyroscopeBiasRadps()[1] > 0.0005,
    "stationary online bias update did not include camera Y");
  require(
    stabilizer.onlineTiltBiasUpdateCount() > 0U,
    "stationary online bias update was never activated");
}

void verifyReferenceLeakBoundsGyroDrift()
{
  auto config = fastConfig();
  config.reference_tilt_leak_time_constant_sec = 0.02;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 0.01),
      timestamp_sec);
  }
  const auto correction = stabilizer.correctionAt(timestamp_sec);
  require(correction.has_value(), "reference-leak lookup failed");
  require(
    std::abs(correction->roll_error_deg) < 0.05,
    "startup-reference leak did not bound accumulated gyro drift");
}

void verifyMotionCompensatedGravityCorrectsMovingGyroDrift()
{
  auto config = fastConfig();
  config.acceleration_correction_stationary_only = false;
  config.roll_acceleration_correction_time_constant_sec = 0.02;
  config.roll_acceleration_direction_gate_deg = 10.0;
  config.reference_tilt_leak_time_constant_sec = 1000.0;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.01),
      timestamp_sec,
      false);
  }
  const auto correction = stabilizer.correctionAt(timestamp_sec);
  require(correction.has_value(), "moving gravity correction lookup failed");
  require(
    std::abs(correction->roll_error_deg) < 0.1,
    "motion-compensated gravity did not bound moving gyro drift");
}

void verifyExternalVehicleStationaryControlsRecovery()
{
  auto config = fastConfig();
  config.stationary_tilt_recovery_time_constant_sec = 0.02;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 40; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 1.0),
      timestamp_sec,
      false);
  }
  const auto drifted = stabilizer.correctionAt(timestamp_sec);
  require(drifted.has_value(), "ERPM-moving tilt lookup failed");
  require(
    std::abs(drifted->roll_error_deg) > 3.0,
    "vehicle moving state unexpectedly activated stationary recovery");

  for (int index = 1; index <= 80; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      true);
  }
  const auto recovered = stabilizer.correctionAt(timestamp_sec);
  require(recovered.has_value(), "ERPM-stationary recovery lookup failed");
  require(
    std::abs(recovered->roll_error_deg) < 0.1,
    "ERPM stationary state did not restore the startup view");
  require(
    stabilizer.stationaryConfirmed(),
    "external vehicle stationary state was not reported");
}

void verifyExternalBevReferenceIsSharedWithErpmStationaryRecovery()
{
  auto config = fastConfig();
  config.external_reference_required = true;
  config.stationary_tilt_recovery_time_constant_sec = 0.02;
  camera_driver::ImuImageStabilizer stabilizer(config);
  const cv::Vec3d startup_acceleration(0.0, -9.80665, 0.0);

  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      0.0025 * static_cast<double>(index));
  }
  require(
    !stabilizer.initialized(),
    "required BEV reference was silently replaced by the IMU reference");

  constexpr double depth_roll_rad = 5.0 * kDegreesToRadians;
  const cv::Vec3d depth_up_camera(
    -std::sin(depth_roll_rad),
    -std::cos(depth_roll_rad),
    0.0);
  require(
    stabilizer.setExternalReferenceUpCamera(depth_up_camera),
    "valid BEV ground reference was rejected");
  require(
    stabilizer.externalReferenceReceived(),
    "BEV ground reference receipt was not recorded");

  for (int index = 1; index <= 6; ++index) {
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      0.01 + 0.0025 * static_cast<double>(index));
  }
  require(
    stabilizer.initialized(),
    "calibration did not finish after the BEV reference arrived");

  double timestamp_sec = 0.025;
  const auto startup_correction = stabilizer.correctionAt(timestamp_sec);
  require(
    startup_correction.has_value() &&
    startup_correction->correction_angle_deg < 1.0e-9,
    "shared BEV startup reference produced a non-identity startup warp");

  for (int index = 0; index < 40; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      std::nullopt, cv::Vec3d(0.0, 0.0, 1.0), timestamp_sec);
  }
  for (int index = 0; index < 200; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      true);
  }
  const auto recovered = stabilizer.correctionAt(timestamp_sec);
  require(
    stabilizer.stationaryConfirmed(),
    "ERPM stationary state was not accepted with a different depth reference");
  require(
    recovered.has_value() && std::abs(recovered->roll_error_deg) < 0.1,
    "stationary recovery did not return to the shared BEV reference");
}

}  // namespace

int main()
{
  verifyStartupSamplesAreDiscarded();
  verifyLateralAccelerationDoesNotCreateRoll();
  verifyPitchedUturnDoesNotCreateTilt();
  verifyStationaryRecoveryAndThreeAxisBiasUpdate();
  verifyExternalVehicleStationaryControlsRecovery();
  verifyReferenceLeakBoundsGyroDrift();
  verifyMotionCompensatedGravityCorrectsMovingGyroDrift();
  verifyExternalBevReferenceIsSharedWithErpmStationaryRecovery();

  const auto config = fastConfig();
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(0.0, 0.0, 1.0),
    0.0125);
  const auto roll_correction = stabilizer.correctionAt(0.0125);
  require(roll_correction.has_value(), "roll correction lookup failed");
  require(
    std::abs(roll_correction->roll_error_deg) > 0.1,
    "camera-Z rotation did not move the fixed roll reference");
  require(
    !roll_correction->predicted,
    "exact timestamp was incorrectly marked as predicted");

  const auto predicted = stabilizer.correctionAt(0.0200);
  require(predicted.has_value(), "short gyro prediction failed");
  require(predicted->predicted, "future frame did not use gyro prediction");
  require(
    std::abs(predicted->prediction_horizon_sec - 0.0075) < 1.0e-9,
    "prediction horizon is incorrect");
  require(
    !stabilizer.correctionAt(0.0300).has_value(),
    "prediction exceeded its configured time limit");

  const auto homography = camera_driver::makeImageStabilizationHomography(
    500.0, 500.0, 640.0, 360.0, *roll_correction);
  require(
    cv::checkRange(cv::Mat(homography)),
    "fixed-reference homography is not finite");

  camera_driver::ImuImageStabilizer pitch_stabilizer(config);
  calibrate(pitch_stabilizer);
  pitch_stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(-1.0, 0.0, 0.0),
    0.0125);
  const auto pitch_correction = pitch_stabilizer.correctionAt(0.0125);
  require(pitch_correction.has_value(), "pitch correction lookup failed");
  require(
    pitch_correction->pitch_error_deg > 0.1,
    "negative camera-X gyro must produce positive downward pitch");

  const auto pitch_homography =
    camera_driver::makeImageStabilizationHomography(
    500.0, 500.0, 640.0, 360.0, *pitch_correction);
  const cv::Vec3d current_horizon_ray(
    0.0,
    -std::sin(pitch_correction->pitch_error_deg * kDegreesToRadians),
    std::cos(pitch_correction->pitch_error_deg * kDegreesToRadians));
  const cv::Vec3d current_horizon_pixel(
    500.0 * current_horizon_ray[0] + 640.0 * current_horizon_ray[2],
    500.0 * current_horizon_ray[1] + 360.0 * current_horizon_ray[2],
    current_horizon_ray[2]);
  const cv::Vec3d stabilized_horizon_pixel =
    pitch_homography * current_horizon_pixel;
  require(
    std::abs(
      stabilized_horizon_pixel[1] / stabilized_horizon_pixel[2] - 360.0) <
    1.0e-5,
    "pitch homography did not restore the startup horizon");

  camera_driver::ImuImageStabilizer yaw_stabilizer(config);
  calibrate(yaw_stabilizer);
  yaw_stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(0.0, 1.0, 0.0),
    0.0125);
  const auto yaw_correction = yaw_stabilizer.correctionAt(0.0125);
  require(yaw_correction.has_value(), "yaw attitude lookup failed");
  require(
    yaw_correction->correction_angle_deg < 1.0e-6,
    "gravity-axis yaw must not be stabilized");

  const cv::Matx33d camera_matrix(
    500.0, 0.0, 640.0,
    0.0, 500.0, 360.0,
    0.0, 0.0, 1.0);
  const auto zoom = camera_driver::makeFixedViewZoomHomography(
    camera_matrix, 1.25);
  require(
    camera_driver::outputIsCoveredBySource(
      zoom, cv::Size(1280, 720), cv::Size(1280, 720), 1.5),
    "fixed 1.25x view is not covered by the source image");

  auto moving_config = fastConfig();
  moving_config.calibration_maximum_angular_speed_degps = 0.5;
  camera_driver::ImuImageStabilizer moving_stabilizer(moving_config);
  for (int index = 0; index <= 8; ++index) {
    moving_stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.1),
      0.0025 * static_cast<double>(index));
  }
  require(
    !moving_stabilizer.initialized(),
    "moving camera was accepted as the fixed startup reference");
  require(
    moving_stabilizer.calibrationProgress().reset_count > 0U,
    "moving calibration did not report a reset");

  std::cout << "imu_image_stabilizer_test passed\n";
  return 0;
}
