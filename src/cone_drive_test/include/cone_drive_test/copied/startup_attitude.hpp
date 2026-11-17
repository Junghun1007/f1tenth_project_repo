// Copied from src/oak_startup/include/oak_startup/startup_attitude.hpp (origin/1113ML)
#ifndef CONE_DRIVE_TEST_COPIED__STARTUP_ATTITUDE_HPP_
#define CONE_DRIVE_TEST_COPIED__STARTUP_ATTITUDE_HPP_

#include <string>

#include <opencv2/core.hpp>

namespace cone_drive_test
{

enum class StartupAttitudeSource
{
  kDepth,
  kImu,
};

struct StartupAttitudeSelection
{
  StartupAttitudeSource source{StartupAttitudeSource::kDepth};
  double roll_deg{0.0};
  double pitch_down_deg{0.0};
  double corrected_imu_roll_deg{0.0};
  double corrected_imu_pitch_down_deg{0.0};
  double depth_roll_deg{0.0};
  double depth_pitch_down_deg{0.0};
  double imu_depth_difference_deg{0.0};
};

StartupAttitudeSource parseStartupAttitudeSource(const std::string & value);

const char * startupAttitudeSourceName(StartupAttitudeSource source);

cv::Vec3d attitudeUpVector(
  double roll_deg,
  double pitch_down_deg);

StartupAttitudeSelection selectStartupAttitude(
  const cv::Vec3d & raw_imu_up_camera,
  const cv::Vec3d & depth_up_camera,
  StartupAttitudeSource source,
  double imu_roll_bias_deg,
  double imu_pitch_bias_deg);

}  // namespace cone_drive_test

#endif  // CONE_DRIVE_TEST_COPIED__STARTUP_ATTITUDE_HPP_
