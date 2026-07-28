#include "bev_processor/imu_attitude_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bev_processor
{

namespace
{

constexpr double kRadiansToDegrees =
  180.0 / 3.141592653589793238462643383279502884;

bool finiteVector(const cv::Vec3d & value)
{
  return
    std::isfinite(value[0]) &&
    std::isfinite(value[1]) &&
    std::isfinite(value[2]);
}

cv::Vec3d normalized(const cv::Vec3d & value)
{
  const double norm = cv::norm(value);
  if (!finiteVector(value) || !std::isfinite(norm) || norm <= 1.0e-12) {
    throw std::invalid_argument("IMU attitude vector must be finite/non-zero");
  }
  return value / norm;
}

double angleDegrees(const cv::Vec3d & first, const cv::Vec3d & second)
{
  return std::acos(std::clamp(first.dot(second), -1.0, 1.0)) *
         kRadiansToDegrees;
}

cv::Vec3d integrateGyroscope(
  const cv::Vec3d & up_camera,
  const cv::Vec3d & angular_velocity_camera_radps,
  const double dt_sec)
{
  const double angular_speed = cv::norm(angular_velocity_camera_radps);
  if (angular_speed <= 1.0e-12) {
    return up_camera;
  }

  const cv::Vec3d axis = angular_velocity_camera_radps / angular_speed;
  const double angle = angular_speed * dt_sec;
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);

  // A world-fixed up vector expressed in the rotating camera frame evolves
  // with the inverse of the measured camera angular motion.
  return normalized(
    cosine * up_camera -
    sine * axis.cross(up_camera) +
    (1.0 - cosine) * axis.dot(up_camera) * axis);
}

bool validConfig(const ImuAttitudeTrackerConfig & config)
{
  return
    std::isfinite(config.minimum_acceleration_mps2) &&
    config.minimum_acceleration_mps2 > 0.0 &&
    std::isfinite(config.maximum_acceleration_mps2) &&
    config.maximum_acceleration_mps2 >
    config.minimum_acceleration_mps2 &&
    std::isfinite(config.acceleration_correction_time_constant_sec) &&
    config.acceleration_correction_time_constant_sec > 0.0 &&
    std::isfinite(config.acceleration_correction_gate_deg) &&
    config.acceleration_correction_gate_deg > 0.0 &&
    config.acceleration_correction_gate_deg < 90.0 &&
    std::isfinite(config.maximum_sample_interval_sec) &&
    config.maximum_sample_interval_sec > 0.0;
}

}  // namespace

ImuAttitudeTracker::ImuAttitudeTracker(
  const ImuAttitudeTrackerConfig & config)
: config_(config)
{
  if (!validConfig(config_)) {
    throw std::invalid_argument("invalid real-time IMU attitude configuration");
  }
}

std::optional<ImuAttitudeEstimate> ImuAttitudeTracker::update(
  const cv::Vec3d & acceleration_camera_mps2,
  const cv::Vec3d & angular_velocity_camera_radps,
  const double timestamp_sec)
{
  if (
    !finiteVector(acceleration_camera_mps2) ||
    !finiteVector(angular_velocity_camera_radps) ||
    !std::isfinite(timestamp_sec))
  {
    return std::nullopt;
  }

  const double acceleration_magnitude = cv::norm(
    acceleration_camera_mps2);
  const bool acceleration_valid =
    std::isfinite(acceleration_magnitude) &&
    acceleration_magnitude >= config_.minimum_acceleration_mps2 &&
    acceleration_magnitude <= config_.maximum_acceleration_mps2;

  if (!initialized_) {
    if (!acceleration_valid) {
      return std::nullopt;
    }
    up_camera_ = acceleration_camera_mps2 / acceleration_magnitude;
    last_timestamp_sec_ = timestamp_sec;
    initialized_ = true;
    return estimate(true);
  }

  const double dt_sec = timestamp_sec - last_timestamp_sec_;
  if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
    return std::nullopt;
  }
  last_timestamp_sec_ = timestamp_sec;

  if (dt_sec > config_.maximum_sample_interval_sec) {
    if (acceleration_valid) {
      up_camera_ = acceleration_camera_mps2 / acceleration_magnitude;
      return estimate(true);
    }
    return estimate(false);
  }

  up_camera_ = integrateGyroscope(
    up_camera_, angular_velocity_camera_radps, dt_sec);

  bool acceleration_correction_used = false;
  if (acceleration_valid) {
    const cv::Vec3d measured_up =
      acceleration_camera_mps2 / acceleration_magnitude;
    if (
      angleDegrees(up_camera_, measured_up) <=
      config_.acceleration_correction_gate_deg)
    {
      const double correction_gain =
        1.0 -
        std::exp(
        -dt_sec /
        config_.acceleration_correction_time_constant_sec);
      up_camera_ = normalized(
        (1.0 - correction_gain) * up_camera_ +
        correction_gain * measured_up);
      acceleration_correction_used = true;
    }
  }

  return estimate(acceleration_correction_used);
}

void ImuAttitudeTracker::reset()
{
  up_camera_ = cv::Vec3d(0.0, -1.0, 0.0);
  last_timestamp_sec_ = 0.0;
  initialized_ = false;
}

bool ImuAttitudeTracker::initialized() const
{
  return initialized_;
}

ImuAttitudeEstimate ImuAttitudeTracker::estimate(
  const bool acceleration_correction_used) const
{
  return ImuAttitudeEstimate{
    up_camera_,
    std::atan2(-up_camera_[0], -up_camera_[1]) *
    kRadiansToDegrees,
    std::atan2(
      -up_camera_[2],
      std::hypot(up_camera_[0], up_camera_[1])) *
    kRadiansToDegrees,
    acceleration_correction_used};
}

}  // namespace bev_processor
