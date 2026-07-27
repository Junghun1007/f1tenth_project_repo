#include "bev_processor/attitude_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bev_processor
{

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kDegreesToRadians = kPi / 180.0;

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
    throw std::invalid_argument("attitude normal must be finite/non-zero");
  }
  return value / norm;
}

double rollDegrees(const cv::Vec3d & up)
{
  return std::atan2(-up[0], -up[1]) * kRadiansToDegrees;
}

double pitchDownDegrees(const cv::Vec3d & up)
{
  return std::atan2(
    -up[2], std::hypot(up[0], up[1])) * kRadiansToDegrees;
}

double angleDegrees(const cv::Vec3d & first, const cv::Vec3d & second)
{
  return std::acos(std::clamp(first.dot(second), -1.0, 1.0)) *
         kRadiansToDegrees;
}

bool validConfig(const AttitudeFusionConfig & config)
{
  return
    std::isfinite(config.imu_roll_bias_deg) &&
    std::isfinite(config.imu_pitch_bias_deg) &&
    std::isfinite(config.imu_uncertainty_floor_deg) &&
    config.imu_uncertainty_floor_deg > 0.0 &&
    std::isfinite(config.depth_uncertainty_floor_deg) &&
    config.depth_uncertainty_floor_deg > 0.0 &&
    std::isfinite(config.agreement_gate_sigma) &&
    config.agreement_gate_sigma > 0.0 &&
    std::isfinite(config.minimum_agreement_gate_deg) &&
    config.minimum_agreement_gate_deg > 0.0 &&
    std::isfinite(config.maximum_agreement_gate_deg) &&
    config.maximum_agreement_gate_deg >=
    config.minimum_agreement_gate_deg &&
    config.maximum_agreement_gate_deg < 90.0 &&
    std::isfinite(config.minimum_dominance_ratio) &&
    config.minimum_dominance_ratio > 1.0;
}

void reject(std::string * rejection, const char * reason)
{
  if (rejection != nullptr) {
    *rejection = reason;
  }
}

}  // namespace

cv::Vec3d attitudeUpVector(
  const double roll_deg,
  const double pitch_down_deg)
{
  const double roll_rad = roll_deg * kDegreesToRadians;
  const double pitch_rad = pitch_down_deg * kDegreesToRadians;
  return cv::Vec3d(
    -std::sin(roll_rad) * std::cos(pitch_rad),
    -std::cos(roll_rad) * std::cos(pitch_rad),
    -std::sin(pitch_rad));
}

std::optional<AttitudeFusionResult> fuseAttitudeNormals(
  const cv::Vec3d & raw_imu_up_camera,
  const double imu_direction_rms_deg,
  const cv::Vec3d & depth_up_camera,
  const double depth_normal_rms_deg,
  const AttitudeFusionConfig & config,
  std::string * rejection)
{
  if (!validConfig(config)) {
    throw std::invalid_argument("invalid attitude-fusion configuration");
  }
  if (
    !std::isfinite(imu_direction_rms_deg) ||
    imu_direction_rms_deg < 0.0 ||
    !std::isfinite(depth_normal_rms_deg) ||
    depth_normal_rms_deg < 0.0)
  {
    throw std::invalid_argument(
            "attitude uncertainty must be finite/non-negative");
  }

  const cv::Vec3d raw_imu_up = normalized(raw_imu_up_camera);
  const cv::Vec3d depth_up = normalized(depth_up_camera);
  const double corrected_imu_roll_deg =
    rollDegrees(raw_imu_up) - config.imu_roll_bias_deg;
  const double corrected_imu_pitch_down_deg =
    pitchDownDegrees(raw_imu_up) - config.imu_pitch_bias_deg;
  const cv::Vec3d corrected_imu_up = attitudeUpVector(
    corrected_imu_roll_deg, corrected_imu_pitch_down_deg);

  const double imu_uncertainty_deg = std::hypot(
    imu_direction_rms_deg, config.imu_uncertainty_floor_deg);
  const double depth_uncertainty_deg = std::hypot(
    depth_normal_rms_deg, config.depth_uncertainty_floor_deg);
  const double imu_information =
    1.0 / (imu_uncertainty_deg * imu_uncertainty_deg);
  const double depth_information =
    1.0 / (depth_uncertainty_deg * depth_uncertainty_deg);
  const double information_sum = imu_information + depth_information;
  const double imu_weight = imu_information / information_sum;
  const double depth_weight = depth_information / information_sum;

  const double disagreement_deg =
    angleDegrees(corrected_imu_up, depth_up);
  const double agreement_gate_deg = std::clamp(
    config.agreement_gate_sigma *
    std::hypot(imu_uncertainty_deg, depth_uncertainty_deg),
    config.minimum_agreement_gate_deg,
    config.maximum_agreement_gate_deg);

  cv::Vec3d result_up;
  AttitudeFusionSource source = AttitudeFusionSource::kFused;
  if (disagreement_deg <= agreement_gate_deg) {
    result_up = normalized(
      imu_weight * corrected_imu_up + depth_weight * depth_up);
  } else {
    const double depth_dominance =
      depth_information / imu_information;
    const double imu_dominance =
      imu_information / depth_information;
    if (depth_dominance >= config.minimum_dominance_ratio) {
      source = AttitudeFusionSource::kDepthSelected;
      result_up = depth_up;
    } else if (imu_dominance >= config.minimum_dominance_ratio) {
      source = AttitudeFusionSource::kImuSelected;
      result_up = corrected_imu_up;
    } else {
      reject(
        rejection,
        "IMU and depth attitudes disagree without a reliable winner");
      return std::nullopt;
    }
  }

  return AttitudeFusionResult{
    result_up,
    source,
    rollDegrees(result_up),
    pitchDownDegrees(result_up),
    corrected_imu_roll_deg,
    corrected_imu_pitch_down_deg,
    imu_uncertainty_deg,
    depth_uncertainty_deg,
    imu_weight,
    depth_weight,
    disagreement_deg,
    agreement_gate_deg};
}

const char * attitudeFusionSourceName(const AttitudeFusionSource source)
{
  switch (source) {
    case AttitudeFusionSource::kFused:
      return "imu_depth_fused";
    case AttitudeFusionSource::kImuSelected:
      return "imu_selected";
    case AttitudeFusionSource::kDepthSelected:
      return "depth_selected";
  }
  return "unknown";
}

}  // namespace bev_processor
