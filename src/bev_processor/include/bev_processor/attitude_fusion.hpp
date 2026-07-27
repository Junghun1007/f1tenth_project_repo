#ifndef BEV_PROCESSOR__ATTITUDE_FUSION_HPP_
#define BEV_PROCESSOR__ATTITUDE_FUSION_HPP_

#include <optional>
#include <string>

#include <opencv2/core.hpp>

namespace bev_processor
{

enum class AttitudeFusionSource
{
  kFused,
  kImuSelected,
  kDepthSelected,
};

struct AttitudeFusionConfig
{
  double imu_roll_bias_deg{0.0};
  double imu_pitch_bias_deg{0.0};
  double imu_uncertainty_floor_deg{2.0};
  double depth_uncertainty_floor_deg{0.25};
  double agreement_gate_sigma{3.0};
  double minimum_agreement_gate_deg{1.0};
  double maximum_agreement_gate_deg{8.0};
  double minimum_dominance_ratio{4.0};
};

struct AttitudeFusionResult
{
  cv::Vec3d up_camera{0.0, -1.0, 0.0};
  AttitudeFusionSource source{AttitudeFusionSource::kFused};
  double roll_deg{0.0};
  double pitch_down_deg{0.0};
  double corrected_imu_roll_deg{0.0};
  double corrected_imu_pitch_down_deg{0.0};
  double imu_uncertainty_deg{0.0};
  double depth_uncertainty_deg{0.0};
  double imu_weight{0.0};
  double depth_weight{0.0};
  double disagreement_deg{0.0};
  double agreement_gate_deg{0.0};
};

cv::Vec3d attitudeUpVector(
  double roll_deg,
  double pitch_down_deg);

std::optional<AttitudeFusionResult> fuseAttitudeNormals(
  const cv::Vec3d & raw_imu_up_camera,
  double imu_direction_rms_deg,
  const cv::Vec3d & depth_up_camera,
  double depth_normal_rms_deg,
  const AttitudeFusionConfig & config,
  std::string * rejection = nullptr);

const char * attitudeFusionSourceName(AttitudeFusionSource source);

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__ATTITUDE_FUSION_HPP_
