#ifndef BEV_PROCESSOR__IMU_ATTITUDE_TRACKER_HPP_
#define BEV_PROCESSOR__IMU_ATTITUDE_TRACKER_HPP_

#include <optional>

#include <opencv2/core.hpp>

namespace bev_processor
{

struct ImuAttitudeTrackerConfig
{
  double minimum_acceleration_mps2{7.5};
  double maximum_acceleration_mps2{12.0};
  double acceleration_correction_time_constant_sec{1.5};
  double acceleration_correction_gate_deg{8.0};
  double maximum_sample_interval_sec{0.1};
};

struct ImuAttitudeEstimate
{
  cv::Vec3d up_camera{0.0, -1.0, 0.0};
  double roll_deg{0.0};
  double pitch_down_deg{0.0};
  bool acceleration_correction_used{false};
};

class ImuAttitudeTracker
{
public:
  explicit ImuAttitudeTracker(
    const ImuAttitudeTrackerConfig & config = {});

  std::optional<ImuAttitudeEstimate> update(
    const cv::Vec3d & acceleration_camera_mps2,
    const cv::Vec3d & angular_velocity_camera_radps,
    double timestamp_sec);

  void reset();
  bool initialized() const;

private:
  ImuAttitudeEstimate estimate(bool acceleration_correction_used) const;

  ImuAttitudeTrackerConfig config_;
  cv::Vec3d up_camera_{0.0, -1.0, 0.0};
  double last_timestamp_sec_{0.0};
  bool initialized_{false};
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__IMU_ATTITUDE_TRACKER_HPP_
