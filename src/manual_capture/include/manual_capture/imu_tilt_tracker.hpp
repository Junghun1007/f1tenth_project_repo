#ifndef MANUAL_CAPTURE__IMU_TILT_TRACKER_HPP_
#define MANUAL_CAPTURE__IMU_TILT_TRACKER_HPP_

// IMU pitch/roll correction for the BEV, same model as camera_driver (auto_control):
// the startup/Refit ground normal is the fixed reference, the gyro tracks the
// camera tilt relative to it, and CAN vehicle acceleration (vehicle_dynamics_monitor)
// plus the lever-arm terms are removed from the accelerometer before it is used
// as a gravity anchor. Vehicle standstill comes from measured ERPM.
// Camera/Qt/ROS independent; every method is thread safe.

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "manual_capture/imu_image_stabilizer.hpp"

namespace manual_capture
{

struct ImuTiltTrackerConfig
{
  bool enabled{true};
  double imu_rate_hz{200.0};
  int imu_queue_size{80};
  int imu_max_batch_reports{5};
  double maximum_accel_gyro_timestamp_skew_sec{0.003};
  double maximum_timestamp_domain_delta_sec{1.0};

  // Standstill from |ERPM| (filtered for entry, raw for exit).
  int stationary_erpm_enter_threshold{100};
  int stationary_erpm_exit_threshold{500};
  double stationary_erpm_filter_time_constant_sec{0.15};
  double stationary_erpm_enter_duration_sec{1.0};
  double measured_erpm_timeout_sec{1.0};

  // CAN vehicle acceleration (base_link X forward, Y left) removed from the IMU.
  bool vehicle_motion_compensation_enabled{true};
  double can_acceleration_timeout_sec{0.10};
  double can_longitudinal_compensation_gain{1.0};
  double can_lateral_compensation_gain{1.0};
  double maximum_longitudinal_acceleration_mps2{15.0};
  double maximum_lateral_acceleration_mps2{15.0};
  // Camera distance in front of the rotation center (rear axle): -w^2 r forward, (dw/dt) r left.
  double lever_arm_m{0.24};

  ImuImageStabilizerConfig stabilizer{defaultStabilizerConfig()};

  // camera_driver/config/camera_config.yaml values (auto_control).
  static ImuImageStabilizerConfig defaultStabilizerConfig();
  void validate() const;  // throws std::invalid_argument
};

struct ImuSample
{
  cv::Vec3d acceleration_camera_mps2;   // CAM_A optical frame
  cv::Vec3d angular_velocity_camera_radps;
  double acceleration_timestamp_sec{0.0};
  double gyroscope_timestamp_sec{0.0};
};

enum class FrameTiltKind
{
  OFF,          // stabilization disabled (parameter / GUI) or no IMU
  CALIBRATING,  // reference calibration (keep the vehicle still)
  APPLIED,
  REJECTED,     // larger than maximum_correction_deg
  MISSING,      // no IMU sample close enough to the frame time
};

const char * frameTiltName(FrameTiltKind kind);

struct FrameTilt
{
  FrameTiltKind kind{FrameTiltKind::OFF};
  cv::Matx33d camera_to_reference_tilt{cv::Matx33d::eye()};  // CAM_A optical frame
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double angle_deg{0.0};

  bool applied() const {return kind == FrameTiltKind::APPLIED;}
};

struct ImuTiltStatus
{
  bool enabled{false};
  bool apply{false};
  bool imu_streaming{false};
  bool calibrated{false};
  bool reference_from_ground{false};
  bool vehicle_stationary{false};
  ImageStabilizerCalibrationProgress progress;
  double roll_deg{0.0};
  double pitch_deg{0.0};
  std::uint64_t samples{0U};
  std::uint64_t motion_compensated{0U};
  std::uint64_t motion_misses{0U};
  std::uint64_t timestamp_errors{0U};
  std::string last_error;
};

class ImuTiltTracker
{
public:
  explicit ImuTiltTracker(ImuTiltTrackerConfig config);

  const ImuTiltTrackerConfig & config() const {return config_;}

  // Live GUI toggle; the estimator keeps running so re-enabling is immediate.
  void setApply(bool apply);
  bool apply() const;

  // Camera engine: IMU stream state and calibrated samples (one device batch).
  void setImuStreaming(bool streaming);
  void processBatch(const std::vector<ImuSample> & samples, double batch_received_steady_sec);

  // ROS thread. Times are std::chrono::steady_clock seconds (same as the OAK host timestamps).
  void onMeasuredErpm(int erpm, double steady_now_sec);
  void onVehicleAcceleration(double timestamp_sec, double longitudinal_mps2, double lateral_mps2);

  // Ground normal of the measured mount pose (camera optical frame). Before the
  // first calibration it becomes the reference; after it (Refit) the
  // calibration restarts with the new normal. Returns false if rejected.
  bool setGroundReference(const cv::Vec3d & up_camera);

  // Frame lookup (may wait a few ms for the IMU sample after the frame).
  FrameTilt tiltAt(double frame_timestamp_sec) const;
  ImuTiltStatus status() const;

  // Exposed for tests.
  bool vehicleStationary(double steady_now_sec);

private:
  struct VehicleAxes
  {
    cv::Vec3d forward;
    cv::Vec3d left;
    cv::Vec3d up;
  };
  struct TimedAcceleration
  {
    double timestamp_sec;
    double longitudinal_mps2;
    double lateral_mps2;
  };

  static std::optional<VehicleAxes> axesFromUp(const cv::Vec3d & up_camera);
  std::optional<TimedAcceleration> accelerationAt(double timestamp_sec) const;

  const ImuTiltTrackerConfig config_;
  ImuImageStabilizer stabilizer_;
  std::atomic<bool> apply_{true};
  std::atomic<bool> imu_streaming_{false};

  mutable std::mutex mutex_;  // everything below
  std::optional<cv::Vec3d> ground_reference_;
  std::optional<VehicleAxes> axes_;
  std::deque<TimedAcceleration> accelerations_;
  // ERPM standstill filter
  double erpm_received_sec_{0.0};
  double filtered_abs_erpm_{0.0};
  double stationary_candidate_sec_{0.0};
  bool erpm_stationary_{false};
  // Lever-arm yaw filter (IMU thread)
  double lever_yaw_{0.0};
  double lever_yaw_rate_{0.0};
  double lever_yaw_t_{-1.0};
  double latest_imu_sec_{0.0};
  bool last_stationary_{false};
  std::uint64_t samples_{0U};
  std::uint64_t motion_compensated_{0U};
  std::uint64_t motion_misses_{0U};
  std::uint64_t timestamp_errors_{0U};
  std::string last_error_;
  mutable double last_roll_deg_{0.0};
  mutable double last_pitch_deg_{0.0};
};

}  // namespace manual_capture

#endif  // MANUAL_CAPTURE__IMU_TILT_TRACKER_HPP_
