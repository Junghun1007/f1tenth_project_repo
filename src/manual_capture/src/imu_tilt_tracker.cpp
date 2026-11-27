#include "manual_capture/imu_tilt_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace manual_capture
{

namespace
{

bool finite(const cv::Vec3d & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

bool positive(double value) {return std::isfinite(value) && value > 0.0;}

}  // namespace

const char * frameTiltName(const FrameTiltKind kind)
{
  switch (kind) {
    case FrameTiltKind::OFF: return "off";
    case FrameTiltKind::CALIBRATING: return "calibrating";
    case FrameTiltKind::APPLIED: return "applied";
    case FrameTiltKind::REJECTED: return "rejected";
    case FrameTiltKind::MISSING: return "missing";
  }
  return "off";
}

ImuImageStabilizerConfig ImuTiltTrackerConfig::defaultStabilizerConfig()
{
  // Struct defaults differ from the values auto_control runs with; copy those.
  ImuImageStabilizerConfig c;
  c.acceleration_correction_stationary_only = false;
  c.moving_accelerometer_nudge_enabled = true;
  c.maximum_frame_imu_wait_sec = 0.005;
  c.maximum_frame_imu_age_sec = 0.012;
  c.maximum_frame_imu_prediction_sec = 0.015;
  return c;
}

void ImuTiltTrackerConfig::validate() const
{
  if (!enabled) {
    return;
  }
  if (!positive(imu_rate_hz) || imu_rate_hz > 500.0 || imu_queue_size <= 0 ||
    imu_max_batch_reports <= 1 || imu_max_batch_reports > imu_queue_size ||
    !positive(maximum_accel_gyro_timestamp_skew_sec) ||
    !positive(maximum_timestamp_domain_delta_sec))
  {
    throw std::invalid_argument("invalid IMU rate, batching or timestamp skew limit");
  }
  if (stationary_erpm_enter_threshold < 0 ||
    stationary_erpm_exit_threshold <= stationary_erpm_enter_threshold ||
    !positive(stationary_erpm_filter_time_constant_sec) ||
    !positive(stationary_erpm_enter_duration_sec) || !positive(measured_erpm_timeout_sec))
  {
    throw std::invalid_argument("invalid measured-ERPM stationary detection parameters");
  }
  if (vehicle_motion_compensation_enabled &&
    (!positive(can_acceleration_timeout_sec) ||
    !std::isfinite(can_longitudinal_compensation_gain) ||
    can_longitudinal_compensation_gain < 0.0 || can_longitudinal_compensation_gain > 1.0 ||
    !std::isfinite(can_lateral_compensation_gain) ||
    can_lateral_compensation_gain < 0.0 || can_lateral_compensation_gain > 1.0 ||
    !positive(maximum_longitudinal_acceleration_mps2) ||
    !positive(maximum_lateral_acceleration_mps2)))
  {
    throw std::invalid_argument("invalid CAN vehicle acceleration compensation parameters");
  }
  if (!std::isfinite(lever_arm_m) || std::abs(lever_arm_m) > 1.0) {
    throw std::invalid_argument("imu_stabilization.lever_arm_m must be within +-1 m");
  }
}

ImuTiltTracker::ImuTiltTracker(ImuTiltTrackerConfig config)
: config_([&config]() {
      config.validate();
      if (config.enabled && !config.vehicle_motion_compensation_enabled) {
        // Same safety rule as camera_driver: raw moving acceleration is not gravity.
        config.stabilizer.moving_accelerometer_nudge_enabled = false;
        config.stabilizer.acceleration_correction_stationary_only = true;
      }
      return std::move(config);
    }()),
  stabilizer_(config_.stabilizer)  // throws on an invalid estimator configuration
{
  apply_.store(config_.enabled);
}

void ImuTiltTracker::setApply(const bool apply) {apply_.store(apply && config_.enabled);}
bool ImuTiltTracker::apply() const {return apply_.load();}
void ImuTiltTracker::setImuStreaming(const bool streaming) {imu_streaming_.store(streaming);}

void ImuTiltTracker::onMeasuredErpm(const int erpm, const double now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double previous = erpm_received_sec_;
  erpm_received_sec_ = now;
  const bool reset = previous <= 0.0 || now - previous > config_.measured_erpm_timeout_sec;
  if (reset) {
    erpm_stationary_ = false;
    stationary_candidate_sec_ = 0.0;
  }
  const double raw = std::abs(static_cast<double>(erpm));
  double filtered = raw;
  if (!reset) {
    const double alpha = -std::expm1(
      -(now - previous) / config_.stationary_erpm_filter_time_constant_sec);
    filtered = filtered_abs_erpm_ + alpha * (raw - filtered_abs_erpm_);
  }
  filtered_abs_erpm_ = filtered;

  // A real departure cancels standstill immediately; only entry is filtered.
  if (raw >= config_.stationary_erpm_exit_threshold) {
    stationary_candidate_sec_ = 0.0;
    erpm_stationary_ = false;
    return;
  }
  if (filtered <= config_.stationary_erpm_enter_threshold) {
    if (erpm_stationary_) {
      return;
    }
    if (stationary_candidate_sec_ <= 0.0) {
      stationary_candidate_sec_ = now;
      return;
    }
    if (now - stationary_candidate_sec_ >= config_.stationary_erpm_enter_duration_sec) {
      erpm_stationary_ = true;
    }
    return;
  }
  stationary_candidate_sec_ = 0.0;
}

bool ImuTiltTracker::vehicleStationary(const double now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool fresh = erpm_received_sec_ > 0.0 && now >= erpm_received_sec_ &&
    now - erpm_received_sec_ <= config_.measured_erpm_timeout_sec;
  if (!fresh) {
    // Missing telemetry must not lock the estimator into standstill.
    erpm_stationary_ = false;
    stationary_candidate_sec_ = 0.0;
    return false;
  }
  return erpm_stationary_;
}

void ImuTiltTracker::onVehicleAcceleration(
  const double timestamp_sec, const double longitudinal_mps2, const double lateral_mps2)
{
  if (!std::isfinite(timestamp_sec) || !std::isfinite(longitudinal_mps2) ||
    !std::isfinite(lateral_mps2))
  {
    return;
  }
  const TimedAcceleration sample{
    timestamp_sec,
    std::clamp(longitudinal_mps2, -config_.maximum_longitudinal_acceleration_mps2,
      config_.maximum_longitudinal_acceleration_mps2),
    std::clamp(lateral_mps2, -config_.maximum_lateral_acceleration_mps2,
      config_.maximum_lateral_acceleration_mps2)};
  std::lock_guard<std::mutex> lock(mutex_);
  if (!accelerations_.empty() && timestamp_sec <= accelerations_.back().timestamp_sec) {
    accelerations_.back().longitudinal_mps2 = sample.longitudinal_mps2;
    accelerations_.back().lateral_mps2 = sample.lateral_mps2;
  } else {
    accelerations_.push_back(sample);
  }
  const double keep_sec = std::max(1.0, 2.0 * config_.can_acceleration_timeout_sec);
  while (accelerations_.size() > 2U &&
    accelerations_.back().timestamp_sec - accelerations_.front().timestamp_sec > keep_sec)
  {
    accelerations_.pop_front();
  }
}

std::optional<ImuTiltTracker::TimedAcceleration> ImuTiltTracker::accelerationAt(
  const double timestamp_sec) const
{
  // Caller holds mutex_.
  if (!std::isfinite(timestamp_sec) || accelerations_.empty()) {
    return std::nullopt;
  }
  const double timeout = config_.can_acceleration_timeout_sec;
  const auto next = std::lower_bound(
    accelerations_.begin(), accelerations_.end(), timestamp_sec,
    [](const TimedAcceleration & sample, double time) {return sample.timestamp_sec < time;});
  if (next == accelerations_.begin()) {
    if (next->timestamp_sec - timestamp_sec > timeout) {return std::nullopt;}
    return *next;
  }
  if (next == accelerations_.end()) {
    const auto & latest = accelerations_.back();
    if (timestamp_sec - latest.timestamp_sec > timeout) {return std::nullopt;}
    return latest;
  }
  const auto & previous = *std::prev(next);
  if (timestamp_sec - previous.timestamp_sec > timeout ||
    next->timestamp_sec - timestamp_sec > timeout)
  {
    return std::nullopt;
  }
  const double interval = next->timestamp_sec - previous.timestamp_sec;
  if (interval <= 0.0) {
    return *next;
  }
  const double amount = std::clamp((timestamp_sec - previous.timestamp_sec) / interval, 0.0, 1.0);
  return TimedAcceleration{
    timestamp_sec,
    previous.longitudinal_mps2 + amount * (next->longitudinal_mps2 - previous.longitudinal_mps2),
    previous.lateral_mps2 + amount * (next->lateral_mps2 - previous.lateral_mps2)};
}

std::optional<ImuTiltTracker::VehicleAxes> ImuTiltTracker::axesFromUp(const cv::Vec3d & up_camera)
{
  const double up_norm = cv::norm(up_camera);
  if (!std::isfinite(up_norm) || up_norm <= 1.0e-6) {
    return std::nullopt;
  }
  const cv::Vec3d up = up_camera / up_norm;
  const cv::Vec3d optical_forward(0.0, 0.0, 1.0);
  cv::Vec3d forward = optical_forward - up * optical_forward.dot(up);
  const double forward_norm = cv::norm(forward);
  if (!std::isfinite(forward_norm) || forward_norm <= 1.0e-6) {
    return std::nullopt;
  }
  forward /= forward_norm;
  cv::Vec3d left = up.cross(forward);
  const double left_norm = cv::norm(left);
  if (!std::isfinite(left_norm) || left_norm <= 1.0e-6) {
    return std::nullopt;
  }
  return VehicleAxes{forward, left / left_norm, up};
}

bool ImuTiltTracker::setGroundReference(const cv::Vec3d & up_camera)
{
  if (!config_.enabled || !finite(up_camera) || cv::norm(up_camera) <= 1.0e-9) {
    return false;
  }
  const cv::Vec3d up = up_camera / cv::norm(up_camera);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stabilizer_.setExternalReferenceUpCamera(up)) {
    // Already calibrated: Refit restarts the calibration around the new normal.
    if (!stabilizer_.restartCalibration(up)) {
      return false;
    }
  }
  ground_reference_ = up;
  axes_ = axesFromUp(up);
  return true;
}

void ImuTiltTracker::processBatch(
  const std::vector<ImuSample> & samples, const double batch_received_steady_sec)
{
  if (!config_.enabled || samples.empty()) {
    return;
  }
  double newest_gyro_sec = -std::numeric_limits<double>::infinity();
  for (const auto & sample : samples) {
    newest_gyro_sec = std::max(newest_gyro_sec, sample.gyroscope_timestamp_sec);
  }
  const bool stationary = vehicleStationary(batch_received_steady_sec);

  for (const auto & sample : samples) {
    const double gyro_t = sample.gyroscope_timestamp_sec;
    const cv::Vec3d & w_camera = sample.angular_velocity_camera_radps;
    std::optional<cv::Vec3d> acceleration;
    const double skew = std::abs(sample.acceleration_timestamp_sec - gyro_t);
    if (std::isfinite(skew) && skew <= config_.maximum_accel_gyro_timestamp_skew_sec &&
      finite(sample.acceleration_camera_mps2))
    {
      acceleration = sample.acceleration_camera_mps2;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++samples_;
      latest_imu_sec_ = gyro_t;
      last_stationary_ = stationary;
      if (!axes_ && stabilizer_.initialized()) {
        // No measured ground normal (startup measurement failed): IMU startup gravity.
        if (const auto reference = stabilizer_.referenceUpCamera()) {
          axes_ = axesFromUp(*reference);
        }
      }
      if (config_.vehicle_motion_compensation_enabled && stabilizer_.initialized()) {
        // CAN dynamics are stamped on reception; keep each report's age in the batch
        // so the vehicle acceleration is queried at the same instant.
        double query_sec = batch_received_steady_sec;
        const double age = newest_gyro_sec - gyro_t;
        if (std::isfinite(age) && age >= 0.0 && age <= config_.maximum_timestamp_domain_delta_sec) {
          query_sec -= age;
        }
        const auto vehicle = accelerationAt(query_sec);
        if (axes_ && config_.lever_arm_m != 0.0) {
          // Yaw rate and its derivative (time constants 5 ms / 30 ms).
          const double w = w_camera.dot(axes_->up);
          const double dt = gyro_t - lever_yaw_t_;
          if (lever_yaw_t_ < 0.0 || !(dt > 1e-5) || dt > 0.1) {
            lever_yaw_ = w;
            lever_yaw_rate_ = 0.0;
          } else {
            const double w_new = lever_yaw_ + (1.0 - std::exp(-dt / 0.005)) * (w - lever_yaw_);
            lever_yaw_rate_ += (1.0 - std::exp(-dt / 0.03)) * ((w_new - lever_yaw_) / dt - lever_yaw_rate_);
            lever_yaw_ = w_new;
          }
          lever_yaw_t_ = gyro_t;
        }
        if (acceleration && vehicle && axes_) {
          const cv::Vec3d vehicle_camera =
            axes_->forward * (config_.can_longitudinal_compensation_gain * vehicle->longitudinal_mps2 -
            lever_yaw_ * lever_yaw_ * config_.lever_arm_m) +
            axes_->left * (config_.can_lateral_compensation_gain * vehicle->lateral_mps2 +
            lever_yaw_rate_ * config_.lever_arm_m);
          acceleration = *acceleration - vehicle_camera;
          ++motion_compensated_;
        } else {
          // Raw moving acceleration is not a gravity observation: omit it.
          acceleration.reset();
          ++motion_misses_;
        }
      }
    }
    try {
      stabilizer_.update(acceleration, w_camera, gyro_t, stationary);
    } catch (const std::exception & exception) {
      // e.g. the gap across a camera restart; the next sample continues.
      std::lock_guard<std::mutex> lock(mutex_);
      ++timestamp_errors_;
      last_error_ = exception.what();
    }
  }
}

FrameTilt ImuTiltTracker::tiltAt(const double frame_timestamp_sec) const
{
  FrameTilt tilt;
  if (!apply_.load() || !imu_streaming_.load()) {
    return tilt;
  }
  if (!stabilizer_.initialized()) {
    tilt.kind = FrameTiltKind::CALIBRATING;
    return tilt;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_imu_sec_ > 0.0 &&
      std::abs(frame_timestamp_sec - latest_imu_sec_) > config_.maximum_timestamp_domain_delta_sec)
    {
      tilt.kind = FrameTiltKind::MISSING;  // not the same clock domain
      return tilt;
    }
  }
  // Must not hold mutex_: this may wait for the IMU thread.
  const auto correction = stabilizer_.correctionAt(frame_timestamp_sec);
  if (!correction) {
    tilt.kind = FrameTiltKind::MISSING;
    return tilt;
  }
  tilt.roll_deg = correction->roll_error_deg;
  tilt.pitch_deg = correction->pitch_error_deg;
  tilt.angle_deg = correction->correction_angle_deg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_roll_deg_ = tilt.roll_deg;
    last_pitch_deg_ = tilt.pitch_deg;
  }
  if (!correction->within_correction_limit) {
    tilt.kind = FrameTiltKind::REJECTED;
    return tilt;
  }
  tilt.kind = FrameTiltKind::APPLIED;
  tilt.camera_to_reference_tilt = correction->camera_to_reference_tilt;
  return tilt;
}

ImuTiltStatus ImuTiltTracker::status() const
{
  ImuTiltStatus s;
  s.enabled = config_.enabled;
  s.apply = apply_.load();
  s.imu_streaming = imu_streaming_.load();
  s.calibrated = stabilizer_.initialized();
  s.progress = stabilizer_.calibrationProgress();
  std::lock_guard<std::mutex> lock(mutex_);
  s.reference_from_ground = ground_reference_.has_value();
  s.vehicle_stationary = last_stationary_;
  s.roll_deg = last_roll_deg_;
  s.pitch_deg = last_pitch_deg_;
  s.samples = samples_;
  s.motion_compensated = motion_compensated_;
  s.motion_misses = motion_misses_;
  s.timestamp_errors = timestamp_errors_;
  s.last_error = last_error_;
  return s;
}

}  // namespace manual_capture
