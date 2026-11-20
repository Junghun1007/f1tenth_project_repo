// Copied from src/cone_drive_test/src/core/steering.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/steering.hpp"
#include "auto_control/drive/copied/control_core.hpp"
#include <cmath>

namespace auto_control::drive {

double SteeringFilter::update(double raw, double dt, double heading_error, double target, double v) {
  // 정지 중 조향 유지 (ERPM 노이즈로 깜빡이지 않게 히스테리시스)
  if (!p_.stop_hold || target > 0.0 || std::abs(v) > 0.10) held_ = false;
  else if (std::abs(v) < 0.05) held_ = true;
  const bool corner_reset = !held_ && std::abs(heading_error) >= p_.corner_heading_threshold_rad && raw * angle_ < 0.0;
  double filtered = raw;
  if (held_) {
    filtered = angle_;
  } else if (corner_reset) {
    angle_ = 0.0;
  } else {
    const double w = 1.0 - std::pow(1.0 - p_.steering_current_weight, dt * p_.control_rate_hz);
    filtered = w * raw + (1.0 - w) * angle_;
  }
  angle_ = move_toward(angle_, filtered, p_.rate_limit_rad_s * dt);
  return angle_;
}

double SteeringFilter::servo(double angle) const {
  return steering_angle_to_servo(angle, p_.max_steer_rad, p_.servo_left, p_.servo_center, p_.servo_right, p_.inverted);
}

}  // namespace auto_control::drive
