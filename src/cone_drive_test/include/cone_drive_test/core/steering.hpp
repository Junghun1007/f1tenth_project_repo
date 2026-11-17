// 조향 필터·변화율 제한·정지 시 유지 (auto_control_node update_control_from_path의 조향 부분)
#pragma once

namespace cone_drive_test {

struct SteeringParams {
  double steering_current_weight{0.4};   // control_rate_hz 기준 현재값 비중
  double control_rate_hz{100.0};
  double rate_limit_rad_s{240.0 * 3.14159265358979323846 / 180.0};
  double max_steer_rad{30.0 * 3.14159265358979323846 / 180.0};
  double corner_heading_threshold_rad{60.0 * 3.14159265358979323846 / 180.0};
  double servo_left{1.0}, servo_center{0.5}, servo_right{0.0};
  bool inverted{true};
  bool stop_hold{true};
};

class SteeringFilter {
public:
  explicit SteeringFilter(const SteeringParams & p) : p_(p) {}
  void set_params(const SteeringParams & p) { p_ = p; }
  // raw: Stanley 출력, heading_error: Stanley 방향 오차, target/current: 목표·실제 속도
  double update(double raw, double dt, double heading_error, double target_speed, double current_speed = 1.0);
  double angle() const { return angle_; }
  double servo(double angle) const;
  void reset() { angle_ = 0.0; held_ = false; }

private:
  SteeringParams p_;
  double angle_{0.0};
  bool held_{false};
};

}  // namespace cone_drive_test
