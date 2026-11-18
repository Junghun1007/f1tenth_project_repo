#include "cone_drive_test/core/duty_only.hpp"
#include <algorithm>
#include <cmath>

namespace cone_drive_test {

double duty_feedforward(double speed_mps, double offset_duty, double duty_per_mps, double fade_speed_mps, double maximum_duty) {
  const double speed = std::isfinite(speed_mps) ? std::max(0.0, speed_mps) : 0.0;
  const double fade = fade_speed_mps > 0.0 ? std::min(1.0, speed / fade_speed_mps) : 1.0;
  return std::clamp(offset_duty * fade + duty_per_mps * speed, 0.0, maximum_duty);
}

double duty_only_stop_speed_limit(double remaining_m, double planning_deceleration_mps2, double coast_deceleration_mps2,
                                  double release_speed_mps, double response_sec) {
  if (!std::isfinite(remaining_m) || remaining_m <= 0.0) return 0.0;
  const double a = std::max(1.0e-6, planning_deceleration_mps2);
  const double coast = std::max(1.0e-6, coast_deceleration_mps2);
  const double release = std::max(0.0, release_speed_mps);
  const double delay = std::max(0.0, response_sec);
  const double coast_length = release * release / (2.0 * coast);
  const double at = a * delay;
  // v·delay + (v² − release²)/(2a) = 굴러가기 구간 앞까지의 거리
  const auto duty_segment = [&](double distance) { return std::sqrt(at * at + release * release + 2.0 * a * std::max(0.0, distance)) - at; };
  if (remaining_m > coast_length) return duty_segment(remaining_m - coast_length);
  return std::min(std::sqrt(2.0 * coast * remaining_m), duty_segment(0.0));
}

bool duty_only_coast_release(double remaining_m, double speed_mps, double coast_deceleration_mps2, double release_delay_sec,
                             double maximum_release_speed_mps) {
  if (!std::isfinite(remaining_m) || !std::isfinite(speed_mps)) return false;
  if (remaining_m <= 0.0) return true;
  const double speed = std::max(0.0, speed_mps);
  if (speed > std::max(0.0, maximum_release_speed_mps)) return false;
  const double coast = std::max(1.0e-6, coast_deceleration_mps2);
  return remaining_m <= speed * std::max(0.0, release_delay_sec) + speed * speed / (2.0 * coast);
}

}  // namespace cone_drive_test
