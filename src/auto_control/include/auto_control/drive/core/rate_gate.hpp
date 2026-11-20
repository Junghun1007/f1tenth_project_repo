// Copied from src/cone_drive_test/include/cone_drive_test/core/rate_gate.hpp (1113ML) — auto_control로 옮김
// 프레임 선택기: 카메라가 목표보다 빠를 때 평균 목표 FPS가 되도록 프레임을 고른다.
// (직전 실행 후 1/fps 경과 비교만 하면 70 fps 카메라에서 40 목표가 35로 떨어진다)
#pragma once
#include <algorithm>

namespace auto_control::drive {

class RateGate {
public:
  explicit RateGate(double fps) { set_rate(fps); }
  void set_rate(double fps) { period_ = 1.0 / std::max(fps, 1e-3); }
  // t: 프레임 촬영 시각 (s). true면 이 프레임을 처리
  bool accept(double t) {
    if (have_ && t < next_ - 1e-6) return false;
    // 예정 시각 기준으로 다음 시각을 잡아 평균을 맞추되, 오래 멈췄다 돌아오면 몰아서 돌리지 않는다
    next_ = (have_ && t - next_ < period_) ? next_ + period_ : t + period_;
    have_ = true;
    return true;
  }

private:
  double period_{0.025}, next_{0};
  bool have_{false};
};

}  // namespace auto_control::drive
