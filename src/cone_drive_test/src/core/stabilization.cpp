#include "cone_drive_test/core/stabilization.hpp"
#include <cstdio>

namespace cone_drive_test {

StabilizationDecision decide_stabilization(const StabilizationCamera & cam, const ImageStabilizationCorrection & c) {
  StabilizationDecision d;
  if (!c.within_correction_limit) { d.reason = StabilizationReason::OverLimit; return d; }
  d.source_to_stabilized = makeImageStabilizationHomography(cam.fx, cam.fy, cam.cx, cam.cy, c);
  d.apply = true;
  d.reason = StabilizationReason::Applied;
  return d;
}

void StabilizationStats::count(StabilizationReason r, double deg) {
  std::lock_guard<std::mutex> l(m_);
  ++n_[int(r)];
  if (r == StabilizationReason::Applied || r == StabilizationReason::Held) deg_sum_ += deg;
}

std::string StabilizationStats::summary() {
  std::lock_guard<std::mutex> l(m_);
  int total = 0;
  for (int v : n_) total += v;
  if (total == 0) return last_;
  const int ok = n_[int(StabilizationReason::Applied)] + n_[int(StabilizationReason::Held)];
  char buf[200];
  if (n_[int(StabilizationReason::Disabled)] == total) std::snprintf(buf, sizeof(buf), "꺼짐");
  else if (n_[int(StabilizationReason::NotInitialized)] == total) std::snprintf(buf, sizeof(buf), "IMU 기준 측정 중 (정지 유지)");
  else
    std::snprintf(buf, sizeof(buf), "적용 %d%% · 평균 %.2f° · IMU 매칭 실패 %d · 한계 초과 %d", 100 * ok / total,
                  ok ? deg_sum_ / ok : 0.0, n_[int(StabilizationReason::NoImuMatch)], n_[int(StabilizationReason::OverLimit)]);
  last_ = buf;
  for (int & v : n_) v = 0;
  deg_sum_ = 0;
  return last_;
}

}  // namespace cone_drive_test
