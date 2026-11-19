// 차선 좌우 판정 고정 (lane.side_tracking). 차선 모델이 붙인 왼쪽/오른쪽 라벨을 그대로 믿지 않고
// 조각마다 다시 정한다. 오도메트리는 쓰지 않는다: 기준은 "지금 프레임"과 "max_age 안의 직전 프레임"뿐.
//  1) 직전 프레임의 왼쪽·오른쪽 선 중 match_gate 안에서 더 잘 맞는 쪽으로 배정 (라벨보다 우선)
//  2) 직전 기준이 없는 조각: 이번 프레임에 확정된 반대쪽 선에서 차선 폭(±tol)만큼 떨어져 있으면 그 반대편으로,
//     너무 가까우면 그 선과 같은 쪽으로 (사라졌던 쪽은 남은 선에서 차선 폭만큼 떨어져야만 받아들임)
//  3) 폭 검사: 같은 앞쪽 거리에서 왼쪽~오른쪽 폭이 차선 폭 ±tol을 벗어나면 짝이 틀린 것 → 근거가 약한 쪽 조각을 뺀다
// 좌표는 BEV 라벨 픽셀 (x=열: 왼쪽이 작음, y=행: 위가 멀리)
#pragma once
#include <array>
#include <vector>
#include <opencv2/core.hpp>

namespace semi_slam {

struct LaneSideTrackerConfig {
  double meter_per_pixel{0.01};
  double lane_width_m{0.65}, width_tolerance_m{0.12};
  double match_gate_m{0.15};
  double max_age_sec{0.1};
};

struct LaneSideFixStats { int relabeled{0}, dropped{0}; };

class LaneSideTracker {
public:
  using Paths = std::array<std::vector<std::vector<cv::Point2f>>, 2>;   // [0]=왼쪽, [1]=오른쪽
  explicit LaneSideTracker(const LaneSideTrackerConfig & c = {}) : c_(c) {}
  // paths를 고쳐 쓴다. rows: 라벨 영상 높이
  LaneSideFixStats apply(Paths & paths, int rows, double t);
  void reset() { prev_t_ = {-1e9, -1e9}; prev_[0].clear(); prev_[1].clear(); }

private:
  LaneSideTrackerConfig c_;
  std::array<std::vector<float>, 2> prev_;   // 행별 열 위치 (NaN = 없음)
  std::array<double, 2> prev_t_{{-1e9, -1e9}};
};

}  // namespace semi_slam
