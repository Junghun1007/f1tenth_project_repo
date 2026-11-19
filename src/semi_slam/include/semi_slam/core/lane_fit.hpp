// 차선 점 직선 맞춤 (차량 좌표: x 앞, y 왼쪽). BEV 차선 각도·옆 위치를 재는 곳(위치 추정, yaw 측정, 정지선 보정)이 모두 같은 계산을 쓴다
#pragma once
#include <limits>
#include <vector>
#include "semi_slam/core/types.hpp"

namespace semi_slam {

struct LineFit {
  double a{0}, b{0};   // y = a·x + b
  bool ok{false};
  double y_at(double x) const { return a * x + b; }
};
// x0 ≤ x ≤ x1 점만 써서 최소제곱 직선. 점이 min_n개 미만이면 ok = false
LineFit fit_lane_line(const std::vector<Vec2> & pts, double x0, double x1, int min_n = 6);

struct LaneFrameFit {
  LineFit left, right;
  bool both{false};   // 양쪽 선이 나란하고 폭이 그럴듯함
  double angle{std::numeric_limits<double>::quiet_NaN()};      // rad, 차선이 차 기준 왼쪽(+)으로 기울어 보이는 각
  double center_y{std::numeric_limits<double>::quiet_NaN()};   // at_x에서 두 선 가운데의 옆 위치 (+ = 차보다 왼쪽)
  double width{std::numeric_limits<double>::quiet_NaN()};      // 차선 폭 (테이프 가운데 간격)
};
LaneFrameFit fit_lane_frame(const std::vector<Vec2> & left, const std::vector<Vec2> & right, double x0 = 0.15, double x1 = 0.8,
                            double at_x = 0.3);

}  // namespace semi_slam
