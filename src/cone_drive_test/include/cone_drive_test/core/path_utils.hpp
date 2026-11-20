// 시뮬레이터 Path/stanley와 Frenet 기준선 유틸 (planners.py, control.py와 같은 수치 동작)
#pragma once
#include <vector>
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

struct ControlParams;

double interp(double x, const std::vector<double> & xp, const std::vector<double> & fp);   // np.interp (끝값 고정)

struct SimPath {   // control.py Path: 앞차축 기준 점열 + 호 길이
  std::vector<Vec2> points;
  std::vector<double> arc;
  double window{0.16};
  SimPath(const std::vector<Vec2> & pts, double geometry_window);                                  // 중복점 제거, 호 길이 계산
  SimPath(const std::vector<Vec2> & pts, const std::vector<double> & arc, double geometry_window); // 강체 변환한 같은 경로
  double length() const { return arc.empty() ? 0.0 : arc.back(); }
  Vec2 point(double a) const;
  double heading(double a) const;
};

double sim_stanley(const SimPath & path, double speed, const ControlParams & c, double max_steer);

struct FrenetFrame {
  std::vector<double> arc;
  std::vector<Vec2> center, tang, norm;
};
std::vector<Vec2> smooth_center(const std::vector<Vec2> & c);
std::vector<Vec2> extend_backward(const std::vector<Vec2> & c, double length, double spacing = 0.05);
// 끝을 마지막 fit_length 구간의 곡률 그대로 원호로 length만큼 잇는다 (|곡률| ≤ max_curvature).
// 코너 도중 BEV 옆 끝에서 잘린 중앙선을 직선이 아니라 계속 꺾이는 길로 늘린다
std::vector<Vec2> extend_forward_arc(const std::vector<Vec2> & c, double length, double fit_length, double max_curvature,
                                     double spacing = 0.05);
FrenetFrame frenet_frame(const std::vector<Vec2> & center);
Vec2 project_sl(const Vec2 & p, const FrenetFrame & f);                 // (s, l)
Vec2 from_frenet(double s, double l, const FrenetFrame & f);

}  // namespace cone_drive_test
