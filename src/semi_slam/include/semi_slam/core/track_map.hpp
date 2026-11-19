// 정답 트랙 (maps/*.csv: line,x_m,y_m). 좌표는 트랙 영역 왼쪽 아래 = 원점, x 오른쪽, y 위 (m), 주행은 반시계.
// 보정·평가에는 실제 흰 선(outer, inner)만 쓴다. center는 그림에 표시만 한 중앙선이라 화면 참고용
#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "semi_slam/core/types.hpp"

namespace semi_slam {

struct TrackMap {
  std::vector<Vec2> outer, inner, center;   // 닫힌 선 (반시계)
  Vec2 stop_a{}, stop_b{}, start_a{}, start_b{};   // 정지선 S, 출발선 T1 양 끝

  static TrackMap load_csv(const std::string & path);   // 실패 시 예외
  // 선까지 거리 (m). 지도 밖 0.5 m 여유까지, 그 밖은 큰 값
  double dist_outer(const Vec2 & p) const { return field(outer_field_, p); }
  double dist_inner(const Vec2 & p) const { return field(inner_field_, p); }
  double lap_length() const { return lap_length_; }       // 바깥·안쪽 선 길이 평균 (참고 값)
  double lane_width() const { return lane_width_; }       // 두 흰 선(테이프 가운데) 간격
  Vec2 stop_center() const { return {(stop_a.x + stop_b.x) / 2, (stop_a.y + stop_b.y) / 2}; }
  Vec2 start_center() const { return {(start_a.x + start_b.x) / 2, (start_a.y + start_b.y) / 2}; }
  // 주행 방향 (단위 벡터, 직선 구간 축 방향): 반시계면 트랙 중심 기준 반시계, cw면 반대
  Vec2 travel_dir_at(const Vec2 & p, bool cw = false) const;
  // 아래 직선이 시작되는 점 (도로 가운데): 반시계 = 왼쪽 아래 코너가 끝나는 곳, 시계 = 오른쪽 아래 코너가 끝나는 곳
  Vec2 straight_start(bool cw) const;
  // 차선 인식의 왼쪽(side 0)/오른쪽(1) 선이 대응하는 정답 선까지 거리. 반시계: 왼쪽 = 안쪽, 시계: 왼쪽 = 바깥
  double dist_side(int side, bool cw, const Vec2 & p) const { return ((side == 0) != cw) ? dist_inner(p) : dist_outer(p); }
  void build_fields(double resolution_m = 0.01);   // load_csv가 호출

  // 중앙선 (지도에 그린 점선 = 두 흰 선 가운데) 위 거리 좌표 s: 반시계 방향으로 증가, 0 ~ center_length()
  double center_length() const { return center_len_; }
  Vec2 center_at(double s, Vec2 * tangent = nullptr) const;   // tangent: 반시계 방향 단위 벡터
  double center_project(const Vec2 & p) const;                 // p에서 가장 가까운 중앙선 위치의 s
  double arc_diff(double a, double b) const;                   // a − b를 (−L/2, L/2]로
  double wrap_s(double s) const;

private:
  double field(const cv::Mat & f, const Vec2 & p) const;
  cv::Mat outer_field_, inner_field_;
  double res_{0.01}, x0_{-0.5}, y0_{-0.5};
  double lap_length_{0}, lane_width_{0.67};
  Vec2 middle_{2, 2};
  std::vector<double> center_s_;   // 중앙선 점마다 누적 거리
  double center_len_{0};
};

}  // namespace semi_slam
