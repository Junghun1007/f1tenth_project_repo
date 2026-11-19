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
  Vec2 stop_center() const { return {(stop_a.x + stop_b.x) / 2, (stop_a.y + stop_b.y) / 2}; }
  Vec2 start_center() const { return {(start_a.x + start_b.x) / 2, (start_a.y + start_b.y) / 2}; }
  // 주행 방향 (정지선을 반시계로 지날 때 진행 방향, 단위 벡터): 정지선의 법선 중 트랙 안쪽 중심 기준 반시계
  Vec2 travel_dir_at(const Vec2 & p) const;
  void build_fields(double resolution_m = 0.01);   // load_csv가 호출

private:
  double field(const cv::Mat & f, const Vec2 & p) const;
  cv::Mat outer_field_, inner_field_;
  double res_{0.01}, x0_{-0.5}, y0_{-0.5};
  double lap_length_{0};
  Vec2 middle_{2, 2};
};

}  // namespace semi_slam
