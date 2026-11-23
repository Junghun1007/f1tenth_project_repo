// 차선 기준 단기 콘 기억 (memory.mode = 1).
// 콘을 차선 중심선 기준 (s: 중심선 따라 앞쪽 거리, l: 왼쪽 횡거리)로만 들고 있는다.
//  - 보이는 구간(s ≥ visible_s_min): 관측되는 콘만 사용. 안 보이면 miss_delete 번 만에 지움 → 유령 콘 없음
//  - 위치는 거리 가중 평균: 사라지기 직전(가림·화면 끝)의 나쁜 관측 하나로 위치가 끌려가지 않게
//  - 사각지대(s < visible_s_min): 카메라에서 벗어난 콘을 뒷범퍼를 지날 때까지 유지 (뒷바퀴 보호)
//  - 차가 움직이면 s만 바퀴 이동 거리만큼 줄인다. l은 매 프레임 인식하는 차선 기준이라 자이로가 필요 없다
#pragma once
#include <cstdint>
#include <vector>
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

struct LaneMemoryParams {
  double match_radius{0.12};             // 같은 콘으로 볼 (s,l) 거리
  double visible_s_min{0.45};            // 이보다 가까우면 카메라 사각지대 (앞차축 기준)
  int miss_delete{3};                    // 보이는 구간에서 연속으로 안 보이면 삭제
  int confirm_hits{2};                   // 이만큼 관측돼야 계획에 사용
  double front_axle_to_rear_bumper{0.45};
  double delete_behind{0.10};
  double range_noise_base{0.01}, range_noise_per_m{0.02};   // 거리 오차 σ → 1/σ² 가중 평균
};

struct LaneCone { double s{0}, l{0}; int hits{0}, misses{0}; std::uint32_t id{0}; double weight{1}; };

class LaneConeMemory {
public:
  explicit LaneConeMemory(const LaneMemoryParams & p = LaneMemoryParams{}) : p_(p) {}
  void set_params(const LaneMemoryParams & p) { p_ = p; }
  void advance(double ds);                              // 차가 차선을 따라 ds 전진
  // 새 검출 프레임의 관측 (현재 차선 기준 s,l)과 각 관측의 카메라 거리. 거리 가중 평균으로 위치를 다듬어
  // 화면 끝·가림 때문에 한 번 크게 틀린 관측이 기억 위치를 끌고 가지 않게 한다 (ranges 비면 모두 1 m로 봄)
  void update(const std::vector<Vec2> & observed_sl, const std::vector<double> & ranges = {}, bool complete = true);
  std::vector<Vec2> cones_sl() const;                   // 계획에 쓸 콘 (확인된 것)
  const std::vector<LaneCone> & tracks() const { return tracks_; }
  bool confirmed(const LaneCone & c) const { return c.hits >= p_.confirm_hits; }
  void clear() { tracks_.clear(); }

private:
  void drop_behind();
  LaneMemoryParams p_;
  std::vector<LaneCone> tracks_;
  std::uint32_t next_id_{1};
};

}  // namespace cone_drive_test
