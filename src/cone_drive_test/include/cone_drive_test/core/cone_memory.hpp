// 콘 기억: 최근접 이웃 추적 (tools/avoidance_sim planners.py ConeMemory 이식). 위치는 odom 좌표.
#pragma once
#include <cstdint>
#include <functional>
#include <vector>
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

struct MemoryParams {
  double gate{0.06};               // 거리 방향 게이트 기본
  double gate_radial_max{0.17};    // 거리 방향 게이트 상한
  double gate_lateral{0.06};       // 옆 방향 게이트
  double merge_distance{0.10};     // 이보다 가까운 두 기억은 하나로
  int confirm_hits{3};
  int miss_delete{4};              // 보여야 하는데 안 보인 횟수
  double delete_behind{0.10};      // 뒷범퍼 뒤 이 거리 지나면 삭제
  double unconfirmed_timeout{0.5}; // 미확정 기억이 이 시간 동안 안 보이면 삭제
  double range_noise_base{0.01};   // 거리 오차 σ = base + per_m × 거리 (가중치·게이트)
  double range_noise_per_m{0.02};
  double camera_x{-0.09};          // 앞차축 기준 카메라 x
  double front_axle_to_rear_bumper{0.45};
  double bev_x_max{3.0}, bev_y_max{0.6};   // miss 판정 영역
};

struct TrackedCone {
  Vec2 world;
  double weight{1.0};
  int hits{0}, misses{0};
  double last_seen{0};
  std::uint32_t id{0};
};

// 지금 보여야 하는 기억 콘의 인덱스 (centers_world는 병합 후 기억 위치)
using ExpectedVisibleFn = std::function<std::vector<int>(const std::vector<Vec2> & centers_world, const Pose2 & pose)>;

class ConeMemory {
public:
  explicit ConeMemory(const MemoryParams & p = {});
  void set_params(const MemoryParams & p) { p_ = p; }
  const MemoryParams & params() const { return p_; }
  // pose: 촬영 시점 odom 앞차축 자세, cones: 그 시점 차량 좌표 관측
  void update(const Pose2 & pose, double capture_time, const std::vector<ConeObservation> & cones,
              const ExpectedVisibleFn & expected);
  std::vector<Vec2> confirmed_world() const;
  const std::vector<TrackedCone> & tracks() const { return tracks_; }
  bool confirmed(const TrackedCone & t) const { return t.hits >= p_.confirm_hits; }
  void clear() { tracks_.clear(); }

private:
  void merge_duplicates();
  MemoryParams p_;
  std::vector<TrackedCone> tracks_;
  std::uint32_t next_id_{1};
};

Vec2 to_world(const Vec2 & local, const Pose2 & pose);
Vec2 to_local(const Vec2 & world, const Pose2 & pose);

}  // namespace cone_drive_test
