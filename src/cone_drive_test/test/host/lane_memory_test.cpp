#include "check.hpp"
#include "cone_drive_test/core/lane_memory.hpp"

using namespace cone_drive_test;

TEST(visible_cone_not_seen_is_dropped_quickly) {   // 보이는 구간에서 안 보이면 바로 지움 (유령 콘 없음)
  LaneMemoryParams p;
  LaneConeMemory m(p);
  m.update({{1.5, 0.2}});
  m.update({{1.5, 0.2}});
  CHECK(m.cones_sl().size() == 1);
  for (int k = 0; k < p.miss_delete; ++k) m.update({});
  CHECK(m.cones_sl().empty());
}

TEST(cone_leaving_view_is_held_until_rear_passes) {   // 사각지대로 들어간 콘은 뒷범퍼를 지날 때까지 유지
  LaneMemoryParams p;
  LaneConeMemory m(p);
  m.update({{0.6, 0.25}});
  m.update({{0.6, 0.25}});
  m.advance(0.4);                       // s = 0.2: 사각지대 (visible_s_min 0.45)
  for (int k = 0; k < 5; ++k) m.update({});   // 안 보여도 놓침으로 세지 않음
  auto c = m.cones_sl();
  CHECK(c.size() == 1);
  CHECK_NEAR(c[0].x, 0.2, 1e-9);
  CHECK_NEAR(c[0].y, 0.25, 1e-9);       // 옆 위치는 그대로
  m.advance(0.6);                       // s = -0.4: 아직 뒷범퍼(−0.45) 옆
  CHECK(m.cones_sl().size() == 1);
  m.advance(0.2);                       // s = -0.6 < −(0.45+0.10) → 삭제
  CHECK(m.cones_sl().empty());
}

TEST(single_detection_needs_confirmation) {
  LaneMemoryParams p;
  p.confirm_hits = 2;
  LaneConeMemory m(p);
  m.update({{1.2, 0.1}});
  CHECK(m.cones_sl().empty());
  m.update({{1.21, 0.1}});
  CHECK(m.cones_sl().size() == 1);
}

TEST(one_bad_observation_does_not_drag_held_cone) {   // 화면 끝 가림으로 한 번 크게 틀린 관측이 위치를 끌고 가지 않음
  LaneMemoryParams p;
  LaneConeMemory m(p);
  for (int k = 0; k < 8; ++k) m.update({{0.9, 0.34}}, {0.99});   // 가까이서 여러 번 정상 관측
  m.update({{0.9, 0.24}}, {0.99});                                // 앞 콘 깊이로 잡힌 나쁜 관측 (10 cm 안쪽)
  const auto c = m.cones_sl();
  CHECK(c.size() == 1);
  CHECK(std::abs(c[0].y - 0.34) < 0.03);
}
