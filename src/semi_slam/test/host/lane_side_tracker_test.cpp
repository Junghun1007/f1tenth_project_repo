#include "check.hpp"
#include "semi_slam/core/lane_side_tracker.hpp"

using namespace semi_slam;
using Frag = std::vector<cv::Point2f>;

static Frag vline(float col, int r0 = 0, int r1 = 299) {
  Frag f;
  for (int r = r0; r <= r1; ++r) f.push_back({col, float(r)});
  return f;
}

TEST(swapped_labels_follow_previous_frame) {   // 직전엔 왼쪽 열 30, 오른쪽 95 → 이번에 모델이 좌우를 뒤집어도 원래대로
  LaneSideTracker tr;
  LaneSideTracker::Paths p{{{vline(30)}, {vline(95)}}};
  tr.apply(p, 300, 0.0);
  LaneSideTracker::Paths q{{{vline(96)}, {vline(31)}}};
  const auto st = tr.apply(q, 300, 0.016);
  CHECK(q[0].size() == 1 && q[1].size() == 1);
  CHECK_NEAR(q[0][0][0].x, 31.0, 1e-6);
  CHECK_NEAR(q[1][0][0].x, 96.0, 1e-6);
  CHECK(st.relabeled == 2);
}

TEST(reacquired_side_needs_lane_width_from_remaining_line) {
  LaneSideTracker tr;
  LaneSideTracker::Paths p{{{vline(30)}, {}}};   // 오른쪽 선이 안 보이는 상태
  tr.apply(p, 300, 0.0);
  LaneSideTracker::Paths q{{{vline(30), vline(95)}, {}}};   // 새 선을 모델이 '왼쪽'이라고 붙임 → 65 px 오른쪽이니 오른쪽
  tr.apply(q, 300, 0.016);
  CHECK(q[0].size() == 1 && q[1].size() == 1);
  CHECK_NEAR(q[1][0][0].x, 95.0, 1e-6);
  LaneSideTracker::Paths w{{{vline(30)}, {vline(70)}}};   // 40 px: 차선 폭도 같은 선도 아님 → 제외
  const auto st = tr.apply(w, 300, 0.032);
  CHECK(w[1].empty());
  CHECK(st.dropped == 1);
}

TEST(width_check_without_history_drops_weaker_side) {   // 기준 없이 폭 30 px(너무 좁음) → 점이 적은 쪽 제외
  LaneSideTracker tr;
  LaneSideTracker::Paths p{{{vline(30)}, {vline(60, 100, 200)}}};
  const auto st = tr.apply(p, 300, 0.0);
  CHECK(p[0].size() == 1);
  CHECK(p[1].empty());
  CHECK(st.dropped == 1);
}

