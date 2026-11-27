#include "check.hpp"
#include "auto_control/drive/core/corner_planner.hpp"
#include <cmath>

using namespace auto_control::drive;

namespace {
void line(std::vector<Vec2> & v, Vec2 a, Vec2 b) {
  const int n = int(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.02));
  for (int i = 0; i < n; ++i) v.push_back({a.x + (b.x - a.x) * i / n, a.y + (b.y - a.y) * i / n});
}
// 시계 방향(오른쪽) 원호: 중심 c, 반지름 r, 각 a0 → a1
void arc(std::vector<Vec2> & v, Vec2 c, double r, double a0, double a1) {
  const int n = int(std::ceil(std::abs(a1 - a0) * r / 0.02)) + 1;
  for (int i = 0; i < n; ++i) { const double a = a0 + (a1 - a0) * i / n; v.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)}); }
}

// 오른쪽 90° 코너. 안쪽(오른쪽) 차선은 꼭짓점, 바깥(왼쪽) 차선은 반지름 0.9 m — 양쪽 곡률이 다르다
struct Corner { std::vector<Vec2> ref, left, right; };
Corner right_corner() {
  const double w = 0.65, ro = 0.9, xv = 1.2;          // 안쪽 꼭짓점 x
  const Vec2 co{xv + w - ro, w / 2 - ro};              // 바깥 원호 중심
  Corner c;
  line(c.right, {0.25, -w / 2}, {xv, -w / 2});
  line(c.right, {xv, -w / 2}, {xv, -1.0});
  line(c.left, {0.25, w / 2}, {co.x, w / 2});
  arc(c.left, co, ro, M_PI / 2, 0.0);
  line(c.left, {xv + w, co.y}, {xv + w, -1.0});
  line(c.ref, {0.25, 0.0}, {co.x, 0.0});
  arc(c.ref, co, ro - w / 2, M_PI / 2, 0.0);
  line(c.ref, {xv + w / 2, co.y}, {xv + w / 2, -1.0});
  return c;
}
}  // namespace

TEST(straight_lane_keeps_center) {
  std::vector<Vec2> ref, left, right;
  line(ref, {0.25, 0}, {2.8, 0});
  line(left, {0.25, 0.325}, {2.8, 0.325});
  line(right, {0.25, -0.325}, {2.8, -0.325});
  const auto r = plan_corner_path(ref, left, right, 0.8, CornerPlanParams{});
  CHECK(r.ok);
  CHECK_NEAR(r.path.front().x, 0.0, 0.03);
  for (const auto & q : r.path) CHECK_NEAR(q.y, 0.0, 0.01);
  CHECK(r.min_radius > 5.0);
  CHECK(r.min_clearance > 0.1);
  CHECK(!r.infeasible);
}

TEST(sharp_inner_corner_widens_radius_without_touching) {
  const auto c = right_corner();
  const auto r = plan_corner_path(c.ref, c.left, c.right, 0.8, CornerPlanParams{});
  CHECK(r.ok);
  CHECK(r.min_radius > 0.575 + 0.05);   // 차선 중심선(0.575 m)보다 크게 돈다
  CHECK(r.min_clearance >= 0.0);         // 관측한 차선을 밟지 않는다
  CHECK_NEAR(r.path.front().x, 0.0, 0.03);
  CHECK_NEAR(r.path.front().y, 0.0, 0.01);
}

TEST(one_side_missing_is_estimated_from_lane_width) {
  const auto c = right_corner();
  const auto r = plan_corner_path(c.ref, c.left, {}, 0.8, CornerPlanParams{});
  CHECK(r.ok);
  CHECK(r.estimated_fraction > 0.5);
}

TEST(short_or_invalid_reference_is_rejected) {
  CHECK(!plan_corner_path({{0, 0}, {0.1, 0}}, {}, {}, 0.8, CornerPlanParams{}).ok);
  std::vector<Vec2> ref;
  line(ref, {0.25, 0}, {1.0, 0});
  ref[3].x = std::nan("");
  CHECK(!plan_corner_path(ref, {}, {}, 0.8, CornerPlanParams{}).ok);
}

TEST(offset_vehicle_starts_on_itself_and_returns_slowly) {   // 코너 뒤 차선 중앙 복귀: 출발은 차량 위치·방향, 복귀는 천천히
  std::vector<Vec2> ref, left, right;
  line(ref, {0.25, -0.12}, {2.9, -0.12});      // 차량이 차선 중앙보다 왼쪽으로 12 cm
  line(left, {0.25, 0.205}, {2.9, 0.205});
  line(right, {0.25, -0.445}, {2.9, -0.445});
  const auto r = plan_corner_path(ref, left, right, 0.8, CornerPlanParams{});
  CHECK(r.ok);
  CHECK_NEAR(r.path.front().x, 0.0, 0.02);
  CHECK_NEAR(r.path.front().y, 0.0, 0.02);
  const double heading = std::atan2(r.path[2].y - r.path[0].y, r.path[2].x - r.path[0].x);
  CHECK(std::abs(heading) < 2.0 * M_PI / 180.0);   // 출발 방향 = 차량 방향 (차선 중심 쪽으로 꺾어 시작하지 않음)
  for (const auto & q : r.path) if (q.x < 0.3) CHECK(q.y > -0.03);   // 출발 직후 중앙으로 급히 꺾지 않는다
  CHECK(r.min_radius > 1.5);
  for (const auto & q : r.path) if (q.x > 1.6) CHECK_NEAR(q.y, -0.12, 0.04);   // 복귀 거리(0.8 m 이상) 뒤엔 중앙에 와 있다
}
