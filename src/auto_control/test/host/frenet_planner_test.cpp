#include "check.hpp"
#include "planner_params.hpp"
#include "auto_control/drive/core/path_utils.hpp"
#include <chrono>

using namespace auto_control::drive;

static DesiredOffset l_des_from(const nlohmann::json & j) {
  DesiredOffset d;
  for (const auto & v : j[0]) d.s.push_back(v);
  for (const auto & v : j[1]) d.l.push_back(v);
  return d;
}

static bool mirror_symmetric(const nlohmann::json & c) {
  if (std::abs(c["l0"].get<double>()) > 1e-9 || std::abs(c["rear_sl"][1].get<double>()) > 1e-9) return false;
  for (const auto & l : c["l_des"][1]) if (std::abs(l.get<double>()) > 1e-9) return false;
  const auto cones = v2list(c["cones_sl"]);
  for (const auto & a : cones) {
    bool found = false;
    for (const auto & b : cones) found |= std::abs(a.x - b.x) < 1e-9 && std::abs(a.y + b.y) < 1e-9;
    if (!found) return false;
  }
  return true;
}

TEST(dp_matches_simulator) {
  const auto fx = load_fixture("planner.json");
  const auto P = planner_params_from(fx["params"]);
  int n = 0, worst_case = -1;
  double worst = 0;
  for (const auto & c : fx["cases"]) {
    const auto out = frenet_dp(P, c["l0"], v2(c["rear_sl"]), v2list(c["cones_sl"]), c["s_end"], c["margin_extra"],
                               c["lane_hard"], c["relaxed"], l_des_from(c["l_des"]));
    if (c["result"].is_null()) {
      if (out.has_value()) std::cout << "  expected None: " << c["name"] << "\n";
      CHECK(!out.has_value());
    } else {
      if (!out.has_value()) std::cout << "  expected path: " << c["name"] << "\n";
      CHECK(out.has_value());
      const auto & want = c["result"]["l"];
      CHECK(out->l.size() == want.size());
      // 좌우 대칭 입력은 두 우회 경로의 비용이 같아 부동소수점 차이로 방향이 갈릴 수 있다 → 거울상도 허용
      double e_same = 0, e_mirror = 0;
      for (std::size_t i = 0; i < want.size(); ++i) {
        e_same = std::max(e_same, std::abs(out->l[i] - want[i].get<double>()));
        e_mirror = std::max(e_mirror, std::abs(out->l[i] + want[i].get<double>()));
      }
      const double e = std::min(e_same, mirror_symmetric(c) ? e_mirror : e_same);
      if (e > 0.01) std::cout << "  case " << n << " " << c["name"] << " max |dl| " << e << "\n";
      if (e > worst) { worst = e; worst_case = n; }
    }
    ++n;
  }
  std::cout << "  cases " << n << ", worst |dl| " << worst << " (case " << worst_case << ")\n";
  CHECK(worst <= 0.01);
}

TEST(rollout_matches_simulator) {
  const auto fx = load_fixture("rollout.json");
  const auto P = planner_params_from(fx["params"]);
  const auto C = control_params_from(fx["params"]);
  int agree = 0, n = 0;
  for (const auto & c : fx["cases"]) {
    const bool ok = rollout_ok(P, C, v2list(c["path"]), v2list(c["cones_local"]), c["speed"], c["steer"]).ok;
    agree += ok == c["ok"].get<bool>();
    ++n;
  }
  std::cout << "  agree " << agree << "/" << n << "\n";
  CHECK(agree >= n - 1);
}

TEST(dp_leaves_lane_only_when_blocked) {
  PlannerParams P;
  std::vector<Vec2> wall;
  for (double l = -0.3; l <= 0.31; l += 0.12) wall.push_back({1.5, l});
  CHECK(!frenet_dp(P, 0.0, {-0.33, 0.0}, wall, 3.0, 0.0, true, false, {}).has_value());
  const auto free = frenet_dp(P, 0.0, {-0.33, 0.0}, wall, 3.0, 0.0, false, false, {});
  CHECK(free.has_value());
  double m = 0;
  for (double l : free->l) m = std::max(m, std::abs(l));
  CHECK(m > 0.4);
  const auto side = frenet_dp(P, 0.0, {-0.33, 0.0}, {{1.5, 0.21}}, 3.0, 0.0, true, false, {});
  CHECK(side.has_value());
  CHECK(side->l[15] < 0.0);    // s=1.5 (ds_lane 0.1)
}

static std::vector<Vec2> slalom(double spacing) {
  std::vector<Vec2> c;
  for (int k = 0; k < 4; ++k) { const double s = 1.0 + k * spacing, side = k % 2 ? -1 : 1; c.push_back({s, side * 0.21}); c.push_back({s, side * 0.34}); }
  return c;
}

TEST(zigzag_stays_in_lane) {
  PlannerParams P;
  for (double spacing : {0.5, 0.75}) {
    auto out = frenet_dp(P, 0.0, {-0.33, 0.0}, slalom(spacing), 3.0, 0.0, true, false, {});
    if (!out) out = frenet_dp(P, 0.0, {-0.33, 0.0}, slalom(spacing), 3.0, 0.0, true, true, {});
    CHECK(out.has_value());
    double m = 0;
    for (double l : out->l) m = std::max(m, std::abs(l));
    CHECK(m < 0.2);
  }
}

TEST(dp_runtime) {
  PlannerParams P;
  const auto cones = slalom(0.6);
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 100; ++i) frenet_dp(P, 0.0, {-0.33, 0.0}, cones, 3.0, 0.0, true, true, {});
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 100;
  std::cout << "  lane DP avg " << ms << " ms\n";
  CHECK(ms < 3.0);
}

TEST(forward_arc_extension_keeps_turning) {   // 반지름 1 m로 꺾이다 잘린 중앙선 → 늘린 부분은 곡률을 0까지 줄이며 k·L/2만큼 더 꺾인다
  std::vector<Vec2> c;
  for (double a = 0; a <= 0.6 + 1e-9; a += 0.02) c.push_back({std::sin(a), -(1 - std::cos(a))});   // 오른쪽으로 0.6 rad
  const auto e = extend_forward_arc(c, 0.6, 0.25, 1.75);
  const auto n = e.size();
  const double head = std::atan2(e[n - 1].y - e[n - 2].y, e[n - 1].x - e[n - 2].x);
  CHECK_NEAR(head, -(0.6 + 1.0 * 0.6 / 2), 0.05);
  const auto tight = extend_forward_arc(c, 0.6, 0.25, 0.5);   // 곡률 상한으로 덜 꺾임
  CHECK(tight.back().y > e.back().y + 0.02);
}
