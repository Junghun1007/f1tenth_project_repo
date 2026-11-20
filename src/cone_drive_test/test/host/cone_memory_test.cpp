#include "check.hpp"
#include "fixture.hpp"
#include "cone_drive_test/core/cone_memory.hpp"
#include <algorithm>

using namespace cone_drive_test;

static MemoryParams params_from(const nlohmann::json & P) {
  const auto & pp = P["proposed"];
  const auto & pc = P["perception"];
  MemoryParams m;
  m.gate = pp["gate"]; m.gate_radial_max = pp["gate_radial_max"]; m.gate_lateral = pp["gate_lateral"];
  m.merge_distance = pp["merge_distance"]; m.confirm_hits = pp["confirm_hits"]; m.miss_delete = pp["miss_delete"];
  m.delete_behind = pp["delete_behind"];
  m.range_noise_base = pc["range_noise_base"]; m.range_noise_per_m = pc["range_noise_per_m"];
  m.bev_x_max = pc["bev_x_max"]; m.bev_y_max = pc["bev_y_max"];
  m.camera_x = P["camera_x"];
  const auto & vp = P["vehicle"];
  m.front_axle_to_rear_bumper = vp["wheelbase"].get<double>() + vp["rear_overhang"].get<double>();
  return m;
}

static ConeObservation obs(Vec2 c) { ConeObservation o; o.center = c; return o; }

static std::vector<int> match_indices(const std::vector<Vec2> & centers, const std::vector<Vec2> & expected) {
  std::vector<int> out;
  for (const auto & e : expected)
    for (std::size_t i = 0; i < centers.size(); ++i)
      if (std::hypot(centers[i].x - e.x, centers[i].y - e.y) < 1e-6) { out.push_back(int(i)); break; }
  return out;
}

TEST(memory_matches_simulator) {
  const auto fx = load_fixture("memory.json");
  const auto P = params_from(fx["params"]);
  ConeMemory mem(P);
  std::string scenario;
  int compared = 0;
  for (const auto & st : fx["steps"]) {
    if (st["scenario"] != scenario) { mem.clear(); scenario = st["scenario"]; }
    std::vector<ConeObservation> dets;
    for (const auto & d : st["detections"]) dets.push_back(obs(v2(d)));
    const auto expected = v2list(st["expected_world"]);
    const Pose2 pose{st["pose"][0], st["pose"][1], st["pose"][2]};
    mem.update(pose, st["t"].get<double>(), dets,
               [&](const std::vector<Vec2> & centers, const Pose2 &) { return match_indices(centers, expected); });
    auto got = mem.confirmed_world();
    const auto want = v2list(st["confirmed"]);
    CHECK(got.size() == want.size());
    for (const auto & w : want) {
      double best = 1e9;
      for (const auto & g : got) best = std::min(best, std::hypot(g.x - w.x, g.y - w.y));
      CHECK(best < 1e-3);
    }
    ++compared;
  }
  CHECK(compared > 300);
}

TEST(keeps_cone_after_leaving_view_and_drops_behind) {
  MemoryParams P;
  ConeMemory mem(P);
  const auto none = [](const std::vector<Vec2> &, const Pose2 &) { return std::vector<int>{}; };
  for (int k = 0; k < 3; ++k) mem.update({0, 0, 0}, k * 0.03, {obs({0.5, 0.25})}, none);
  CHECK(mem.confirmed_world().size() == 1);
  mem.update({0.6, 0, 0}, 1.0, {}, none);              // 차량 기준 x=-0.1: 시야 밖이지만 유지
  auto pos = mem.confirmed_world();
  CHECK(pos.size() == 1);
  CHECK_NEAR(pos[0].x, 0.5, 1e-6);
  CHECK_NEAR(pos[0].y, 0.25, 1e-6);
  mem.update({1.1, 0, 0}, 2.0, {}, none);              // 뒷범퍼(0.45)+0.10 뒤 → 삭제
  CHECK(mem.confirmed_world().empty());
}

TEST(merges_noisy_duplicates) {
  MemoryParams P;
  ConeMemory mem(P);
  const auto none = [](const std::vector<Vec2> &, const Pose2 &) { return std::vector<int>{}; };
  int k = 0;
  for (double dx : {0.0, 0.06, -0.06, 0.05, -0.04, 0.06}) mem.update({0, 0, 0}, 0.03 * k++, {obs({2.0 + dx, 0.25})}, none);
  CHECK(mem.confirmed_world().size() == 1);
}

TEST(missed_when_expected_then_deleted) {
  MemoryParams P;
  ConeMemory mem(P);
  const auto none = [](const std::vector<Vec2> &, const Pose2 &) { return std::vector<int>{}; };
  const auto all = [](const std::vector<Vec2> & c, const Pose2 &) { std::vector<int> v(c.size()); for (std::size_t i = 0; i < c.size(); ++i) v[i] = int(i); return v; };
  for (int k = 0; k < 3; ++k) mem.update({0, 0, 0}, k * 0.03, {obs({1.0, 0.1})}, none);
  for (int k = 0; k < P.miss_delete; ++k) mem.update({0, 0, 0}, 0.1 + k * 0.03, {}, all);
  CHECK(mem.confirmed_world().empty());
}

TEST(update_reports_track_ids_per_observation) {
  ConeMemory mem;
  const auto none = [](const std::vector<Vec2> &, const Pose2 &) { return std::vector<int>{}; };
  std::vector<std::uint32_t> ids;
  mem.update({0, 0, 0}, 0.0, {obs({0.5, 0.25}), obs({0.9, -0.2})}, none, &ids);
  CHECK(ids.size() == 2 && ids[0] != ids[1]);
  std::vector<std::uint32_t> ids2;
  mem.update({0, 0, 0}, 0.03, {obs({0.9, -0.2})}, none, &ids2);
  CHECK(ids2.size() == 1 && ids2[0] == ids[1]);
  CHECK(mem.find(ids[1]) != nullptr);
}
