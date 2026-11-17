#include "cone_drive_test/core/avoidance_planner.hpp"
#include "cone_drive_test/core/cone_memory.hpp"
#include "cone_drive_test/core/path_utils.hpp"
#include <algorithm>
#include <chrono>
#include <numeric>

namespace cone_drive_test {

const char * plan_mode_name(PlanMode m) {
  switch (m) {
    case PlanMode::Center: return "center";
    case PlanMode::Lane: return "lane";
    case PlanMode::LaneRelaxed: return "lane-tight";
    case PlanMode::Free: return "free";
    default: return "fallback";
  }
}

PlanResult AvoidancePlanner::plan(const LaneObservation & lane, const std::vector<Vec2> & cones_world, const Pose2 & pose,
                                  double speed, double steer, const LanePin * pin,
                                  const std::vector<Vec2> * blind_world) {
  const auto t0 = std::chrono::steady_clock::now();
  PlanResult out;
  const auto & ref = lane.center.size() >= 4 ? lane.center : lane.path;
  if (ref.size() < 4 || lane.path.size() < 2) {
    out.mode = PlanMode::Fallback;
    out.rollout_ok = false;
    return out;
  }
  const auto center = extend_backward(smooth_center(ref), p_.backward_extension);
  auto f = frenet_frame(center);
  const Vec2 origin = project_sl({0.0, 0.0}, f);   // 앞차축의 s를 0으로
  for (auto & a : f.arc) a -= origin.x;
  const double l0 = origin.y;
  std::vector<Vec2> mem_local, cones_sl;
  for (const auto & w : cones_world) {
    mem_local.push_back(to_local(w, pose));
    cones_sl.push_back(project_sl(mem_local.back(), f));
  }
  const Vec2 rear_sl = project_sl({-p_.wheelbase, 0.0}, f);
  const double s_end = std::min(p_.s_max, f.arc.back());
  // line_detactor 경로(코너 바깥 이동)를 기준선 대비 목표 오프셋으로
  const auto drive = smooth_center(lane.path);
  std::vector<Vec2> dsl;
  for (const auto & q : drive) dsl.push_back(project_sl(q, f));
  std::vector<std::size_t> order(dsl.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return dsl[a].x < dsl[b].x; });
  DesiredOffset l_des;
  for (auto i : order) { l_des.s.push_back(dsl[i].x); l_des.l.push_back(dsl[i].y); }

  // 차선 안 경로에 영향을 줄 수 있는 콘만 회피 계획을 켠다: 차선 안 경로의 차체 끝(차선 반폭 + 허용 이탈)에서
  // 권장 여유보다 멀리 있는 콘(도로 밖 콘)은 비용이 0이라 계획해도 중앙선과 같은 결과 → 중앙선을 그대로 쓴다
  const double reach = p_.lane_half + p_.lane_tolerance + p_.soft_clearance + p_.cone_half + 0.02;
  bool near = false;
  for (const auto & c : cones_sl) {
    if (c.x <= -0.6 || c.x >= s_end + 0.3 || std::abs(c.y) > reach) continue;
    near = true;
    if (c.x < 3.0) out.obstacle_active = true;
  }
  const bool pin_active = pin && pin->s.size() >= 2 && pin->end_s > 0;
  if (pin_active) { near = true; out.obstacle_active = true; }   // 화면 밖 콘 옆을 지나는 중: 중앙선 추종으로 바꾸지 않음

  bool have = false;
  if (!near) {
    for (const auto & q : drive) if (q.x >= -0.05) out.path_local.push_back(q);
    out.mode = PlanMode::Center;
    have = true;
  } else {
    // 차선 안(기본 여유) → 차선 안(여유 축소) → 차선 밖. 차선 밖은 차가 지나갈 틈이 아예 없을 때만
    struct Tier { bool lane_hard, relaxed; PlanMode mode; };
    const Tier tiers[] = {{true, false, PlanMode::Lane}, {true, true, PlanMode::LaneRelaxed}, {false, false, PlanMode::Free}};
    const double v_roll = std::max(speed, c_.obstacle_speed);   // 곧 도달할 장애물 구간 속도까지 가정
    std::vector<const LanePin *> pins{pin_active ? pin : nullptr};
    if (pin_active) pins.push_back(nullptr);   // 고정을 지킬 수 없으면(새로 보인 콘과 충돌) 고정 없이 다시
    std::vector<Vec2> all_sl = cones_sl, all_local = mem_local;
    if (blind_world)
      for (const auto & w : *blind_world) { all_local.push_back(to_local(w, pose)); all_sl.push_back(project_sl(all_local.back(), f)); }
    for (const LanePin * use_pin : pins) {
      const auto & csl = use_pin || !pin_active ? cones_sl : all_sl;
      const auto & cloc = use_pin || !pin_active ? mem_local : all_local;
      for (const auto & tier : tiers) {
        std::optional<std::pair<std::vector<Vec2>, bool>> found;
        for (double extra : {0.0, p_.rollout_retry_margin}) {
          const auto ls = frenet_dp(p_, l0, rear_sl, csl, s_end, extra, tier.lane_hard, tier.relaxed, l_des, use_pin);
          if (!ls) break;
          std::vector<Vec2> path;
          path.reserve(ls->s.size());
          for (std::size_t i = 0; i < ls->s.size(); ++i) path.push_back(from_frenet(ls->s[i], ls->l[i], f));
          const bool ok = rollout_ok(p_, c_, path, cloc, v_roll, steer).ok;
          if (!found || ok) found = std::make_pair(std::move(path), ok);
          if (ok) break;   // 충돌 예상이면 여유를 늘려 한 번 더
        }
        if (!found) continue;
        out.path_local = std::move(found->first);
        out.rollout_ok = found->second;
        out.mode = tier.mode;
        out.pinned = use_pin != nullptr;
        have = true;
        break;
      }
      if (have) break;
    }
  }
  if (!have) {
    out.mode = PlanMode::Fallback;
    out.rollout_ok = false;
    if (!last_path_world_.empty()) {   // 계획 실패: 이전 경로 유지
      for (const auto & w : last_path_world_) out.path_local.push_back(to_local(w, pose));
    } else {
      for (std::size_t i = 0; i < f.center.size(); ++i) if (f.arc[i] >= 0) out.path_local.push_back(f.center[i]);
    }
  } else {
    last_path_world_.clear();
    for (const auto & q : out.path_local) last_path_world_.push_back(to_world(q, pose));
  }
  if (!out.rollout_ok) out.speed_cap = c_.rollout_fail_speed;
  out.compute_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return out;
}

}  // namespace cone_drive_test
