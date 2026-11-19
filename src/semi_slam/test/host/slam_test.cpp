#include "check.hpp"
#include "semi_slam/core/slam.hpp"
#include <cmath>
#include <cstdio>

using namespace semi_slam;

// 모의 주행: 실제 차는 트랙 가운데(두 흰 선 사이)를 일정 속도로 반시계 주행.
// 센서는 거리 5% 크게, 자이로는 3% 작게 + 잔여 영점 0.01 rad/s로 읽는다고 가정하고,
// 차선 인식은 정답 흰 선을 차량 좌표로 옮긴 점(1.5 m 안)과 정지선 거리를 준다.
namespace {
struct Sim {
  std::vector<Vec2> path;   // 실제 주행선 (출발선에서 시작, 여러 바퀴)
  std::vector<double> s;
};

Sim make_path(const TrackMap & m, int laps) {
  // 바깥 선 점마다 가장 가까운 안쪽 선과의 중점 → 실제 주행선 (출발선 T1 근처에서 시작하도록 회전)
  std::vector<Vec2> mid;
  for (std::size_t i = 0; i < m.outer.size(); i += 2) {
    const auto & o = m.outer[i];
    Vec2 best = m.inner[0];
    double bd = 1e9;
    for (const auto & q : m.inner) { const double d = std::hypot(q.x - o.x, q.y - o.y); if (d < bd) { bd = d; best = q; } }
    mid.push_back({(o.x + best.x) / 2, (o.y + best.y) / 2});
  }
  std::size_t k0 = 0;   // T1 (x 2.03, 아래쪽 도로) 바로 앞 점부터
  for (std::size_t i = 0; i < mid.size(); ++i) if (mid[i].y < 1.0 && mid[i].x >= 1.0 && mid[i].x < 1.2) { k0 = i; break; }   // 아래 직선 시작점 (S보다 약 1 m 앞)
  Sim sim;
  for (int l = 0; l < laps; ++l) for (std::size_t i = 0; i < mid.size(); ++i) sim.path.push_back(mid[(k0 + i) % mid.size()]);
  sim.s.push_back(0);
  for (std::size_t i = 1; i < sim.path.size(); ++i) sim.s.push_back(sim.s.back() + std::hypot(sim.path[i].x - sim.path[i - 1].x, sim.path[i].y - sim.path[i - 1].y));
  return sim;
}

Pose true_pose(const Sim & sim, double s) {
  auto it = std::lower_bound(sim.s.begin(), sim.s.end(), s);
  std::size_t i = std::clamp<std::size_t>(std::size_t(it - sim.s.begin()), 1, sim.path.size() - 1);
  const auto & a = sim.path[i - 1], & b = sim.path[i];
  const double u = std::clamp((s - sim.s[i - 1]) / std::max(1e-9, sim.s[i] - sim.s[i - 1]), 0.0, 1.0);
  // 진행 방향은 앞뒤 5 cm 평균 (코너 다각형 꺾임 완화)
  auto at = [&](double ss) { auto j = std::lower_bound(sim.s.begin(), sim.s.end(), ss); std::size_t k = std::clamp<std::size_t>(std::size_t(j - sim.s.begin()), 0, sim.path.size() - 1); return sim.path[k]; };
  const Vec2 f = at(s + 0.05), r = at(std::max(0.0, s - 0.05));
  return {0, a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), std::atan2(f.y - r.y, f.x - r.x), s};
}

double unwrap_rate(double y1, double y0, double dt) {
  double d = y1 - y0;
  while (d > M_PI) d -= 2 * M_PI;
  while (d < -M_PI) d += 2 * M_PI;
  return d / dt;
}

// SlamCore로 실제처럼 실시간 처리 (정지선 바퀴 세기 포함) 후 RunData 반환
RunData simulate(SlamCore & core, const TrackMap & m, int laps, double v, double wheel_err, double gyro_err, double bias_res,
                 double stamp_delay = 0.0, double noise = 0.0, bool fake_corner_stop = false) {
  unsigned rng = 12345u;
  auto gauss = [&]() { double a = 0; for (int i = 0; i < 6; ++i) { rng = rng * 1664525u + 1013904223u; a += (rng >> 8) / double(1u << 24); } return (a - 3.0) / std::sqrt(0.5); };
  const Sim sim = make_path(m, laps + 1);
  const double bias0 = 0.02;
  double t = 100.0;
  for (int k = 0; k < 400; ++k) { core.on_speed(0.0, t); core.on_gyro(bias0, t); t += 0.0025; }   // 출발 전 정지 영점
  Pose start = true_pose(sim, 0.0);
  start.t = t;
  core.begin_run(v, laps, start);
  const Vec2 S = m.stop_center();
  double s = 0, yaw_prev = start.yaw, next_lane = t, next_speed = t;
  const double dt = 0.0025;
  for (int k = 0; k < 400000; ++k) {
    t += dt; s += v * dt;
    const Pose p = true_pose(sim, s);
    const double w_true = unwrap_rate(p.yaw, yaw_prev, dt);
    yaw_prev = p.yaw;
    if (t >= next_speed) { core.on_speed(v * wheel_err, t); next_speed += 0.02; }
    core.on_gyro(w_true * gyro_err + bias0 + bias_res, t);
    if (t >= next_lane) {
      next_lane += 1.0 / 60.0;
      LaneObservation L;
      L.valid = true;
      L.capture_time = t - stamp_delay;   // 영상 시각이 실제 장면보다 stamp_delay만큼 이르게 찍힘
      const double c = std::cos(p.yaw), sn = std::sin(p.yaw);
      auto local = [&](const Vec2 & w) { const double dx = w.x - p.x, dy = w.y - p.y; return Vec2{c * dx + sn * dy, -sn * dx + c * dy}; };
      for (std::size_t i = 0; i < m.inner.size(); i += 3) { const Vec2 q = local(m.inner[i]); if (q.x > 0.05 && q.x < 1.5 && std::abs(q.y) < 0.6) L.left.push_back({q.x + noise * gauss(), q.y + noise * gauss()}); }
      for (std::size_t i = 0; i < m.outer.size(); i += 3) { const Vec2 q = local(m.outer[i]); if (q.x > 0.05 && q.x < 1.5 && std::abs(q.y) < 0.6) L.right.push_back({q.x + noise * gauss(), q.y + noise * gauss()}); }
      const Vec2 sq = local(S);
      if (sq.x > 0.15 && sq.x < 1.5 && std::abs(sq.y) < 0.3) L.stop_line_m = sq.x;
      // 오인식: 코너(바퀴의 60~65% 지점)에서 휜 흰 선을 정지선으로 0.6 m 앞에 봤다고 가정
      const double frac = std::fmod(s, sim.s.back() / (laps + 1)) / (sim.s.back() / (laps + 1));
      if (fake_corner_stop && frac > 0.60 && frac < 0.65) L.stop_line_m = 0.6;
      if (core.on_lane(L)) break;   // 목표 바퀴 완료
    }
  }
  core.end_run();
  return core.run_data();
}
}  // namespace

TEST(track_map_loads_and_measures_line_distance) {
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  CHECK(m.outer.size() > 100 && m.inner.size() > 100);
  CHECK_NEAR(m.dist_outer({2.0, 0.175}), 0.0, 0.011);
  CHECK_NEAR(m.dist_inner({2.0, 0.5}), 0.325, 0.011);
  CHECK(m.lap_length() > 10.0 && m.lap_length() < 13.0);
  const Vec2 d = m.travel_dir_at(m.stop_center());   // 아래쪽 도로를 반시계로 → +x
  CHECK_NEAR(d.x, 1.0, 1e-9);
}

TEST(calibration_value_round_trips_json) {
  OdomCalib c;
  c.gyro_scale = 1.031; c.gyro_bias = -0.004; c.latency = 0.05;
  c.set_wheel_scale(1.2, 0.97);
  const auto d = OdomCalib::from_json(c.to_json());
  CHECK_NEAR(d.gyro_scale, 1.031, 1e-6);
  CHECK_NEAR(d.wheel_scale_for(1.2), 0.97, 1e-6);
  CHECK_NEAR(d.latency, 0.05, 1e-6);
}

TEST(laps_are_counted_at_stop_line_and_calibration_recovers_sensor_errors) {
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  SlamCore core(m);
  const auto run = simulate(core, m, 5, 1.0, 1.05, 0.97, 0.008);
  const auto live = core.live();
  std::printf("  lap %d finished %d crossings %zu\n", live.lap, int(live.finished), run.crossing_t.size());
  CHECK(live.finished);
  CHECK(live.lap == 5);
  CHECK(run.crossing_t.size() == 5);
  const auto r = calibrate(run, OdomCalib{}, m);
  std::printf("  before rms %.3f m (acc %.1f%%), after rms %.3f m (acc %.1f%%), evals %d\n", r.before.rms_m, r.before.accuracy * 100,
              r.after.rms_m, r.after.accuracy * 100, r.evaluations);
  std::printf("  wheel %.4f (truth %.4f) gyro %.4f (truth %.4f) bias %.4f (truth %.4f) latency %.3f\n", r.calib.wheel_scale_for(1.0), 1 / 1.05,
              r.calib.gyro_scale, 1 / 0.97, r.calib.gyro_bias, 0.008, r.calib.latency);
  CHECK(r.improved);
  CHECK(r.after.rms_m < 0.02);
  CHECK(r.after.accuracy > 0.95);
  CHECK_NEAR(r.calib.wheel_scale_for(1.0), 1 / 1.05, 0.01);
  // 일정 속도·같은 트랙에서는 자이로 배율과 잔여 영점이 서로 일부 대신할 수 있어(궤적이 거의 같음) 넓게 본다
  CHECK_NEAR(r.calib.gyro_scale, 1 / 0.97, 0.015);
  CHECK_NEAR(r.calib.gyro_bias, 0.008, 0.01);
}

#include "semi_slam/core/run_store.hpp"
#include <filesystem>

TEST(run_folder_round_trips_and_calibration_store_reverts) {
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  SlamCore core(m);
  const auto run = simulate(core, m, 2, 1.2, 1.03, 0.98, 0.004);
  const std::string data = std::filesystem::temp_directory_path().string() + "/semi_slam_test_store";
  std::filesystem::remove_all(data);
  const auto dir = make_run_dir(data, 1.2);
  std::size_t cur = 0;
  write_run(dir, run, OdomCalib{}, core.lap_metrics(), core.corrections(), core.track_since(&cur));
  const auto back = load_run(dir);
  CHECK(back.gyro.size() == run.gyro.size());
  CHECK(back.lanes.size() == run.lanes.size());
  CHECK(back.crossing_t.size() == 2);
  CHECK_NEAR(back.gyro_bias0, run.gyro_bias0, 1e-9);
  // 저장한 기록으로 다시 보정해도 같은 결과 (PC에서 재보정 가능)
  const auto r1 = calibrate(run, OdomCalib{}, m), r2 = calibrate(back, OdomCalib{}, m);
  CHECK_NEAR(r1.after.rms_m, r2.after.rms_m, 2e-3);   // 저장 반올림으로 개별 값은 조금 달라도 궤적 정확도는 같음
  write_calibration_result(dir, r1);
  CalibStore store(data);
  const OdomCalib before = store.load();
  store.save(r1.calib);
  store.record(r1, dir, before, 1.2, 2, true);
  CHECK_NEAR(store.load().gyro_scale, r1.calib.gyro_scale, 1e-6);
  OdomCalib reverted;
  CHECK(store.revert(&reverted));
  CHECK_NEAR(reverted.gyro_scale, 1.0, 1e-9);
  CHECK_NEAR(store.load().gyro_scale, 1.0, 1e-9);
  CHECK(!store.revert(&reverted));   // 되돌릴 적용이 더 없음
  CHECK(store.history().size() == 2);
}

TEST(calibration_handles_lane_noise_and_camera_delay) {   // 차선 점 잡음 σ 1.5 cm + 영상 시각 40 ms 지연
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  SlamCore core(m);
  const auto run = simulate(core, m, 5, 1.4, 0.96, 1.02, -0.006, 0.04, 0.015);
  const auto r = calibrate(run, OdomCalib{}, m);
  std::printf("  acc %.1f%% -> %.1f%%  wheel %.4f (truth %.4f) gyro %.4f (truth %.4f) bias %.4f (truth -0.006) latency %.3f (truth 0.040)\n",
              r.before.accuracy * 100, r.after.accuracy * 100, r.calib.wheel_scale_for(1.4), 1 / 0.96, r.calib.gyro_scale, 1 / 1.02, r.calib.gyro_bias, r.calib.latency);
  OdomCalib truth;
  truth.set_wheel_scale(1.4, 1 / 0.96); truth.gyro_scale = 1 / 1.02; truth.gyro_bias = -0.006; truth.latency = 0.04;
  const auto et = evaluate(run, truth, m);
  std::printf("  truth params: acc %.1f%% rms %.4f stop %.4f cost %.6f / found rms %.4f stop %.4f cost %.6f\n", et.accuracy * 100, et.rms_m, et.stop_rms_m, et.cost, r.after.rms_m, r.after.stop_rms_m, r.after.cost);
  CHECK(r.improved);
  CHECK(r.after.accuracy > 0.93);
  CHECK_NEAR(r.calib.wheel_scale_for(1.4), 1 / 0.96, 0.015);
  CHECK_NEAR(r.calib.gyro_scale, 1 / 1.02, 0.01);
  CHECK_NEAR(r.calib.latency, 0.04, 0.025);
}

TEST(false_stop_lines_at_corners_are_not_counted_as_laps) {
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  SlamCore core(m);
  const auto run = simulate(core, m, 4, 1.0, 1.05, 0.97, 0.004, 0.0, 0.0, true);
  const auto live = core.live();
  std::printf("  laps %d crossings %zu rejected_far %d rejected_lap %d\n", live.lap, run.crossing_t.size(), live.rejected_far, live.rejected_lap);
  CHECK(live.lap == 4);
  CHECK(run.crossing_t.size() == 4);
  CHECK(live.rejected_far >= 3);   // 바퀴마다 코너 오인식을 무시
}
