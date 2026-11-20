#include "check.hpp"
#include "semi_slam/core/slam.hpp"
#include <algorithm>
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

Sim make_path(const TrackMap & m, int laps, bool cw = false) {
  // 바깥 선 점마다 가장 가까운 안쪽 선과의 중점 → 실제 주행선 (출발선 T1 근처에서 시작하도록 회전)
  std::vector<Vec2> mid;
  for (std::size_t i = 0; i < m.outer.size(); i += 2) {
    const auto & o = m.outer[i];
    Vec2 best = m.inner[0];
    double bd = 1e9;
    for (const auto & q : m.inner) { const double d = std::hypot(q.x - o.x, q.y - o.y); if (d < bd) { bd = d; best = q; } }
    mid.push_back({(o.x + best.x) / 2, (o.y + best.y) / 2});
  }
  if (cw) std::reverse(mid.begin(), mid.end());   // 시계 방향: 같은 주행선을 거꾸로
  std::size_t k0 = 0;   // 아래 직선 시작점 (S보다 약 1 m 앞): 반시계 x 1.0~1.2, 시계 x 2.8~3.0
  for (std::size_t i = 0; i < mid.size(); ++i)
    if (mid[i].y < 1.0 && (cw ? (mid[i].x > 2.8 && mid[i].x <= 3.0) : (mid[i].x >= 1.0 && mid[i].x < 1.2))) { k0 = i; break; }   // 아래 직선 시작점 (S보다 약 1 m 앞)
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

double g_stop_bias = 0.0;   // 정지선 거리 측정 치우침 (− = 실제보다 가깝게 → S 통과를 일찍 판정)
struct SnapErr { double sq{0}, max{0}; int n{0}; double rms() const { return n ? std::sqrt(sq / n) : 0; } };
SnapErr g_snap;

// SlamCore로 실제처럼 실시간 처리 (정지선 바퀴 세기 포함) 후 RunData 반환
RunData simulate(SlamCore & core, const TrackMap & m, int laps, double v, double wheel_err, double gyro_err, double bias_res,
                 double stamp_delay = 0.0, double noise = 0.0, bool fake_corner_stop = false, bool cw = false, double bev_rot = 0.0) {
  unsigned rng = 12345u;
  auto gauss = [&]() { double a = 0; for (int i = 0; i < 6; ++i) { rng = rng * 1664525u + 1013904223u; a += (rng >> 8) / double(1u << 24); } return (a - 3.0) / std::sqrt(0.5); };
  const Sim sim = make_path(m, laps + 1, cw);
  const double bias0 = 0.02;
  g_snap = SnapErr{};
  double t = 100.0;
  for (int k = 0; k < 400; ++k) { core.on_speed(0.0, t); core.on_gyro(bias0, t); t += 0.0025; }   // 출발 전 정지 영점
  Pose start = true_pose(sim, 0.0);
  start.t = t;
  core.begin_run(v, laps, start, cw);
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
      const double cr = std::cos(bev_rot), sr = std::sin(bev_rot);   // BEV가 bev_rot만큼 반시계로 돌아 보임 (카메라 yaw 오차)
      auto local = [&](const Vec2 & w) {
        const double dx = w.x - p.x, dy = w.y - p.y, lx = c * dx + sn * dy, ly = -sn * dx + c * dy;
        return Vec2{cr * lx - sr * ly, sr * lx + cr * ly};
      };
      auto & to_inner = cw ? L.right : L.left;   // 반시계: 안쪽 선이 왼쪽, 시계: 오른쪽
      auto & to_outer = cw ? L.left : L.right;
      for (std::size_t i = 0; i < m.inner.size(); i += 3) { const Vec2 q = local(m.inner[i]); if (q.x > 0.05 && q.x < 1.5 && std::abs(q.y) < 0.6) to_inner.push_back({q.x + noise * gauss(), q.y + noise * gauss()}); }
      for (std::size_t i = 0; i < m.outer.size(); i += 3) { const Vec2 q = local(m.outer[i]); if (q.x > 0.05 && q.x < 1.5 && std::abs(q.y) < 0.6) to_outer.push_back({q.x + noise * gauss(), q.y + noise * gauss()}); }
      const Vec2 sq = local(S);
      if (sq.x > 0.15 && sq.x < 1.5 && std::abs(sq.y) < 0.3) L.stop_line_m = sq.x + g_stop_bias;
      // 오인식: 코너(바퀴의 60~65% 지점)에서 휜 흰 선을 정지선으로 0.6 m 앞에 봤다고 가정
      const double frac = std::fmod(s, sim.s.back() / (laps + 1)) / (sim.s.back() / (laps + 1));
      if (fake_corner_stop && frac > 0.60 && frac < 0.65) L.stop_line_m = 0.6;
      const bool done = core.on_lane(L);
      const auto lv = core.live();
      if (!lv.waiting_s) {   // 위치 추정 오차 (앞차축 실제 위치 대비)
        const double e = std::hypot(lv.pose.x - p.x, lv.pose.y - p.y);
        g_snap.sq += e * e; ++g_snap.n; g_snap.max = std::max(g_snap.max, e);
      }
      if (done) break;   // 목표 바퀴 완료
    }
  }
  core.end_run();
  return core.run_data();
}
}  // namespace

TEST(track_map_loads_and_measures_line_distance) {
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  CHECK(m.outer.size() > 100 && m.inner.size() > 100);
  CHECK_NEAR(m.dist_outer({2.0, 0.165}), 0.0, 0.011);
  CHECK_NEAR(m.dist_inner({2.0, 0.5}), 0.335, 0.011);
  CHECK_NEAR(m.lane_width(), 0.67, 0.011);   // 테이프 안쪽 64 cm + 두께 3 cm
  CHECK(m.lap_length() > 10.0 && m.lap_length() < 13.0);
  const Vec2 d = m.travel_dir_at(m.stop_center());   // 아래쪽 도로를 반시계로 → +x
  CHECK_NEAR(d.x, 1.0, 1e-9);
}

TEST(calibration_value_round_trips_json) {
  OdomCalib c;
  c.gyro_scale = 1.031; c.gyro_laps = 7; c.latency = 0.05;
  c.set_wheel_scale(1.2, 0.97); c.wheel_laps[120] = 4;
  const auto d = OdomCalib::from_json(c.to_json());
  CHECK(d.version == OdomCalib::kVersion);
  CHECK(d.gyro_laps == 7 && d.wheel_laps.at(120) == 4);
  CHECK(OdomCalib::from_json(R"({"gyro_scale":1.02,"cam_yaw":-0.1})").version == 1);   // 예전 형식 → 노드가 초기화
  CHECK_NEAR(d.gyro_scale, 1.031, 1e-6);
  CHECK_NEAR(d.wheel_scale_for(1.2), 0.97, 1e-6);
  CHECK_NEAR(d.latency, 0.05, 1e-6);
}

TEST(laps_are_counted_at_stop_line_and_calibration_recovers_sensor_errors) {
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  SlamCore core(m);
  const auto run = simulate(core, m, 5, 1.0, 1.05, 0.97, 0.0015);   // 잔여 영점: 출발 직전 정지 측정 뒤 남는 정도 (0.09°/s)
  const auto live = core.live();
  CHECK(live.finished);
  CHECK(live.lap == 5);
  CHECK(run.crossing_t.size() == 5);
  const auto r = calibrate(run, OdomCalib{}, m);
  std::printf("  gyro %.4f (truth %.4f) wheel %.4f (truth %.4f) per lap %.2f deg %.3f m\n", r.calib.gyro_scale, 1 / 0.97,
              r.calib.wheel_scale_for(1.0), 1 / 1.05, r.est.raw_turn_deg_per_lap, r.est.raw_dist_per_lap);
  CHECK(r.applied);
  CHECK_NEAR(r.calib.gyro_scale, 1 / 0.97, 0.005);   // 잔여 영점 0.0015 rad/s × 11 s = 바퀴당 0.9° → 0.3%
  CHECK_NEAR(r.calib.wheel_scale_for(1.0), 1 / 1.05, 0.01);
  CHECK(r.calib.gyro_laps == 5);
  CHECK(std::abs(r.est.bev_yaw_residual_deg) < 1.0);   // 카메라 yaw 오차 없음
}

TEST(calibration_accumulates_over_runs_and_rejects_outliers) {
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  SlamCore core(m);
  const auto run = simulate(core, m, 3, 1.0, 1.0, 1.0, 0.0);
  OdomCalib prev;
  prev.gyro_scale = 1.010; prev.gyro_laps = 6;
  prev.set_wheel_scale(1.0, 1.0); prev.wheel_laps[100] = 3;
  const auto r = calibrate(run, prev, m);
  // 자이로: 이전 1.010(6바퀴)과 이번 ≈1.000 차이 1% → 2% 안이라 가중 평균 (6·1.010 + 3·1.000) / 9
  CHECK(r.gyro_used);
  CHECK_NEAR(r.calib.gyro_scale, (6 * 1.010 + 3 * r.est.gyro_scale) / 9, 1e-9);
  CHECK(r.calib.gyro_laps == 9);
  prev.gyro_scale = 1.05;   // 이번 값과 5% 차이 → 반영 안 함
  const auto r2 = calibrate(run, prev, m);
  CHECK(!r2.gyro_used);
  CHECK_NEAR(r2.calib.gyro_scale, 1.05, 1e-12);
}

TEST(position_on_center_line_tracks_true_position) {   // 보정이 맞으면 위치 추정 = 실제 앞차축 위치 (중앙선 거리 + BEV 옆 위치)
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  SlamCore core(m);
  OdomCalib truth;
  truth.gyro_scale = 1 / 0.97; truth.set_wheel_scale(1.0, 1 / 1.05);
  core.set_calib(truth);
  simulate(core, m, 3, 1.0, 1.05, 0.97, 0.0, 0.0, 0.01);
  std::printf("  snap error rms %.3f m max %.3f m (frames %d)\n", g_snap.rms(), g_snap.max, g_snap.n);
  CHECK(g_snap.rms() < 0.03);
  CHECK(g_snap.max < 0.10);
}

TEST(corner_anchoring_bounds_along_error_when_distance_scale_is_off) {   // 거리 배율이 5% 틀려도 코너마다 다시 잡아 바퀴 중간 오차가 쌓이지 않음
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  double max_on = 0, max_off = 0;
  for (const bool on : {true, false}) {
    SlamCore core(m);
    SlamConfig cfg; cfg.corner_anchor = on;
    core.set_config(cfg);
    OdomCalib c; c.gyro_scale = 1 / 0.97;   // 자이로는 맞고 거리만 5% 틀림
    core.set_calib(c);
    simulate(core, m, 2, 1.0, 1.05, 0.97, 0.0);
    (on ? max_on : max_off) = g_snap.max;
  }
  std::printf("  max position error: corner anchoring on %.3f m, off %.3f m\n", max_on, max_off);
  CHECK(max_on < 0.20);
  CHECK(max_off > 0.35);
}

TEST(stop_line_offset_is_estimated_and_used) {   // 정지선을 16 cm 가깝게 재면 S 통과를 16 cm 일찍 판정 → 코너 한가운데와 비교해 찾아냄
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  struct Reset { ~Reset() { g_stop_bias = 0.0; } } reset;   // CHECK 실패로 중간에 끝나도 다음 테스트에 남지 않게
  g_stop_bias = -0.16;
  SlamCore core(m);
  const auto run = simulate(core, m, 4, 1.0, 1.0, 1.0, 0.0);
  const auto r = calibrate(run, OdomCalib{}, m);
  std::printf("  stop offset est %.3f m (truth -0.16), laps %d\n", r.est.stop_offset, r.est.stop_laps);
  CHECK(r.stop_used);
  CHECK_NEAR(r.calib.stop_offset, -0.16, 0.04);
  // 오프셋을 넣고 다시 달리면 위치 추정 오차가 줄어든다
  SlamCore core2(m);
  core2.set_calib(r.calib);
  simulate(core2, m, 2, 1.0, 1.0, 1.0, 0.0);
  const double with = g_snap.rms();
  SlamCore core3(m);
  simulate(core3, m, 2, 1.0, 1.0, 1.0, 0.0);
  std::printf("  snap rms with offset %.3f m, without %.3f m\n", with, g_snap.rms());
  CHECK(with < g_snap.rms());
}

TEST(driving_check_detects_bev_rotation) {   // BEV가 3° 돌아 보이면 주행 검산이 ≈ +3°를 낸다
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  SlamCore core(m);
  const auto run = simulate(core, m, 3, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, false, false, 3.0 * M_PI / 180.0);
  const auto e = estimate_odometry(run, m);
  std::printf("  straights %d angle %.2f slip %.2f residual %.2f deg\n", e.straights, e.straight_angle_deg, e.straight_slip_deg, e.bev_yaw_residual_deg);
  CHECK(e.straights >= 4);
  CHECK_NEAR(e.bev_yaw_residual_deg, 3.0, 0.7);
}

#include "semi_slam/core/lane_post.hpp"
#include <opencv2/imgproc.hpp>

TEST(stop_line_angle_from_mask) {   // 가로선이 5° 반시계로 돌아 보이면 +5°: 왼쪽 끝(작은 열)이 가깝다(큰 행)
  LanePostprocessor post{LanePostConfig()};
  cv::Mat mask(300, 120, CV_8UC1, cv::Scalar(0));
  const double t = std::tan(5.0 * M_PI / 180.0);
  cv::line(mask, cv::Point(10, int(std::lround(250 + 50 * t))), cv::Point(110, int(std::lround(250 - 50 * t))), cv::Scalar(255), 3);
  const double a = post.stop_line_angle(mask) * 180.0 / M_PI;
  std::printf("  stop line angle %.2f deg\n", a);
  CHECK_NEAR(a, 5.0, 0.8);
  CHECK(std::isnan(post.stop_line_angle(cv::Mat(300, 120, CV_8UC1, cv::Scalar(0)))));
}

#include "semi_slam/core/run_store.hpp"
#include <filesystem>
TEST(run_folder_round_trips_and_calibration_store_reverts) {
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  SlamCore core(m);
  auto run = simulate(core, m, 2, 1.2, 1.03, 0.98, 0.004);
  run.imu.push_back({run.start.t + 0.1, 0.1, -0.2, -9.81, 0.01, -0.02, 0.5});
  run.imu.push_back({run.start.t + 0.2, std::nan(""), std::nan(""), std::nan(""), 0.0, 0.0, 0.4});   // 가속도 시각 안 맞음
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
  CHECK(back.imu.size() == 2);
  CHECK_NEAR(back.imu[0].az, -9.81, 1e-9);
  CHECK(std::isnan(back.imu[1].ax));
  CHECK_NEAR(back.imu[1].gz, 0.4, 1e-9);
  // 저장한 기록으로 다시 보정해도 같은 결과 (PC에서 재보정 가능)
  const auto r1 = calibrate(run, OdomCalib{}, m), r2 = calibrate(back, OdomCalib{}, m);
  CHECK(r1.applied);
  CHECK_NEAR(r1.calib.gyro_scale, r2.calib.gyro_scale, 1e-4);
  CHECK_NEAR(r1.calib.wheel_scale_for(1.2), r2.calib.wheel_scale_for(1.2), 1e-4);
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
  store.save(r1.calib);
  const auto backup = store.reset();   // 초기화: 기본값으로, 이전 값·이력은 백업
  CHECK_NEAR(store.load().gyro_scale, 1.0, 1e-12);
  CHECK(store.history().empty());
  CHECK(std::filesystem::exists(backup + "/calibration_history.jsonl"));
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

TEST(clockwise_run_counts_laps_and_calibrates) {   // 시계 방향: 오른쪽 아래(x≈3 m)에서 −x로 출발, 왼쪽 선 = 바깥 흰 선
  const auto m = TrackMap::load_csv(std::string(SLAM_MAPS) + "/track_4x4.csv");
  const Vec2 a = m.straight_start(true), d = m.travel_dir_at(a, true);
  CHECK_NEAR(a.x, 3.0, 0.02);
  CHECK_NEAR(d.x, -1.0, 1e-9);
  SlamCore core(m);
  const auto run = simulate(core, m, 4, 1.0, 1.05, 0.97, 0.0015, 0.0, 0.0, false, true);
  const auto live = core.live();
  CHECK(live.finished);
  CHECK(live.lap == 4);
  CHECK(run.cw);
  const auto r = calibrate(run, OdomCalib{}, m);
  std::printf("  cw wheel %.4f gyro %.4f rms after %.3f\n", r.calib.wheel_scale_for(1.0), r.calib.gyro_scale, r.after.rms_m);
  CHECK(r.applied);
  CHECK_NEAR(r.calib.gyro_scale, 1 / 0.97, 0.005);
  CHECK_NEAR(r.calib.wheel_scale_for(1.0, true), 1 / 1.05, 0.01);   // 시계 방향 표에 따로
  CHECK(r.calib.wheel_scale.empty());
  CHECK_NEAR(r.calib.wheel_scale_for(1.0, false), 1.0, 1e-12);
  CHECK(r.after.rms_m < 0.03);
}
