#include "check.hpp"
#include "auto_control/drive/core/control_loop.hpp"
#include <atomic>
#include <chrono>
#include <thread>

using namespace auto_control::drive;

static LaneObservation lane_at(double t) {
  LaneObservation l;
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.02) { l.path.push_back({x, 0.0}); l.left.push_back({x, 0.33}); l.right.push_back({x, -0.33}); }
  l.capture_time = t;
  l.valid = true;
  return l;
}

struct Rig {
  ParamRegistry params;
  RuntimeConfig rc;
  std::unique_ptr<ControlLoop> loop;
  double t{100.0};
  explicit Rig(DriveMode mode) {
    register_params(params);
    loop = std::make_unique<ControlLoop>(mode, rc);
    loop->apply_settings(settings_from(params));
    loop->set_attitude({true, 0.20, 0.0, 13.0, "PASS"});
    loop->set_model_status(0, {"lane", "READY", "ready", "", 1.0});
    loop->set_model_status(1, {"yolo", "READY", "ready", "", 1.0});
    loop->on_inspection_done(true, "", t);
    loop->on_vesc_connected(true, t);
  }
  StepOutput step(double speed_mps = 0.0) {
    t += 0.025;
    loop->on_speed(speed_mps, t);
    loop->on_yaw_rate(0.0, t);
    loop->on_heartbeat("web", t);
    return loop->on_lane(lane_at(t - 0.03), t);
  }
};

TEST(detect_mode_never_publishes) {
  Rig r(DriveMode::Detect);
  for (int k = 0; k < 10; ++k) CHECK(!r.step().publish);
  CHECK(!r.loop->request_start(r.t).first);
}

TEST(tunnel_override_requires_fresh_standstill_and_manual_handover) {
  Rig r(DriveMode::Drive);
  CHECK(!r.loop->tunnel_override_allowed(r.t));
  r.step();
  CHECK(r.loop->tunnel_override_allowed(r.t));
  CHECK(!r.loop->tunnel_override_allowed(r.t + r.rc.erpm_timeout_sec + 0.01));
  r.step(0.5);
  CHECK(!r.loop->tunnel_override_allowed(r.t));
  for (int k = 0; k < 15; ++k) r.step();
  CHECK(r.loop->tunnel_override_allowed(r.t));
  CHECK(r.loop->request_start(r.t).first);
  CHECK(!r.loop->tunnel_override_allowed(r.t));
  r.loop->on_handover_mode("AUTO", r.t);
  CHECK(!r.loop->tunnel_override_allowed(r.t));
  r.loop->request_stop("test", r.t);
  CHECK(!r.loop->tunnel_override_allowed(r.t));
  r.loop->on_handover_mode("MANUAL", r.t);
  CHECK(r.loop->tunnel_override_allowed(r.t));
}

TEST(drive_flow_start_to_commands) {
  Rig r(DriveMode::Drive);
  r.step();
  const auto res = r.loop->request_start(r.t);
  CHECK(res.first);
  auto out = r.step();
  CHECK(out.handover_request == "auto");
  CHECK(!out.publish);                            // 안전 프로세스가 AUTO를 알리기 전
  r.loop->on_handover_mode("AUTO", r.t);
  out = r.step();
  CHECK(out.publish);
  CHECK(out.duty > 0.0);                          // 정지 출발 시작 duty
  CHECK_NEAR(out.servo, r.rc.steering.servo_center, 0.02);
  CHECK(r.loop->telemetry(true, r.t).state == "DRIVING");
}

TEST(stop_brakes_then_requests_manual) {
  Rig r(DriveMode::Drive);
  r.step();
  r.loop->request_start(r.t);
  r.step();
  r.loop->on_handover_mode("AUTO", r.t);
  for (int k = 0; k < 40; ++k) r.step(0.8);
  r.loop->request_stop("user", r.t);
  auto out = r.step(0.8);
  CHECK(out.publish && out.brake_current > 0.0 && out.duty == 0.0);
  std::string req;
  for (int k = 0; k < 40 && req.empty(); ++k) req = r.step(0.0).handover_request;
  CHECK(req == "manual");
}

TEST(stale_detection_is_dropped) {   // Review Focus 1
  Rig r(DriveMode::Detect);
  for (int k = 0; k < 5; ++k) r.step();
  DetectionFrame old;
  old.capture_time = r.t - 5.0;                   // 오도메트리 이력(2 s)보다 오래됨
  ConeObservation c; c.center = {1.0, 0.0};
  old.cones.push_back(c);
  r.loop->on_detections(old);
  r.step();
  CHECK(r.loop->dropped_detections() == 1);
  CHECK(r.loop->telemetry(true, r.t).memory.empty());
  DetectionFrame future = old;
  future.capture_time = r.t + 1.0;
  r.loop->on_detections(future);
  r.step();
  CHECK(r.loop->dropped_detections() == 2);
}

TEST(cone_in_lane_triggers_avoidance_plan) {
  Rig r(DriveMode::Detect);
  for (int k = 0; k < 4; ++k) {
    r.step();
    DetectionFrame d;
    d.capture_time = r.t - 0.01;
    ConeObservation c; c.center = {1.5, 0.21}; c.score = 0.9f;
    d.cones.push_back(c);
    r.loop->on_detections(d);
  }
  r.step();
  const auto tf = r.loop->telemetry(true, r.t);
  CHECK(tf.memory.size() == 1);
  CHECK(tf.plan_mode != "center");
  CHECK(r.loop->cones_nearby());
}

TEST(params_apply_reaches_controllers) {
  Rig r(DriveMode::Drive);
  CHECK(r.params.apply({{"drive.maximum_speed_mps", 1.4}}).first);
  r.loop->apply_settings(settings_from(r.params));
  CHECK_NEAR(r.loop->settings().lon.maximum_speed_mps, 1.4, 1e-12);
}

TEST(registry_has_both_groups_and_ordering) {
  ParamRegistry p;
  register_params(p);
  int obstacle = 0, driving = 0;
  for (const auto & s : p.list()) { obstacle += s.group == "obstacle"; driving += s.group == "driving"; }
  CHECK(obstacle >= 20 && driving >= 15);
  CHECK(!p.apply({{"drive.minimum_speed_mps", 1.5}, {"drive.maximum_speed_mps", 1.2}}).first);
}

TEST(lane_gap_within_timeout_keeps_driving) {   // I1: 50 ms 넘는 한 번의 차선 지연에 제동하지 않음
  Rig r(DriveMode::Drive);
  r.step();
  r.loop->request_start(r.t);
  r.step();
  r.loop->on_handover_mode("AUTO", r.t);
  for (int k = 0; k < 40; ++k) r.step(0.8);
  StepOutput out;
  for (int k = 0; k < 5; ++k) {            // 100 ms 차선 없음 (timeout 0.25 s 안)
    r.t += 0.02;
    r.loop->on_speed(0.8, r.t);
    r.loop->on_yaw_rate(0.0, r.t);
    r.loop->on_heartbeat("web", r.t);
    out = r.loop->tick(r.t);
  }
  CHECK(out.publish);
  CHECK(out.brake_current == 0.0);
  CHECK(out.duty > 0.0);
  CHECK(r.loop->state() == DriveState::Driving);
}

TEST(detection_without_depth_does_not_erase_memory) {   // I2
  Rig r(DriveMode::Detect);
  auto put = [&](bool depth, bool with_cone) {
    r.step();
    DetectionFrame d;
    d.capture_time = r.t - 0.01;
    d.depth_valid = depth;
    if (with_cone) { ConeObservation c; c.center = {0.45, 0.0}; c.score = 0.9f; d.cones.push_back(c); }
    r.loop->on_detections(d);
  };
  for (int k = 0; k < 4; ++k) put(true, true);
  r.step();
  CHECK(r.loop->telemetry(true, r.t).memory.size() == 1);
  for (int k = 0; k < 10; ++k) put(false, false);   // 깊이 없는 프레임: 관측 안 함으로 취급
  r.step();
  CHECK(r.loop->telemetry(true, r.t).memory.size() == 1);
}

TEST(roi_change_updates_visibility_geometry) {   // I7
  Rig r(DriveMode::Detect);
  CHECK(r.params.apply({{"yolo.roi_height_percent", 40.0}}).first);
  r.loop->apply_settings(settings_from(r.params));
  CHECK_NEAR(r.loop->camera_geometry().roi_height_ratio, 0.40, 1e-12);
}

TEST(imu_update_not_blocked_by_planning) {   // IMU 스레드(안정화·오도메트리)가 회피 계획을 기다리지 않아야 한다
  Rig r(DriveMode::Detect);
  for (int k = 0; k < 4; ++k) {             // 차선을 막는 콘 벽 → 매 프레임 세 단계 계획 (가장 무거운 경우)
    r.step();
    DetectionFrame d;
    d.capture_time = r.t - 0.01;
    for (double l = -0.3; l <= 0.31; l += 0.12) { ConeObservation c; c.center = {1.5, l}; c.score = 0.9f; d.cones.push_back(c); }
    r.loop->on_detections(d);
  }
  r.step();
  CHECK(r.loop->telemetry(true, r.t).plan_mode != "center");
  std::atomic<bool> run{true};
  const double t0 = r.t;
  std::thread planner([&] { double t = t0; while (run) { t += 0.001; r.loop->on_lane(lane_at(t), t); } });
  double worst = 0, t = t0;
  for (int k = 0; k < 3000; ++k) {
    t += 0.0001;
    const auto a = std::chrono::steady_clock::now();
    r.loop->on_yaw_rate(0.0, t);
    worst = std::max(worst, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count());
  }
  run = false;
  planner.join();
  std::cout << "  worst on_yaw_rate " << worst << " ms\n";
  CHECK(worst < 0.3);
}

TEST(lane_memory_mode_holds_passed_cone_without_gyro) {   // 차선 기준 단기 기억: 자이로가 틀려도 옆 위치 유지
  Rig r(DriveMode::Detect);
  CHECK(r.params.apply({{"memory.mode", 1.0}}).first);
  r.loop->apply_settings(settings_from(r.params));
  for (int k = 0; k < 3; ++k) {
    r.step(0.0);
    DetectionFrame d;
    d.capture_time = r.t - 0.005;
    ConeObservation c; c.center = {0.8, 0.25}; c.score = 0.9f;
    d.cones.push_back(c);
    r.loop->on_detections(d);
  }
  r.step(0.0);
  // 1 m/s로 0.5 s 전진하는 동안 자이로는 큰 가짜 회전(0.5 rad/s)을 보고하고, 콘은 보이지 않음
  for (int k = 0; k < 20; ++k) {
    r.t += 0.025;
    r.loop->on_speed(1.0, r.t);
    r.loop->on_yaw_rate(0.5, r.t);
    r.loop->on_heartbeat("web", r.t);
    r.loop->on_lane(lane_at(r.t - 0.005), r.t);
  }
  const auto tf = r.loop->telemetry(true, r.t);
  CHECK(tf.memory.size() == 1);
  CHECK_NEAR(tf.memory[0].y, 0.25, 0.03);          // 월드 기억이었다면 회전 오차로 크게 밀림
  CHECK(tf.memory[0].x < 0.4);                      // 전진한 만큼 뒤로
}

TEST(speed_read_does_not_wait_for_planning) {   // IMU 스레드의 정지 판정(speed())도 계획을 기다리지 않음
  Rig r(DriveMode::Detect);
  for (int k = 0; k < 4; ++k) {
    r.step(0.5);
    DetectionFrame d;
    d.capture_time = r.t - 0.01;
    for (double l = -0.3; l <= 0.31; l += 0.12) { ConeObservation c; c.center = {1.5, l}; c.score = 0.9f; d.cones.push_back(c); }
    r.loop->on_detections(d);
  }
  std::atomic<bool> run{true};
  const double t0 = r.t;
  std::thread planner([&] { double t = t0; while (run) { t += 0.001; r.loop->on_lane(lane_at(t), t); } });
  double worst = 0;
  for (int k = 0; k < 3000; ++k) {
    const auto a = std::chrono::steady_clock::now();
    volatile double v = r.loop->speed();
    (void)v;
    worst = std::max(worst, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count());
  }
  run = false;
  planner.join();
  CHECK(worst < 0.3);
  CHECK_NEAR(r.loop->speed(), 0.5, 0.1);
}

static void see_cone(Rig & r, Vec2 at, int frames) {
  for (int k = 0; k < frames; ++k) {
    r.step(0.0);
    DetectionFrame d;
    d.capture_time = r.t - 0.005;
    ConeObservation c; c.center = at; c.score = 0.9f;
    d.cones.push_back(c);
    r.loop->on_detections(d);
  }
  r.step(0.0);
}

TEST(lane_check_modes_apply_the_width_rule_only_in_mode_two) {
  for (const int mode : {0, 1, 2}) {
    Rig r(DriveMode::Detect);
    CHECK(!r.params.apply({{"lane.check_mode", 1.5}}).first);
    CHECK(r.params.apply({{"lane.check_mode", double(mode)}, {"drive.lane_timeout_sec", 0.15}}).first);
    r.loop->apply_settings(settings_from(r.params));
    see_cone(r, {1.1, 0.21}, 5);
    CHECK(r.loop->telemetry(true, r.t).plan_mode != "center");

    for (int i = 0; i < 8; ++i) {
      r.t += 0.025;
      r.loop->on_speed(0.0, r.t);
      r.loop->on_yaw_rate(0.0, r.t);
      auto noise = lane_at(r.t - 0.005);
      for (auto & p : noise.path) p.y = 0.45;
      for (auto & p : noise.left) p.y = 0.60;
      for (auto & p : noise.right) p.y = 0.30;
      CHECK(!r.loop->on_lane(noise, r.t).publish);
    }
    const auto frame = r.loop->telemetry(true, r.t);
    CHECK_NEAR(frame.lane_path.front().y, mode == 2 ? 0.0 : 0.45, 1e-12);
    CHECK(frame.readiness.lane_fresh == (mode != 2));

    if (mode == 2) {
      r.t += 0.025;
      r.loop->on_yaw_rate(0.0, r.t);
      auto ir_lane = lane_at(r.t - 0.005);
      for (auto & point : ir_lane.path) point.y -= 0.3;
      r.loop->on_lane(ir_lane, r.t, LaneSource::IR);
      const auto switched = r.loop->telemetry(true, r.t);
      CHECK_NEAR(switched.lane_path.front().y, -0.3, 1e-12);
      CHECK(switched.readiness.lane_fresh);
    }
  }
}

static void drive_blind(Rig & r, int frames) {   // 1 m/s 전진, 콘 검출 없음
  for (int k = 0; k < frames; ++k) {
    r.t += 0.025;
    r.loop->on_speed(1.0, r.t);
    r.loop->on_yaw_rate(0.0, r.t);
    r.loop->on_heartbeat("web", r.t);
    r.loop->on_lane(lane_at(r.t - 0.005), r.t);
  }
}

TEST(visible_only_mode_locks_path_when_cone_leaves_view) {   // 모드 2: 사각지대 콘은 계획에 안 넣고 경로 고정
  Rig r(DriveMode::Detect);
  CHECK(r.params.apply({{"memory.mode", 2.0}}).first);
  r.loop->apply_settings(settings_from(r.params));
  see_cone(r, {0.8, 0.22}, 6);   // 차선 기준 기억은 memory.lane_confirm_hits(기본 4)번 넘게 봐야 계획에 씀
  auto tf = r.loop->telemetry(true, r.t);
  CHECK(tf.plan_mode != "center");
  CHECK(tf.lock_m == 0.0);
  drive_blind(r, 22);   // 약 0.45 m 전진 → 콘이 사각지대(s < 0.45)로
  tf = r.loop->telemetry(true, r.t);
  CHECK(tf.lock_m > 0.3);
  CHECK(tf.plan_mode != "center");
  drive_blind(r, 50);   // 뒷범퍼가 지나면 고정 해제 → 중앙선
  tf = r.loop->telemetry(true, r.t);
  CHECK(tf.lock_m == 0.0);
  CHECK(tf.plan_mode == "center");
}

TEST(no_memory_mode_forgets_unseen_cone) {   // 모드 3: 안 보이면 바로 잊는다
  Rig r(DriveMode::Detect);
  CHECK(r.params.apply({{"memory.mode", 3.0}}).first);
  r.loop->apply_settings(settings_from(r.params));
  see_cone(r, {1.2, 0.22}, 2);
  auto tf = r.loop->telemetry(true, r.t);
  CHECK(tf.memory.empty());               // 기억이 없으니 분홍(기억 콘)도 없음
  CHECK(tf.detections.size() == 1);
  CHECK(tf.plan_mode != "center");
  drive_blind(r, 16);   // 0.4 s 검출 없음
  tf = r.loop->telemetry(true, r.t);
  CHECK(tf.plan_mode == "center");
}

TEST(steering_holds_while_stopped_behind_lead) {   // 앞차 뒤 정지 중엔 차선 흔들림을 따라 조향하지 않음
  Rig r(DriveMode::Drive);
  r.step();
  CHECK(r.loop->request_start(r.t).first);
  r.step();
  r.loop->on_handover_mode("AUTO", r.t);
  for (int k = 0; k < 3; ++k) {   // 정지 거리에 서 있는 앞차
    r.step(0.0);
    DetectionFrame d;
    d.capture_time = r.t - 0.005;
    ConeObservation car; car.class_id = 2; car.length = 0.5; car.width = 0.3; car.center = {0.30 + 0.25, 0.0}; car.score = 0.9f;
    d.cones.push_back(car);
    r.loop->on_detections(d);
  }
  r.step(0.0);
  CHECK(r.loop->telemetry(true, r.t).lead.holding);
  double first = -1, worst = 0;
  for (int k = 0; k < 40; ++k) {   // 차선이 좌우로 4 cm씩 흔들림
    r.t += 0.025;
    r.loop->on_speed(0.0, r.t);
    r.loop->on_yaw_rate(0.0, r.t);
    r.loop->on_heartbeat("web", r.t);
    DetectionFrame d;   // 앞차는 계속 보인다 (안 보이면 lost_timeout 뒤 크루즈 종료)
    d.capture_time = r.t - 0.005;
    ConeObservation car; car.class_id = 2; car.length = 0.5; car.width = 0.3; car.center = {0.30 + 0.25, 0.0}; car.score = 0.9f;
    d.cones.push_back(car);
    r.loop->on_detections(d);
    auto lane = lane_at(r.t - 0.005);
    const double off = (k % 2 ? 0.04 : -0.04);
    for (auto & p : lane.path) p.y += off;
    const auto out = r.loop->on_lane(lane, r.t);
    if (first < 0) first = out.servo;
    worst = std::max(worst, std::abs(out.servo - first));
  }
  CHECK(worst < 1e-6);
}

TEST(red_light_and_stop_line_reach_control_cap) {   // 차선 결과의 정지선 + 검출의 빨간불 → 속도 상한
  Rig r(DriveMode::Detect);
  for (int k = 0; k < 5; ++k) {
    r.t += 0.025;
    r.loop->on_speed(1.0, r.t);
    r.loop->on_yaw_rate(0.0, r.t);
    r.loop->on_heartbeat("web", r.t);
    r.loop->on_traffic_signal(TrafficSignal::Red, r.t - 0.01);
    auto lane = lane_at(r.t - 0.005);
    lane.stop_line_m = 1.2;
    r.loop->on_lane(lane, r.t);
  }
  const auto tf = r.loop->telemetry(true, r.t);
  CHECK(tf.traffic_enabled);
  CHECK(tf.traffic.signal == TrafficSignal::Red);
  CHECK(tf.traffic.committed);
  CHECK(tf.traffic.cap < 1.5);
}

TEST(gyro_bias_learned_while_stopped_and_cone_log_written) {   // 정지 중 자이로 영점을 배우고, 콘 관측 기록에 id·자세 시각이 남는다
  Rig r(DriveMode::Detect);
  for (int k = 0; k < 400; ++k) {   // 10 s 정지, 자이로 영점 0.003 rad/s
    r.t += 0.025;
    r.loop->on_speed(0.0, r.t);
    r.loop->on_yaw_rate(0.003, r.t);
  }
  CHECK_NEAR(r.loop->gyro_bias(), 0.003, 2e-4);
  std::vector<ConeLogRow> rows;
  r.loop->on_cone_log = [&](const std::vector<ConeLogRow> & v) { rows.insert(rows.end(), v.begin(), v.end()); };
  for (int k = 0; k < 3; ++k) {
    r.step();
    DetectionFrame d;
    d.capture_time = r.t - 0.02;
    d.depth_time = r.t - 0.01;   // 깊이 영상 시각으로 자세를 찾는다
    ConeObservation c; c.center = {0.6, 0.1}; c.score = 0.9f; d.cones.push_back(c);
    r.loop->on_detections(d);
  }
  r.step();
  CHECK(rows.size() >= 2);
  CHECK(rows.front().id == rows.back().id);
  CHECK_NEAR(rows.back().t_pose, rows.back().t_depth + 0.015, 1e-9);   // 깊이 시각 + object.time_offset_ms (기본 15 ms)
  CHECK_NEAR(rows.back().world_x, 0.6, 0.02);
}
