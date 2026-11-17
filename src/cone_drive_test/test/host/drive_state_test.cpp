#include "check.hpp"
#include "cone_drive_test/core/drive_state.hpp"

using namespace cone_drive_test;

static Readiness all_ok() { return {true, true, true, true, true, true, true}; }

// 점검 완료 → drive 모드 → 클라이언트 접속 → START → 안전 프로세스 AUTO 수락 → DRIVING
static DriveStateMachine driving(double & now) {
  DriveStateMachine sm(DriveMode::Drive, 1.0);
  sm.on_inspection_done(true, now);
  sm.on_heartbeat("a", now);
  CHECK(sm.request_start(all_ok(), now).first);
  CHECK(sm.state() == DriveState::Arming);
  CHECK(sm.wants_auto_rearm());
  sm.on_handover_mode("AUTO", now);
  CHECK(sm.state() == DriveState::Driving);
  return sm;
}

TEST(detect_mode_never_allows_auto) {
  DriveStateMachine sm(DriveMode::Detect, 1.0);
  sm.on_inspection_done(true, 0.0);
  sm.on_heartbeat("a", 0.0);
  const auto r = sm.request_start(all_ok(), 0.0);
  CHECK(!r.first);
  CHECK(r.second.find("detect") != std::string::npos);
  sm.on_handover_mode("AUTO", 0.1);
  CHECK(!sm.auto_output_allowed());
  CHECK(sm.state() == DriveState::Ready);
}

TEST(start_requires_all_readiness) {
  const char * names[] = {"attitude", "lane model", "yolo model", "vesc", "erpm", "lane input"};
  for (int k = 0; k < 6; ++k) {
    DriveStateMachine sm(DriveMode::Drive, 1.0);
    sm.on_inspection_done(true, 0.0);
    sm.on_heartbeat("a", 0.0);
    Readiness r = all_ok();
    bool * flags[] = {&r.attitude_ok, &r.lane_model_ready, &r.yolo_model_ready, &r.vesc_connected, &r.erpm_fresh, &r.lane_fresh};
    *flags[k] = false;
    const auto res = sm.request_start(r, 0.0);
    CHECK(!res.first);
    CHECK(res.second.find(names[k]) != std::string::npos);
  }
}

TEST(start_refused_without_any_client) {   // Review Focus 3
  DriveStateMachine sm(DriveMode::Drive, 1.0);
  sm.on_inspection_done(true, 0.0);
  const auto r = sm.request_start(all_ok(), 0.0);
  CHECK(!r.first);
  CHECK(r.second.find("browser") != std::string::npos);
}

TEST(multi_client_heartbeat) {   // Review Focus 3
  double now = 0.0;
  auto sm = driving(now);
  sm.on_heartbeat("b", now);
  for (int k = 1; k <= 15; ++k) { now = k * 0.1; sm.on_heartbeat("b", now); sm.tick(all_ok(), 1.0, now); }   // a는 1.5 s 끊김
  CHECK(sm.state() == DriveState::Driving);
  const double last_b = now;
  now = last_b + 1.05;
  sm.tick(all_ok(), 1.0, now);
  CHECK(sm.state() == DriveState::Stopping);
  CHECK(sm.last_reason().find("browser") != std::string::npos);
}

TEST(arming_times_out_without_safety_ack) {
  DriveStateMachine sm(DriveMode::Drive, 1.0);
  sm.on_inspection_done(true, 0.0);
  sm.on_heartbeat("a", 0.0);
  CHECK(sm.request_start(all_ok(), 0.0).first);
  sm.on_heartbeat("a", 1.4);
  sm.tick(all_ok(), 0.0, 1.5);
  CHECK(sm.state() == DriveState::Ready);
  CHECK(!sm.auto_output_allowed());
}

TEST(lane_loss_while_driving_stops) {   // Review Focus 2
  double now = 0.0;
  auto sm = driving(now);
  Readiness r = all_ok();
  r.lane_fresh = false;
  sm.tick(r, 1.0, 0.1);
  CHECK(sm.state() == DriveState::Stopping);
  CHECK(sm.last_reason().find("lane") != std::string::npos);
  CHECK(sm.auto_output_allowed());   // 정지 중 제동 명령은 계속 낸다
}

TEST(stopping_to_manual_after_standstill) {
  double now = 0.0;
  auto sm = driving(now);
  sm.request_stop("user stop", 0.0);
  CHECK(sm.state() == DriveState::Stopping);
  for (int k = 1; k <= 4; ++k) { sm.on_heartbeat("a", k * 0.1); sm.tick(all_ok(), 0.03, k * 0.1); }
  CHECK(!sm.wants_manual_handover());   // 0.4 s만 정지
  for (int k = 5; k <= 7; ++k) { sm.on_heartbeat("a", k * 0.1); sm.tick(all_ok(), 0.03, k * 0.1); }
  CHECK(sm.wants_manual_handover());
  sm.on_handover_mode("MANUAL", 0.8);
  CHECK(sm.state() == DriveState::Manual);
  CHECK(!sm.auto_output_allowed());
}

TEST(joystick_x_while_driving_stops) {
  double now = 0.0;
  auto sm = driving(now);
  sm.on_handover_mode("STOPPING", 0.1);
  CHECK(sm.state() == DriveState::Stopping);
  CHECK(!sm.auto_output_allowed());   // 안전 프로세스가 자동 명령을 차단한 상태
  sm.on_handover_mode("MANUAL", 0.5);
  CHECK(sm.state() == DriveState::Manual);
}

TEST(no_auto_restart_after_recovery) {
  double now = 0.0;
  auto sm = driving(now);
  Readiness r = all_ok();
  r.erpm_fresh = false;
  sm.tick(r, 1.0, 0.1);
  CHECK(sm.state() == DriveState::Stopping);
  sm.on_heartbeat("a", 0.2);
  sm.tick(all_ok(), 1.0, 0.2);
  CHECK(sm.state() == DriveState::Stopping);
}

TEST(refit_while_driving_stops_first) {   // Review Focus 5
  double now = 0.0;
  auto sm = driving(now);
  CHECK(sm.request_refit(0.0).first);
  CHECK(sm.state() == DriveState::Stopping);
  for (int k = 1; k <= 6; ++k) { sm.on_heartbeat("a", k * 0.1); sm.tick(all_ok(), 0.0, k * 0.1); }
  sm.on_handover_mode("MANUAL", 0.7);
  CHECK(sm.state() == DriveState::Refitting);
  CHECK(sm.refit_pending());
  sm.on_refit_done(false, 2.0);
  CHECK(sm.state() == DriveState::Ready);
  CHECK(sm.last_reason().find("refit failed") != std::string::npos);
}

TEST(refit_from_ready_is_immediate) {
  DriveStateMachine sm(DriveMode::Detect, 1.0);
  sm.on_inspection_done(true, 0.0);
  CHECK(sm.request_refit(0.0).first);
  CHECK(sm.state() == DriveState::Refitting);
  sm.on_refit_done(true, 1.0);
  CHECK(sm.state() == DriveState::Ready);
}

TEST(mode_change_only_when_stopped) {
  double now = 0.0;
  auto sm = driving(now);
  CHECK(!sm.request_mode(DriveMode::Detect, 0.0).first);
  DriveStateMachine s2(DriveMode::Detect, 1.0);
  s2.on_inspection_done(true, 0.0);
  CHECK(s2.request_mode(DriveMode::Drive, 0.0).first);
  CHECK(s2.mode() == DriveMode::Drive);
}

TEST(inspection_failure_blocks_start) {
  DriveStateMachine sm(DriveMode::Drive, 1.0);
  sm.on_heartbeat("a", 0.0);
  sm.on_inspection_done(false, 0.0);
  CHECK(sm.state() == DriveState::Inspecting);
  CHECK(!sm.request_start(all_ok(), 0.0).first);
}

TEST(arming_ignores_periodic_manual_status) {   // I3: 안전 프로세스가 주기적으로 다시 보내는 MANUAL은 거절이 아님
  DriveStateMachine sm(DriveMode::Drive, 1.0);
  sm.on_inspection_done(true, 0.0);
  sm.on_heartbeat("a", 0.0);
  CHECK(sm.request_start(all_ok(), 0.0).first);
  sm.on_handover_mode("MANUAL", 0.1);
  CHECK(sm.state() == DriveState::Arming);
  sm.on_handover_mode("AUTO", 0.2);
  CHECK(sm.state() == DriveState::Driving);
}

TEST(stray_auto_is_handed_back_to_manual) {   // I3: 출발 승인 없이 안전 프로세스가 AUTO면 수동 전환 요청
  DriveStateMachine sm(DriveMode::Drive, 1.0);
  sm.on_inspection_done(true, 0.0);
  sm.on_handover_mode("AUTO", 0.1);
  CHECK(sm.state() == DriveState::Ready);
  CHECK(sm.wants_manual_handover());
  CHECK(!sm.auto_output_allowed());
}
