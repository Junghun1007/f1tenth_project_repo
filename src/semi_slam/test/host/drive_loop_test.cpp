#include "check.hpp"
#include "semi_slam/core/drive_loop.hpp"

using namespace semi_slam;

static LaneObservation lane_at(double t) {
  LaneObservation l;
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.02) { l.path.push_back({x, 0.0}); l.left.push_back({x, 0.33}); l.right.push_back({x, -0.33}); }
  l.capture_time = t;
  l.valid = true;
  return l;
}

TEST(constant_speed_run_then_stop_hands_over_to_manual) {
  DriveLoop d(RuntimeConfig{});
  Settings s;
  s.run_speed = 1.2;
  d.apply_settings(s);
  double t = 100;
  d.set_attitude({true, 0.2, 0, 6, "PASS", ""});
  d.set_model_status({"lane", "READY", "", "", 0});
  d.on_inspection_done(true, "", t);
  d.on_vesc_connected(true);
  auto step = [&](double v) { t += 0.02; d.on_speed(v, t); d.on_heartbeat("web", t); return d.on_lane(lane_at(t), t); };
  step(0);
  CHECK(d.request_start(t).first);
  auto o = step(0);
  CHECK(o.handover_request == "auto");
  d.on_handover_mode("AUTO", t);
  o = step(0);
  CHECK(o.publish);
  CHECK(o.duty > 0.0);
  CHECK_NEAR(d.snapshot(t).vehicle.target, 0.0 + d.snapshot(t).vehicle.target, 1e-12);
  for (int k = 0; k < 100; ++k) o = step(1.2);
  CHECK_NEAR(d.snapshot(t).vehicle.target, 1.2, 1e-9);   // 코너 감속 없이 일정 속도
  d.request_stop("test", t);
  for (int k = 0; k < 50; ++k) o = step(0.0);
  CHECK(o.handover_request == "manual" || d.state() == DriveState::Stopping || d.state() == DriveState::Manual);
}
