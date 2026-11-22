#include "check.hpp"
#include "auto_control/drive/core/traffic_stop.hpp"
#include <cmath>

using namespace auto_control::drive;

static const double kNaN = std::nan("");

TEST(hsv_color_rule_picks_red_and_green) {
  cv::Mat red(10, 10, CV_8UC3, cv::Scalar(20, 20, 230)), green(10, 10, CV_8UC3, cv::Scalar(40, 220, 40)), dark(10, 10, CV_8UC3, cv::Scalar(20, 20, 20));
  CHECK(classify_traffic_color(red, 80, 60) == TrafficSignal::Red);
  CHECK(classify_traffic_color(green, 80, 60) == TrafficSignal::Green);
  CHECK(classify_traffic_color(dark, 80, 60) == TrafficSignal::Unknown);
}

// 1 m/s로 정지선(앞차축 1.5 m)에 접근하며 매 프레임 같은 정지선을 관측
static TrafficState approach(TrafficStopper & ts, TrafficSignal sig, int frames, double & t, double & d, double v = 1.0) {
  TrafficState s;
  for (int k = 0; k < frames; ++k) {
    ts.on_signal(sig, t);
    s = ts.update(1.5 - d, t, d, v);
    t += 0.02; d += v * 0.02;
  }
  return s;
}

TEST(red_light_with_stop_line_caps_speed_and_holds_until_green) {
  TrafficStopper ts; ts.set_params(TrafficParams{});
  double t = 0, d = 0;
  auto s = approach(ts, TrafficSignal::Red, 5, t, d);
  CHECK(s.line_tracked);
  CHECK(s.committed);
  CHECK(s.cap < 1.6 && s.cap > 0.5);
  // 이후 빨간불이 안 보여도(UNKNOWN) 상한을 그대로 따라 달려서 정지 (Q2 a). 정지선이 가까워지면 관측은 끊기고 이동 거리로 추적
  double v = 1.0;
  for (int k = 0; k < 600; ++k) {
    ts.on_signal(TrafficSignal::Unknown, t);
    const double axle = 1.5 - d;
    s = ts.update(axle - 0.05 > 0.15 ? axle : kNaN, t, d, v);
    v = std::min(1.0, s.cap);
    if (v < 0.03) v = 0.0;
    t += 0.02; d += v * 0.02;
  }
  const double bumper = 1.5 - d - 0.05;
  CHECK(s.committed);
  CHECK(bumper >= 0.0 && bumper <= 0.25);
  CHECK(s.holding);
  CHECK_NEAR(s.cap, 0.0, 1e-12);
  // 초록불 (연속 3프레임) → 출발
  for (int k = 0; k < 3; ++k) { t += 0.02; ts.on_signal(TrafficSignal::Green, t); s = ts.update(1.5 - d, t, d, 0.0); }
  CHECK(!s.committed);
  CHECK(std::isinf(s.cap));
}

TEST(unknown_signal_does_not_stop) {   // Q1 a: 신호등이 안 보이면 정지선이 있어도 지나감
  TrafficStopper ts; ts.set_params(TrafficParams{});
  double t = 0, d = 0;
  const auto s = approach(ts, TrafficSignal::Unknown, 10, t, d);
  CHECK(s.line_tracked);
  CHECK(!s.committed);
  CHECK(std::isinf(s.cap));
}

TEST(stop_line_needs_confirmation_and_disabled_never_caps) {
  TrafficStopper ts; ts.set_params(TrafficParams{});
  ts.on_signal(TrafficSignal::Red, 0.0);
  CHECK(!ts.update(1.0, 0.0, 0.0, 1.0).line_tracked);   // 1회 관측만으론 추적 안 함
  CHECK(std::isinf(ts.update(kNaN, 0.02, 0.02, 1.0).cap));
  TrafficParams off; off.enabled = false;
  TrafficStopper ts2; ts2.set_params(off);
  double t = 0, d = 0;
  CHECK(std::isinf(approach(ts2, TrafficSignal::Red, 10, t, d).cap));
}

TEST(duty_only_releases_motor_near_stop_point) {
  TrafficParams p; p.duty_only = true; p.decel = 0.4;
  TrafficStopper ts; ts.set_params(p);
  double t = 0, d = 0;
  for (int k = 0; k < 3; ++k) { ts.on_signal(TrafficSignal::Red, t); ts.update(0.26, t, d, 0.3); t += 0.02; }
  // 앞범퍼 → 정지선 0.21, 목표점(정지선 0.10 앞)까지 0.11: 0.33 m/s로 굴러가는 거리 ≈ 0.12 → 모터 풀기
  const auto s = ts.update(0.26, t, d, 0.33);
  CHECK(s.committed);
  CHECK_NEAR(s.cap, 0.0, 1e-12);
}

// ---- 2026-11-22 정책 ----
TEST(red_alone_without_line_keeps_driving_then_stops_when_line_found) {   // 정책 1·2
  TrafficStopper ts; ts.set_params(TrafficParams{});
  double t = 0, d = 0;
  for (int k = 0; k < 20; ++k) {   // 빨간불만, 정지선 없음 → 정상 속도 (상한 없음), 정지선 찾는 중
    ts.on_signal(TrafficSignal::Red, t);
    const auto s = ts.update(kNaN, t, d, 1.0);
    CHECK(!s.committed);
    CHECK(std::isinf(s.cap));
    if (k >= 2) CHECK(s.red_pending);
    t += 0.02; d += 0.02;
  }
  for (int k = 0; k < 20; ++k) { ts.on_signal(TrafficSignal::Unknown, t); ts.update(kNaN, t, d, 1.0); t += 0.02; d += 0.02; }   // 신호등이 화면 밖으로
  TrafficState s;
  for (int k = 0; k < 3; ++k) { s = ts.update(2.5 - (d - 0.8), t, d, 1.0); t += 0.02; d += 0.02; }   // 정지선 발견 (기억 안의 거리)
  CHECK(s.committed);
  CHECK(std::isfinite(s.cap));
}

TEST(red_memory_expires_after_distance) {
  TrafficParams p; p.red_memory_m = 1.0;
  TrafficStopper ts; ts.set_params(p);
  double t = 0, d = 0;
  for (int k = 0; k < 3; ++k) { ts.on_signal(TrafficSignal::Red, t); ts.update(kNaN, t, d, 1.0); t += 0.02; d += 0.02; }
  for (int k = 0; k < 100; ++k) { ts.on_signal(TrafficSignal::Unknown, t); ts.update(kNaN, t, d, 1.0); t += 0.02; d += 0.02; }   // 2 m 주행
  TrafficState s;
  for (int k = 0; k < 4; ++k) { s = ts.update(1.5, t, d, 1.0); t += 0.02; }
  CHECK(!s.red_pending);
  CHECK(!s.committed);
}

TEST(single_red_frame_is_not_confirmed) {   // 정책 5: 연속 3프레임
  TrafficStopper ts; ts.set_params(TrafficParams{});
  double t = 0, d = 0;
  TrafficState s;
  for (int k = 0; k < 10; ++k) {
    ts.on_signal(k % 2 ? TrafficSignal::Red : TrafficSignal::Unknown, t);   // 깜빡이는 오검출
    s = ts.update(1.5 - d, t, d, 1.0);
    t += 0.02; d += 0.02;
  }
  CHECK(s.line_tracked);
  CHECK(!s.red_pending);
  CHECK(!s.committed);
}

TEST(too_close_to_stop_passes_the_line) {   // 정책 3
  TrafficStopper ts; ts.set_params(TrafficParams{});   // 최대 감속 1.0, 1.5 m/s: 정지 거리 0.15 + 1.13 = 1.28 m > 정지선 0.5 m
  double t = 0, d = 0;
  TrafficState s;
  for (int k = 0; k < 5; ++k) { ts.on_signal(TrafficSignal::Red, t); s = ts.update(0.6 - d, t, d, 1.5); t += 0.02; d += 0.03; }
  CHECK(s.passing);
  CHECK(!s.committed);
  CHECK(std::isinf(s.cap));
  for (int k = 0; k < 40; ++k) { ts.on_signal(TrafficSignal::Red, t); s = ts.update(0.6 - d > 0.1 ? 0.6 - d : kNaN, t, d, 1.5); t += 0.02; d += 0.03; }
  CHECK(!s.committed);   // 지나가는 동안 다시 정지하지 않음
}

TEST(committed_stop_keeps_line_by_distance_and_waits_for_green) {   // 정책 4·6
  TrafficStopper ts; ts.set_params(TrafficParams{});
  double t = 0, d = 0;
  TrafficState s;
  for (int k = 0; k < 4; ++k) { ts.on_signal(TrafficSignal::Red, t); s = ts.update(1.5 - d, t, d, 1.0); t += 0.02; d += 0.02; }
  CHECK(s.committed);
  double v = 1.0;
  for (int k = 0; k < 1500; ++k) {   // 정지선·신호등 둘 다 안 보임 (30 s)
    ts.on_signal(TrafficSignal::Unknown, t);
    s = ts.update(kNaN, t, d, v);
    v = std::min(1.0, s.cap); if (v < 0.03) v = 0.0;
    t += 0.02; d += v * 0.02;
  }
  CHECK(s.committed);
  CHECK(s.holding);
  const double bumper = 1.5 - d - 0.05;
  CHECK(bumper >= 0.0 && bumper <= 0.25);
  for (int k = 0; k < 2; ++k) { ts.on_signal(TrafficSignal::Green, t); s = ts.update(kNaN, t, d, 0.0); t += 0.02; }
  CHECK(s.committed);   // 초록 2프레임으로는 아직
  ts.on_signal(TrafficSignal::Green, t); s = ts.update(kNaN, t, d, 0.0);
  CHECK(!s.committed);
  CHECK(std::isinf(s.cap));
}
