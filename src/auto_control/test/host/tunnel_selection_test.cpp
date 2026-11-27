#include "check.hpp"
#include "auto_control/tunnel_selection.hpp"

namespace {
using auto_control::tunnel::LaneSelector;
using auto_control::tunnel::PathRules;

constexpr std::int64_t kEntry = 1'000'000'000;

struct Lane {
  struct Point { double x, y, z; };
  struct Header {
    struct Stamp { std::int32_t sec; std::uint32_t nanosec; } stamp;
    std::string frame_id{"front_axle_bev"};
  } header;
  int source_width{120}, source_height{300}, padding_left{0}, padding_right{0}, state{3};
  double centerline_bev_width_m{1.2}, centerline_bev_height_m{3.0};
  bool centerline_valid{true}, centerline_sample_limit_reached{false};
  std::vector<Point> centerline_points;
  std::vector<std::uint8_t> centerline_support;
};

Lane lane_at(std::int64_t stamp, int points) {
  Lane lane;
  lane.header.stamp = {static_cast<std::int32_t>(stamp / kEntry),
    static_cast<std::uint32_t>(stamp % kEntry)};
  for (int i = 0; i < points; ++i) {
    lane.centerline_points.push_back({59.5, 289.5 - 2 * i, 0.});
    lane.centerline_support.push_back(2);
  }
  return lane;
}
}  // namespace

TEST(tcn_entry_selects_ir_before_any_lane_result) {
  for (bool validation : {true, false}) {
    LaneSelector selector(.2, validation, PathRules{});
    CHECK(selector.event("entry", kEntry, kEntry));
    CHECK(selector.state == "INSIDE");
    CHECK(selector.selected == "IR");
    CHECK(selector.camera() == "IR" && selector.compute() == "IR");
    CHECK(!selector.transitioning());
    CHECK(selector.reason == "tcn_entry");
    CHECK(!selector.lane("RGB", lane_at(kEntry + 1, 9), kEntry + 2));
    auto ir = lane_at(kEntry + 3, 0);
    ir.centerline_valid = false;
    CHECK(selector.lane("IR", ir, kEntry + 4));
    CHECK(!selector.lane("IR", ir, kEntry + 5));
  }
}

TEST(tcn_exit_selects_rgb_before_any_lane_result) {
  for (bool validation : {true, false}) {
    LaneSelector selector(.2, validation, PathRules{});
    CHECK(selector.event("entry", kEntry, kEntry));
    CHECK(selector.event("exit", 2 * kEntry, 2 * kEntry));
    CHECK(selector.state == "OUTSIDE" && selector.reason == "tcn_exit");
    CHECK(selector.selected == "RGB" && selector.compute() == "RGB" && selector.camera() == "RGB");
    CHECK(!selector.transitioning());
    CHECK(!selector.lane("IR", lane_at(2 * kEntry + 1, 9), 2 * kEntry + 2));
    CHECK(selector.lane("RGB", lane_at(kEntry + 1, 9), 2 * kEntry + 2));   // 오래된 결과도 넘김 (신선도 정책 없음)
    CHECK(!selector.lane("RGB", lane_at(kEntry + 1, 9), 2 * kEntry + 2));  // 같은 시각을 두 번 넘기지는 않음
    CHECK(selector.selected == "RGB");
    auto invalid = lane_at(2 * kEntry + 3, 0);
    invalid.centerline_valid = false;
    invalid.header.frame_id = "wrong_frame";
    CHECK(selector.lane("RGB", invalid, 2 * kEntry + 4));
    CHECK(!selector.lane("RGB", invalid, 2 * kEntry + 5));
  }
}

TEST(manual_ir_request_switches_immediately_without_a_lane) {   // 전환 정책 제거: 경로를 기다리지 않음
  LaneSelector selector(.2, false, PathRules{});
  CHECK(selector.request_source("IR", kEntry));
  CHECK(selector.state == "INSIDE" && selector.selected == "IR" && selector.reason == "force_ir_on");
  CHECK(selector.camera() == "IR" && selector.compute() == "IR" && !selector.transitioning());
  CHECK(selector.lane("IR", lane_at(kEntry + 1, 0), kEntry + 2));       // 점이 없는 결과도 그대로 넘김
  CHECK(!selector.lane("RGB", lane_at(kEntry + 1, 9), kEntry + 2));
  CHECK(!selector.request_source("IR", kEntry + 3));                     // 이미 IR
  CHECK(selector.request_source("RGB", kEntry + 5));
  CHECK(selector.state == "OUTSIDE" && selector.reason == "force_ir_off");
}

TEST(tcn_reports_are_followed_immediately_and_out_of_order_reports_are_ignored) {
  LaneSelector selector(.2, false, PathRules{});
  CHECK(selector.event("entry", 5 * kEntry, 50 * kEntry));   // 45초 지난 보고여도 따른다 (나이 정책 없음)
  CHECK(selector.selected == "IR");
  CHECK(!selector.event("exit", 4 * kEntry, 50 * kEntry));   // 더 이전 시각의 보고는 무시
  CHECK(selector.selected == "IR");
  CHECK(selector.event("exit", 6 * kEntry, 50 * kEntry));
  CHECK(selector.selected == "RGB" && selector.state == "OUTSIDE");
}

// ---- 실험 기능: 사전 준비(prearm), 유효 경로 전환(valid handover) ----
namespace {
using auto_control::tunnel::TransitionOptions;
constexpr std::int64_t kMs = 1'000'000;

TransitionOptions options(bool entry, bool exit, bool handover) {
  TransitionOptions o;
  o.prearm_entry = entry;
  o.prearm_exit = exit;
  o.valid_handover = handover;
  return o;
}
Lane invalid_at(std::int64_t stamp) { auto l = lane_at(stamp, 9); l.centerline_valid = false; return l; }
}  // namespace

TEST(default_options_keep_immediate_switch_and_ignore_scores) {
  LaneSelector selector(.2, true, PathRules{});
  CHECK(!selector.scores(.9, .1, true, kEntry));
  CHECK(selector.state == "OUTSIDE" && selector.compute() == "RGB");
  CHECK(selector.lane("RGB", lane_at(kEntry, 9), kEntry));
  CHECK(selector.event("entry", kEntry, kEntry));
  CHECK(selector.state == "INSIDE" && selector.selected == "IR");
}

TEST(prearm_entry_starts_ir_without_selecting_it_and_cancels_after_hold) {
  LaneSelector selector(.2, true, PathRules{}, options(true, false, false));
  CHECK(!selector.scores(.2, .0, true, kEntry));               // 기준 아래
  CHECK(!selector.scores(.4, .0, false, kEntry));              // WARMUP 점수는 쓰지 않음
  CHECK(selector.scores(.35, .0, true, kEntry));
  CHECK(selector.state == "ENTERING" && selector.reason == "prearm_entry");
  CHECK(selector.selected == "RGB" && selector.camera() == "IR" && selector.compute() == "BOTH");
  CHECK(selector.lane("RGB", lane_at(kEntry + 1, 9), kEntry + 1));   // 선택은 그대로 RGB
  CHECK(!selector.lane("IR", lane_at(kEntry + 2, 9), kEntry + 2));
  CHECK(!selector.scores(.1, .0, true, kEntry + 900 * kMs));   // 1초 유지 전에는 취소하지 않음
  CHECK(selector.state == "ENTERING");
  CHECK(selector.tick(kEntry + 1000 * kMs));
  CHECK(selector.state == "OUTSIDE" && selector.reason == "prearm_cancelled" && selector.compute() == "RGB");
}

TEST(prearm_entry_then_tcn_entry_selects_ir) {
  LaneSelector selector(.2, true, PathRules{}, options(true, false, false));
  CHECK(selector.scores(.35, .0, true, kEntry));
  CHECK(selector.event("entry", kEntry + 200 * kMs, kEntry + 200 * kMs));
  CHECK(selector.state == "INSIDE" && selector.selected == "IR" && !selector.prearmed());
  CHECK(!selector.tick(kEntry + 5000 * kMs));                  // 끝난 준비는 다시 취소되지 않음
  CHECK(selector.state == "INSIDE");
}

TEST(prearm_exit_only_when_enabled) {
  LaneSelector entry_only(.2, true, PathRules{}, options(true, false, false));
  CHECK(entry_only.event("entry", kEntry, kEntry));
  CHECK(!entry_only.scores(.0, .9, true, kEntry + 1));
  CHECK(entry_only.state == "INSIDE");
  LaneSelector both(.2, true, PathRules{}, options(true, true, false));
  CHECK(both.event("entry", kEntry, kEntry));
  CHECK(both.scores(.0, .4, true, kEntry + 1));
  CHECK(both.state == "EXITING" && both.selected == "IR" && both.camera() == "IR" && both.compute() == "BOTH");
  CHECK(both.event("exit", 2 * kEntry, 2 * kEntry));
  CHECK(both.state == "OUTSIDE" && both.selected == "RGB");
}

TEST(valid_handover_keeps_valid_rgb_until_first_valid_ir) {
  LaneSelector selector(.2, true, PathRules{}, options(false, false, true));
  CHECK(selector.lane("RGB", lane_at(kEntry, 9), kEntry));      // 지금 출처(RGB) 유효
  CHECK(selector.event("entry", kEntry + 10 * kMs, kEntry + 10 * kMs));
  CHECK(selector.transitioning() && selector.state == "ENTERING" && selector.selected == "RGB");
  CHECK(selector.camera() == "IR" && selector.compute() == "BOTH");
  CHECK(!selector.lane("IR", invalid_at(kEntry + 20 * kMs), kEntry + 20 * kMs));   // IR 무효 → RGB 유지
  CHECK(selector.lane("RGB", lane_at(kEntry + 30 * kMs, 9), kEntry + 30 * kMs));
  CHECK(selector.lane("IR", lane_at(kEntry + 40 * kMs, 9), kEntry + 40 * kMs));    // 첫 유효 IR을 넘기며 전환
  CHECK(selector.state == "INSIDE" && selector.selected == "IR" && selector.reason == "first_valid_target");
  CHECK(!selector.lane("RGB", lane_at(kEntry + 50 * kMs, 9), kEntry + 50 * kMs));
}

TEST(valid_handover_switches_when_current_becomes_invalid_or_times_out) {
  LaneSelector invalid(.2, true, PathRules{}, options(false, false, true));
  CHECK(invalid.lane("RGB", lane_at(kEntry, 9), kEntry));
  CHECK(invalid.event("entry", kEntry + 1, kEntry + 1));
  CHECK(!invalid.lane("RGB", invalid_at(kEntry + 2), kEntry + 2));   // RGB도 무효 → 기다릴 이유 없음
  CHECK(invalid.selected == "IR" && invalid.reason == "current_invalid");

  LaneSelector timeout(.2, true, PathRules{}, options(false, false, true));
  CHECK(timeout.lane("RGB", lane_at(kEntry, 9), kEntry));
  CHECK(timeout.event("entry", kEntry + 1, kEntry + 1));
  CHECK(!timeout.tick(kEntry + 400 * kMs));
  CHECK(timeout.tick(kEntry + 1 + 500 * kMs));
  CHECK(timeout.selected == "IR" && timeout.state == "INSIDE" && timeout.reason == "handover_timeout");

  LaneSelector unknown(.2, true, PathRules{}, options(false, false, true));   // RGB 결과가 아직 없으면 바로 전환
  CHECK(unknown.event("entry", kEntry, kEntry));
  CHECK(unknown.selected == "IR" && !unknown.transitioning());
}

TEST(valid_handover_exit_keeps_ir_until_valid_rgb_and_reversal_cancels) {
  LaneSelector selector(.2, true, PathRules{}, options(false, false, true));
  CHECK(selector.lane("RGB", lane_at(kEntry, 9), kEntry));
  CHECK(selector.event("entry", kEntry + 1, kEntry + 1));
  CHECK(selector.lane("IR", lane_at(kEntry + 2, 9), kEntry + 2));
  CHECK(selector.selected == "IR");
  CHECK(selector.event("exit", 2 * kEntry, 2 * kEntry));
  CHECK(selector.state == "EXITING" && selector.selected == "IR" && selector.camera() == "IR");
  CHECK(!selector.lane("RGB", invalid_at(2 * kEntry + 1), 2 * kEntry + 1));
  CHECK(selector.lane("IR", lane_at(2 * kEntry + 2, 9), 2 * kEntry + 2));
  CHECK(selector.event("entry", 3 * kEntry, 2 * kEntry + 3));      // 기다리는 중 반대 판정 → 취소
  CHECK(selector.state == "INSIDE" && selector.selected == "IR" && selector.reason == "reversed_request");
}

TEST(force_ir_is_immediate_even_with_valid_handover) {
  LaneSelector selector(.2, true, PathRules{}, options(true, true, true));
  CHECK(selector.lane("RGB", lane_at(kEntry, 9), kEntry));
  CHECK(selector.request_source("IR", kEntry + 1));
  CHECK(selector.state == "INSIDE" && selector.selected == "IR" && selector.reason == "force_ir_on");
}
