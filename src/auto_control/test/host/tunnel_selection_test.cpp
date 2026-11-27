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
