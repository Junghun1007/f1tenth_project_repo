#include "check.hpp"
#include "auto_control/tunnel_selection.hpp"

namespace {
using auto_control::tunnel::IrLengthExit;
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
  bool observed_lane_length_valid{false};
  double observed_lane_length_m{0.0};
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
    LaneSelector selector(.2, validation, PathRules{}, IrLengthExit{});
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
    LaneSelector selector(.2, validation, PathRules{}, IrLengthExit{});
    CHECK(selector.event("entry", kEntry, kEntry));
    CHECK(selector.event("exit", 2 * kEntry, 2 * kEntry));
    CHECK(selector.state == "OUTSIDE" && selector.reason == "tcn_exit");
    CHECK(selector.selected == "RGB" && selector.compute() == "RGB" && selector.camera() == "RGB");
    CHECK(!selector.transitioning());
    CHECK(!selector.lane("IR", lane_at(2 * kEntry + 1, 9), 2 * kEntry + 2));
    CHECK(!selector.lane("RGB", lane_at(kEntry + 1, 9), 2 * kEntry + 2));
    CHECK(selector.selected == "RGB");
    auto invalid = lane_at(2 * kEntry + 3, 0);
    invalid.centerline_valid = false;
    invalid.header.frame_id = "wrong_frame";
    CHECK(selector.lane("RGB", invalid, 2 * kEntry + 4));
    CHECK(!selector.lane("RGB", invalid, 2 * kEntry + 5));
  }
}

TEST(ir_length_exit_selects_rgb_and_reports_its_trigger) {
  LaneSelector selector(.2, true, PathRules{}, IrLengthExit{true});
  CHECK(selector.event("entry", kEntry, kEntry));
  for (int i = 0; i < 6; ++i) {
    const auto stamp = kEntry + (i + 1) * 40'000'000;
    auto ir = lane_at(stamp, 9);
    ir.observed_lane_length_valid = true;
    ir.observed_lane_length_m = i < 3 ? 2.0 : 0.1;
    CHECK(selector.lane("IR", ir, stamp + 1) == (i < 5));
  }
  CHECK(selector.state == "OUTSIDE" && selector.selected == "RGB");
  CHECK(selector.last_handover.find("\"trigger\": \"ir_length_drop\"") != std::string::npos);
  CHECK(!selector.observe_tcn_state("inside", kEntry + 250'000'000, kEntry + 250'000'001));
}

TEST(manual_ir_request_waits_for_a_valid_path_but_release_is_immediate) {
  LaneSelector selector(.2, true, PathRules{}, IrLengthExit{});
  CHECK(selector.request_source("IR", kEntry));
  CHECK(selector.state == "ENTERING" && selector.selected == "RGB");
  CHECK(!selector.lane("IR", lane_at(kEntry + 1, 5), kEntry + 2));
  CHECK(selector.lane("IR", lane_at(kEntry + 3, 9), kEntry + 4));
  CHECK(selector.state == "INSIDE" && selector.reason == "first_valid_target");
  CHECK(selector.request_source("RGB", kEntry + 5));
  CHECK(selector.state == "OUTSIDE" && selector.reason == "force_ir_off");
  CHECK(!selector.transitioning());
}
