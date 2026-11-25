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

TEST(tcn_exit_waits_for_rgb_and_reentry_cancels_exit) {
  LaneSelector selector(.2, true, PathRules{}, IrLengthExit{});
  CHECK(selector.event("entry", kEntry, kEntry));
  CHECK(selector.event("exit", kEntry + 1, kEntry + 1));
  CHECK(selector.state == "EXITING" && selector.compute() == "BOTH");
  CHECK(!selector.lane("RGB", lane_at(kEntry + 2, 5), kEntry + 3));
  CHECK(selector.selected == "IR");
  CHECK(selector.event("entry", kEntry + 4, kEntry + 4));
  CHECK(selector.state == "INSIDE" && selector.reason == "reversed_request");
  CHECK(selector.event("exit", kEntry + 5, kEntry + 5));
  CHECK(selector.lane("RGB", lane_at(kEntry + 6, 9), kEntry + 7));
  CHECK(selector.state == "OUTSIDE" && selector.reason == "first_valid_target");
}

TEST(manual_ir_request_waits_for_a_valid_path) {
  LaneSelector selector(.2, true, PathRules{}, IrLengthExit{});
  CHECK(selector.request_source("IR", kEntry));
  CHECK(selector.state == "ENTERING" && selector.selected == "RGB");
  CHECK(!selector.lane("IR", lane_at(kEntry + 1, 5), kEntry + 2));
  CHECK(selector.lane("IR", lane_at(kEntry + 3, 9), kEntry + 4));
  CHECK(selector.state == "INSIDE" && selector.reason == "first_valid_target");
}
