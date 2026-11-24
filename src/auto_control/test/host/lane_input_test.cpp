#include "check.hpp"
#include "auto_control/drive/core/cone_memory.hpp"
#include "auto_control/drive/core/lane_input.hpp"
#include "auto_control/drive/runtime/detector.hpp"

using namespace auto_control::drive;

static LanePixels straight() {   // 120×300 BEV (1 cm/px), 좌우 패딩 20 px, 중앙(열 80 = 원본 60)에 세로 중앙선
  LanePixels p;
  p.source_width = 120; p.padding_left = 20; p.bev_width_m = 1.2; p.bev_height_m = 3.0; p.centerline_valid = true;
  for (int v = 299; v >= 0; v -= 10) {
    p.centerline.push_back({79.5f, float(v)});
    p.left.push_back({79.5f - 32.5f, float(v)});
    p.right.push_back({79.5f + 32.5f, float(v)});
  }
  return p;
}

TEST(pixels_map_to_front_axle_vehicle_frame) {
  const auto lane = lane_from_pixels(straight(), 1.0);
  CHECK(lane.valid);
  CHECK_NEAR(lane.path.front().x, 3.0 - 299.5 * 0.01, 1e-9);   // 가장 가까운 행 = 거의 0 m
  CHECK_NEAR(lane.path.front().y, 0.0, 1e-9);                     // 패딩을 빼고 가운데
  CHECK_NEAR(lane.left.front().y, 0.325, 1e-9);                   // 왼쪽 = +y
  CHECK_NEAR(lane.right.front().y, -0.325, 1e-9);
  CHECK(lane.left.front().x <= lane.left.back().x);               // 가까운 것부터 정렬
  CHECK(lane.center.empty());                                     // 기준 중앙선은 요청했을 때만
  CHECK(std::isnan(lane.stop_line_m));
}

TEST(invalid_centerline_gives_invalid_lane) {
  auto p = straight();
  p.centerline_valid = false;
  CHECK(!lane_from_pixels(p, 1.0).valid);
}

TEST(avoidance_continuity_reacquires_either_boundary_after_motion_and_noise) {
  constexpr double inside = 0.175, wheelbase = 0.33;
  for (const int side : {-1, 1}) {
    LaneContinuity continuity;
    const Pose2 initial{0.0, 0.0, 0.0}, outside{0.65, side * 0.42, side * 0.30};
    const Pose2 returning{0.9, side * 0.10, 0.0};
    const auto reference = lane_from_pixels(straight(), 1.0);
    CHECK(continuity.accept(reference, initial, false, inside, wheelbase));
    CHECK(!continuity.accept(LaneObservation{}, outside, true, inside, wheelbase));

    auto observe = [&](const Pose2 & pose, double t, double noise, bool correct_side) {
      auto lane = reference;   // Same world lane, viewed from a new vehicle pose.
      lane.capture_time = t;
      if ((side > 0) == correct_side) lane.right.clear();
      else lane.left.clear();
      for (auto * line : {&lane.path, &lane.left, &lane.right}) {
        for (auto & p : *line) {
          p.y += noise;
          p = to_local(p, pose);
        }
      }
      return lane;
    };
    CHECK(!continuity.accept(observe(outside, 1.04, 0.32, true), outside, true, inside, wheelbase));
    CHECK(!continuity.accept(observe(outside, 1.08, 0.0, true), std::nullopt, true, inside, wheelbase));
    CHECK(!continuity.accept(observe(outside, 1.10, 0.0, true), outside, true, inside, wheelbase));
    CHECK(!continuity.accept(observe(outside, 1.12, 0.0, true), outside, true, inside, wheelbase));
    CHECK(continuity.accept(observe(outside, 1.14, 0.0, true), outside, true, inside, wheelbase));

    auto mislabeled = observe(returning, 1.16, 0.0, true);
    std::swap(mislabeled.left, mislabeled.right);
    CHECK(!continuity.accept(mislabeled, returning, false, inside, wheelbase));
    CHECK(!continuity.accept(observe(returning, 1.18, 0.0, true), returning, false, inside, wheelbase));
    CHECK(!continuity.accept(observe(returning, 1.20, 0.0, true), returning, false, inside, wheelbase));
    CHECK(continuity.accept(observe(returning, 1.22, 0.0, true), returning, false, inside, wheelbase));
    CHECK(continuity.accept(observe(returning, 1.24, 0.0, false), returning, false, inside, wheelbase));
  }
}

TEST(rgb_box_maps_through_intrinsics_to_depth) {   // 1.25배 확대 RGB(1280×800) → CAM_A 정렬 깊이(640×400, 확대 없음)
  RgbIntrinsics rgb{700.0, 700.0, 640.0, 400.0};
  DepthImage d;
  d.width = 640; d.height = 400; d.fx = 280.0; d.fy = 280.0; d.cx = 320.0; d.cy = 200.0;
  const auto b = rgb_box_to_depth(cv::Rect2f(640.f, 400.f, 70.f, 35.f), rgb, d);
  CHECK_NEAR(b.x, 320.0, 1e-4);      // 광축은 광축으로
  CHECK_NEAR(b.y, 200.0, 1e-4);
  CHECK_NEAR(b.width, 28.0, 1e-4);   // 70 px × 280/700
  CHECK_NEAR(b.height, 14.0, 1e-4);
}
