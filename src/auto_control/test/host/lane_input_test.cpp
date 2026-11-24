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

TEST(avoidance_recovers_departure_side_first_even_when_model_changes_its_label) {
  constexpr double lane_half = 0.325, vehicle_half = 0.15, wheelbase = 0.33;
  for (const int side : {-1, 1}) {
    LaneContinuity continuity;
    const Pose2 initial{0.0, 0.0, 0.0}, outside{0.65, side * 0.42, side * 0.30};
    const Pose2 returning{0.9, side * 0.10, 0.0};
    auto reference = lane_from_pixels(straight(), 1.0);
    auto accept = [&](LaneObservation & lane, const Pose2 & pose, bool avoiding) {
      return continuity.accept(lane, pose, avoiding, lane_half, vehicle_half, wheelbase);
    };
    auto observed = [&](bool departure_side, const Pose2 & pose, double noise = 0.0) {
      const auto & boundary = (side > 0) == departure_side ? reference.left : reference.right;
      std::vector<Vec2> points;
      for (auto p : boundary) {
        p.y += noise;
        p = to_local(p, pose);
        if (p.x >= 0.0) points.push_back(p);
      }
      return points;
    };
    CHECK(accept(reference, initial, false));
    LaneObservation blind;
    blind.capture_time = 1.02;
    CHECK(!accept(blind, outside, true));

    LaneObservation outside_noise;
    outside_noise.capture_time = 1.04;
    outside_noise.left = observed(true, outside, side * 0.22);
    CHECK(!accept(outside_noise, outside, true));

    LaneObservation wrong_first;
    wrong_first.capture_time = 1.06;
    wrong_first.right = observed(false, returning);
    CHECK(!accept(wrong_first, returning, false));

    for (int frame = 0; frame < 3; ++frame) {
      LaneObservation mislabeled;
      mislabeled.capture_time = 1.08 + frame * 0.02;
      (side > 0 ? mislabeled.right : mislabeled.left) = observed(true, returning);
      (side > 0 ? mislabeled.left : mislabeled.right) = observed(true, returning, side * 0.22);
      CHECK(accept(mislabeled, returning, false) == (frame == 2));
      if (frame == 2) {
        CHECK((side > 0 ? mislabeled.left : mislabeled.right).size() >= 4);
        CHECK((side > 0 ? mislabeled.right : mislabeled.left).empty());
        CHECK_NEAR(to_world(mislabeled.path.back(), returning).y, 0.0, 1e-9);
      }
    }
    auto both = lane_from_pixels(straight(), 1.14);
    for (auto * points : {&both.path, &both.left, &both.right})
      for (auto & p : *points) p = to_local(p, returning);
    CHECK(accept(both, returning, false));
  }
}

TEST(lane_continuity_rejects_narrow_pairs_and_duplicate_boundaries) {
  LaneContinuity continuity;
  const Pose2 pose{};
  auto narrow = lane_from_pixels(straight(), 1.0);
  for (auto & point : narrow.left) point.y = 0.245;
  for (auto & point : narrow.right) point.y = -0.245;
  CHECK(!continuity.accept(narrow, pose, false, 0.325, 0.15, 0.33));
  auto exactly = narrow;
  exactly.capture_time = 1.02;
  for (auto & point : exactly.left) point.y = 0.25;
  for (auto & point : exactly.right) point.y = -0.25;
  CHECK(continuity.accept(exactly, pose, false, 0.325, 0.15, 0.33));

  auto shared = exactly;
  shared.capture_time = 1.04;
  shared.right = shared.left;
  CHECK(!continuity.accept(shared, pose, false, 0.325, 0.15, 0.33));
  auto duplicate = exactly;
  duplicate.capture_time = 1.06;
  for (const auto & point : exactly.left) duplicate.left.push_back({point.x, point.y + 0.10});
  CHECK(!continuity.accept(duplicate, pose, false, 0.325, 0.15, 0.33));
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
