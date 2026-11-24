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

TEST(avoidance_uses_first_return_boundary_without_matching_the_old_geometry) {
  constexpr double lane_half = 0.325, vehicle_half = 0.15, wheelbase = 0.33;
  for (const int side : {-1, 1}) {
    LaneDepartureCheck departure;
    const Pose2 initial{0.0, 0.0, 0.0}, outside{0.65, side * 0.42, side * 0.30};
    const Pose2 returning{3.1, side * 0.10, 0.0};
    auto reference = lane_from_pixels(straight(), 1.0);
    auto accept = [&](LaneObservation & lane, const Pose2 & pose, bool avoiding) {
      return departure.accept(lane, pose, avoiding, lane_half, vehicle_half, wheelbase);
    };
    auto observed = [&](bool departure_side, const Pose2 & pose, double noise = 0.0) {
      const auto & boundary = (side > 0) == departure_side ? reference.left : reference.right;
      std::vector<Vec2> points;
      for (auto p : boundary) {
        if (pose.x > 3.0) p.x += 3.2;
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

    // The true line is 20 cm from the old edge and beyond its observed extent.
    LaneObservation mislabeled;
    mislabeled.capture_time = 1.08;
    (side > 0 ? mislabeled.right : mislabeled.left) = observed(true, returning, side * 0.20);
    auto & extra = side > 0 ? mislabeled.left : mislabeled.right;
    extra = observed(true, returning, side * 0.40);
    const auto opposite = observed(false, returning, side * 0.20);
    extra.insert(extra.end(), opposite.begin(), opposite.end());
    CHECK(accept(mislabeled, returning, false));
    CHECK((side > 0 ? mislabeled.left : mislabeled.right).size() >= 4);
    CHECK((side > 0 ? mislabeled.right : mislabeled.left).empty());
    CHECK_NEAR(to_world(mislabeled.path.back(), returning).y, side * 0.20, 1e-9);

    LaneObservation both;
    both.capture_time = 1.10;
    (side > 0 ? both.left : both.right) = observed(true, returning, side * 0.20);
    (side > 0 ? both.right : both.left) = opposite;
    CHECK(accept(both, returning, false));
    CHECK(!both.left.empty() && !both.right.empty());
  }
}

TEST(single_return_boundary_gets_departure_label_even_across_the_old_center) {
  LaneDepartureCheck departure;
  const Pose2 centered{}, outside{0.6, 0.42, 0.0}, returning{0.9, 0.10, 0.0};
  auto before = lane_from_pixels(straight(), 1.0);
  CHECK(departure.accept(before, centered, false, 0.325, 0.15, 0.33));
  LaneObservation missing;
  missing.capture_time = 1.02;
  CHECK(!departure.accept(missing, outside, true, 0.325, 0.15, 0.33));
  LaneObservation recovered;
  recovered.capture_time = 1.04;
  for (double x = 0.1; x < 1.0; x += 0.05) recovered.right.push_back({x, -0.30});
  CHECK(departure.accept(recovered, returning, false, 0.325, 0.15, 0.33));
  CHECK(recovered.right.empty() && !recovered.left.empty());
  CHECK_NEAR(recovered.path.back().y, -0.625, 1e-9);
}

TEST(lane_departure_check_rejects_narrow_pairs_and_duplicate_boundaries) {
  LaneDepartureCheck departure;
  const Pose2 pose{};
  auto narrow = lane_from_pixels(straight(), 1.0);
  for (auto & point : narrow.left) point.y = 0.245;
  for (auto & point : narrow.right) point.y = -0.245;
  CHECK(!departure.accept(narrow, pose, false, 0.325, 0.15, 0.33));
  auto exactly = narrow;
  exactly.capture_time = 1.02;
  for (auto & point : exactly.left) point.y = 0.25;
  for (auto & point : exactly.right) point.y = -0.25;
  CHECK(departure.accept(exactly, pose, false, 0.325, 0.15, 0.33));

  auto shared = exactly;
  shared.capture_time = 1.04;
  shared.right = shared.left;
  CHECK(!departure.accept(shared, pose, false, 0.325, 0.15, 0.33));
  auto duplicate = exactly;
  duplicate.capture_time = 1.06;
  for (const auto & point : exactly.left) duplicate.left.push_back({point.x, point.y + 0.10});
  CHECK(!departure.accept(duplicate, pose, false, 0.325, 0.15, 0.33));
  auto disjoint = exactly;
  disjoint.capture_time = 1.08;
  for (auto & point : disjoint.right) point.x += 4.0;
  CHECK(!departure.accept(disjoint, pose, false, 0.325, 0.15, 0.33));
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
