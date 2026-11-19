#include "semi_slam/core/lane_post.hpp"
#include <algorithm>

namespace semi_slam {

LanePostConfig::LanePostConfig() {
  connection.skeleton_downsample_factor = 2;
  auto & c = centerline;
  c.lane_width_m = 0.65; c.bev_width_m = 1.2; c.bev_height_m = 3.0;
  c.sample_spacing_m = 0.035; c.output_spacing_m = 0.02; c.clearance_check_spacing_m = 0.025;
  c.min_fragment_length_m = 0.08; c.tangent_window_m = 0.06; c.width_tolerance_m = 0.12;
  c.pair_along_tolerance_m = 0.055; c.pair_heading_tolerance_deg = 40.0; c.max_gap_m = 0.09;
  c.max_start_distance_m = 0.65; c.min_clearance_m = 0.16; c.outside_margin_m = 0.12;
  c.max_samples = 2000; c.line_width_px = 2;
  c.corner_outer_enabled = true; c.corner_outer_weight = 1.0; c.corner_outward_offset_m = 0.08;
  c.corner_entry_distance_m = 2.2; c.corner_entry_full_offset_distance_m = 0.6;
  c.corner_outer_window_m = 0.60; c.corner_outer_tangent_window_m = 0.15; c.corner_outer_min_length_m = 0.30;
  c.corner_outer_min_turn_deg = 8.0; c.corner_outer_full_turn_deg = 25.0;
  c.smoothing_enabled = true; c.smoothing_sigma_m = 0.04; c.smoothing_window_m = 0.65;
  c.smoothing_max_shift_m = 0.03; c.smoothing_strength = 0.8;
  c.straight_turn_deg = 12.0; c.corner_turn_deg = 35.0; c.turn_window_m = 0.30;
}

LanePostprocessor::LanePostprocessor(const LanePostConfig & c) { set_config(c); }

void LanePostprocessor::set_config(const LanePostConfig & c) {
  validate_lane_connection(c.connection);
  validate_centerline(c.centerline);
  c_ = c;
  reference_ = c.centerline;
  reference_.corner_outer_enabled = false;
  LaneSideTrackerConfig tc;
  tc.meter_per_pixel = c.meter_per_pixel;
  tc.lane_width_m = c.centerline.lane_width_m;
  tc.width_tolerance_m = c.centerline.width_tolerance_m;
  tracker_ = LaneSideTracker(tc);
}

Vec2 LanePostprocessor::pixel_to_vehicle(const cv::Point2f & p) const {
  return {c_.x_max_m - (p.y + 0.5) * c_.meter_per_pixel,
          c_.y_max_m - (p.x - c_.connection.padding_px + 0.5) * c_.meter_per_pixel};
}

LaneObservation LanePostprocessor::process(const cv::Mat & labels, double t, bool want_reference, const cv::Mat & stop_mask) const {
  LaneObservation out;
  out.capture_time = t;
  auto conn = connect_lane_fragments(labels, c_.connection, false, nullptr, false);
  if (c_.side_tracking) {   // 모델 라벨 대신 직전 프레임·차선 폭 기준으로 좌우 재판정
    const auto fix = tracker_.apply(conn.observed_paths, labels.rows, t);
    out.side_relabeled = fix.relabeled;
    out.side_dropped = fix.dropped;
  }
  const auto center = generate_centerline(conn.labels, conn.observed_paths, labels.cols, c_.connection.padding_px,
                                          c_.centerline, false, nullptr);
  for (const auto & p : center.points) out.path.push_back(pixel_to_vehicle(p));
  if (!stop_mask.empty() && center.points.size() >= 2)   // line_detactor와 같은 계산: 정지선 근변과 중앙선 교차점까지 거리
    out.stop_line_m = estimate_stop_line_distance_m(stop_mask, center.points, c_.connection.padding_px,
                                                    c_.centerline.bev_width_m, c_.centerline.bev_height_m, c_.stop_line);
  if (want_reference && !out.path.empty()) {
    const auto ref = generate_centerline(conn.labels, conn.observed_paths, labels.cols, c_.connection.padding_px,
                                         reference_, false, nullptr);
    for (const auto & p : ref.points) out.center.push_back(pixel_to_vehicle(p));
  }
  for (int side = 0; side < 2; ++side) {
    auto & dst = side == 0 ? out.left : out.right;
    for (const auto & frag : conn.observed_paths[side]) for (const auto & p : frag) dst.push_back(pixel_to_vehicle(p));
    std::sort(dst.begin(), dst.end(), [](const Vec2 & a, const Vec2 & b) { return a.x < b.x; });
  }
  out.valid = out.path.size() >= 4;
  return out;
}

}  // namespace semi_slam
