#pragma once

// One schema for the web registry and the detector's live RGB/IR settings.
// Arguments: type, parameter key, detector member, label, unit, min, max, RGB default, IR default.
#define BEV_LANE_DETECTION_SETTINGS(X) \
  X(double, left_mask_threshold, left_mask_threshold, "좌측 차선 임계값", "", 0, 1, .80, .80) \
  X(double, right_mask_threshold, right_mask_threshold, "우측 차선 임계값", "", 0, 1, .80, .80) \
  X(double, stop_line_mask_threshold, stop_line_mask_threshold, "정지선 임계값", "", 0, 1, .80, 1.) \
  X(double, overlay_alpha, overlay_alpha, "오버레이 불투명도", "", 0, 1, .75, .75) \
  X(int, stop_line_min_present_pixels, stop_line_min_present_pixels, "정지선 검출 최소 픽셀", "px", 1, 1000, 1, 1) \
  X(bool, side_tracking, side_tracking, "좌우 판정 고정", "", 0, 1, false, false) \
  X(double, side_track_match_gate_m, side_track_match_gate_m, "좌우 추적의 위치 허용 오차", "m", .02, .4, .15, .15) \
  X(double, side_track_max_age_sec, side_track_max_age_sec, "좌우 추적의 이전 관측 유효 시간", "s", .01, .5, .10, .10) \
  X(bool, bridge_centerline, bridge_centerline, "가장자리 보간선을 중앙선 계산에 사용", "", 0, 1, true, true)

#define BEV_LANE_CONNECTION_SETTINGS(X) \
  X(bool, connection_enabled, enabled, "노이즈 제거 및 가장자리 연결", "", 0, 1, true, true) \
  X(int, connection_min_component_area_px, min_component_area_px, "최소 조각 면적", "px²", 1, 150, 30, 8) \
  X(int, connection_skeleton_downsample_factor, skeleton_downsample_factor, "골격 추출 축소 배율", "", 1, 4, 2, 2) \
  X(double, connection_border_endpoint_distance_px, border_endpoint_distance_px, "끝점과 영상 가장자리 최대 거리", "px", 1, 30, 6., 6.) \
  X(double, connection_min_fragment_length_px, min_fragment_length_px, "보간에 필요한 최소 조각 길이", "px", 1, 100, 8., 8.) \
  X(int, connection_max_fragments, max_fragments, "가장자리 후보 조각 수 상한", "", 1, 64, 24, 24) \
  X(double, connection_tangent_window_px, tangent_window_px, "보간 접선 추정 길이", "px", 1, 60, 8., 8.) \
  X(double, connection_max_gap_px, max_gap_px, "가장자리 최대 연결 간격", "px", 1, 300, 90., 80.) \
  X(double, connection_corridor_half_width_px, corridor_half_width_px, "보간 허용 띠 반폭", "px", .5, 30, 4., 4.) \
  X(double, connection_direction_tolerance_deg, direction_tolerance_deg, "보간 방향 허용 오차", "°", 1, 79, 20., 20.) \
  X(double, connection_max_turn_deg, max_turn_deg, "보간 최대 회전각", "°", 1, 180, 180., 180.) \
  X(double, connection_max_curvature_per_px, max_curvature_per_px, "보간 최대 곡률", "1/px", .01, 1, .12, .12) \
  X(double, connection_max_arc_ratio, max_arc_ratio, "보간선 길이/직선거리 제한", "", 1, 4, 1.8, 1.8) \
  X(int, connection_line_width_px, line_width_px, "보간선 두께", "px", 1, 10, 2, 2)

#define BEV_LANE_CENTERLINE_SETTINGS(X) \
  X(bool, centerline_enabled, enabled, "중앙선 계산", "", 0, 1, true, true) \
  X(double, lane_width_m, lane_width_m, "차선 폭", "m", .30, 1.2, .65, .65) \
  X(double, sample_spacing_m, sample_spacing_m, "중앙선 후보 간격", "m", .005, .10, .035, .035) \
  X(double, output_spacing_m, output_spacing_m, "출력 중앙선 점 간격", "m", .005, .10, .02, .02) \
  X(double, clearance_check_spacing_m, clearance_check_spacing_m, "차선 여유 검사 간격", "m", .001, .05, .025, .025) \
  X(double, min_fragment_length_m, min_fragment_length_m, "중앙선에 쓰는 최소 차선 조각 길이", "m", .01, .8, .08, .08) \
  X(double, tangent_window_m, tangent_window_m, "차선 접선 추정 범위", "m", .01, .5, .06, .06) \
  X(double, width_tolerance_m, width_tolerance_m, "차선 폭 허용 오차", "m", .01, .4, .12, .12) \
  X(double, pair_along_tolerance_m, pair_along_tolerance_m, "좌우 차선 짝의 전후 허용 오차", "m", .01, .3, .055, .055) \
  X(double, pair_heading_tolerance_deg, pair_heading_tolerance_deg, "좌우 차선 짝의 방향 허용 오차", "°", 1, 89, 40., 40.) \
  X(double, max_gap_m, max_gap_m, "중앙선 최대 연결 간격", "m", .02, .5, .09, .09) \
  X(double, max_start_distance_m, max_start_distance_m, "차량부터 중앙선 시작점까지 최대 거리", "m", .05, 1.5, .65, .65) \
  X(double, min_clearance_m, min_clearance_m, "차선과 최소 여유", "m", .005, .4, .16, .16) \
  X(double, outside_margin_m, outside_margin_m, "BEV 바깥쪽 중앙선 허용 거리", "m", 0, .3, .12, .12) \
  X(int, max_samples, max_samples, "중앙선 후보 수 상한", "", 32, 5000, 2000, 2000) \
  X(int, centerline_line_width_px, line_width_px, "중앙선 두께", "px", 1, 10, 2, 2)

#define BEV_LANE_CORNER_SETTINGS(X) \
  X(bool, corner_outer_enabled, corner_outer_enabled, "코너 바깥 차선 반영", "", 0, 1, true, true) \
  X(double, corner_outer_weight, corner_outer_weight, "형상 반영 최대 비중", "", 0, 1, .85, 1.) \
  X(double, corner_outer_min_turn_deg, corner_outer_min_turn_deg, "반영 시작 회전각", "°", 1, 35, 8., 8.) \
  X(double, corner_outer_full_turn_deg, corner_outer_full_turn_deg, "최대 반영 회전각", "°", 2, 90, 25., 25.) \
  X(double, corner_outer_window_m, corner_outer_window_m, "바깥 차선 회전각 평가 범위", "m", .15, 2, .60, .60) \
  X(double, corner_outer_tangent_window_m, corner_outer_tangent_window_m, "바깥 차선 접선 추정 범위", "m", .03, .8, .15, .15) \
  X(double, corner_outer_min_length_m, corner_outer_min_length_m, "바깥 차선 최소 관측 길이", "m", .05, 1, .30, .30) \
  X(double, corner_outward_offset_m, corner_outward_offset_m, "코너 바깥쪽 이동량", "m", 0, .25, 0., .08) \
  X(double, corner_entry_distance_m, corner_entry_distance_m, "코너 진입 거리", "m", 0, 5, 2.2, 2.2) \
  X(double, corner_entry_full_offset_m, corner_entry_full_offset_distance_m, "최대 이동 시작 거리", "m", 0, 5, .6, .6)

#define BEV_LANE_SMOOTHING_SETTINGS(X) \
  X(bool, smoothing_enabled, smoothing_enabled, "경로 평활화", "", 0, 1, true, true) \
  X(double, smoothing_strength, smoothing_strength, "평활화 강도", "", 0, 1, .8, .8) \
  X(double, smoothing_sigma_m, smoothing_sigma_m, "국소 평활화 범위", "m", 0, .4, .04, .04) \
  X(double, smoothing_window_m, smoothing_window_m, "경로 곡선 피팅 범위", "m", .05, 2, .65, .65) \
  X(double, smoothing_max_shift_m, smoothing_max_shift_m, "평활화 최대 이동량", "m", .001, .2, .03, .03) \
  X(double, straight_turn_deg, straight_turn_deg, "직선 판정 회전각", "°", 0, 45, 12., 12.) \
  X(double, corner_turn_deg, corner_turn_deg, "코너 판정 회전각", "°", 1, 90, 35., 35.) \
  X(double, turn_window_m, turn_window_m, "코너 판정 거리 범위", "m", .03, 2, .30, .30)

#define BEV_LANE_STOP_SETTINGS(X) \
  X(bool, stop_line_distance_enabled, enabled, "정지선 거리 계산", "", 0, 1, true, false) \
  X(int, stop_line_min_pixels, min_pixels, "거리 피팅에 필요한 최소 정지선 픽셀", "px", 2, 1000, 30, 6) \
  X(int, stop_line_max_fit_samples, max_fit_samples, "직선 피팅 표본 수 상한", "", 2, 2048, 256, 256) \
  X(double, stop_line_support_trim_quantile, support_trim_quantile, "정지선 피팅 양끝 제외 비율", "", 0, .49, .02, .02) \
  X(double, stop_line_support_margin_px, support_margin_px, "정지선 양끝 확장량", "px", 0, 30, 5., 5.) \
  X(double, stop_line_near_edge_quantile, near_edge_quantile, "차량 쪽 정지선 가장자리 분위수", "", 0, .49, .05, .05) \
  X(double, stop_line_min_crossing_alignment, min_crossing_alignment, "경로와 정지선의 최소 교차 정렬", "", .05, 1, .5, .5) \
  X(double, stop_line_fit_distance_tolerance_px, fit_distance_tolerance_px, "직선 피팅 거리 수렴 기준", "px", .001, 1, .01, .01) \
  X(double, stop_line_fit_angle_tolerance_rad, fit_angle_tolerance_rad, "직선 피팅 각도 수렴 기준", "rad", .001, .2, .01, .01)

namespace bev_handoff
{
struct LaneTuning
{
#define BEV_DECLARE_LANE_FIELD(type, key, member, label, unit, lo, hi, rgb, ir) type key{rgb};
  BEV_LANE_DETECTION_SETTINGS(BEV_DECLARE_LANE_FIELD)
  BEV_LANE_CONNECTION_SETTINGS(BEV_DECLARE_LANE_FIELD)
  BEV_LANE_CENTERLINE_SETTINGS(BEV_DECLARE_LANE_FIELD)
  BEV_LANE_CORNER_SETTINGS(BEV_DECLARE_LANE_FIELD)
  BEV_LANE_SMOOTHING_SETTINGS(BEV_DECLARE_LANE_FIELD)
  BEV_LANE_STOP_SETTINGS(BEV_DECLARE_LANE_FIELD)
#undef BEV_DECLARE_LANE_FIELD
};
}  // namespace bev_handoff
