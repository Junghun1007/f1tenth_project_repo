// Copied from src/cone_drive_test/src/core/settings.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/settings.hpp"
#include <cmath>
#include <type_traits>

namespace auto_control::drive {
namespace {

template<class T>
T lane_setting_value(double value) {
  if constexpr (std::is_same_v<T, int>) return static_cast<int>(std::lround(value));
  return static_cast<T>(value);
}

bev_handoff::LaneTuning read_lane_tuning(const ParamRegistry::ValueLookup & get, bool ir) {
  const std::string prefix = ir ? "ir_lane." : "lane.";
  bev_handoff::LaneTuning tuning;
#define READ_SETTING(type, key, member, label, unit, lo, hi, rgb_default, ir_default) \
  tuning.key = lane_setting_value<type>(get(prefix + #key));
  BEV_LANE_DETECTION_SETTINGS(READ_SETTING)
  BEV_LANE_CONNECTION_SETTINGS(READ_SETTING)
  BEV_LANE_CENTERLINE_SETTINGS(READ_SETTING)
  BEV_LANE_CORNER_SETTINGS(READ_SETTING)
  BEV_LANE_SMOOTHING_SETTINGS(READ_SETTING)
  BEV_LANE_STOP_SETTINGS(READ_SETTING)
#undef READ_SETTING
  if (ir) {
    // Preserve the shared threshold written by existing IR tuning files.
    const double common = get(prefix + "mask_threshold");
    if (tuning.left_mask_threshold == -1.0) tuning.left_mask_threshold = common;
    if (tuning.right_mask_threshold == -1.0) tuning.right_mask_threshold = common;
  }
  return tuning;
}

std::string validate_lane_tuning(const ParamRegistry::ValueLookup & get, bool ir) {
  const auto c = read_lane_tuning(get, ir);
  const std::string prefix = ir ? "ir_lane." : "lane.";
  const double padding = get(prefix + "result_padding_px");
  if (padding != std::trunc(padding))
    return prefix + "result_padding_px must be a whole number";
  if (c.left_mask_threshold < 0.0 || c.right_mask_threshold < 0.0)
    return prefix + "left/right_mask_threshold: use -1 (inherit) or 0..1";
  if (c.min_clearance_m + c.corner_outward_offset_m >= c.lane_width_m / 2.0 ||
      c.width_tolerance_m >= c.lane_width_m)
    return prefix + "lane_width_m: too narrow for the clearance, corner offset or width tolerance";
  if (c.corner_entry_full_offset_m > 0.0 &&
      c.corner_entry_full_offset_m >= c.corner_entry_distance_m)
    return prefix + "corner_entry_full_offset_m must be below corner_entry_distance_m, or zero";
  if (c.corner_outer_full_turn_deg <= c.corner_outer_min_turn_deg ||
      c.corner_turn_deg <= c.straight_turn_deg)
    return prefix + "corner turn thresholds must be in ascending order";
  if (c.max_gap_m < 2.0 * c.sample_spacing_m || c.tangent_window_m < c.sample_spacing_m ||
      c.corner_outer_tangent_window_m < 2.0 * c.sample_spacing_m ||
      c.corner_outer_min_length_m < 3.0 * c.sample_spacing_m ||
      c.smoothing_window_m < 5.0 * c.sample_spacing_m)
    return prefix + "sample_spacing_m is too large for one of the path or corner windows";
  if (c.corner_outer_window_m <= c.corner_outer_min_length_m ||
      c.corner_outer_window_m < c.corner_outer_tangent_window_m)
    return prefix + "corner_outer_window_m: too short for the observation or tangent window";
  return {};
}

void register_lane_tuning(ParamRegistry & registry, bool ir) {
  const std::string prefix = ir ? "ir_lane." : "lane.";
  const std::string sensor = ir ? "IR (터널) · " : "RGB · ";
  std::string section = sensor + "검출";
  const auto add = [&](const char * key, const char * label, const char * unit,
                       double low, double high, double value, bool toggle) {
    const std::string name(key);
    if (ir && (name == "left_mask_threshold" || name == "right_mask_threshold")) {
      registry.add({prefix + key, "lane", section, std::string(label) + " (-1 = 공통값)", unit, -1, high, -1, true});
      return;
    }
    const auto group = name == "bridge_centerline" ? sensor + "차선 연결" : section;
    ParamSpec spec{prefix + key, "lane", group, label, unit, low, high, value, true};
    if (toggle) spec.options = {{0, "끔"}, {1, "켬"}};
    registry.add(spec);
  };
  if (ir) registry.add({prefix + "mask_threshold", "lane", section, "좌·우 차선 공통 임계값", "", .05, .95, .8, true});
#define REGISTER_SETTING(type, key, member, label, unit, lo, hi, rgb_default, ir_default) \
  add(#key, label, unit, lo, hi, ir ? ir_default : rgb_default, std::is_same_v<type, bool>);
  BEV_LANE_DETECTION_SETTINGS(REGISTER_SETTING)
  section = sensor + "차선 연결";
  registry.add({prefix + "result_padding_px", "lane", section,
    "영상 바깥쪽 보간 여백 (-1 = 기본값) · YAML 저장 후 재시작", "px", -1, 300, -1, false});
  BEV_LANE_CONNECTION_SETTINGS(REGISTER_SETTING)
  section = sensor + "중앙선";
  BEV_LANE_CENTERLINE_SETTINGS(REGISTER_SETTING)
  section = sensor + "코너 바깥 차선";
  BEV_LANE_CORNER_SETTINGS(REGISTER_SETTING)
  section = sensor + "경로 평활화";
  BEV_LANE_SMOOTHING_SETTINGS(REGISTER_SETTING)
  section = sensor + "정지선 거리";
  BEV_LANE_STOP_SETTINGS(REGISTER_SETTING)
#undef REGISTER_SETTING
  registry.add_validator([ir](const auto & get) { return validate_lane_tuning(get, ir); });
}

}  // namespace

void register_params(ParamRegistry & r) {
  auto o = [&](const char * sec, const char * name, const char * label, const char * unit, double lo, double hi, double v, bool live = true) {
    r.add({name, "obstacle", sec, label, unit, lo, hi, v, live});
  };
  auto d = [&](const char * sec, const char * name, const char * label, const char * unit, double lo, double hi, double v, bool live = true) {
    r.add({name, "driving", sec, label, unit, lo, hi, v, live});
  };
  auto t = [&](const char * sec, const char * name, const char * label, const char * unit, double lo, double hi, double v, bool live = true) {
    r.add({name, "traffic", sec, label, unit, lo, hi, v, live});   // 웹 파라미터 창 [신호등] 탭
  };
  constexpr double max_tcn_seconds = 86400;
  r.add({"tcn.threshold", "tunnel", "RGB TCN", "진입·이탈 판정 임계값", "", 0, 1, 0.5});
  r.add({"tcn.cooldown_seconds", "tunnel", "RGB TCN", "같은 이벤트 재검출 간격", "s", 0, max_tcn_seconds, 1.0});
  r.add({"tcn.warmup_seconds", "tunnel", "RGB TCN", "워밍업 시간", "s", 0, max_tcn_seconds, 2.5});
  r.add({"tcn.max_gap_seconds", "tunnel", "RGB TCN", "프레임 간 최대 간격", "s", 0.000001, max_tcn_seconds, 0.5});
  r.add({"tcn.max_image_age_seconds", "tunnel", "RGB TCN", "RGB 프레임 최대 지연", "s", 0, max_tcn_seconds, 0.25});
  r.add({"tunnel.validation_mode", "tunnel", "IR 입력 검증", "터널 검증 모드", "", 0, 1, 1, true,
    {{1, "true · 독립 IR 테스트 기준"}, {0, "false · 노출 적용 확인"}}});
  o("YOLO 검출", "yolo.con_threshold", "콘 임계값", "", 0, 1, 0.83);
  o("YOLO 검출", "yolo.truck_threshold", "트럭 임계값", "", 0, 1, 0.70);
  o("YOLO 검출", "yolo.sedan_threshold", "세단 임계값", "", 0, 1, 0.70);
  o("YOLO 검출", "yolo.nms_threshold", "NMS IoU", "", 0.05, 0.95, 0.65);
  o("YOLO 검출", "yolo.roi_height_percent", "콘·차량 인정 영역 (화면 아래에서, 신호등은 화면 전체)", "%", 1, 100, 65);
  o("YOLO 검출", "yolo.max_fps", "검출 FPS 상한", "fps", 1, 60, 40);
  // 스테레오 흑백 카메라(CAM_B/C) — 터널 밖: 장애물 깊이용. 노출 0 = 자동노출 (30 FPS라 최대 33 ms)
  o("IR 카메라 · 터널 밖 (장애물 깊이)", "ir_cam.outside_exposure_us", "노출 (0 = 자동노출)", "µs", 0, 33000, 0);
  o("IR 카메라 · 터널 밖 (장애물 깊이)", "ir_cam.outside_iso", "ISO (수동 노출일 때만)", "", 100, 1600, 400);
  o("IR 카메라 · 터널 밖 (장애물 깊이)", "ir_cam.outside_dot", "점 투사기 세기 (깊이 품질)", "", 0, 1, 0.5);
  o("IR 카메라 · 터널 밖 (장애물 깊이)", "ir_cam.outside_flood", "플러드 조명 세기", "", 0, 1, 0);
  o("위치 추정", "object.box_core_fraction", "박스 가운데 비율", "", 0.2, 1, 0.6);
  o("위치 추정", "object.min_points", "최소 점 수", "", 1, 200, 6);
  o("위치 추정", "object.min_point_share", "최소 점 비율", "", 0, 1, 0.10);
  o("위치 추정", "object.window_base_m", "거리 창", "m", 0.01, 0.3, 0.05);
  o("위치 추정", "object.window_per_m", "거리 창 증가", "/m", 0, 0.2, 0.03);
  o("위치 추정", "object.min_height_m", "지면 제거 높이", "m", 0, 0.3, 0.03);
  o("위치 추정", "object.max_height_m", "최대 높이", "m", 0.05, 1.5, 1.0);
  o("위치 추정", "object.cone_size_m", "콘 크기", "m", 0.05, 0.5, 0.13);
  // 정답 배치(콘 4개, 위치 실측) 녹화 4회로 맞춘 검출 위치 보정: 정답과의 오차 RMS 6.5 → 4.5 cm
  o("위치 추정", "object.bearing_offset_deg", "검출 방향 보정 (+ = 왼쪽으로 돌림). 정답 배치 분석값 −2.4°, 지금은 해제(0)", "°", -10, 10, 0);
  o("위치 추정", "object.range_gain", "검출 거리 배율 (카메라→콘 거리 × 이 값)", "", 0.8, 1.2, 0.971);
  o("위치 추정", "object.range_offset_m", "검출 거리 오프셋 (배율 뒤에 더함)", "m", -0.2, 0.2, -0.046);
  o("위치 추정", "object.ground_check_ratio", "지면 거리 검사: 박스 아래 끝으로 구한 거리와 깊이 거리가 이 배율 넘게 다르면 버림 (0=끔)", "", 0, 5, 1.6);
  o("위치 추정", "object.ground_check_min_m", "지면 거리 검사: 두 거리 차이가 이보다 작으면 통과", "m", 0, 1, 0.3);
  o("위치 추정", "object.time_offset_ms", "검출 시각 보정 (차 자세를 찾을 때 이만큼 뒤 시각으로)", "ms", -100, 100, 15);
  o("콘 기억", "memory.gate_radial_max", "거리 방향 게이트", "m", 0.02, 0.5, 0.17);
  o("콘 기억", "memory.gate_lateral", "옆 방향 게이트", "m", 0.01, 0.3, 0.06);
  o("콘 기억", "memory.merge_distance", "병합 거리", "m", 0.01, 0.3, 0.10);
  o("콘 기억", "memory.confirm_hits", "확정 관측 횟수", "", 1, 20, 3);
  o("콘 기억", "memory.miss_delete", "놓침 삭제 횟수", "", 1, 50, 4);
  o("콘 기억", "memory.delete_behind", "뒤 삭제 거리", "m", 0, 1, 0.10);
  o("콘 기억", "memory.ghost_timeout_sec", "유령 기억 정리: 이 시간 동안 다시 안 보이고 차 옆·바로 앞도 아니면 삭제 (0=끔)", "s", 0, 10, 1.5);
  o("콘 기억", "memory.mode", "기억 방식 (0=월드, 1=차선 기준 단기, 2=보이는 콘+화면 밖 고정, 3=기억 없음)", "", 0, 3, 0);
  o("콘 기억", "memory.lane_confirm_hits", "[차선 기준] 사용 전 관측 횟수", "", 1, 10, 4);
  o("콘 기억", "memory.lane_miss_delete", "[차선 기준] 보이는 구간 놓침 삭제", "", 1, 20, 2);
  o("콘 기억", "memory.lane_match_radius", "[차선 기준] 같은 콘 판정 거리", "m", 0.02, 0.4, 0.12);
  o("콘 기억", "memory.lane_visible_s_min", "[차선 기준] 사각지대 경계 (앞차축 앞)", "m", 0.1, 1.5, 0.45);
  // 경로는 ① 차선 안 → ② 차선 안·여유 축소 → ③ 차선 밖 순서로 처음 길이 나오는 단계를 쓴다.
  // 최소 여유(차체 직사각형 ↔ 콘 사각형 빈틈, 이보다 가까우면 불가)는 세 단계 모두 같은 안전 하한 이상으로 둔다:
  // ①만 올리면 ②(예전 0 고정)로 넘어가 오히려 콘에 더 붙는다
  o("회피 계획", "plan.hard_clearance", "① 차선 안 최소 여유 (안전 하한)", "m", 0, 0.2, 0.02);
  o("회피 계획", "plan.soft_clearance", "① 차선 안 권장 여유 (수동 주행 녹화로 맞춤)", "m", 0, 0.3, 0.10);
  o("회피 계획", "plan.relaxed_hard_clearance", "② 차선 안·여유 축소 최소 여유 (차선 밖으로 나가기 전 마지막 단계)", "m", 0, 0.2, 0.005);
  o("회피 계획", "plan.relaxed_soft_clearance", "② 차선 안·여유 축소 권장 여유", "m", 0, 0.3, 0.05);
  o("회피 계획", "plan.free_hard_clearance", "③ 차선 밖 최소 여유", "m", 0, 0.2, 0.02);
  o("회피 계획", "plan.free_soft_clearance", "③ 차선 밖 권장 여유 (수동 주행 녹화로 맞춤)", "m", 0, 0.3, 0.08);
  o("회피 계획", "plan.vehicle_length_m", "계획에 쓰는 차체 길이 (실측, 범퍼 끝~끝)", "m", 0.3, 0.8, 0.50);
  o("회피 계획", "plan.vehicle_width_m", "계획에 쓰는 차체 폭 (실측, 가장 넓은 곳)", "m", 0.15, 0.5, 0.30);
  o("회피 계획", "plan.vehicle_rear_overhang_m", "뒤 차축 → 뒤 범퍼 (실측)", "m", 0, 0.3, 0.12);
  o("회피 계획", "plan.lane_tolerance", "차선 안 허용 이탈 (수동 주행 녹화로 맞춤)", "m", 0, 0.1, 0.07);
  o("회피 계획", "plan.avoid_vehicles", "차량(트럭·세단)도 회피 (0=끔, 1=켬)", "", 0, 1, 1.0);
  o("회피 계획", "plan.activation_margin_m", "회피 계획을 켜는 범위 여유: 차가 지나갈 폭(차체 반폭 + 권장 여유 + 콘 반폭)에 더함", "m", 0, 0.5, 0.05);
  o("회피 계획", "plan.free_max_excess", "차선 밖 회피 최대 이탈", "m", 0, 1, 0.45);
  // 경로 모양 가중치 (예전엔 코드 고정). 수동 주행 녹화 6회를 다시 돌려 계획 경로가 사람 경로와 가장 비슷한 값으로 맞춤 (tools/fit_planner)
  o("회피 계획", "plan.w_obstacle", "콘 가까이 벌점 가중치 (권장 여유 안쪽을 얼마나 피할지)", "", 10, 40000, 400);
  o("회피 계획", "plan.w_offset", "기준 경로(중앙선)에서 벗어남 벌점 가중치", "", 0.1, 500, 20);
  o("회피 계획", "plan.w_slope", "경로 기울기(옆으로 꺾임) 벌점 가중치", "", 0, 50, 4);
  o("회피 계획", "plan.w_curvature", "경로 곡률(급한 꺾임) 벌점 가중치", "", 0, 2, 0.1);
  o("회피 계획", "plan.lane_return_distance", "차선 복귀 거리", "m", 0.1, 3, 1.2);
  o("회피 계획", "plan.rollout_sec", "롤아웃 시간", "s", 0.2, 3, 1.0);
  o("회피 계획", "plan.rollout_retry_margin", "재계획 여유", "m", 0, 0.1, 0.03);
  o("회피 계획", "plan.forward_extension_m", "코너에서 잘린 중앙선을 끝 곡률로(끝에서 0까지 줄이며) 늘리는 최대 길이 (0=끔, 시험용: 녹화 재생에선 막힘이 늘었음)", "m", 0, 1.5, 0.0);
  o("회피 계획", "plan.s_max", "계획 거리 (수동 주행 녹화로 맞춤: 사람은 1.5 m 앞부터 반응)", "m", 1, 3.5, 1.5);
  d("속도", "drive.maximum_speed_mps", "최대 속도", "m/s", 0, 3, 1.3);
  d("속도", "drive.minimum_speed_mps", "최소 속도(코너)", "m/s", 0, 2, 1.1);
  d("속도", "drive.obstacle_speed_mps", "장애물 구간 속도", "m/s", 0, 2, 1.0);
  d("속도", "drive.rollout_fail_speed_mps", "롤아웃 실패 시 속도", "m/s", 0, 2, 0.8);
  d("속도", "drive.maximum_lateral_accel", "최대 횡가속", "m/s²", 0.2, 5, 1.6);
  d("속도", "drive.planning_decel", "계획 감속도", "m/s²", 0.1, 3, 0.5);
  d("속도", "drive.obstacle_decel", "장애물 감속도", "m/s²", 0.05, 3, 0.4);
  d("속도", "drive.obstacle_accel", "장애물 해제 가속도", "m/s²", 0.05, 3, 0.4);
  r.add({"corner_plan.enabled", "driving", "코너 경로 계획 (실험)", "새 코너 경로 계획 (켜면 RGB 차선의 코너 바깥 이동을 끄고 새 계획 경로로 주행)", "", 0, 1, 0, true,
    {{0, "끔 · 기존 코너 오프셋"}, {1, "켬 · 새 계획"}}});
  d("코너 경로 계획 (실험)", "corner_plan.v0_mps", "내륜차가 0이 되는 속도 V₀ (슬립, 실측으로 맞출 것)", "m/s", 0.3, 4, 1.3);
  d("코너 경로 계획 (실험)", "corner_plan.inner_margin_m", "회전 안쪽 차선과 차체 여유", "m", 0, 0.15, 0.03);
  d("코너 경로 계획 (실험)", "corner_plan.outer_margin_m", "회전 바깥 차선과 차체 여유", "m", 0, 0.15, 0.03);
  d("조향 (Stanley)", "drive.stanley_gain", "Stanley gain", "", 0.1, 5, 1.3);
  d("조향 (Stanley)", "drive.stanley_softening", "softening 속도", "m/s", 0, 2, 0.3);
  d("조향 (Stanley)", "drive.heading_lookahead_m", "heading lookahead", "m", 0, 0.5, 0.08);
  d("조향 (Stanley)", "drive.steering_current_weight", "조향 필터 현재 비중", "", 0.05, 1, 0.5);
  d("조향 (Stanley)", "drive.steering_rate_deg_s", "조향 변화율 한계", "°/s", 10, 1000, 260);
  r.add({"lane.check_mode", "lane", "공통", "차선 검사 모드", "", 0, 2, 1, true,
    {{0, "0 · 추가 검사 끔"}, {1, "1 · 기존 검사"}, {2, "2 · 회피 방향별 차선 복귀"}}});
  // RGB·IR 검출기 모델을 한 번에 고른다. 실행 중 교체: 각 검출기가 뒤에서 새 엔진을 만든 뒤 바꾼다 (재시작 없음).
  // 파일·정밀도 표는 auto_control_tunnel/launch_support.py LANE_MODELS (같은 번호)
  r.add({"lane.model", "lane", "차선 모델 (RGB·IR 공통)", "모델 · 바로 교체 (처음 고르면 엔진 빌드에 수 분)", "",
    0, 3, 2, true, {{0, "0 · 기존 (RGB stop_0910 INT8 + IR 기존 FP32)"}, {1, "1 · v5 (RGB v5 INT8 + IR 기존 FP32)"},
                    {2, "2 · 통합 C3 INT8 (RGB·IR)"}, {3, "3 · 통합 C3 FP32 (RGB·IR)"}}});
  register_lane_tuning(r, false);
  register_lane_tuning(r, true);
  const bev_handoff::RgbCameraSettings rgb_defaults;
  r.add({"rgb_cam.auto_exposure", "lane", "RGB · 카메라", "노출·ISO 모드", "", 0, 2,
    static_cast<double>(rgb_defaults.mode), true, {{1, "노출·ISO 자동"}, {2, "노출 자동 · ISO 고정"}, {0, "노출·ISO 수동"}}});
  r.add({"rgb_cam.exposure_us", "lane", "RGB · 카메라", "수동 노출 시간", "µs",
    rgb_defaults.min_exposure_us, rgb_defaults.max_exposure_us, static_cast<double>(rgb_defaults.exposure_us)});
  r.add({"rgb_cam.iso", "lane", "RGB · 카메라", "수동 ISO", "",
    rgb_defaults.min_iso, rgb_defaults.max_iso, static_cast<double>(rgb_defaults.iso)});
  // 스테레오 흑백 카메라 — 터널 안: IR 차선용 (tunnel_enabled:=true일 때만). 수동 노출 고정
  const auto ir_defaults = Settings::IrCamera{}.inside;
  r.add({"ir_cam.inside_resolution", "lane", "IR (터널) · 카메라",
    "IR 영상 해상도", "", 0, 400, 0, true, {{0, "시작값 (기본 800p)"}, {400, "400p (640×400)"}}});
  r.add({"ir_cam.inside_exposure_us", "lane", "IR (터널) · 카메라", "노출 시간", "µs", 1000, 33000, double(ir_defaults.exposure_us)});
  r.add({"ir_cam.inside_iso", "lane", "IR (터널) · 카메라", "ISO", "", 100, 1600, double(ir_defaults.iso)});
  r.add({"ir_cam.inside_dot", "lane", "IR (터널) · 카메라", "점 투사기 세기", "", 0, 1, ir_defaults.dot});
  r.add({"ir_cam.inside_flood", "lane", "IR (터널) · 카메라", "IR 플러드 조명 세기", "", 0, 1, ir_defaults.flood});
  d("출력", "drive.maximum_duty", "최대 duty", "", 0, 0.5, 0.12);
  d("출력", "drive.start_duty", "정지 출발 duty", "", 0, 0.3, 0.05);
  d("종방향 모드", "drive.longitudinal_mode", "종방향 모드 (1=staged 전류 제동, 2=duty-only 회생 감속) · 정지했을 때 적용", "", 1, 2, 2.0);
  d("duty-only (모드 2 전용)", "duty_only.pid_kp", "PID P", "duty/(m/s)", 0, 1, 0.06);
  d("duty-only (모드 2 전용)", "duty_only.pid_ki", "PID I", "duty/(m/s·s)", 0, 1, 0.03);
  d("duty-only (모드 2 전용)", "duty_only.ff_offset_duty", "기본 duty (굴러가는 데 필요한 값)", "", 0, 0.1, 0.05);
  d("duty-only (모드 2 전용)", "duty_only.ff_duty_per_mps", "속도 1 m/s당 duty (정속 주행 duty로 맞출 것)", "/(m/s)", 0, 0.3, 0.035);
  d("duty-only (모드 2 전용)", "duty_only.maximum_duty", "최대 duty", "", 0.01, 0.5, 0.12);
  d("duty-only (모드 2 전용)", "duty_only.start_duty", "정지 출발 duty", "", 0, 0.3, 0.05);
  d("duty-only (모드 2 전용)", "duty_only.duty_rise_rate", "duty 상승률", "/s", 0.005, 1, 0.04);
  d("duty-only (모드 2 전용)", "duty_only.duty_fall_rate", "duty 하강률 (회생 감속 세기)", "/s", 0.005, 1, 0.06);
  d("duty-only (모드 2 전용)", "duty_only.planning_decel", "계획 감속도 (코너·앞차)", "m/s²", 0.05, 3, 0.40);
  d("duty-only (모드 2 전용)", "duty_only.obstacle_decel", "장애물 감속도", "m/s²", 0.05, 3, 0.35);
  d("duty-only (모드 2 전용)", "duty_only.response_sec", "duty 명령→감속 지연", "s", 0, 0.5, 0.10);
  d("duty-only (모드 2 전용)", "duty_only.coast_decel", "모터 풀었을 때 감속도 (실측으로 맞출 것)", "m/s²", 0.05, 3, 0.50);
  d("duty-only (모드 2 전용)", "duty_only.coast_release_speed", "이 속도까지 줄인 뒤 모터를 풀고 굴러서 정지", "m/s", 0, 1.5, 0.35);
  d("duty-only (모드 2 전용)", "duty_only.hold_current", "정지 유지 전류 (0=안 씀)", "A", 0, 10, 0.0);
  d("duty-only (모드 2 전용)", "duty_only.emergency_decel", "비상 제동: 필요 감속도 이상이면", "m/s²", 0, 5, 1.2);
  d("duty-only (모드 2 전용)", "duty_only.emergency_overspeed", "비상 제동: 목표보다 이만큼 빠르면", "m/s", 0, 3, 0.5);
  d("duty-only (모드 2 전용)", "duty_only.emergency_current", "비상 제동 전류", "A", 0, 10, 2.0);
  d("제동", "drive.brake_maximum_current", "최대 제동 전류", "A", 0, 10, 2.0);
  d("제동", "drive.brake_speed_gain", "속도 초과 → 제동 비율 (전류 = 최대 × 이 값 × 초과 m/s)", "/(m/s)", 0, 5, 0.8);
  d("제동", "drive.brake_deceleration_gain", "장애물 감속 부족 → 제동 비율", "/(m/s²)", 0, 5, 0.5);
  d("제동", "drive.brake_current_rise_aps", "제동 전류 상승 속도", "A/s", 0.1, 30, 1.5);
  d("제동", "drive.brake_entry_speed_error_mps", "제동 시작 속도 초과 (일반 주행)", "m/s", 0.02, 1.5, 0.3);
  d("제동", "drive.brake_exit_speed_error_mps", "제동 해제 속도 초과 (일반 주행)", "m/s", 0, 1.0, 0.1);
  d("차량 추종 (ACC)", "acc.enabled", "차량 추종 (0=끔, 1=켬: 경로에 걸친 차량은 회피 대신 뒤따름)", "", 0, 1, 1);
  d("차량 추종 (ACC)", "acc.stop_gap_m", "정지 거리 (앞차가 서 있을 때)", "m", 0.05, 1.5, 0.25);
  d("차량 추종 (ACC)", "acc.resume_gap_m", "재출발·최소 유지 거리", "m", 0.1, 2.0, 0.40);
  d("차량 추종 (ACC)", "acc.time_gap_s", "시간 간격 (유지 거리 += 이 값 × 내 속도)", "s", 0, 2, 0.3);
  d("차량 추종 (ACC)", "acc.decel_mps2", "거리 기반 감속도", "m/s²", 0.2, 3, 0.8);
  d("차량 추종 (ACC)", "acc.lateral_margin_m", "경로 막힘 판정 옆 여유", "m", 0, 0.3, 0.05);
  d("차량 추종 (ACC)", "acc.min_follow_speed_mps", "이보다 느린 앞차는 정지로 봄", "m/s", 0, 1.5, 0.3);
  d("차량 추종 (ACC)", "acc.hold_near_m", "이 거리 안에서 놓치면 (놓친 동안) 서 있다고 가정", "m", 0, 1.5, 0.6);
  d("차량 추종 (ACC)", "acc.lost_timeout_sec", "앞차를 놓친 뒤 크루즈 유지 시간 (넘으면 정상 주행)", "s", 0.05, 2, 0.3);
  t("신호등 검출", "yolo.traffic_light_threshold", "신호등 임계값", "", 0, 1, 0.70);
  t("신호등·정지선", "traffic.enabled", "신호등 정지 (0=끔, 1=켬: 빨간불 + 정지선이면 정지선 앞에 정지, 초록불에 출발)", "", 0, 1, 1);
  t("신호등·정지선", "traffic.signal_confirm_frames", "신호 색 인정: 같은 색 연속 관측 횟수", "회", 1, 10, 3);
  t("신호등·정지선", "traffic.red_memory_m", "빨간불을 본 뒤 정지선을 찾으며 달리는 거리", "m", 0.5, 20, 4.0);
  t("신호등·정지선", "traffic.max_decel_mps2", "통과 판정용 최대 감속도 (비상 제동 포함 실제로 낼 수 있는 값)", "m/s²", 0.2, 5, 1.0);
  t("신호등·정지선", "traffic.pass_tolerance_m", "못 설 거리 판정 여유 (예상 정지 거리 > 정지선 + 이 값이면 통과)", "m", 0, 0.5, 0.05);
  t("신호등·정지선", "traffic.stop_range_max_m", "허용 정차 범위 (앞범퍼가 정지선 앞 0~이 거리)", "m", 0.05, 1.0, 0.25);
  t("신호등·정지선", "traffic.stop_margin_m", "목표 정차점 (정지선 앞 이 거리)", "m", 0, 0.5, 0.10);
  t("신호등·정지선", "traffic.decel_mps2", "감속 계획 (staged. duty-only는 duty_only 계획 감속도와 작은 쪽)", "m/s²", 0.1, 3, 0.8);
  t("신호등·정지선", "traffic.reapproach_speed_mps", "정차 범위보다 짧게 섰을 때 다시 다가가는 속도", "m/s", 0.05, 1.0, 0.20);
  t("신호등·정지선", "traffic.state_timeout_sec", "신호 관측 유효 시간", "s", 0.05, 3, 0.30);
  t("신호등·정지선", "traffic.line_timeout_sec", "정지선을 못 봐도 추적 유지", "s", 0.05, 3, 0.50);
  t("신호등·정지선", "traffic.line_confirm_frames", "정지선 확인 횟수", "", 1, 10, 3);
  t("신호등·정지선", "traffic.color_min_saturation", "신호 색 판별 최소 채도 (HSV S)", "", 0, 255, 80);
  t("신호등·정지선", "traffic.color_min_value", "신호 색 판별 최소 밝기 (HSV V)", "", 0, 255, 60);
  d("안전", "web.link_loss_stop_sec", "연결 끊김 정지 시간", "s", 0.2, 5, 1.0);
  d("안전", "drive.lane_timeout_sec", "차선 끊김 허용 시간 (0=제한 없음, 그동안 직전 경로로 주행)", "s", 0, 60, 20.0);
  r.add_order_constraint("drive.minimum_speed_mps", "drive.maximum_speed_mps");
  r.add_order_constraint("acc.stop_gap_m", "acc.resume_gap_m");
  r.add_order_constraint("traffic.stop_margin_m", "traffic.stop_range_max_m");
  r.add_order_constraint("duty_only.start_duty", "duty_only.maximum_duty");
  r.add_order_constraint("duty_only.ff_offset_duty", "duty_only.maximum_duty");
  r.add_order_constraint("drive.brake_exit_speed_error_mps", "drive.brake_entry_speed_error_mps");
  r.add_order_constraint("plan.hard_clearance", "plan.soft_clearance");
  r.add_order_constraint("plan.free_hard_clearance", "plan.free_soft_clearance");
  r.add_order_constraint("plan.relaxed_hard_clearance", "plan.relaxed_soft_clearance");
  r.add_order_constraint("object.min_height_m", "object.max_height_m");
}

Settings settings_from(const ParamRegistry & r) {
  Settings s;
  std::map<std::string, double> values;
  for (const auto & spec : r.list()) values.emplace(spec.name, spec.value);
  const ParamRegistry::ValueLookup g = [&](const std::string & name) { return values.at(name); };
  s.rgb_lane = read_lane_tuning(g, false);
  s.ir_lane = read_lane_tuning(g, true);
  s.yolo_thresholds = {float(g("yolo.con_threshold")), float(g("yolo.truck_threshold")), float(g("yolo.sedan_threshold")),
                       float(g("yolo.traffic_light_threshold"))};
  s.nms_threshold = g("yolo.nms_threshold");
  s.roi_height_percent = g("yolo.roi_height_percent");
  s.yolo_max_fps = g("yolo.max_fps");
  s.cone_size_m = g("object.cone_size_m");
  s.box.box_core_fraction = g("object.box_core_fraction");
  s.box.bearing_offset_rad = g("object.bearing_offset_deg") * 3.14159265358979323846 / 180.0;
  s.box.range_gain = g("object.range_gain");
  s.box.range_offset_m = g("object.range_offset_m");
  s.box.ground_check_ratio = g("object.ground_check_ratio");
  s.box.ground_check_min_m = g("object.ground_check_min_m");
  s.obs_time_offset_s = g("object.time_offset_ms") / 1000.0;
  s.box.loc.min_points = std::size_t(std::lround(g("object.min_points")));
  s.box.loc.min_fraction = g("object.min_point_share");
  s.box.loc.window_base_m = g("object.window_base_m");
  s.box.loc.window_per_m = g("object.window_per_m");
  s.box.min_height_m = g("object.min_height_m");
  s.box.max_height_m = g("object.max_height_m");
  s.memory.gate_radial_max = g("memory.gate_radial_max");
  s.memory.gate_lateral = g("memory.gate_lateral");
  s.memory.merge_distance = g("memory.merge_distance");
  s.memory.confirm_hits = int(std::lround(g("memory.confirm_hits")));
  s.memory.miss_delete = int(std::lround(g("memory.miss_delete")));
  s.memory.delete_behind = g("memory.delete_behind");
  s.memory.ghost_timeout = g("memory.ghost_timeout_sec");
  s.memory_mode = int(std::lround(g("memory.mode")));
  s.lane_memory.confirm_hits = int(std::lround(g("memory.lane_confirm_hits")));
  s.lane_memory.miss_delete = int(std::lround(g("memory.lane_miss_delete")));
  s.lane_memory.match_radius = g("memory.lane_match_radius");
  s.lane_memory.visible_s_min = g("memory.lane_visible_s_min");
  s.lane_memory.delete_behind = s.memory.delete_behind;
  auto & p = s.planner;
  p.hard_clearance = g("plan.hard_clearance");
  p.soft_clearance = g("plan.soft_clearance");
  p.relaxed_hard_clearance = g("plan.relaxed_hard_clearance");
  p.relaxed_soft_clearance = g("plan.relaxed_soft_clearance");
  p.length = g("plan.vehicle_length_m");
  p.width = g("plan.vehicle_width_m");
  p.rear_overhang = g("plan.vehicle_rear_overhang_m");
  p.free_hard_clearance = g("plan.free_hard_clearance");
  p.free_soft_clearance = g("plan.free_soft_clearance");
  p.lane_tolerance = g("plan.lane_tolerance");
  p.activation_margin = g("plan.activation_margin_m");
  p.w_obstacle = g("plan.w_obstacle");
  p.w_offset = g("plan.w_offset");
  p.w_slope = g("plan.w_slope");
  p.w_curvature = g("plan.w_curvature");
  p.free_max_excess = g("plan.free_max_excess");
  p.lane_return_distance = g("plan.lane_return_distance");
  p.rollout_sec = g("plan.rollout_sec");
  p.rollout_retry_margin = g("plan.rollout_retry_margin");
  p.s_max = g("plan.s_max");
  p.forward_extension = g("plan.forward_extension_m");
  p.cone_half = s.cone_size_m / 2;
  s.corner_plan.enabled = g("corner_plan.enabled") > 0.5;
  s.corner_plan.v0 = g("corner_plan.v0_mps");
  s.corner_plan.inner_margin = g("corner_plan.inner_margin_m");
  s.corner_plan.outer_margin = g("corner_plan.outer_margin_m");
  s.maximum_lateral_accel = g("drive.maximum_lateral_accel");
  s.planning_decel = g("drive.planning_decel");
  s.stanley_gain = g("drive.stanley_gain");
  s.stanley_softening = g("drive.stanley_softening");
  s.heading_lookahead_m = g("drive.heading_lookahead_m");
  s.steering_current_weight = g("drive.steering_current_weight");
  s.steering_rate_rad_s = g("drive.steering_rate_deg_s") * 3.14159265358979323846 / 180.0;
  auto & c = s.control;
  c.stanley_gain = s.stanley_gain;
  c.softening_speed = s.stanley_softening;
  c.heading_lookahead = s.heading_lookahead_m;
  c.steering_current_weight = s.steering_current_weight;
  c.max_speed = g("drive.maximum_speed_mps");
  c.obstacle_speed = g("drive.obstacle_speed_mps");
  c.rollout_fail_speed = g("drive.rollout_fail_speed_mps");
  auto & l = s.lon;
  l.maximum_speed_mps = g("drive.maximum_speed_mps");
  l.minimum_speed_mps = g("drive.minimum_speed_mps");
  l.obstacle_speed_mps = g("drive.obstacle_speed_mps");
  l.obstacle_planning_deceleration_mps2 = g("drive.obstacle_decel");
  l.obstacle_planning_acceleration_mps2 = g("drive.obstacle_accel");
  l.maximum_duty = g("drive.maximum_duty");
  l.longitudinal_start_duty = std::min(g("drive.start_duty"), l.maximum_duty);
  l.brake_maximum_current_amps = g("drive.brake_maximum_current");
  l.mode = int(std::lround(g("drive.longitudinal_mode")));
  {
    auto & d = l.duty_only;
    d.kp = g("duty_only.pid_kp");
    d.ki = g("duty_only.pid_ki");
    d.ff_offset_duty = g("duty_only.ff_offset_duty");
    d.ff_duty_per_mps = g("duty_only.ff_duty_per_mps");
    d.maximum_duty = g("duty_only.maximum_duty");
    d.start_duty = g("duty_only.start_duty");
    d.duty_rise_rate_per_sec = g("duty_only.duty_rise_rate");
    d.duty_fall_rate_per_sec = g("duty_only.duty_fall_rate");
    d.planning_deceleration_mps2 = g("duty_only.planning_decel");
    d.obstacle_planning_deceleration_mps2 = g("duty_only.obstacle_decel");
    d.response_sec = g("duty_only.response_sec");
    d.coast_deceleration_mps2 = g("duty_only.coast_decel");
    d.coast_release_speed_mps = g("duty_only.coast_release_speed");
    d.hold_current_amps = g("duty_only.hold_current");
    d.emergency_deceleration_mps2 = g("duty_only.emergency_decel");
    d.emergency_overspeed_mps = g("duty_only.emergency_overspeed");
    d.emergency_brake_current_amps = g("duty_only.emergency_current");
  }
  l.brake_speed_gain = g("drive.brake_speed_gain");
  l.brake_deceleration_gain = g("drive.brake_deceleration_gain");
  l.brake_current_rise_amps_per_sec = g("drive.brake_current_rise_aps");
  l.brake_entry_speed_error_mps = g("drive.brake_entry_speed_error_mps");
  l.brake_exit_speed_error_mps = g("drive.brake_exit_speed_error_mps");
  s.link_loss_stop_sec = g("web.link_loss_stop_sec");
  s.lane_timeout_sec = g("drive.lane_timeout_sec");
  s.lane_check_mode = static_cast<int>(g("lane.check_mode"));
  s.rgb_camera = {static_cast<bev_handoff::RgbExposureMode>(int(g("rgb_cam.auto_exposure"))), int(std::lround(g("rgb_cam.exposure_us"))),
                  int(std::lround(g("rgb_cam.iso")))};
  s.ir_camera.inside = {int(std::lround(g("ir_cam.inside_exposure_us"))), int(std::lround(g("ir_cam.inside_iso"))),
                        g("ir_cam.inside_dot"), g("ir_cam.inside_flood")};
  s.ir_camera.outside = {int(std::lround(g("ir_cam.outside_exposure_us"))), int(std::lround(g("ir_cam.outside_iso"))),
                         g("ir_cam.outside_dot"), g("ir_cam.outside_flood")};
  s.ir_camera.use_400p_ir_output = g("ir_cam.inside_resolution") == 400;
  s.avoid_vehicles = g("plan.avoid_vehicles") >= 0.5;
  s.traffic.enabled = g("traffic.enabled") >= 0.5;
  s.traffic.stop_range_max = g("traffic.stop_range_max_m");
  s.traffic.stop_margin = g("traffic.stop_margin_m");
  s.traffic.decel = g("traffic.decel_mps2");
  s.traffic.reapproach_speed = g("traffic.reapproach_speed_mps");
  s.traffic.state_timeout = g("traffic.state_timeout_sec");
  s.traffic.line_timeout = g("traffic.line_timeout_sec");
  s.traffic.line_confirm = int(std::lround(g("traffic.line_confirm_frames")));
  s.traffic.signal_confirm = int(std::lround(g("traffic.signal_confirm_frames")));
  s.traffic.red_memory_m = g("traffic.red_memory_m");
  s.traffic.pass_tolerance = g("traffic.pass_tolerance_m");
  s.traffic.max_decel = g("traffic.max_decel_mps2");
  s.traffic_min_saturation = int(std::lround(g("traffic.color_min_saturation")));
  s.traffic_min_value = int(std::lround(g("traffic.color_min_value")));
  s.lead.enabled = g("acc.enabled") >= 0.5;
  s.lead.stop_gap = g("acc.stop_gap_m");
  s.lead.resume_gap = g("acc.resume_gap_m");
  s.lead.time_gap = g("acc.time_gap_s");
  s.lead.decel = g("acc.decel_mps2");
  s.lead.lateral_margin = g("acc.lateral_margin_m");
  s.lead.min_follow_speed = g("acc.min_follow_speed_mps");
  s.lead.hold_near = g("acc.hold_near_m");
  s.lead.lost_timeout = g("acc.lost_timeout_sec");
  return s;
}

}  // namespace auto_control::drive
