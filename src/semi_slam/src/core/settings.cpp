#include "semi_slam/core/settings.hpp"
#include <algorithm>
#include <cmath>

namespace semi_slam {

void register_params(ParamRegistry & r) {
  auto m = [&](const char * sec, const char * name, const char * label, const char * unit, double lo, double hi, double v) {
    r.add({name, "slam", sec, label, unit, lo, hi, v, true});
  };
  auto d = [&](const char * sec, const char * name, const char * label, const char * unit, double lo, double hi, double v) {
    r.add({name, "driving", sec, label, unit, lo, hi, v, true});
  };
  m("주행 계획", "run.speed_mps", "일정 속도 (코너 감속 없음)", "m/s", 0.2, 2.5, 1.0);
  m("주행 계획", "run.laps", "목표 바퀴 수 (정지선 S 통과로 셈)", "", 1, 30, 10);
  m("주행 계획", "slam.direction", "주행 방향 (0=반시계: 왼쪽 아래에서 출발, 1=시계: 오른쪽 아래에서 출발)", "", 0, 1, 0);
  m("주행 계획", "slam.start_shift_m", "출발 위치: 앞범퍼가 아래 직선 시작점에서 진행 방향 앞(+)/뒤(−)로", "m", -0.5, 1.0, 0.0);
  m("주행 계획", "slam.start_lateral_m", "출발 위치: 도로 가운데에서 왼쪽(+)으로", "m", -0.3, 0.3, 0.0);
  m("카메라", "camera.yaw_deg", "카메라 yaw (+ = 카메라가 왼쪽을 봄). BEV를 이만큼 돌려 만든다 · 바로 반영", "°", -20, 20, 0);
  m("매핑·보정", "camera.stabilization", "IMU 영상 안정화 (0=끄기, 1=전체)", "", 0, 1, 1);
  m("매핑·보정", "slam.stopline_correction", "정지선 통과 시 위치 보정 (0=끔, 1=켬)", "", 0, 1, 1);
  m("매핑·보정", "slam.corner_anchor", "코너마다 위치 다시 잡기 (자이로 45°·135°·225°·315° = 코너 한가운데) (0=끔, 1=켬)", "", 0, 1, 1);
  m("매핑·보정", "slam.auto_calibrate", "목표 바퀴 완료 시 보정 자동 적용 (0=끔, 1=켬)", "", 0, 1, 1);
  m("매핑·보정", "slam.max_range_m", "매핑·보정에 쓰는 차선 점 최대 거리", "m", 0.5, 3.0, 1.5);
  m("매핑·보정", "slam.stop_confirm_frames", "정지선 확인 횟수", "", 1, 10, 3);
  m("매핑·보정", "slam.stop_gate_m", "정지선 인정 범위: 지도상 정지선 S에서 이 거리 안", "m", 0.2, 3.0, 1.2);
  m("매핑·보정", "slam.lap_window", "바퀴 인정 범위: 한 바퀴 길이의 (1 ± 이 비율)", "", 0.05, 0.6, 0.25);
  d("조향 (Stanley)", "drive.stanley_gain", "Stanley gain", "", 0.1, 5, 1.2);
  d("조향 (Stanley)", "drive.stanley_softening", "softening 속도", "m/s", 0, 2, 0.3);
  d("조향 (Stanley)", "drive.heading_lookahead_m", "heading lookahead", "m", 0, 0.5, 0.12);
  d("조향 (Stanley)", "drive.steering_current_weight", "조향 필터 현재 비중", "", 0.05, 1, 0.4);
  d("조향 (Stanley)", "drive.steering_rate_deg_s", "조향 변화율 한계", "°/s", 10, 1000, 240);
  d("차선", "lane.side_tracking", "좌우 판정 고정 (0=끔, 1=켬)", "", 0, 1, 1);
  d("차선", "lane.corner_outward_offset_m", "코너 바깥 이동 (주행 경로만, 매핑과 무관)", "m", 0, 0.15, 0.08);
  d("차선", "lane.corner_entry_distance_m", "코너 진입 거리", "m", 0, 4, 2.2);
  d("차선", "lane.corner_entry_full_offset_m", "최대 이동 시작 거리", "m", 0, 4, 0.6);
  d("종방향 모드", "drive.longitudinal_mode", "종방향 모드 (1=staged 전류 제동, 2=duty-only 회생 감속) · 정지했을 때 적용", "", 1, 2, 2);
  d("duty-only (모드 2 전용)", "duty_only.pid_kp", "PID P", "duty/(m/s)", 0, 1, 0.06);
  d("duty-only (모드 2 전용)", "duty_only.pid_ki", "PID I", "duty/(m/s·s)", 0, 1, 0.03);
  d("duty-only (모드 2 전용)", "duty_only.ff_offset_duty", "기본 duty (굴러가는 데 필요한 값)", "", 0, 0.1, 0.05);
  d("duty-only (모드 2 전용)", "duty_only.ff_duty_per_mps", "속도 1 m/s당 duty", "/(m/s)", 0, 0.3, 0.035);
  d("duty-only (모드 2 전용)", "duty_only.maximum_duty", "최대 duty", "", 0.01, 0.5, 0.12);
  d("duty-only (모드 2 전용)", "duty_only.start_duty", "정지 출발 duty", "", 0, 0.3, 0.05);
  d("duty-only (모드 2 전용)", "duty_only.duty_rise_rate", "duty 상승률", "/s", 0.005, 1, 0.04);
  d("duty-only (모드 2 전용)", "duty_only.duty_fall_rate", "duty 하강률 (회생 감속 세기)", "/s", 0.005, 1, 0.06);
  d("duty-only (모드 2 전용)", "duty_only.emergency_overspeed", "비상 제동: 목표보다 이만큼 빠르면", "m/s", 0, 3, 0.5);
  d("duty-only (모드 2 전용)", "duty_only.emergency_current", "비상 제동 전류", "A", 0, 10, 2.0);
  d("staged (모드 1 전용)", "drive.maximum_duty", "최대 duty", "", 0, 0.5, 0.12);
  d("staged (모드 1 전용)", "drive.start_duty", "정지 출발 duty", "", 0, 0.3, 0.05);
  d("staged (모드 1 전용)", "drive.brake_maximum_current", "최대 제동 전류", "A", 0, 10, 2.0);
  d("안전", "drive.lane_timeout_sec", "차선 끊김 허용 시간 (0=제한 없음)", "s", 0, 60, 1.0);
  d("안전", "web.link_loss_stop_sec", "연결 끊김 정지 시간", "s", 0.2, 5, 1.0);
  r.add_order_constraint("duty_only.start_duty", "duty_only.maximum_duty");
  r.add_order_constraint("duty_only.ff_offset_duty", "duty_only.maximum_duty");
  r.add_order_constraint("lane.corner_entry_full_offset_m", "lane.corner_entry_distance_m");
}

Settings settings_from(const ParamRegistry & r) {
  Settings s;
  auto g = [&](const char * n) { return r.get(n); };
  s.run_speed = g("run.speed_mps");
  s.run_laps = int(std::lround(g("run.laps")));
  s.clockwise = g("slam.direction") >= 0.5;
  s.start_offset_m = g("slam.start_shift_m");
  s.start_lateral_m = g("slam.start_lateral_m");
  s.stabilization = g("camera.stabilization") >= 0.5;
  s.camera_yaw_deg = g("camera.yaw_deg");
  s.slam.stopline_correction = g("slam.stopline_correction") >= 0.5;
  s.slam.corner_anchor = g("slam.corner_anchor") >= 0.5;
  s.auto_calibrate = g("slam.auto_calibrate") >= 0.5;
  s.slam.max_range_m = g("slam.max_range_m");
  s.slam.stop_confirm = int(std::lround(g("slam.stop_confirm_frames")));
  s.slam.stop_gate_m = g("slam.stop_gate_m");
  s.slam.lap_window = g("slam.lap_window");
  s.stanley_gain = g("drive.stanley_gain");
  s.stanley_softening = g("drive.stanley_softening");
  s.heading_lookahead_m = g("drive.heading_lookahead_m");
  s.steering_current_weight = g("drive.steering_current_weight");
  s.steering_rate_rad_s = g("drive.steering_rate_deg_s") * 3.14159265358979323846 / 180.0;
  s.lane_side_tracking = g("lane.side_tracking") >= 0.5;
  s.corner_outward_offset_m = g("lane.corner_outward_offset_m");
  s.corner_entry_distance_m = g("lane.corner_entry_distance_m");
  s.corner_entry_full_offset_m = g("lane.corner_entry_full_offset_m");
  auto & l = s.lon;
  l.maximum_speed_mps = 2.5;               // 목표는 run.speed_mps 하나 (코너 감속 없음)
  l.minimum_speed_mps = 0.0;
  l.obstacle_speed_mps = 2.5;
  l.maximum_duty = g("drive.maximum_duty");
  l.longitudinal_start_duty = std::min(g("drive.start_duty"), l.maximum_duty);
  l.brake_maximum_current_amps = g("drive.brake_maximum_current");
  l.mode = int(std::lround(g("drive.longitudinal_mode")));
  auto & d = l.duty_only;
  d.kp = g("duty_only.pid_kp");
  d.ki = g("duty_only.pid_ki");
  d.ff_offset_duty = g("duty_only.ff_offset_duty");
  d.ff_duty_per_mps = g("duty_only.ff_duty_per_mps");
  d.maximum_duty = g("duty_only.maximum_duty");
  d.start_duty = g("duty_only.start_duty");
  d.duty_rise_rate_per_sec = g("duty_only.duty_rise_rate");
  d.duty_fall_rate_per_sec = g("duty_only.duty_fall_rate");
  d.emergency_overspeed_mps = g("duty_only.emergency_overspeed");
  d.emergency_brake_current_amps = g("duty_only.emergency_current");
  s.lane_timeout_sec = g("drive.lane_timeout_sec");
  s.link_loss_stop_sec = g("web.link_loss_stop_sec");
  return s;
}

}  // namespace semi_slam
