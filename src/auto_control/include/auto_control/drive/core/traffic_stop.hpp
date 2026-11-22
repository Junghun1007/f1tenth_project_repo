// Copied from src/cone_drive_test/include/cone_drive_test/core/traffic_stop.hpp (1113ML) — auto_control로 옮김
// 신호등 정지 (traffic.enabled). 정책 (2026-11-22):
//  1) 정지 조건 = 빨간불 + 정지선. 빨간불만 보인다고 서지 않는다
//  2) 빨간불을 봤는데 정지선이 아직 없으면 정상 속도로 계속 달리며 찾는다 (빨간불은 red_memory_m 달리는 동안 기억).
//     정지선을 찾으면 그 앞(허용 정차 범위 0~stop_range_max)에 서도록 속도 상한
//  3) 찾았을 때 이미 너무 가까워 그 정지선 앞에 설 수 없으면 정지하지 않고 통과 (그 정지선을 지날 때까지)
//  4) 정지에 들어간 뒤에는 신호등이 안 보여도 초록불을 볼 때까지 정지 유지. 정지선이 안 보이면 바퀴 이동 거리로 위치를 이어 간다
//  5) 빨간불·초록불은 같은 색을 연속 signal_confirm 번 봐야 인정
//  6) 초록불 확인 → 바로 출발
//  신호 색: 화면에서 가장 크고 가운데에 가까운 traffic_light 박스의 HSV (검출기에서 고름)
//  정지선: 차선 모델 정지선 채널 + 중앙선 교차점 거리. 연속 확인 후 추적, 관측 사이는 이동 거리로
#pragma once
#include <cstdint>
#include <limits>
#include <opencv2/core.hpp>

namespace auto_control::drive {

enum class TrafficSignal : std::uint8_t { Unknown = 0, Red = 1, Green = 2 };

// BGR 박스 영상 → 신호 색 (traffic_detection_test traffic_light_node::color 와 같은 규칙)
TrafficSignal classify_traffic_color(const cv::Mat & bgr, int min_saturation, int min_value);

struct TrafficParams {
  bool enabled{true};
  double state_timeout{0.30};        // RED·GREEN 관측 유효 시간
  double line_timeout{0.50};         // 정지선을 못 봐도 추적을 유지하는 시간
  int line_confirm{3};               // 정지선 관측 확인 횟수
  double match_tolerance{0.30};      // 예측(이동 거리 보정)과 관측의 일치 허용
  double current_weight{0.35};       // 새 관측 반영 비율
  double terminal_distance{0.10};    // 앞범퍼→정지선이 이 안이면 근접 추적 (안 보여도 이동 거리로)
  double terminal_timeout{1.50};
  double stop_range_max{0.25};       // 허용 정차 범위: 앞범퍼가 정지선 앞 0 ~ 이 거리
  double stop_margin{0.10};          // 목표 정차점: 정지선 앞 이 거리
  double front_overhang{0.05};
  double decel{0.8};                 // staged 감속 계획
  double reapproach_speed{0.20};     // 정차 범위보다 짧게 섰을 때 다시 다가가는 속도 상한
  double pass_overshoot{0.30};       // 정지선을 이만큼 지나치면 정지·통과 상태 해제 (다음 정지선을 받음)
  int signal_confirm{3};             // 같은 색 연속 관측 횟수
  double red_memory_m{4.0};          // 빨간불을 마지막으로 본 뒤 이만큼 달릴 때까지 정지선을 찾는다
  double pass_tolerance{0.05};       // 예상 정지 거리가 (앞범퍼 → 정지선 + 이 값)보다 길면 못 선다고 보고 통과
  double max_decel{1.0};             // 통과 판정에 쓰는 실제 낼 수 있는 최대 감속도 (계획 감속도 decel보다 큼, 비상 제동 포함)
  bool duty_only{false};
  double coast_decel{0.5}, release_speed{0.35}, release_delay{0.03}, response{0.10};
};

struct TrafficState {
  TrafficSignal signal{TrafficSignal::Unknown};   // 유효 시간 안의 마지막 신호
  bool line_tracked{false};
  double line_m{std::numeric_limits<double>::quiet_NaN()};   // 앞범퍼 → 정지선 (추적 중)
  bool committed{false};   // 빨간불 정지 중 (GREEN 볼 때까지)
  bool holding{false};     // 정차 범위 안에 서 있음
  bool red_pending{false}; // 빨간불을 봤고 정지선을 찾는 중 (아직 정지 아님)
  bool passing{false};     // 너무 가까워 못 서서 이 정지선은 통과 중
  double cap{std::numeric_limits<double>::infinity()};
};

class TrafficStopper {
public:
  void set_params(const TrafficParams & p) { p_ = p; if (!p.enabled) reset(); }
  void reset() { s_ = TrafficState{}; cand_n_ = 0; red_t_ = green_t_ = -1e9; streak_n_ = 0; streak_sig_ = TrafficSignal::Unknown; }
  void on_signal(TrafficSignal sig, double t);   // 검출 프레임마다 (UNKNOWN도 전달)
  // stop_line_m: 이번 차선 프레임의 앞차축 → 정지선 거리 (NaN = 없음), t: 그 촬영 시각, dist: 그 시각 바퀴 누적 거리
  const TrafficState & update(double stop_line_m, double t, double dist, double v_ego);
  const TrafficState & state() const { return s_; }

private:
  double cap_for(double v_ego);
  double stop_distance(double v) const;   // 지금 속도에서 멈추는 데 필요한 거리 (응답 지연 + 감속)
  TrafficParams p_;
  TrafficState s_;
  double red_t_{-1e9}, green_t_{-1e9};
  TrafficSignal last_sig_{TrafficSignal::Unknown};
  double sig_t_{-1e9};
  double meas_line_{0}, meas_dist_{0}, meas_t_{0};   // 앞차축 기준
  double cand_line_{0}, cand_dist_{0};
  int cand_n_{0};
  TrafficSignal streak_sig_{TrafficSignal::Unknown};
  int streak_n_{0};
  double red_seen_dist_{0};   // 빨간불을 마지막으로 인정한 시점의 누적 거리
};

}  // namespace auto_control::drive
