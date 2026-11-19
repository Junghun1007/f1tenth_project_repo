// duty-only 종방향 모드 보조 함수 (auto_control control_core 1113ML c791340 복사).
// duty는 속도 명령처럼 동작해서, 지금 속도에 필요한 duty보다 낮추면 VESC가 전류 제동 없이 회생 감속한다.
#pragma once

namespace semi_slam {

// 속도를 유지하는 데 필요한 duty. 정지 근처에서는 offset을 0까지 줄인다
double duty_feedforward(double speed_mps, double offset_duty, double duty_per_mps, double fade_speed_mps, double maximum_duty);

// 목표 지점까지 남은 거리에 따른 속도 상한. 마지막 구간(release²/(2·coast))은 release 속도로 들어가 모터를 풀고 굴러가고,
// 그 앞은 응답 지연을 두고 planning_decel로 duty 감속한다
double duty_only_stop_speed_limit(double remaining_m, double planning_deceleration_mps2, double coast_deceleration_mps2,
                                  double release_speed_mps, double response_sec);

// 지금 모터를 풀면 굴러서 목표 지점에 서는지. maximum_release_speed보다 빠르면 풀지 않는다
bool duty_only_coast_release(double remaining_m, double speed_mps, double coast_deceleration_mps2, double release_delay_sec,
                             double maximum_release_speed_mps);

}  // namespace semi_slam
