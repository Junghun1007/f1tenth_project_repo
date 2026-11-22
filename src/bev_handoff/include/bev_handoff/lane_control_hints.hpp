#pragma once
// 같은 프로세스 안의 주행 컴포넌트(auto_control::AutoDriveNode)가 차선 검출(line_detactor)에 주는 실시간 힌트.
// 주행 컴포넌트가 없으면(단독 실행) 아무도 쓰지 않아 line_detactor는 자기 파라미터대로 동작한다.
//  - want_reference: 콘이 근처에 있을 때만 코너 바깥 이동 없는 기준 중앙선을 한 번 더 만든다 (회피 계획의 기준선)
//  - 코너 바깥 이동 값과 차선 좌우 재판정: 웹 파라미터 창에서 바꾼 값을 다음 프레임부터 적용
#include <memory>

namespace bev_handoff
{
struct LaneControlHints
{
  bool want_reference{false};
  bool side_tracking{true};
  double corner_outward_offset_m{0.08};
  double corner_entry_distance_m{2.2};
  double corner_entry_full_offset_distance_m{0.6};
  // 터널 IR 차선 검출기(lane_hints_profile: ir) 전용 값. mask_threshold·lane_width_m < 0이면 검출기 자기 파라미터
  struct Ir
  {
    double mask_threshold{-1.0};
    double lane_width_m{-1.0};
    bool side_tracking{true};
    double corner_outward_offset_m{0.08};
    double corner_entry_distance_m{2.2};
    double corner_entry_full_offset_distance_m{0.6};
  } ir;
};
void publishLaneControlHints(std::shared_ptr<const LaneControlHints> hints);
std::shared_ptr<const LaneControlHints> laneControlHints();   // 없으면 nullptr
}  // namespace bev_handoff
