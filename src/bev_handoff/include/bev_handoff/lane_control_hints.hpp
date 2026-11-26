#pragma once
// 같은 프로세스 안의 주행 컴포넌트(auto_control::AutoDriveNode)가 차선 검출(line_detactor)에 주는 실시간 힌트.
// 주행 컴포넌트가 없으면(단독 실행) 아무도 쓰지 않아 line_detactor는 자기 파라미터대로 동작한다.
//  - want_reference: 콘이 근처에 있을 때만 코너 바깥 이동 없는 기준 중앙선을 한 번 더 만든다 (회피 계획의 기준선)
//  - RGB/IR 검출·차선 연결·중앙선 프로필: 웹에서 적용한 값을 다음 프레임부터 사용
//  - rgb_model_path/rgb_engine_precision: 웹에서 고른 RGB 차선 모델. 비어 있으면 실행 시 모델 유지.
//    RGB 검출기는 다른 값이면 뒤에서 새 엔진을 만들고, 준비되면 재시작 없이 교체한다 (IR 검출기는 무시)
#include <memory>
#include <string>
#include "bev_handoff/lane_tuning.hpp"

namespace bev_handoff
{
struct LaneControlHints
{
  bool want_reference{false};
  LaneTuning rgb;
  LaneTuning ir;
  std::string rgb_model_path;
  std::string rgb_engine_precision;
};
void publishLaneControlHints(std::shared_ptr<const LaneControlHints> hints);
std::shared_ptr<const LaneControlHints> laneControlHints();   // 없으면 nullptr
}  // namespace bev_handoff
