#pragma once
// camera_driver의 IMU 영상 안정화 보정 상태를 같은 프로세스의 주행 컴포넌트(auto_control)에 알린다.
// 시작할 때와 웹 [Refit] 뒤에 차가 정지한 채로 보정(기본 4 s)이 끝나야 ready가 된다.
// 보정 전에는 RGB는 보정 없이 지나가지만 터널 IR 입력은 IMU 보정값을 기다리다 모두 버려진다 → 출발 전 점검에 쓴다.
#include <memory>
#include <string>

namespace bev_handoff
{
enum class ImuCalibrationState : int
{
  unknown = 0,       // camera_driver가 아직 알리지 않음 (시작·카메라 초기화 직후, 단독 실행)
  disabled = 1,      // IMU 영상 안정화를 쓰지 않음 → 점검하지 않음
  calibrating = 2,   // 보정 중 (정지 유지 필요)
  ready = 3,
};
void publishImuCalibrationState(ImuCalibrationState state);
ImuCalibrationState imuCalibrationState();

// 보정 중 진행 (점검표 표시용, 약 5 Hz). 보정이 끝나면 nullptr
struct ImuCalibrationProgress
{
  bool discarding{false};    // 시작 직후 IMU 표본을 버리는 구간
  double elapsed_sec{0.0}, target_sec{0.0};
  std::string reason;        // 마지막으로 보정을 다시 시작한 이유 (움직임 등), 없으면 빈 문자열
};
void publishImuCalibrationProgress(std::shared_ptr<const ImuCalibrationProgress> progress);
std::shared_ptr<const ImuCalibrationProgress> imuCalibrationProgress();
}  // namespace bev_handoff
