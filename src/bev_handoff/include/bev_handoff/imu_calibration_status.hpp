#pragma once
// camera_driver의 IMU 영상 안정화 보정 상태를 같은 프로세스의 주행 컴포넌트(auto_control)에 알린다.
// 시작할 때와 웹 [Refit] 뒤에 차가 정지한 채로 보정(기본 4 s)이 끝나야 ready가 된다.
// 보정 전에는 RGB는 보정 없이 지나가지만 터널 IR 입력은 IMU 보정값을 기다리다 모두 버려진다 → 출발 전 점검에 쓴다.
namespace bev_handoff
{
enum class ImuCalibrationState : int
{
  unknown = 0,       // camera_driver가 아직 알리지 않음 (단독 실행 등) → 점검하지 않음
  disabled = 1,      // IMU 영상 안정화를 쓰지 않음 → 점검하지 않음
  calibrating = 2,   // 보정 중 (정지 유지 필요)
  ready = 3,
};
void publishImuCalibrationState(ImuCalibrationState state);
ImuCalibrationState imuCalibrationState();
}  // namespace bev_handoff
