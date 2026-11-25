#pragma once
// 웹(auto_control::AutoDriveNode) → camera_driver: 스테레오 흑백 카메라(CAM_B/C) 노출·ISO와 IR 조명 세기.
// 터널 안(차선 IR)과 터널 밖(장애물 깊이)을 따로 둔다. 아무도 안 내면 camera_driver는 자기 파라미터·기본값대로.
#include <cstdint>
#include <memory>

namespace bev_handoff
{
struct IrCameraProfile
{
  int exposure_us{0};   // 0 = 자동노출 (iso 무시)
  int iso{800};
  double dot_intensity{0.0};     // 점 투사기 0..1
  double flood_intensity{0.0};   // 플러드 조명 0..1

  bool operator==(const IrCameraProfile & other) const
  {
    return exposure_us == other.exposure_us && iso == other.iso &&
      dot_intensity == other.dot_intensity && flood_intensity == other.flood_intensity;
  }
};

struct IrCameraSettings
{
  IrCameraProfile inside{20000, 500, 0.0, 1.0};   // 터널 안: IR 차선
  IrCameraProfile outside{0, 400, 0.5, 0.0};      // 터널 밖: 장애물 스테레오 깊이
  bool use_400p_ir_output{false};                 // false: camera_driver의 시작 해상도
};

void publishIrCameraSettings(std::shared_ptr<const IrCameraSettings> settings);
std::shared_ptr<const IrCameraSettings> irCameraSettings();   // 없으면 nullptr
std::uint64_t irCameraSettingsRevision();                     // 바뀔 때마다 +1
// Runtime settings shared by the controller, camera, IR BEV and selector in one container.
struct TunnelRuntimeSettings
{
  bool validation_mode{true};
  bool ir_length_exit_enabled{false};
  double ir_length_exit_ratio{0.15};
  int ir_length_exit_confirm_frames{3};
  double ir_length_exit_minimum_reference_m{0.5};
};

void publishTunnelRuntimeSettings(std::shared_ptr<const TunnelRuntimeSettings> settings);
std::shared_ptr<const TunnelRuntimeSettings> tunnelRuntimeSettings();
}  // namespace bev_handoff
