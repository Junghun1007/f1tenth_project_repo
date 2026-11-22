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
};

struct IrCameraSettings
{
  IrCameraProfile inside{30000, 800, 0.0, 0.5};   // 터널 안: IR 차선
  IrCameraProfile outside{0, 400, 0.5, 0.0};      // 터널 밖: 장애물 스테레오 깊이
};

void publishIrCameraSettings(std::shared_ptr<const IrCameraSettings> settings);
std::shared_ptr<const IrCameraSettings> irCameraSettings();   // 없으면 nullptr
std::uint64_t irCameraSettingsRevision();                     // 바뀔 때마다 +1
}  // namespace bev_handoff
