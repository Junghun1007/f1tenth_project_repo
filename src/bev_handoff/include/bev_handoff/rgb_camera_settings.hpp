#pragma once
#include <memory>

namespace bev_handoff
{
struct RgbCameraSettings
{
  static constexpr int min_exposure_us{20}, max_exposure_us{14000};
  static constexpr int min_iso{100}, max_iso{1600};
  bool auto_exposure{true};
  int exposure_us{8000};
  int iso{400};

  bool operator==(const RgbCameraSettings & other) const
  {
    return auto_exposure == other.auto_exposure &&
      exposure_us == other.exposure_us && iso == other.iso;
  }
};

void publishRgbCameraSettings(std::shared_ptr<const RgbCameraSettings> settings);
std::shared_ptr<const RgbCameraSettings> rgbCameraSettings();
}  // namespace bev_handoff
