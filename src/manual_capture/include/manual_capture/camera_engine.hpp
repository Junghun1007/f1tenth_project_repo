#ifndef MANUAL_CAPTURE__CAMERA_ENGINE_HPP_
#define MANUAL_CAPTURE__CAMERA_ENGINE_HPP_

// Owns the OAK device. The header is DepthAI free so the GUI does not depend
// on the SDK headers.

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "manual_capture/bev_projector.hpp"
#include "manual_capture/capture_logic.hpp"
#include "manual_capture/imu_tilt_tracker.hpp"
#include "opencv2/core.hpp"

namespace manual_capture
{

enum class RgbFormat
{
  NV12,
  RGB888,
};

// Values that need a pipeline restart.
struct CameraConfig
{
  std::string device_id;
  int rgb_width{1280};
  int rgb_height{800};
  double rgb_fps{30.0};
  RgbFormat rgb_format{RgbFormat::NV12};
  bool rgb_undistort{true};
  int stereo_width{1280};
  int stereo_height{800};
  double stereo_fps{30.0};
  StereoMode stereo_mode{StereoMode::LEFT_RIGHT};
  double virtual_camera_position_ratio{0.5};
  double sync_threshold_ms{2.0};
};

struct StereoExposureReadback
{
  double sent_at_sec{0.0};
  std::optional<ExposureReading> latest;
  std::string send_error;
};

// Values that change while the pipeline runs.
struct LiveControls
{
  ExposureSetting rgb;
  ExposureSetting left;
  ExposureSetting right;
  double flood{0.0};
  double dot{0.0};
};

struct RgbFrame
{
  cv::Mat bgr;
  std::uint64_t sequence{0U};
  double timestamp_sec{0.0};
  int exposure_us{0};
  int iso{0};
  FrameTilt tilt;  // IMU tilt at this frame's mid-exposure time
};

struct StereoFrame
{
  cv::Mat left;
  cv::Mat right;      // empty in LEFT_ONLY mode
  cv::Mat disparity;  // empty in LEFT_ONLY mode
  std::uint64_t sequence{0U};
  double timestamp_sec{0.0};
  int left_exposure_us{0};
  int left_iso{0};
  int right_exposure_us{0};
  int right_iso{0};
  FrameTilt tilt;  // IMU tilt at the left frame's mid-exposure time
};

// Called on the IMU thread for every calibrated sample (CAM_A optical frame).
using ImuObserver = std::function<void(const ImuSample &)>;

class CameraEngine
{
public:
  CameraEngine();
  ~CameraEngine();

  CameraEngine(const CameraEngine &) = delete;
  CameraEngine & operator=(const CameraEngine &) = delete;

  // Set before start(). The tracker outlives restarts so its calibration is kept.
  void setImu(std::shared_ptr<ImuTiltTracker> tracker, ImuObserver observer);
  // Empty while the IMU streams; otherwise why it is not available.
  std::string imuError() const;

  // Blocking. Throws on failure, leaving the engine stopped.
  void start(const CameraConfig & config, const LiveControls & controls);
  void stop();
  bool running() const;

  const CameraConfig & config() const;
  std::string deviceId() const;
  // Increments on every successful start; BEV projectors reconfigure on change.
  std::uint64_t generation() const;

  std::shared_ptr<const RgbFrame> latestRgb() const;
  std::shared_ptr<const StereoFrame> latestStereo() const;
  std::optional<LensCalibration> rgbCalibration() const;
  std::optional<LensCalibration> leftCalibration() const;

  void setRgbExposure(const ExposureSetting & setting);
  void setStereoExposure(const ExposureSetting & left, const ExposureSetting & right);
  std::array<StereoExposureReadback, 2> stereoExposureReadback() const;  // CAM_B, CAM_C
  void setFlood(double intensity);
  void setDot(double intensity);
  LiveControls controls() const;

  double measuredRgbFps() const;
  double measuredStereoFps() const;
  std::string lastError() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manual_capture

#endif  // MANUAL_CAPTURE__CAMERA_ENGINE_HPP_
