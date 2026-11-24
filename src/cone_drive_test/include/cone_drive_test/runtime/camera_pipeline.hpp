// OAK 한 대를 직접 연다: 시작 자세 측정(oak_startup 복사본) → 본 파이프라인(CAM_A NV12, CAM_A에 정렬한 스테레오 깊이, IMU)
// IMU는 camera_driver 복사본 ImuImageStabilizer로 영상 흔들림 보정값을 만든다. Jetson 전용 (DepthAI).
#pragma once
#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <opencv2/core.hpp>
#include "cone_drive_test/copied/imu_image_stabilizer.hpp"
#include "cone_drive_test/copied/oak_startup_measurement.hpp"
#include "cone_drive_test/core/stabilization.hpp"
#include "cone_drive_test/core/stage_stats.hpp"
#include "cone_drive_test/runtime/latest.hpp"

namespace dai { class Device; class Pipeline; class MessageQueue; class ImgFrame; }

namespace cone_drive_test {

enum class StereoPreset { FastDensity = 0, FastAccuracy = 1, Default = 2 };

struct CameraConfig {
  std::string device_id;
  int width{1280}, height{800};
  double fps{70.0};
  double depth_fps{30.0};   // 스테레오 깊이 FPS (RGB와 따로)   // 기존 camera_driver/bev_processor와 같음 (차선은 매 프레임, YOLO는 yolo.max_fps)
  bool undistort{true};
  // 원본(확대 없는) 1280×800 rectified CAM_A 내부 파라미터. bev_config.yaml의 fx/fy(701.75)는 camera_driver의 1.25배 고정 확대가
  // 들어간 값이라 원본 영상에 쓰면 BEV·깊이 투영이 1.25배 어긋난다 → drive_node가 camera.fx / camera.fixed_view_zoom으로 넣는다
  double fx{701.751174926 / 1.25}, fy{701.420440674 / 1.25}, cx{643.032653809}, cy{392.621124268};
  int depth_width{640}, depth_height{400};
  StereoPreset stereo_preset{StereoPreset::FastDensity};
  int stereo_confidence{200};
  int stereo_extended_disparity{-1};  // -1이면 프리셋 설정 사용
  int stereo_subpixel_bits{-1};       // -1이면 프리셋 설정, 0이면 끔
  double dot_intensity{0.5};
  double imu_rate_hz{400.0};
  double depth_sync_ms{25.0};  // 30 FPS 깊이면 가장 가까운 깊이는 항상 ±17 ms 안
  double depth_wait_ms{20.0};   // 검출 후 깊이가 아직 안 왔으면 기다리는 최대 시간
  double maximum_imu_pair_skew_sec{0.003};
  bool stabilization_enabled{true};
};

struct CameraFrame {
  std::shared_ptr<dai::ImgFrame> rgb;        // NV12 width×height
  std::shared_ptr<dai::ImgFrame> depth;      // RAW16 mm, CAM_A 정렬, depth_width×depth_height (없을 수 있음)
  double t{0};                               // 촬영 시각 (steady clock s)
  std::uint64_t seq{0};
  cv::Matx33d source_to_stabilized{cv::Matx33d::eye()};
  bool stabilized{false};
};

class CameraPipeline {
public:
  CameraPipeline(const CameraConfig & c, const ImuImageStabilizerConfig & s);
  ~CameraPipeline();
  // 정지 상태 자세 측정 (측정 전용 파이프라인을 열고 닫음). 본 파이프라인이 열려 있으면 먼저 닫는다
  OakStartupMeasurement measure(const OakStartupMeasurementConfig & mc, const std::function<bool()> & should_stop);
  // 측정 자세로 본 파이프라인 시작. 자세는 안정화 기준(지면 법선)으로 쓰인다
  void open(const OakStartupMeasurement & pose, double camera_yaw_deg);
  void close();
  bool is_open() const { return running_; }
  const Latest<CameraFrame> & frames() const { return frames_; }
  const CameraConfig & config() const { return c_; }
  std::function<void(double yaw_rate_vehicle, double t)> on_yaw_rate;   // IMU 스레드에서 호출
  // 저주파 IMU 보정(옵션): 차가 만드는 가속도 (차량 좌표, 없으면 nullopt). 있으면 가속도계에서 빼고 안정화에 넘긴다 → 중력만 남음
  std::function<std::optional<cv::Vec3d>(double t)> vehicle_accel;
  std::function<std::optional<bool>()> stationary_hint;
  StageStats * stats{nullptr};   // 카메라 실제 수신 FPS 기록 (선택)                   // ERPM 기반 정지 여부 (camera_driver와 같이 안정화에 전달)
  StabilizationStats & stabilization_stats() { return stab_stats_; }
  std::string device_id() const;
  // 촬영 시각 t에 가장 가까운 깊이 (tolerance 안). 깊이는 같은 시각의 RGB보다 늦게 도착하므로 검출 후에 찾는다
  std::shared_ptr<dai::ImgFrame> nearest_depth(double t, double tolerance_sec) const;

private:
  void frame_loop();
  void imu_loop();
  CameraConfig c_;
  ImuImageStabilizerConfig sc_;
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> rgb_q_, depth_q_, imu_q_;
  std::unique_ptr<ImuImageStabilizer> stabilizer_;
  LastValidStabilizationHomography held_{2};
  bool intrinsics_logged_{false};   // frame_loop 스레드 전용
  StabilizationStats stab_stats_;
  cv::Matx33d imu_to_camera_{cv::Matx33d::eye()};
  cv::Matx33d vehicle_from_camera_{cv::Matx33d::eye()};
  Latest<CameraFrame> frames_;
  std::thread frame_thread_, imu_thread_;
  std::atomic<bool> running_{false};
  std::uint64_t seq_{0};
  mutable std::mutex mutex_;
  std::string device_id_;
  mutable std::mutex depth_mutex_;
  std::deque<std::pair<double, std::shared_ptr<dai::ImgFrame>>> depths_;
};

}  // namespace cone_drive_test
