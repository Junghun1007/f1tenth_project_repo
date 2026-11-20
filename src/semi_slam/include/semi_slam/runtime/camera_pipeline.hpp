// OAK 한 대를 직접 연다: 시작 자세 측정(oak_startup 복사본) → 본 파이프라인(CAM_A NV12, IMU). 깊이는 쓰지 않음
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
#include "semi_slam/copied/imu_image_stabilizer.hpp"
#include "semi_slam/copied/oak_startup_measurement.hpp"
#include "semi_slam/core/stabilization.hpp"
#include "semi_slam/core/stage_stats.hpp"
#include "semi_slam/runtime/latest.hpp"

namespace dai { class Device; class Pipeline; class MessageQueue; class ImgFrame; }

namespace semi_slam {

struct CameraConfig {
  std::string device_id;
  int width{1280}, height{800};
  double fps{70.0};
  bool undistort{true};
  // 원본(확대 없는) 1280×800 rectified CAM_A 내부 파라미터. bev_config.yaml의 fx/fy(701.75)는 camera_driver의 1.25배 고정 확대가
  // 들어간 값이라 원본 영상에 쓰면 BEV·깊이 투영이 1.25배 어긋난다 → drive_node가 camera.fx / camera.fixed_view_zoom으로 넣는다
  double fx{701.751174926 / 1.25}, fy{701.420440674 / 1.25}, cx{643.032653809}, cy{392.621124268};
  double imu_rate_hz{400.0};
  double maximum_imu_pair_skew_sec{0.003};
};

struct CameraFrame {
  std::shared_ptr<dai::ImgFrame> rgb;        // NV12 width×height
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
  // IMU 원값 6축 (차량 좌표 x 앞 y 왼쪽 z 위, 영점 보정 전). acc_ok = 가속도와 자이로 시각이 맞을 때만 acc가 유효. IMU 스레드에서 호출
  std::function<void(const cv::Vec3d & acc, bool acc_ok, const cv::Vec3d & gyro, double t)> on_imu;
  // 저주파 IMU 보정(옵션): 차가 만드는 가속도 (차량 좌표, 없으면 nullopt). 있으면 가속도계에서 빼고 안정화에 넘긴다 → 중력만 남음
  std::function<std::optional<cv::Vec3d>(double t)> vehicle_accel;
  std::function<std::optional<bool>()> stationary_hint;
  StageStats * stats{nullptr};   // 카메라 실제 수신 FPS 기록 (선택)
  std::atomic<bool> stabilization_on{true};   // IMU 영상 안정화 켜기/끄기 (실시간)
  StabilizationStats & stabilization_stats() { return stab_stats_; }
  std::string device_id() const;

private:
  void frame_loop();
  void imu_loop();
  CameraConfig c_;
  ImuImageStabilizerConfig sc_;
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> rgb_q_, imu_q_;
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
};

}  // namespace semi_slam
