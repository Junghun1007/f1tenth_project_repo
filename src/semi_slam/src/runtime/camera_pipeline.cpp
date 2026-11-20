#include "semi_slam/runtime/camera_pipeline.hpp"
#include "semi_slam/copied/bev_geometry.hpp"
#include <depthai/depthai.hpp>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <thread>

namespace semi_slam {

namespace {
double seconds(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
  return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

cv::Matx33d matrix3(const std::vector<std::vector<float>> & rows, const char * label) {
  if (rows.size() < 3 || rows[0].size() < 3 || rows[1].size() < 3 || rows[2].size() < 3)
    throw std::runtime_error(std::string(label) + " is missing or smaller than 3x3");
  cv::Matx33d m;
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) m(r, c) = rows[r][c];
  return m;
}

}  // namespace

CameraPipeline::CameraPipeline(const CameraConfig & c, const ImuImageStabilizerConfig & s) : c_(c), sc_(s) {}

CameraPipeline::~CameraPipeline() { close(); }

std::string CameraPipeline::device_id() const { std::lock_guard<std::mutex> l(mutex_); return device_id_; }

OakStartupMeasurement CameraPipeline::measure(const OakStartupMeasurementConfig & mc, const std::function<bool()> & should_stop) {
  close();
  auto cfg = mc;
  if (cfg.device_id.empty()) cfg.device_id = device_id();
  cfg.roi_preview_enabled = false;
  const auto m = measureOakStartupExtrinsics(cfg, should_stop);
  std::lock_guard<std::mutex> l(mutex_);
  device_id_ = m.device_id;
  return m;
}

void CameraPipeline::open(const OakStartupMeasurement & pose, double camera_yaw_deg) {
  close();
  const std::string id = device_id();
  device_ = id.empty() ? std::make_shared<dai::Device>(dai::UsbSpeed::SUPER)
                       : std::make_shared<dai::Device>(dai::DeviceInfo(id), dai::UsbSpeed::SUPER);
  pipeline_ = std::make_unique<dai::Pipeline>(device_);
  pipeline_->setXLinkChunkSize(0);
  const float fps = float(c_.fps);
  auto camera = pipeline_->create<dai::node::Camera>()->build(
    dai::CameraBoardSocket::CAM_A, std::make_pair(std::uint32_t(c_.width), std::uint32_t(c_.height)), fps);
  auto * rgb_out = camera->requestOutput(std::make_pair(std::uint32_t(c_.width), std::uint32_t(c_.height)),
                                         dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, fps, c_.undistort);
  rgb_q_ = rgb_out->createOutputQueue(1U, false);
  // IMU: 보정된 가속도+각속도. 보정 출력 → 카메라 광학 좌표 상대 회전만 적용 (camera_driver와 같음)
  const auto calibration = device_->getCalibration();
  const auto imu_to_camera = matrix3(calibration.getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_A, false), "IMU-to-camera rotation");
  const auto imu_to_output = matrix3(calibration.getEepromData().imuExtrinsics.rotationMatrix, "IMU output rotation");
  imu_to_camera_ = imu_to_camera * imu_to_output.t();
  auto imu = pipeline_->create<dai::node::IMU>();
  imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_CALIBRATED, dai::IMUSensor::GYROSCOPE_CALIBRATED}, int(std::lround(c_.imu_rate_hz)));
  imu->setBatchReportThreshold(1);
  imu->setMaxBatchReports(5);
  imu_q_ = imu->out.createOutputQueue(64U, false);
  pipeline_->build();
  {   // semi_slam은 깊이를 쓰지 않는다 (RGB + IMU만)
    auto bridge = rgb_out->getXLinkBridge();
    if (!bridge || !bridge->xLinkOut) throw std::runtime_error("Missing XLink bridge");
    bridge->xLinkOut->input.setMaxSize(1);
    bridge->xLinkOut->input.setBlocking(false);
  }
  pipeline_->start();
  // 안정화 기준: 시작 측정 자세의 지면 법선 (카메라 광학 좌표)
  vehicle_from_camera_ = mountRotationVehicleFromCamera(degToRad(pose.roll_deg), degToRad(pose.pitch_down_deg), degToRad(camera_yaw_deg));
  stabilizer_ = std::make_unique<ImuImageStabilizer>(sc_);
  stabilizer_->setExternalReferenceUpCamera(vehicle_from_camera_.t() * cv::Vec3d(0, 0, 1));
  held_.clear();
  frames_.reopen();
  intrinsics_logged_ = false;
  running_ = true;
  imu_thread_ = std::thread([this] { imu_loop(); });
  frame_thread_ = std::thread([this] { frame_loop(); });
}

void CameraPipeline::close() {
  if (running_.exchange(false)) {
    frames_.close();
    if (frame_thread_.joinable()) frame_thread_.join();
    if (imu_thread_.joinable()) imu_thread_.join();
  }
  if (pipeline_) {
    pipeline_->stop();
    pipeline_->wait();
  }
  rgb_q_.reset();
  imu_q_.reset();
  pipeline_.reset();
  device_.reset();
}

void CameraPipeline::imu_loop() {
  while (running_) {
    auto data = imu_q_->tryGet<dai::IMUData>();   // 기존 패키지와 같은 non-blocking 읽기
    if (!data) { std::this_thread::sleep_for(std::chrono::microseconds(500)); continue; }
    for (const auto & p : data->packets) {
      const auto & a = p.acceleroMeter;
      const auto & g = p.gyroscope;
      const double ta = seconds(a.getTimestamp()), tg = seconds(g.getTimestamp());
      const cv::Vec3d acc = imu_to_camera_ * cv::Vec3d(a.x, a.y, a.z);
      const cv::Vec3d gyro = imu_to_camera_ * cv::Vec3d(g.x, g.y, g.z);
      std::optional<cv::Vec3d> sync_acc;
      if (std::abs(ta - tg) <= c_.maximum_imu_pair_skew_sec) sync_acc = acc;
      if (stabilizer_) stabilizer_->update(sync_acc, gyro, tg, stationary_hint ? stationary_hint() : std::nullopt);
      if (on_yaw_rate) on_yaw_rate((vehicle_from_camera_ * gyro)[2], tg);   // 차량 z축(위) 요레이트
      if (on_imu) on_imu(vehicle_from_camera_ * acc, sync_acc.has_value(), vehicle_from_camera_ * gyro, tg);
    }
  }
}

void CameraPipeline::frame_loop() {
  while (running_) {
    auto rgb = rgb_q_->tryGet<dai::ImgFrame>();
    if (!rgb) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    if (!rgb || rgb->getType() != dai::ImgFrame::Type::NV12) continue;
    if (!intrinsics_logged_) {   // 실제 프레임 내부 파라미터와 설정값 비교 (camera_driver는 프레임 값을 직접 씀)
      intrinsics_logged_ = true;
      const auto & tr = rgb->getTransformation();
      if (tr.isValid()) {
        const auto k = tr.getIntrinsicMatrix();
        const double fx = k[0][0], fy = k[1][1], cx = k[0][2], cy = k[1][2];
        const bool off = std::abs(fx / c_.fx - 1.0) > 0.01 || std::abs(fy / c_.fy - 1.0) > 0.01 || std::abs(cx - c_.cx) > 5.0 || std::abs(cy - c_.cy) > 5.0;
        std::fprintf(stderr, "[semi_slam] frame intrinsics fx=%.2f fy=%.2f cx=%.2f cy=%.2f / config fx=%.2f fy=%.2f cx=%.2f cy=%.2f%s\n",
                     fx, fy, cx, cy, c_.fx, c_.fy, c_.cx, c_.cy, off ? "  << MISMATCH: camera.fx/fy/cx/cy 확인 (BEV가 어긋남)" : "  (ok)");
      }
    }
    auto frame = std::make_shared<CameraFrame>();
    frame->rgb = rgb;
    frame->t = seconds(rgb->getTimestamp(dai::CameraExposureOffset::MIDDLE));
    frame->seq = ++seq_;
    if (stats) stats->record("cam_rgb", frame->t, frame->t);   // FPS 칸 = 실제 카메라 수신율
    if (!stabilization_on.load(std::memory_order_relaxed) || !stabilizer_) {   // 웹에서 실시간으로 켜고 끔 (IMU 적분은 계속)
      stab_stats_.count(StabilizationReason::Disabled, 0.0);
    } else if (!stabilizer_->initialized()) {
      stab_stats_.count(StabilizationReason::NotInitialized, 0.0);   // 시작 1 s 버림 + 4 s 정지 기준 측정 중
    } else {
      const auto corr = stabilizer_->correctionAt(frame->t);
      const StabilizationCamera cam{c_.fx, c_.fy, c_.cx, c_.cy, c_.width, c_.height};
      const auto d = corr ? decide_stabilization(cam, *corr) : StabilizationDecision{};
      if (d.apply) {
        held_.remember(d.source_to_stabilized);
        frame->source_to_stabilized = d.source_to_stabilized;
        frame->stabilized = true;
        stab_stats_.count(StabilizationReason::Applied, corr->correction_angle_deg);
      } else if (auto h = (d.reason == StabilizationReason::NoImuMatch ? held_.reuseForFrame(frame->seq) : std::nullopt)) {
        frame->source_to_stabilized = *h;   // 일시적 IMU 매칭 실패는 직전 보정을 짧게 유지 (camera_driver와 같음)
        frame->stabilized = true;
        stab_stats_.count(StabilizationReason::Held, 0.0);
      } else {
        if (d.reason == StabilizationReason::OverLimit) held_.clear();
        stab_stats_.count(d.reason, 0.0);
      }
    }
    frames_.put(frame);
  }
}

}  // namespace semi_slam
