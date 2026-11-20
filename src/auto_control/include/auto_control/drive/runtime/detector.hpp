// 검출 워커 (auto_control 주행 컴포넌트): 최신 카메라 NV12 프레임만 → YOLOX(TensorRT) → 신호등 색, 박스 깊이 → 콘·차량 위치.
// cone_drive_test/src/runtime/perception.cpp의 DetectWorker를 auto_control 파이프라인(camera_driver 직접 전달 프레임,
// CAM_A 정렬 깊이)에 맞춘 것. ROS·DepthAI 없이 빌드된다 (TensorRT가 없으면 tensorrt_disabled로 링크).
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "auto_control/drive/copied/yolox_detector.hpp"
#include "auto_control/drive/core/box_depth.hpp"
#include "auto_control/drive/core/control_loop.hpp"

namespace auto_control::drive {

struct ModelConfig {
  std::string yolo_model, yolo_engine_cache, yolo_precision{"fp16"};
  std::string yolo_backend{"TENSORRT"};   // CPU = OpenCV DNN (개발 PC 확인용, 느림)
  std::size_t workspace_bytes{1024u * 1024u * 1024u};
};

// 카메라 NV12 한 장 (owner가 버퍼를 붙잡고 있음). t = 촬영 시각 (steady, s)
//  호스트: nv12 (신호등 색·카메라 보기에 씀)  /  GPU: device (camera_driver 공유 업로드, 있으면 추론은 이걸로)
struct Nv12Frame {
  std::shared_ptr<const void> owner;
  const std::uint8_t * nv12{nullptr};
  std::size_t size{0}, stride{0};
  int width{0}, height{0};
  double t{0};
  std::shared_ptr<const void> device_owner;
  const std::uint8_t * device{nullptr};   // Y 평면, UV는 height×device_pitch 위치
  std::size_t device_pitch{0};
};

// 깊이 한 장 (RAW16 mm, CAM_A 정렬). 내부 파라미터는 이 깊이 영상 기준
struct DepthImage {
  std::shared_ptr<const void> owner;
  const std::uint8_t * data{nullptr};
  std::size_t stride{0};   // 바이트
  int width{0}, height{0};
  double t{0};
  double fx{0}, fy{0}, cx{0}, cy{0};
};

// 검출 영상(카메라 NV12)의 내부 파라미터 (bev_processor가 공유하는 값, 고정 확대 포함)
struct RgbIntrinsics { double fx{0}, fy{0}, cx{0}, cy{0}; };

// RGB 박스 → 깊이 영상 박스 (두 영상의 내부 파라미터가 달라도 같은 시선으로 맞춘다. 정렬 깊이라 회전·이동은 없음)
cv::Rect2f rgb_box_to_depth(const cv::Rect2f & box, const RgbIntrinsics & rgb, const DepthImage & depth);

class DetectWorker {
public:
  explicit DetectWorker(ControlLoop & loop);
  ~DetectWorker();
  void load_model(const ModelConfig & m, const std::function<void(const std::string &)> & progress);   // 엔진 생성 + 시험 추론
  void set_settings(const Settings & s);
  void set_mount(const CameraMount & m, const RgbIntrinsics & k);
  void set_paused(bool paused) { paused_ = paused; }   // 터널(IR) 구간: 추론하지 않음
  bool ready() const;
  void push_frame(std::shared_ptr<const Nv12Frame> f);   // 카메라 스레드 (호스트 NV12): 최신 것만 남기고 바로 돌아감
  // GPU 원본 (camera_driver 공유 업로드). 들어오기 시작하면 추론은 이것으로 하고, 같은 시각의 호스트 프레임을 짝지어 둔다
  void push_device_frame(std::shared_ptr<const Nv12Frame> f);
  void push_depth(std::shared_ptr<const DepthImage> d);  // 깊이 스레드
  void start();
  void stop();
  double fps() const { return fps_.load(); }   // 최근 1초 추론 수
  float last_signal_score() const { return signal_score_.load(); }
  std::function<bool()> camera_wanted;
  std::function<void(std::vector<std::uint8_t>)> publish_camera_jpeg;

private:
  void loop_();
  void jpeg_loop_();   // 웹 카메라 보기: 변환·그리기·인코딩을 검출 스레드 밖에서
  struct CameraView { std::shared_ptr<const Nv12Frame> frame; std::vector<Detection> detections; int roi_y{0}; };
  std::shared_ptr<const DepthImage> nearest_depth(double t, double max_dt) const;
  ControlLoop & ctl_;
  std::unique_ptr<YoloxDetector> detector_;
  mutable std::mutex mutex_;          // detector_, s_, mount_, rgb_k_
  Settings s_;
  CameraMount mount_{};
  RgbIntrinsics rgb_k_{};
  bool mount_ok_{false};
  std::mutex frame_m_;
  std::condition_variable frame_cv_;
  std::shared_ptr<const Nv12Frame> pending_;
  std::deque<std::shared_ptr<const Nv12Frame>> host_recent_;   // GPU 프레임과 짝지을 최근 호스트 프레임 (최대 4장)
  double last_device_t_{-1e9};                                 // 마지막 GPU 프레임 도착 (now_sec)
  mutable std::mutex depth_m_;
  std::deque<std::shared_ptr<const DepthImage>> depths_;   // 최근 깊이 (최대 8장)
  std::thread thread_, jpeg_thread_;
  std::mutex jpeg_m_;
  std::condition_variable jpeg_cv_;
  std::unique_ptr<CameraView> jpeg_pending_;
  std::atomic<bool> running_{false}, paused_{false};
  std::atomic<double> fps_{0};
  std::atomic<float> signal_score_{0.f};
  double last_jpeg_t_{0};
  double depth_sync_s_{0.025}, depth_wait_s_{0.020};   // 30 FPS 깊이: 가장 가까운 깊이는 ±17 ms 안. 아직 안 왔으면 최대 20 ms 기다림
};

}  // namespace auto_control::drive
