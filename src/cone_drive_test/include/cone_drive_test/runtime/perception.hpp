// 차선 워커(매 프레임: CUDA BEV → TensorRT Fast-SCNN → 후처리 → 제어)와 검출 워커(최신 프레임만: YOLOX → 박스 깊이 → 콘)
// Jetson 전용 (CUDA, TensorRT).
#pragma once
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "cone_drive_test/copied/bev_geometry.hpp"
#include "cone_drive_test/copied/cuda_bev_processor.hpp"
#include "cone_drive_test/copied/tensorrt_lane_backend.hpp"
#include "cone_drive_test/copied/yolox_detector.hpp"
#include "cone_drive_test/core/control_loop.hpp"
#include "cone_drive_test/core/lane_post.hpp"
#include "cone_drive_test/runtime/camera_pipeline.hpp"

namespace cone_drive_test {

struct ModelConfig {
  std::string lane_model, lane_engine_cache, lane_precision{"fp16"};
  std::string yolo_model, yolo_engine_cache, yolo_precision{"fp16"};
  std::size_t workspace_bytes{1024u * 1024u * 1024u};
  float lane_mask_threshold{0.5f};
};

class LaneWorker {
public:
  using Publish = std::function<void(const StepOutput &)>;
  LaneWorker(CameraPipeline & cam, ControlLoop & loop, Publish publish);
  ~LaneWorker();
  void load_model(const ModelConfig & m, const std::function<void(const std::string &)> & progress);   // 엔진 생성 + 시험 추론
  void set_geometry(const RectifiedCameraModel & camera, const BevConfig & bev);   // 자세가 바뀌면 LUT 재생성
  void set_post_config(const LanePostConfig & c);
  void start();
  void stop();
  std::function<bool()> bev_wanted;                                  // 웹에서 BEV 원본을 볼 때만 다운로드·인코딩
  std::function<void(std::vector<std::uint8_t>)> publish_bev_jpeg;
  double last_frame_time() const { return last_t_; }

private:
  void loop_();
  CameraPipeline & cam_;
  ControlLoop & ctl_;
  Publish publish_;
  std::unique_ptr<TensorRtLaneBackend> backend_;
  std::shared_ptr<CudaBevProcessor> bev_;
  std::unique_ptr<LanePostprocessor> post_;
  std::mutex mutex_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<double> last_t_{0};
  double last_jpeg_t_{0};
};

class DetectWorker {
public:
  DetectWorker(CameraPipeline & cam, ControlLoop & loop);
  ~DetectWorker();
  void load_model(const ModelConfig & m, const std::function<void(const std::string &)> & progress);
  void set_settings(const Settings & s);
  void set_mount(const CameraMount & m);
  void start();
  void stop();
  std::function<bool()> camera_wanted;
  std::function<void(std::vector<std::uint8_t>)> publish_camera_jpeg;

private:
  void loop_();
  CameraPipeline & cam_;
  ControlLoop & ctl_;
  std::unique_ptr<YoloxDetector> detector_;
  std::mutex mutex_;
  Settings s_;
  CameraMount mount_{};
  std::thread thread_;
  std::atomic<bool> running_{false};
  double last_jpeg_t_{0};
};

}  // namespace cone_drive_test
