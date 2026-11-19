// 차선 워커 (매 프레임: IMU 안정화 homography → CUDA BEV → TensorRT Fast-SCNN → 후처리) → on_lane 콜백 (제어·매핑)
// Jetson 전용 (CUDA, TensorRT).
#pragma once
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "semi_slam/copied/bev_geometry.hpp"
#include "semi_slam/copied/cuda_bev_processor.hpp"
#include "semi_slam/copied/tensorrt_lane_backend.hpp"
#include "semi_slam/core/lane_post.hpp"
#include "semi_slam/core/stage_stats.hpp"
#include "semi_slam/runtime/camera_pipeline.hpp"

namespace semi_slam {

struct ModelConfig {
  std::string lane_model, lane_engine_cache, lane_precision{"int8"};
  std::size_t workspace_bytes{1024u * 1024u * 1024u};
  float lane_mask_threshold{0.5f};
};

class LaneWorker {
public:
  LaneWorker(CameraPipeline & cam, StageStats & stats, std::function<void(const LaneObservation &)> on_lane);
  ~LaneWorker();
  void load_model(const ModelConfig & m, const std::function<void(const std::string &)> & progress);
  void set_geometry(const RectifiedCameraModel & camera, const BevConfig & bev);
  void set_post_config(const LanePostConfig & c);
  void start();
  void stop();
  std::function<bool()> bev_wanted;
  std::function<void(std::vector<std::uint8_t>)> publish_bev_jpeg;
  double last_frame_time() const { return last_t_; }

private:
  void loop_();
  CameraPipeline & cam_;
  StageStats & stats_;
  std::function<void(const LaneObservation &)> on_lane_;
  std::unique_ptr<TensorRtLaneBackend> backend_;
  std::shared_ptr<CudaBevProcessor> bev_;
  std::unique_ptr<LanePostprocessor> post_;
  std::mutex mutex_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<double> last_t_{0};
  double last_jpeg_t_{0};
};

}  // namespace semi_slam
