#include "semi_slam/runtime/lane_worker.hpp"
#include <depthai/depthai.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace semi_slam {

// ---------------------------------------------------------------- 차선
LaneWorker::LaneWorker(CameraPipeline & cam, StageStats & stats, std::function<void(const LaneObservation &)> on_lane)
  : cam_(cam), stats_(stats), on_lane_(std::move(on_lane)), post_(std::make_unique<LanePostprocessor>(LanePostConfig{})) {}

LaneWorker::~LaneWorker() { stop(); }

void LaneWorker::load_model(const ModelConfig & m, const std::function<void(const std::string &)> & progress) {
  auto backend = std::make_unique<TensorRtLaneBackend>(m.lane_model, m.lane_engine_cache, m.lane_precision, 120, 300,
                                                       m.workspace_bytes, m.lane_mask_threshold, 0.5f, true, -1.0f, progress);
  if (progress) progress("시험 추론");
  std::vector<std::uint8_t> zeros(120 * 300 * 3, 0);
  backend->infer_bgr(zeros.data(), zeros.size(), 120 * 3);
  std::lock_guard<std::mutex> l(mutex_);
  backend_ = std::move(backend);
}

void LaneWorker::set_geometry(const RectifiedCameraModel & camera, const BevConfig & bev) {
  const auto lut = generateRemap(camera, bev);
  auto proc = std::make_shared<CudaBevProcessor>(camera.image_width, camera.image_height, lut.map_x, lut.map_y,
                                                 BevInterpolation::Bilinear, EdgeAdaptiveConfig{});
  std::lock_guard<std::mutex> l(mutex_);
  bev_ = std::move(proc);
}

void LaneWorker::set_post_config(const LanePostConfig & c) {
  auto p = std::make_unique<LanePostprocessor>(c);
  std::lock_guard<std::mutex> l(mutex_);
  post_ = std::move(p);
}

void LaneWorker::start() {
  if (running_.exchange(true)) return;
  thread_ = std::thread([this] { loop_(); });
}

void LaneWorker::stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
}

void LaneWorker::loop_() {
  std::uint64_t seen = 0;
  auto & stats = stats_;
  while (running_) {
    const auto frame = cam_.frames().wait_newer(seen, 0.1);
    if (!frame) continue;
    std::shared_ptr<CudaBevProcessor> bev;
    { std::lock_guard<std::mutex> l(mutex_); bev = bev_; }
    if (!bev || !backend_) continue;
    const double t0 = now_sec();
    stats.record("receive", frame->t, t0);
    const auto & data = frame->rgb->getData();
    const std::size_t stride = frame->rgb->getStride() > 0 ? frame->rgb->getStride() : frame->rgb->getWidth();
    const bool want_jpeg = bev_wanted && bev_wanted() && t0 - last_jpeg_t_ >= 1.0 / 15.0;
    // 안정화: BEV LUT는 안정화된 영상 기준 → GPU 샘플링에 안정화→원본 homography 전달 (bev_processor와 같음)
    auto result = bev->process(data.data(), data.size(), stride, frame->source_to_stabilized.inv(cv::DECOMP_LU), 0, false);
    const double t1 = now_sec();
    stats.record("stabilize_bev", t0, t1);
    LaneObservation lane;
    {
      std::lock_guard<std::mutex> l(mutex_);
      backend_->infer_device_bgr(result.device_bgr, result.device_stride);
      const double t2 = now_sec();
      stats.record("lane_infer", t1, t2);
      const cv::Mat labels(300, 120, CV_8UC1, const_cast<std::uint8_t *>(backend_->label_data()));
      const auto * stop_data = backend_->stop_line_mask_data();
      const cv::Mat stop_mask = stop_data ? cv::Mat(300, 120, CV_8UC1, const_cast<std::uint8_t *>(stop_data)) : cv::Mat();
      lane = post_->process(labels, frame->t, false, stop_mask);
      stats.record("lane_post", t2, now_sec());
      for (int k = 0; k < lane.side_relabeled; ++k) stats.record("lane_side_fix", t2, t2);   // 진단: 좌우 교정 (회/s)
      for (int k = 0; k < lane.side_dropped; ++k) stats.record("lane_side_drop", t2, t2);    // 진단: 짝 틀린 조각 제외 (회/s)
    }
    if (want_jpeg && result.download_bgr) {   // 켜져 있을 때만 GPU→CPU 다운로드
      const cv::Mat bgr = result.download_bgr();
      std::vector<std::uint8_t> jpeg;
      if (!bgr.empty() && cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70}) && publish_bev_jpeg) publish_bev_jpeg(std::move(jpeg));
      last_jpeg_t_ = t0;
    }
    last_t_ = now_sec();
    if (on_lane_) on_lane_(lane);
  }
}

}  // namespace semi_slam
