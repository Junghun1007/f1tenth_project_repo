#include "cone_drive_test/runtime/perception.hpp"
#include "cone_drive_test/core/rate_gate.hpp"
#include <depthai/depthai.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace cone_drive_test {

namespace {
const char * kClassNames[] = {"con", "truck", "sedan", "traffic_light"};
}

// ---------------------------------------------------------------- 차선
LaneWorker::LaneWorker(CameraPipeline & cam, ControlLoop & loop, Publish publish)
  : cam_(cam), ctl_(loop), publish_(std::move(publish)), post_(std::make_unique<LanePostprocessor>(LanePostConfig{})) {}

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
  auto & stats = ctl_.stats();
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
      lane = post_->process(labels, frame->t, ctl_.cones_nearby());
      stats.record("lane_post", t2, now_sec());
    }
    if (want_jpeg && result.download_bgr) {   // 켜져 있을 때만 GPU→CPU 다운로드
      const cv::Mat bgr = result.download_bgr();
      std::vector<std::uint8_t> jpeg;
      if (!bgr.empty() && cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70}) && publish_bev_jpeg) publish_bev_jpeg(std::move(jpeg));
      last_jpeg_t_ = t0;
    }
    last_t_ = now_sec();
    publish_(ctl_.on_lane(lane, now_sec()));
  }
}

// ---------------------------------------------------------------- 검출
DetectWorker::DetectWorker(CameraPipeline & cam, ControlLoop & loop) : cam_(cam), ctl_(loop) {}

DetectWorker::~DetectWorker() { stop(); }

void DetectWorker::load_model(const ModelConfig & m, const std::function<void(const std::string &)> & progress) {
  auto det = std::make_unique<YoloxDetector>(m.yolo_model, "TENSORRT", m.yolo_engine_cache, m.yolo_precision, 512, 320, 0.0f, 0.65f,
                                             m.workspace_bytes, progress, 100);
  if (progress) progress("시험 추론");
  cv::Mat zeros(320, 512, CV_8UC3, cv::Scalar(114, 114, 114));
  det->detect(zeros);
  std::lock_guard<std::mutex> l(mutex_);
  detector_ = std::move(det);
  detector_->set_thresholds(s_.yolo_thresholds);
}

void DetectWorker::set_settings(const Settings & s) {
  std::lock_guard<std::mutex> l(mutex_);
  s_ = s;
  if (detector_) detector_->set_thresholds(s.yolo_thresholds);
}

void DetectWorker::set_mount(const CameraMount & m) { std::lock_guard<std::mutex> l(mutex_); mount_ = m; }

void DetectWorker::start() {
  if (running_.exchange(true)) return;
  thread_ = std::thread([this] { loop_(); });
}

void DetectWorker::stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
}

void DetectWorker::loop_() {
  std::uint64_t seen = 0;
  RateGate gate(40.0);   // 카메라(70 fps)보다 낮은 검출 FPS를 평균으로 정확히 맞춤
  auto & stats = ctl_.stats();
  const auto & cc = cam_.config();
  while (running_) {
    const auto frame = cam_.frames().wait_newer(seen, 0.1);   // 최신 프레임만 (처리 중 들어온 프레임은 건너뜀)
    if (!frame || !detector_) continue;
    Settings s;
    CameraMount mount;
    { std::lock_guard<std::mutex> l(mutex_); s = s_; mount = mount_; }
    gate.set_rate(s.yolo_max_fps);
    if (!gate.accept(frame->t)) continue;
    const double t0 = now_sec();
    const int w = int(frame->rgb->getWidth()), h = int(frame->rgb->getHeight());
    const int roi_h = std::clamp(int(std::lround(h * s.roi_height_percent / 100.0)), 1, h);
    const cv::Rect roi(0, h - roi_h, w, roi_h);
    const auto & data = frame->rgb->getData();
    const std::size_t stride = frame->rgb->getStride() > 0 ? frame->rgb->getStride() : frame->rgb->getWidth();
    YoloxDetectionResult res;
    {
      std::lock_guard<std::mutex> l(mutex_);
      res = detector_->detect_nv12(data.data(), data.size(), stride, w, h, roi);
    }
    const double t1 = now_sec();
    stats.record("yolo_pre", t0, t0 + res.timing.preprocessing_nanoseconds * 1e-9);
    stats.record("yolo_infer", t0, t0 + (res.timing.input_transfer_nanoseconds + res.timing.forward_nanoseconds + res.timing.output_transfer_nanoseconds) * 1e-9);
    stats.record("yolo_post", t0, t0 + res.timing.postprocessing_nanoseconds * 1e-9);
    DetectionFrame out;
    out.capture_time = frame->t;
    // 추론하는 동안 도착한 깊이까지 포함해 다시 짝짓기. 끝내 없으면 이 프레임은 관측으로 치지 않는다
    const auto depth = frame->depth ? frame->depth : cam_.nearest_depth(frame->t, cc.depth_sync_ms / 1000.0);
    out.depth_valid = depth != nullptr;
    if (depth) {
      auto & d = *depth;
      const int dw = int(d.getWidth()), dh = int(d.getHeight());
      const double sx = double(dw) / w, sy = double(dh) / h;
      const DepthIntrinsics k{cc.fx * sx, cc.fy * sy, cc.cx * sx, cc.cy * sy, dw, dh};
      const auto * mm = reinterpret_cast<const std::uint16_t *>(d.getData().data());
      const int stride_px = d.getStride() > 0 ? int(d.getStride() / 2) : dw;
      for (const auto & det : res.detections) {
        const Footprint fp = det.class_id == 0 ? Footprint{s.cone_size_m, s.cone_size_m}
                           : det.class_id == 1 ? Footprint{0.60, 0.36} : det.class_id == 2 ? Footprint{0.50, 0.30} : Footprint{};
        const cv::Rect2f box(det.box.x * sx, det.box.y * sy, det.box.width * sx, det.box.height * sy);
        if (auto o = localize_box(mm, stride_px, k, mount, box, det.class_id, det.score, fp, s.box)) out.cones.push_back(*o);
      }
    }
    stats.record("box_depth", t1, now_sec());
    ctl_.on_detections(out);
    if (camera_wanted && camera_wanted() && t0 - last_jpeg_t_ >= 0.1) {   // 검출 확인용 카메라 보기 (켰을 때만)
      const cv::Mat nv12(h * 3 / 2, w, CV_8UC1, const_cast<std::uint8_t *>(data.data()), stride);
      cv::Mat bgr;
      cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
      cv::line(bgr, {0, roi.y}, {w, roi.y}, {0, 220, 255}, 2);
      for (const auto & det : res.detections) {
        cv::rectangle(bgr, det.box, {60, 60, 255}, 2);
        char label[48];
        std::snprintf(label, sizeof(label), "%s %.2f", kClassNames[std::clamp(det.class_id, 0, 3)], det.score);
        cv::putText(bgr, label, det.box.tl() + cv::Point2f(0, -4), cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 2);
      }
      cv::Mat small;
      cv::resize(bgr, small, {640, 400}, 0, 0, cv::INTER_AREA);
      std::vector<std::uint8_t> jpeg;
      if (cv::imencode(".jpg", small, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70}) && publish_camera_jpeg) publish_camera_jpeg(std::move(jpeg));
      last_jpeg_t_ = t0;
    }
  }
}

}  // namespace cone_drive_test
