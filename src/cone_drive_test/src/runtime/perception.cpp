#include "cone_drive_test/runtime/perception.hpp"
#include "cone_drive_test/core/rate_gate.hpp"
#include "cone_drive_test/core/segmentation.hpp"
#include "cone_drive_test/copied/tensorrt_yolox_backend.hpp"
#include <depthai/depthai.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace cone_drive_test {

namespace {
// 박스 아래 끝이 보이나: 다른 박스(더 가까운 물체)가 아래 끝 줄을 가로로 겹쳐 덮고 있으면 가려진 것으로 본다
bool bottom_visible(const Detection & d, const std::vector<Detection> & all) {
  const float bottom = d.box.y + d.box.height;
  for (const auto & o : all) {
    if (&o == &d) continue;
    const float ob = o.box.y + o.box.height;
    const bool covers_row = o.box.y < bottom && ob > bottom;   // 아래 끝 줄이 상대 박스 안 (상대가 더 아래까지 내려옴 = 더 가까움)
    const bool overlaps_x = std::min(d.box.x + d.box.width, o.box.x + o.box.width) > std::max(d.box.x, o.box.x);
    if (covers_row && overlaps_x) return false;
  }
  return true;
}
}  // namespace

namespace {
const char * kClassNames[] = {"con", "truck", "sedan", "traffic_light"};

Footprint footprint_for(int cls, const Settings & s) {
  return cls == 0 ? Footprint{s.cone_size_m, s.cone_size_m}
       : cls == 1 ? Footprint{0.60, 0.36} : cls == 2 ? Footprint{0.50, 0.30} : Footprint{};
}

void draw_mask_outline(cv::Mat & small, const SegDetection & det, const SegTransform & transform,
                       const std::uint16_t * prototypes, int width, int height) {
  // The optional preview traces the outline at one quarter of its display resolution.
  constexpr int step = 4;
  const int preview_w = small.cols / step, preview_h = small.rows / step;
  const double sx = double(preview_w) / width, sy = double(preview_h) / height;
  const auto & box = det.box;
  const cv::Rect region = cv::Rect(cv::Point(int(std::floor(box.x * sx)), int(std::floor(box.y * sy))),
    cv::Point(int(std::ceil((box.x + box.width) * sx)), int(std::ceil((box.y + box.height) * sy)))) &
    cv::Rect(0, 0, preview_w, preview_h);
  if (region.empty()) return;
  cv::Mat mask(region.height, region.width, CV_8UC1);
  for (int y = 0; y < region.height; ++y) for (int x = 0; x < region.width; ++x) {
    const auto pixel = transform.model_pixel((region.x + x + .5) / sx, (region.y + y + .5) / sy);
    mask.at<std::uint8_t>(y, x) = mask_contains(prototypes, det, pixel) ? 255 : 0;
  }
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, region.tl());
  for (auto & contour : contours) for (auto & p : contour) p = p * step + cv::Point(step / 2, step / 2);
  cv::drawContours(small, contours, -1, cv::Scalar(245, 217, 55), 2, cv::LINE_AA);
}
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
      const auto * stop_data = backend_->stop_line_mask_data();
      const cv::Mat stop_mask = stop_data ? cv::Mat(300, 120, CV_8UC1, const_cast<std::uint8_t *>(stop_data)) : cv::Mat();
      lane = post_->process(labels, frame->t, ctl_.cones_nearby(), stop_mask);
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
    publish_(ctl_.on_lane(lane, now_sec()));
  }
}

// ---------------------------------------------------------------- 검출
DetectWorker::DetectWorker(CameraPipeline & cam, ControlLoop & loop) : cam_(cam), ctl_(loop) {}

DetectWorker::~DetectWorker() { stop(); }

void DetectWorker::load_model(const ModelConfig & m, const std::function<void(const std::string &)> & progress) {
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (detector_) return;
  }
  auto det = std::make_unique<YoloxDetector>(m.yolo_model, "TENSORRT", m.yolo_engine_cache, m.yolo_precision, 512, 320, 0.0f, 0.65f,
                                             m.workspace_bytes, progress, 100);
  if (progress) progress("시험 추론");
  cv::Mat zeros(320, 512, CV_8UC3, cv::Scalar(114, 114, 114));
  det->detect(zeros);
  std::lock_guard<std::mutex> l(mutex_);
  detector_ = std::move(det);
  detector_->set_thresholds(s_.yolo_thresholds);
}

void DetectWorker::load_segmentation(const ModelConfig & m, const std::function<void(const std::string &)> & progress) {
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (segmentation_) return;
  }
  auto model = std::make_unique<TensorRtYoloxBackend>(m.seg_model, m.seg_engine_cache, "fp16", seg_width, seg_height,
                                                     m.workspace_bytes, progress, true);
  if (progress) progress("시험 추론");
  std::vector<std::uint8_t> nv12(seg_width * seg_height * 3 / 2, 128);
  std::vector<std::uint16_t> packed(seg_output_values);
  model->infer_seg_nv12(nv12.data(), nv12.size(), seg_width, seg_width, seg_height,
                       0, 0, seg_width, seg_height, packed.data(), packed.size());
  std::lock_guard<std::mutex> l(mutex_);
  segmentation_ = std::move(model);
  packed_segmentation_ = std::move(packed);
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
    if (!frame) continue;
    const auto model = ctl_.detector_model();
    const bool use_seg = model == DetectorModel::Yolo11Seg;
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
    const bool preview = camera_wanted && camera_wanted() && t0 - last_jpeg_t_ >= 0.1;
    YoloxDetectionResult res;
    std::vector<SegDetection> seg;
    SegTransform transform;
    const std::uint16_t * prototypes = nullptr;
    {
      std::lock_guard<std::mutex> l(mutex_);
      if (use_seg) {
        if (!segmentation_) continue;
        // The checkpoint was trained on full-frame RGB with centered letterboxing.
        transform = seg_transform({0, 0, w, h});
        const auto t = segmentation_->infer_seg_nv12(data.data(), data.size(), stride, w, h,
          0, 0, w, h, packed_segmentation_.data(), packed_segmentation_.size());
        const double post_start = now_sec();
        seg = decode_seg(packed_segmentation_.data(), transform, s.yolo_thresholds, float(s.nms_threshold));
        prototypes = packed_segmentation_.data() + seg_prediction_values;
        for (const auto & d : seg) res.detections.push_back({d.box, d.score, d.class_id});
        res.timing = {t.preprocessing_nanoseconds, t.input_transfer_nanoseconds, t.execution_nanoseconds,
                      t.output_transfer_nanoseconds, std::uint64_t((now_sec() - post_start) * 1e9)};
      } else {
        if (!detector_) continue;
        res = detector_->detect_nv12(data.data(), data.size(), stride, w, h, s.traffic.enabled ? cv::Rect(0, 0, w, h) : roi);
      }
    }
    // 콘·차량은 지금처럼 하단 ROI 안(박스 아래 끝 기준)만, 신호등은 점수가 가장 높은 것 하나의 색을 본다
    std::vector<Detection> obstacles;
    std::vector<std::size_t> seg_indices;
    const Detection * light = nullptr;
    for (const auto & det : res.detections) {
      if (det.class_id == 3) { if (!light || det.score > light->score) light = &det; continue; }
      if (det.box.y + det.box.height > roi.y) {
        obstacles.push_back(det);
        if (use_seg) seg_indices.push_back(&det - res.detections.data());
      }
    }
    if (s.traffic.enabled) {
      TrafficSignal sig = TrafficSignal::Unknown;
      if (light) {   // 이긴 박스만 BGR로 (traffic_light_node와 같음)
        const int l = std::clamp(int(std::floor(light->box.x)), 0, w) & ~1, t = std::clamp(int(std::floor(light->box.y)), 0, h) & ~1;
        const int r = std::min(w, (std::clamp(int(std::ceil(light->box.x + light->box.width)), 0, w) + 1) & ~1);
        const int b = std::min(h, (std::clamp(int(std::ceil(light->box.y + light->box.height)), 0, h) + 1) & ~1);
        if (r > l && b > t) {
          const cv::Mat yp(b - t, r - l, CV_8UC1, const_cast<std::uint8_t *>(data.data()) + t * stride + l, stride);
          const cv::Mat uv((b - t) / 2, (r - l) / 2, CV_8UC2, const_cast<std::uint8_t *>(data.data()) + h * stride + (t / 2) * stride + l, stride);
          cv::Mat bgr;
          cv::cvtColorTwoPlane(yp, uv, bgr, cv::COLOR_YUV2BGR_NV12);
          sig = classify_traffic_color(bgr, s.traffic_min_saturation, s.traffic_min_value);
        }
      }
      ctl_.on_traffic_signal(sig, frame->t, model);
    }
    const double t1 = now_sec();
    stats.record("yolo_pre", t0, t0 + res.timing.preprocessing_nanoseconds * 1e-9);
    stats.record("yolo_infer", t0, t0 + (res.timing.input_transfer_nanoseconds + res.timing.forward_nanoseconds + res.timing.output_transfer_nanoseconds) * 1e-9);
    stats.record("yolo_post", t0, t0 + res.timing.postprocessing_nanoseconds * 1e-9);
    DetectionFrame out;
    out.capture_time = frame->t;
    out.model = model;
    out.work.detect_ms = (t1 - t0) * 1000;
    // 추론하는 동안 도착한 깊이까지 포함해 다시 짝짓기. 끝내 없으면 이 프레임은 관측으로 치지 않는다
    // 스테레오 깊이는 같은 시각 RGB보다 늦게 나온다. 추론 뒤에도 아직 없으면 잠깐(최대 depth_wait_ms) 기다린다.
    // 짝을 못 지은 프레임은 관측으로 치지 않으므로, 여기서 놓치면 BEV에서 콘이 깜빡이고 늦게 나타난다
    auto depth = frame->depth ? frame->depth : cam_.nearest_depth(frame->t, cc.depth_sync_ms / 1000.0);
    for (double waited = now_sec(); !depth && running_ && now_sec() - waited < cc.depth_wait_ms / 1000.0;) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      depth = cam_.nearest_depth(frame->t, cc.depth_sync_ms / 1000.0);
    }
    out.depth_valid = depth != nullptr;
    const double depth_started = now_sec();
    std::vector<MaskDepthSample> samples;
    if (!depth && !obstacles.empty()) {
      out.work.no_depth = int(obstacles.size());
      out.observations_complete = false;
      stats.record("depth_miss", t1, t1);
    }
    if (depth) {
      auto & d = *depth;
      out.depth_time = std::chrono::duration<double>(d.getTimestamp(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();
      const int dw = int(d.getWidth()), dh = int(d.getHeight());
      const double sx = double(dw) / w, sy = double(dh) / h;
      const DepthIntrinsics k{cc.fx * sx, cc.fy * sy, cc.cx * sx, cc.cy * sy, dw, dh};
      const auto * mm = reinterpret_cast<const std::uint16_t *>(d.getData().data());
      const int stride_px = d.getStride() > 0 ? int(d.getStride() / 2) : dw;
      const auto depth_box = [&](const Detection & det) {
        return cv::Rect2f(det.box.x * sx, det.box.y * sy, det.box.width * sx, det.box.height * sy);
      };
      if (use_seg) {
        std::vector<MaskDepthTarget> targets;
        for (std::size_t i = 0; i < obstacles.size(); ++i) {
          const auto & det = obstacles[i];
          const auto * mask = &seg[seg_indices[i]];
          targets.push_back({depth_box(det), det.class_id, det.score, footprint_for(det.class_id, s),
            bottom_visible(det, obstacles), [&, mask](int x, int y) {
              return mask_contains(prototypes, *mask, transform.model_pixel((x + .5) / sx, (y + .5) / sy));
            }});
        }
        auto localized = localize_masks(mm, stride_px, k, mount, targets, s.box, preview);
        samples = std::move(localized.samples);
        out.work.candidates = localized.work.candidates;
        out.work.depth_reads = localized.work.reads;
        out.work.mask_ms = localized.mask_ms;
        for (const auto & o : localized.objects) {
          if (o.observation) { out.cones.push_back(*o.observation); continue; }
          out.observations_complete = false;
          if (o.status == MaskDepthStatus::BudgetExhausted) {
            ++out.work.budget_exhausted;
            stats.record("depth_budget", t1, t1);
          } else if (o.status == MaskDepthStatus::GroundRejected) {
            ++out.work.ground_rejected;
            stats.record("ground_reject", t1, t1);
          } else {
            ++out.work.insufficient;
            stats.record("loc_fail", t1, t1);
          }
        }
      } else {
        DepthWork work;
        for (const auto & det : obstacles) {
          GroundCheck gc = GroundCheck::Skipped;
          if (auto o = localize_box(mm, stride_px, k, mount, depth_box(det), det.class_id, det.score,
                                    footprint_for(det.class_id, s), s.box, bottom_visible(det, obstacles), &gc, &work)) {
            out.cones.push_back(*o);
          } else if (gc == GroundCheck::Rejected) {
            ++out.work.ground_rejected;
            stats.record("ground_reject", t1, t1);
          } else {
            ++out.work.insufficient;
            stats.record("loc_fail", t1, t1);
          }
        }
        out.work.candidates = work.candidates;
        out.work.depth_reads = work.reads;
      }
    }
    out.work.distance_ms = std::max(0.0, (now_sec() - depth_started) * 1000 - out.work.mask_ms);
    out.work.total_ms = out.work.detect_ms + out.work.mask_ms + out.work.distance_ms;
    if (use_seg) stats.record("mask_post", depth_started, depth_started + out.work.mask_ms * .001);
    stats.record("box_depth", depth_started, depth_started + out.work.distance_ms * .001);
    ctl_.on_detections(out);
    if (preview && ctl_.detector_model() == model) {
      const cv::Mat yp(h, w, CV_8UC1, const_cast<std::uint8_t *>(data.data()), stride);
      const cv::Mat uv(h / 2, w / 2, CV_8UC2, const_cast<std::uint8_t *>(data.data()) + h * stride, stride);
      cv::Mat bgr;
      cv::cvtColorTwoPlane(yp, uv, bgr, cv::COLOR_YUV2BGR_NV12);
      cv::Mat small;
      cv::resize(bgr, small, {640, 400}, 0, 0, cv::INTER_AREA);
      const float px = float(small.cols) / w, py = float(small.rows) / h;
      cv::line(small, {0, int(roi.y * py)}, {small.cols, int(roi.y * py)}, {0, 220, 255}, 1);
      if (use_seg) for (const auto & d : seg) draw_mask_outline(small, d, transform, prototypes, w, h);
      for (const auto & det : res.detections) {
        const cv::Rect2f box(det.box.x * px, det.box.y * py, det.box.width * px, det.box.height * py);
        if (!use_seg) cv::rectangle(small, box, {60, 60, 255}, 1);
        char label[48];
        std::snprintf(label, sizeof(label), "%s %.2f", kClassNames[std::clamp(det.class_id, 0, 3)], det.score);
        cv::putText(small, label, {int(box.x), std::max(10, int(box.y) - 3)}, cv::FONT_HERSHEY_SIMPLEX, 0.37, {255, 255, 255}, 1);
      }
      if (depth) for (const auto & sample : samples) {
        const cv::Point p(int((sample.pixel.x + .5) * small.cols / depth->getWidth()),
                          int((sample.pixel.y + .5) * small.rows / depth->getHeight()));
        cv::circle(small, p, 2, !sample.in_mask ? cv::Scalar(227, 226, 226) :
          sample.valid_depth ? cv::Scalar(253, 231, 59) : cv::Scalar(42, 175, 253), -1, cv::LINE_AA);
      }
      std::vector<std::uint8_t> jpeg;
      if (cv::imencode(".jpg", small, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70}) && publish_camera_jpeg) publish_camera_jpeg(std::move(jpeg));
      last_jpeg_t_ = t0;
    }
  }
}

}  // namespace cone_drive_test
