// cone_drive_test/src/runtime/perception.cpp의 DetectWorker를 auto_control 입력(직접 전달 NV12, CAM_A 정렬 깊이)에 맞춤
#include "auto_control/drive/runtime/detector.hpp"
#include "auto_control/drive/core/rate_gate.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace auto_control::drive {

namespace {
const char * kClassNames[] = {"con", "truck", "sedan", "traffic_light"};

// 박스 아래 끝이 보이나: 다른 박스(더 가까운 물체)가 아래 끝 줄을 가로로 겹쳐 덮고 있으면 가려진 것으로 본다
bool bottom_visible(const Detection & d, const std::vector<Detection> & all) {
  const float bottom = d.box.y + d.box.height;
  for (const auto & o : all) {
    if (&o == &d) continue;
    const float ob = o.box.y + o.box.height;
    const bool covers_row = o.box.y < bottom && ob > bottom;
    const bool overlaps_x = std::min(d.box.x + d.box.width, o.box.x + o.box.width) > std::max(d.box.x, o.box.x);
    if (covers_row && overlaps_x) return false;
  }
  return true;
}
}  // namespace

cv::Rect2f rgb_box_to_depth(const cv::Rect2f & box, const RgbIntrinsics & rgb, const DepthImage & depth) {
  auto map_x = [&](double u) { return float((u - rgb.cx) / rgb.fx * depth.fx + depth.cx); };
  auto map_y = [&](double v) { return float((v - rgb.cy) / rgb.fy * depth.fy + depth.cy); };
  const float x0 = map_x(box.x), x1 = map_x(box.x + box.width);
  const float y0 = map_y(box.y), y1 = map_y(box.y + box.height);
  return {x0, y0, x1 - x0, y1 - y0};
}

DetectWorker::DetectWorker(ControlLoop & loop) : ctl_(loop) {}

DetectWorker::~DetectWorker() { stop(); }

void DetectWorker::load_model(const ModelConfig & m, const std::function<void(const std::string &)> & progress) {
  auto det = std::make_unique<YoloxDetector>(m.yolo_model, m.yolo_backend, m.yolo_engine_cache, m.yolo_precision, 512, 320, 0.0f, 0.65f,
                                             m.workspace_bytes, progress, 100);
  if (progress) progress("시험 추론");
  cv::Mat zeros(320, 512, CV_8UC3, cv::Scalar(114, 114, 114));
  det->detect(zeros);
  std::lock_guard<std::mutex> l(mutex_);
  detector_ = std::move(det);
  detector_->set_thresholds(s_.yolo_thresholds);
}

bool DetectWorker::ready() const { std::lock_guard<std::mutex> l(mutex_); return detector_ != nullptr; }

void DetectWorker::set_settings(const Settings & s) {
  std::lock_guard<std::mutex> l(mutex_);
  s_ = s;
  if (detector_) detector_->set_thresholds(s.yolo_thresholds);
}

void DetectWorker::set_mount(const CameraMount & m, const RgbIntrinsics & k) {
  std::lock_guard<std::mutex> l(mutex_);
  mount_ = m;
  rgb_k_ = k;
  mount_ok_ = k.fx > 0 && k.fy > 0;
}

void DetectWorker::push_frame(std::shared_ptr<const Nv12Frame> f) {
  {
    std::lock_guard<std::mutex> l(frame_m_);
    if (now_sec() - last_device_t_ < 0.5) {   // GPU 원본이 오는 중: 짝지을 수 있게 남겨만 둔다
      host_recent_.push_back(std::move(f));
      while (host_recent_.size() > 4) host_recent_.pop_front();
      return;
    }
    pending_ = std::move(f);
  }
  frame_cv_.notify_one();
}

void DetectWorker::push_device_frame(std::shared_ptr<const Nv12Frame> f) {
  auto combined = std::make_shared<Nv12Frame>(*f);
  {
    std::lock_guard<std::mutex> l(frame_m_);
    last_device_t_ = now_sec();
    for (const auto & h : host_recent_)   // 같은 촬영 시각의 호스트 프레임 (신호등 색·카메라 보기용)
      if (std::abs(h->t - f->t) < 1e-4) {
        combined->owner = h->owner; combined->nv12 = h->nv12; combined->size = h->size; combined->stride = h->stride;
        break;
      }
    pending_ = std::move(combined);
  }
  frame_cv_.notify_one();
}

void DetectWorker::push_depth(std::shared_ptr<const DepthImage> d) {
  std::lock_guard<std::mutex> l(depth_m_);
  depths_.push_back(std::move(d));
  while (depths_.size() > 8) depths_.pop_front();
}

std::shared_ptr<const DepthImage> DetectWorker::nearest_depth(double t, double max_dt) const {
  std::lock_guard<std::mutex> l(depth_m_);
  std::shared_ptr<const DepthImage> best;
  double best_dt = max_dt;
  for (const auto & d : depths_) {
    const double dt = std::abs(d->t - t);
    if (dt <= best_dt) { best_dt = dt; best = d; }
  }
  return best;
}

void DetectWorker::start() {
  if (running_.exchange(true)) return;
  thread_ = std::thread([this] { loop_(); });
  jpeg_thread_ = std::thread([this] { jpeg_loop_(); });
}

void DetectWorker::stop() {
  if (!running_.exchange(false)) return;
  frame_cv_.notify_all();
  jpeg_cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  if (jpeg_thread_.joinable()) jpeg_thread_.join();
}

void DetectWorker::loop_() {
  RateGate gate(40.0);   // 카메라(70 fps)보다 낮은 검출 FPS를 평균으로 맞춤
  auto & stats = ctl_.stats();
  double fps_t0 = now_sec();
  int fps_n = 0;
  while (running_) {
    std::shared_ptr<const Nv12Frame> frame;
    {
      std::unique_lock<std::mutex> l(frame_m_);
      frame_cv_.wait_for(l, std::chrono::milliseconds(100), [this] { return pending_ != nullptr || !running_; });
      frame = std::move(pending_);
      pending_.reset();
    }
    const double tn = now_sec();
    if (tn - fps_t0 >= 1.0) { fps_ = fps_n / (tn - fps_t0); fps_n = 0; fps_t0 = tn; }
    if (!frame || paused_) continue;
    Settings s;
    CameraMount mount;
    RgbIntrinsics rgb_k;
    bool mount_ok;
    {
      std::lock_guard<std::mutex> l(mutex_);
      if (!detector_) continue;
      s = s_; mount = mount_; rgb_k = rgb_k_; mount_ok = mount_ok_;
    }
    gate.set_rate(s.yolo_max_fps);
    if (!gate.accept(frame->t)) continue;
    const double t0 = now_sec();
    const int w = frame->width, h = frame->height;
    const int roi_h = std::clamp(int(std::lround(h * s.roi_height_percent / 100.0)), 1, h);
    const cv::Rect roi(0, h - roi_h, w, roi_h);
    const std::uint8_t * data = frame->nv12;
    const std::size_t stride = frame->stride;
    YoloxDetectionResult res;
    {
      std::lock_guard<std::mutex> l(mutex_);
      // 신호등을 켜면 화면 전체를 추론한다. 모델 입력(512×320)은 가로 기준 축소라 축소 비율·연산량은 그대로 (위쪽이 패딩 대신 영상)
      const cv::Rect infer_roi = s.traffic.enabled ? cv::Rect(0, 0, w, h) : roi;
      if (frame->device && detector_->supports_nv12_input()) {   // GPU 원본 그대로 (업로드 없음)
        res = detector_->detect_nv12_device(frame->device, frame->device_pitch, w, h, infer_roi);
      } else if (!data) {
        continue;   // 호스트 영상도 없음 (CPU 백엔드인데 GPU 프레임만 옴)
      } else if (detector_->supports_nv12_input()) {
        res = detector_->detect_nv12(data, frame->size, stride, w, h, infer_roi);
      } else {   // CPU 백엔드: NV12 → BGR 변환 후 추론
        const cv::Mat nv12(h * 3 / 2, w, CV_8UC1, const_cast<std::uint8_t *>(data), stride);
        cv::Mat bgr;
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        res = detector_->detect(bgr, infer_roi);
      }
    }
    ++fps_n;
    // 콘·차량은 하단 ROI 안(박스 아래 끝 기준)만, 신호등은 점수가 가장 높은 것 하나의 색을 본다
    std::vector<Detection> obstacles;
    const Detection * light = nullptr;
    for (const auto & det : res.detections) {
      if (det.class_id == 3) {   // 내 신호등: 크고(가깝고) 화면 가로 가운데에 가까운 것
        auto weight = [w](const Detection & d) {
          const double center = std::abs(d.box.x + d.box.width / 2.0 - w / 2.0) / (w / 2.0);
          return double(d.box.area()) * std::max(0.1, 1.0 - center);
        };
        if (!light || weight(det) > weight(*light)) light = &det;
        continue;
      }
      if (det.box.y + det.box.height > roi.y) obstacles.push_back(det);
    }
    if (s.traffic.enabled) {
      TrafficSignal sig = TrafficSignal::Unknown;
      float score = 0.f;
      if (light && data) {   // 이긴 박스만 BGR로 (traffic_light_node와 같음)
        score = light->score;
        const int l = std::clamp(int(std::floor(light->box.x)), 0, w) & ~1, t = std::clamp(int(std::floor(light->box.y)), 0, h) & ~1;
        const int r = std::min(w, (std::clamp(int(std::ceil(light->box.x + light->box.width)), 0, w) + 1) & ~1);
        const int b = std::min(h, (std::clamp(int(std::ceil(light->box.y + light->box.height)), 0, h) + 1) & ~1);
        if (r > l && b > t) {
          const cv::Mat yp(b - t, r - l, CV_8UC1, const_cast<std::uint8_t *>(data) + t * stride + l, stride);
          const cv::Mat uv((b - t) / 2, (r - l) / 2, CV_8UC2, const_cast<std::uint8_t *>(data) + h * stride + (t / 2) * stride + l, stride);
          cv::Mat bgr;
          cv::cvtColorTwoPlane(yp, uv, bgr, cv::COLOR_YUV2BGR_NV12);
          sig = classify_traffic_color(bgr, s.traffic_min_saturation, s.traffic_min_value);
        }
      }
      signal_score_ = score;
      ctl_.on_traffic_signal(sig, frame->t);
    }
    const double t1 = now_sec();
    stats.record("yolo_pre", t0, t0 + res.timing.preprocessing_nanoseconds * 1e-9);
    stats.record("yolo_infer", t0, t0 + (res.timing.input_transfer_nanoseconds + res.timing.forward_nanoseconds + res.timing.output_transfer_nanoseconds) * 1e-9);
    stats.record("yolo_post", t0, t0 + res.timing.postprocessing_nanoseconds * 1e-9);
    DetectionFrame out;
    out.capture_time = frame->t;
    // 스테레오 깊이는 같은 시각 RGB보다 늦게 나온다. 아직 없으면 잠깐(최대 depth_wait) 기다린다.
    // 짝을 못 지은 프레임은 관측으로 치지 않는다 (놓침으로 세지 않음)
    auto depth = nearest_depth(frame->t, depth_sync_s_);
    for (double waited = now_sec(); !depth && running_ && now_sec() - waited < depth_wait_s_;) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      depth = nearest_depth(frame->t, depth_sync_s_);
    }
    const double t_loc = now_sec();
    stats.record("depth_wait", t1, t_loc);   // 깊이 짝을 기다린 시간 (대부분 0, 깊이가 늦으면 최대 depth_wait)
    // 장애물 박스 수 (개/프레임): ms 칸에 개수를 담는다. 박스가 많을수록 위치 계산 연산이 늘어난다
    stats.record("yolo_boxes", t1, t1 + double(obstacles.size()) * 1e-3);
    out.depth_valid = depth != nullptr && mount_ok;
    if (!depth && !obstacles.empty()) stats.record("depth_miss", t1, t1);   // 진단: 박스는 있는데 깊이 짝이 없음 (회/s)
    if (depth && mount_ok) {
      const auto & d = *depth;
      if (!geometry_logged_) {   // 위치 계산 실패 진단용: RGB·깊이 내부 파라미터와 박스 변환 한 번 출력
        geometry_logged_ = true;
        std::fprintf(stderr, "[auto_drive detector] rgb %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f | depth %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f stride=%zu "
                     "| mount x=%.3f h=%.3f pitch=%.1fdeg\n", w, h, rgb_k.fx, rgb_k.fy, rgb_k.cx, rgb_k.cy, d.width, d.height, d.fx, d.fy,
                     d.cx, d.cy, d.stride, mount.x, mount.height, mount.pitch_down_rad * 57.2958);
      }
      out.depth_time = d.t;
      const DepthIntrinsics k{d.fx, d.fy, d.cx, d.cy, d.width, d.height};
      const auto * mm = reinterpret_cast<const std::uint16_t *>(d.data);
      const int stride_px = d.stride > 0 ? int(d.stride / 2) : d.width;
      for (const auto & det : obstacles) {
        const Footprint fp = det.class_id == 0 ? Footprint{s.cone_size_m, s.cone_size_m}
                           : det.class_id == 1 ? Footprint{0.60, 0.36} : det.class_id == 2 ? Footprint{0.50, 0.30} : Footprint{};
        const cv::Rect2f box = rgb_box_to_depth(det.box, rgb_k, d);
        GroundCheck gc = GroundCheck::Skipped;
        if (auto o = localize_box(mm, stride_px, k, mount, box, det.class_id, det.score, fp, s.box, bottom_visible(det, obstacles), &gc))
          out.cones.push_back(*o);
        else if (gc == GroundCheck::Rejected) stats.record("ground_reject", t1, t1);
        else {
          stats.record("loc_fail", t1, t1);   // 진단: 깊이는 있는데 박스 안 유효 점이 모자람
          if (loc_fail_logged_ < 3) {   // 처음 몇 번은 RGB 박스 → 깊이 박스를 남긴다
            ++loc_fail_logged_;
            std::fprintf(stderr, "[auto_drive detector] localize failed: class=%d rgb box=(%.0f,%.0f %.0fx%.0f) -> depth box=(%.0f,%.0f %.0fx%.0f)\n",
                         det.class_id, det.box.x, det.box.y, det.box.width, det.box.height, box.x, box.y, box.width, box.height);
          }
        }
      }
    }
    stats.record("box_depth", t_loc, now_sec());   // 박스 안 깊이로 위치 계산 (깊이 대기 제외)
    stats.record("yolo_total", t0, now_sec());     // 검출 1회 전체 (추론·신호등 색·깊이 대기·위치 계산)
    ctl_.on_detections(out);
    if (data && camera_wanted && camera_wanted() && t0 - last_jpeg_t_ >= 0.1) {   // 검출 확인용 카메라 보기 (켰을 때만, 10 fps)
      auto view = std::make_unique<CameraView>();   // 프레임 포인터와 박스만 넘기고 바로 다음 검출로
      view->frame = frame;
      view->detections = res.detections;
      view->roi_y = roi.y;
      {
        std::lock_guard<std::mutex> l(jpeg_m_);
        jpeg_pending_ = std::move(view);   // 인코딩이 밀리면 이전 것은 버림
      }
      jpeg_cv_.notify_one();
      last_jpeg_t_ = t0;
    }
  }
}

void DetectWorker::jpeg_loop_() {
  while (running_) {
    std::unique_ptr<CameraView> v;
    {
      std::unique_lock<std::mutex> l(jpeg_m_);
      jpeg_cv_.wait_for(l, std::chrono::milliseconds(200), [this] { return jpeg_pending_ != nullptr || !running_; });
      v = std::move(jpeg_pending_);
    }
    if (!v) continue;
    const auto & f = *v->frame;
    // 원본(1280x800)을 통째로 BGR로 바꾼 뒤 줄이지 않고, Y·UV 평면을 먼저 640x400·320x200으로 줄여 작은 영상만 변환한다
    // (단일 스레드 1.4 ms → 0.3 ms, 화질 차이 PSNR 약 37 dB). 박스·ROI 선은 줄인 좌표로 그린다
    constexpr int kViewW = 640, kViewH = 400;
    const cv::Mat y(f.height, f.width, CV_8UC1, const_cast<std::uint8_t *>(f.nv12), f.stride);
    const cv::Mat uv(f.height / 2, f.width / 2, CV_8UC2, const_cast<std::uint8_t *>(f.nv12) + f.height * f.stride, f.stride);
    cv::Mat y_small, uv_small, small;
    cv::resize(y, y_small, {kViewW, kViewH}, 0, 0, cv::INTER_AREA);
    cv::resize(uv, uv_small, {kViewW / 2, kViewH / 2}, 0, 0, cv::INTER_NEAREST);   // 2채널 AREA 축소는 느림, 색은 저해상도라 차이 작음
    const double sx = double(kViewW) / f.width, sy = double(kViewH) / f.height;
    v->frame.reset();   // 줄인 복사본을 만들었으면 카메라 버퍼는 바로 돌려준다
    cv::cvtColorTwoPlane(y_small, uv_small, small, cv::COLOR_YUV2BGR_NV12);
    const int roi_y = int(std::lround(v->roi_y * sy));
    cv::line(small, {0, roi_y}, {kViewW, roi_y}, {0, 220, 255}, 1);
    for (const auto & det : v->detections) {
      const cv::Rect2f box(det.box.x * float(sx), det.box.y * float(sy), det.box.width * float(sx), det.box.height * float(sy));
      cv::rectangle(small, box, {60, 60, 255}, 1);
      char label[48];
      std::snprintf(label, sizeof(label), "%s %.2f", kClassNames[std::clamp(det.class_id, 0, 3)], det.score);
      cv::putText(small, label, box.tl() + cv::Point2f(0, -3), cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 255, 255}, 1);
    }
    std::vector<std::uint8_t> jpeg;
    if (cv::imencode(".jpg", small, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70}) && publish_camera_jpeg) publish_camera_jpeg(std::move(jpeg));
  }
}

}  // namespace auto_control::drive
