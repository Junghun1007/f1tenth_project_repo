#include "multiclass_yolo/yolox_detector.hpp"
#include "multiclass_yolo/preview_exit.hpp"
#include "preview_panel.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace multiclass_yolo {
class Preview : public rclcpp::Node {
 public:
  Preview() : Node("multiclass_yolo") {
    const auto share = ament_index_cpp::get_package_share_directory("multiclass_yolo");
    auto model = declare_parameter<std::string>("model_path", share + "/models/multiclass_yolox_tiny_512x320_holdout10.onnx");
    auto backend = declare_parameter<std::string>("inference_backend", "TENSORRT");
    auto precision = declare_parameter<std::string>("engine_precision", "fp16");
    // User cache is writable even when ROS install/share is read-only.
    const char * home = std::getenv("HOME");
    auto cache = declare_parameter<std::string>("engine_cache_path", "");
    if (!std::filesystem::is_regular_file(model)) throw std::runtime_error("Model missing: " + model);
    if (cache.empty()) {
      const auto modified = std::filesystem::last_write_time(model).time_since_epoch().count();
      const auto identity = std::filesystem::canonical(model).string() +
        std::to_string(std::filesystem::file_size(model)) +
        std::to_string(static_cast<long long>(modified));
      cache = std::string(home ? home : "/tmp") + "/.cache/multiclass_yolo/" +
        std::to_string(std::hash<std::string>{}(identity)) + "." + precision + ".engine";
    }
    format_ = declare_parameter<std::string>("input_format", "nv12");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 800);
    fps_ = declare_parameter<double>("sensor_fps", 30.0);
    roi_height_percent_ = declare_parameter<int>("roi_height_percent", 65);
    if (roi_height_percent_ < 1 || roi_height_percent_ > 100)
      throw std::invalid_argument("roi_height_percent must be in [1, 100]");
    undistort_ = declare_parameter<bool>("undistort_enabled", true);
    timeout_ = declare_parameter<double>("frame_timeout_sec", 5.0);
    auto workspace = declare_parameter<int>("tensorrt_workspace_size_mb", 1024);
    double nms = declare_parameter<double>("nms_threshold", 0.65);
    if ((format_ != "rgb" && format_ != "nv12") || width_ <= 0 || height_ <= 0 ||
        width_ % 2 || height_ % 2 || !std::isfinite(fps_) || fps_ <= 0 ||
        !std::isfinite(timeout_) || timeout_ <= 0 || workspace < 1 || workspace > 16384)
      throw std::invalid_argument("Invalid input_format/dimensions/FPS/timeout/workspace");
    for (int i=0; i<4; ++i) {
      double v = declare_parameter<double>(std::string(names_[i])+"_threshold", 0.25);
      if (!std::isfinite(v) || v < 0 || v > 1) throw std::invalid_argument("Thresholds must be in [0,1]");
      thresholds_[i] = static_cast<float>(v);
    }
    if (!std::filesystem::is_regular_file(model)) throw std::runtime_error("Model missing: " + model);
    detector_ = std::make_unique<YoloxDetector>(model, backend, cache, precision,
      512, 320, 0.0f, static_cast<float>(nms), std::size_t(workspace)*1024*1024,
      [this](const std::string & s) {RCLCPP_INFO(get_logger(), "%s", s.c_str());});
    if (format_ == "nv12" && !detector_->supports_nv12_input())
      throw std::invalid_argument("input_format=nv12 requires TensorRT; CPU: use input_format=rgb engine_precision=fp32");
    auto device = std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
    camera_id_ = device->getDeviceId();
    for (const auto & camera : device->getConnectedCameraFeatures()) {
      if (camera.socket != dai::CameraBoardSocket::CAM_A) continue;
      for (const auto & config : camera.configs) {
        if (config.width == sensor_width_ && config.height == sensor_height_) {
          min_fps_ = config.minFps;
          max_fps_ = config.maxFps;
          break;
        }
      }
      break;
    }
    start_camera(fps_, device);
    last_frame_ = Clock::now();
    capture_ = std::thread([this] { capture_loop(); });
  }
  ~Preview() override {
    stop_ = true;
    if (capture_.joinable()) capture_.join();
    try { close_camera(); } catch (...) {}
  }
  void run() {
    const std::string window = "multiclass_yolo";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    PreviewPanel panel;
    cv::setMouseCallback(window, [](int event, int x, int y, int, void * data) {
      if (event == cv::EVENT_LBUTTONDOWN) static_cast<PreviewPanel *>(data)->click(x, y);
    }, &panel);
    constexpr int video_width = static_cast<int>(1024 * 1.1 + .5);
    constexpr int video_top = 70;
    const int video_height = std::max(1, static_cast<int>(std::lround(
      static_cast<double>(video_width) * height_ / width_)));
    const int video_left = PreviewPanel::width;
    cv::Mat canvas(std::max(PreviewPanel::min_height, video_top + video_height),
      video_left + video_width, CV_8UC3, cv::Scalar(20,20,20));
    cv::Mat video = canvas(cv::Rect(video_left, video_top, video_width, video_height));
    cv::resizeWindow(window, canvas.cols, canvas.rows);
    cv::putText(video, "Waiting for camera frames...", {20, video_height / 2},
      cv::FONT_HERSHEY_SIMPLEX, .7, {240,240,240}, 1, cv::LINE_AA);
    std::vector<Detection> detections;
    auto detection_thresholds = thresholds_;
    PreviewExit exit_state;
    std::string exit_reason = "ROS shutdown / stop requested";
    const auto property = [&window](int id) {
      try { return cv::getWindowProperty(window, id); }
      catch (const cv::Exception &) { return -1.; }
    };
    std::uint64_t previous = 0, outputs = 0, skipped = 0, session = 0;
    double displayed_fps = 0;
    std::string displayed_status;
    auto last_log = Clock::now();
    while (rclcpp::ok() && !stop_) {
      rclcpp::spin_some(get_node_base_interface());
      std::shared_ptr<dai::ImgFrame> frame;
      Clock::time_point received, first;
      std::uint64_t count;
      double fps;
      bool changing_fps;
      std::string camera_status;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!error_.empty()) throw std::runtime_error(error_);
        if (session != camera_session_) {
          session = camera_session_;
          previous = outputs = skipped = 0;
          last_log = Clock::now();
        }
        count = received_; first = first_frame_; received = last_frame_;
        fps = fps_;
        changing_fps = requested_fps_.has_value();
        camera_status = fps_status_;
        if (count != previous) frame = latest_;
      }
      if (!changing_fps && std::chrono::duration<double>(Clock::now()-received).count() > timeout_)
        throw std::runtime_error("No camera frame within frame_timeout_sec; check OAK/USB and other camera processes");
      if (frame) {
        if (previous == 0) RCLCPP_INFO(get_logger(), "First camera frame received; starting inference");
        skipped += count > previous ? count - previous - 1 : 0;
        previous = count;
        detector_->set_thresholds(thresholds_);
        const auto began = Clock::now();
        const int roi_percent = roi_height_percent_;
        const int roi_height = std::max(1, static_cast<int>(std::lround(height_ * (roi_percent / 100.0))));
        const cv::Rect roi(0, height_ - roi_height, width_, roi_height);
        const auto & bytes = frame->getData();
        const std::size_t stride = frame->getStride() > 0 ? frame->getStride() : width_;
        if (stride < std::size_t(width_) || bytes.size() < stride*std::size_t(height_)*3/2)
          throw std::runtime_error("Invalid NV12 buffer/pitch");
        YoloxDetectionResult result;
        cv::Mat display;
        const auto conversion_started = Clock::now();
        double conversion_ms = 0;
        if (format_ == "nv12") {
          result = detector_->detect_nv12(bytes.data(), bytes.size(), stride, width_, height_, roi);
          const auto t = Clock::now();
          cv::Mat y(height_, width_, CV_8UC1, const_cast<std::uint8_t *>(bytes.data()), stride);
          cv::Mat uv(height_/2, width_/2, CV_8UC2, const_cast<std::uint8_t *>(bytes.data())+stride*height_, stride);
          cv::cvtColorTwoPlane(y, uv, display, cv::COLOR_YUV2BGR_NV12);
          conversion_ms = std::chrono::duration<double,std::milli>(Clock::now()-t).count();
        } else {
          cv::Mat y(height_, width_, CV_8UC1, const_cast<std::uint8_t *>(bytes.data()), stride);
          cv::Mat uv(height_/2, width_/2, CV_8UC2, const_cast<std::uint8_t *>(bytes.data())+stride*height_, stride);
          cv::Mat rgb;
          cv::cvtColorTwoPlane(y, uv, rgb, cv::COLOR_YUV2RGB_NV12);
          conversion_ms = std::chrono::duration<double,std::milli>(Clock::now()-conversion_started).count();
          // Model metadata requires BGR: blobFromImage swaps this RGB buffer to BGR.
          result = detector_->detect(rgb, roi, true);
          cv::cvtColor(rgb, display, cv::COLOR_RGB2BGR);
        }
        if (roi.y > 0)
          cv::line(display, {0, roi.y}, {width_ - 1, roi.y}, {0, 220, 255}, 1, cv::LINE_AA);
        detector_->draw(display, result.detections);
        const double total_ms = std::chrono::duration<double,std::milli>(Clock::now()-began).count();
        // Common epoch: first host receipt, engine build and camera startup excluded.
        const double elapsed = std::chrono::duration<double>(Clock::now()-first).count();
        std::uint64_t current_received;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (session != camera_session_) continue;
          current_received = received_;
        }
        const double in_fps = elapsed > 0.5 ? double(current_received-1)/elapsed : 0;
        const double out_fps = elapsed > 0.5 ? double(outputs)/elapsed : 0;
        cv::resize(display, video, video.size(), 0, 0, cv::INTER_LINEAR);
        canvas(cv::Rect(video_left, 0, video_width, video_top)).setTo(cv::Scalar(20,20,20));
        cv::putText(canvas, cv::format("AVG input %.2f FPS | AVG output %.2f FPS | target %.6g", in_fps, out_fps, fps),
          {video_left + 10, 25}, cv::FONT_HERSHEY_SIMPLEX, .65, {255,255,255}, 1, cv::LINE_AA);
        cv::putText(canvas, cv::format("%s | total %.1f ms | convert %.1f ms | skipped %llu | Q/ESC quit",
          format_.c_str(), total_ms, conversion_ms, static_cast<unsigned long long>(skipped)),
          {video_left + 10, 53}, cv::FONT_HERSHEY_SIMPLEX, .5, {200,220,200}, 1, cv::LINE_AA);
        detections = std::move(result.detections);
        detection_thresholds = thresholds_;
        ++outputs;
        if (Clock::now()-last_log >= 2s) {
          RCLCPP_INFO(get_logger(),
            "AVG input %.2f / output %.2f FPS; ROI %d%%; total %.2f ms; upload %.2f ms; preprocess %.2f ms; execute %.2f ms; skipped %llu",
            in_fps, out_fps, roi_percent, total_ms, result.timing.input_transfer_nanoseconds/1e6,
            result.timing.preprocessing_nanoseconds/1e6, result.timing.forward_nanoseconds/1e6,
            static_cast<unsigned long long>(skipped));
          last_log = Clock::now();
        }
      }
      if (frame || panel.dirty || fps != displayed_fps || camera_status != displayed_status) {
        panel.draw(canvas, fps, roi_height_percent_, names_, thresholds_,
          detection_thresholds, detections, camera_status);
        cv::imshow(window, canvas);
        if (frame && outputs == 1) RCLCPP_INFO(get_logger(), "First detection frame submitted to preview");
        panel.dirty = false;
        displayed_fps = fps;
        displayed_status = camera_status;
      }
      int key = cv::waitKey(1) & 255;
      if (panel.handle_key(key)) key = -1;
      if (const auto edit = panel.take_submission()) panel.finish(apply_preview_input(*edit));
      const auto reason = exit_state.reason(key, property(cv::WND_PROP_VISIBLE),
        property(cv::WND_PROP_AUTOSIZE));
      if (reason) {exit_reason = reason; break;}
      if (!frame) std::this_thread::sleep_for(1ms);
    }
    RCLCPP_INFO(get_logger(), "Preview stopped: %s", exit_reason.c_str());
    cv::destroyAllWindows();
  }
 private:
  void start_camera(double fps, const std::shared_ptr<dai::Device> & device) {
    auto pipeline = std::make_unique<dai::Pipeline>(device);
    pipeline->setXLinkChunkSize(0);
    auto camera = pipeline->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_A, std::make_pair(std::uint32_t(sensor_width_), std::uint32_t(sensor_height_)), float(fps));
    auto output = camera->requestOutput(std::make_pair(std::uint32_t(width_), std::uint32_t(height_)),
      dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, float(fps), undistort_);
    auto queue = output->createOutputQueue(1U, false);
    pipeline->build();
    auto bridge = output->getXLinkBridge();
    if (!bridge || !bridge->xLinkOut) throw std::runtime_error("Missing XLink bridge");
    bridge->xLinkOut->input.setMaxSize(1);
    bridge->xLinkOut->input.setBlocking(false);
    pipeline->start();
    pipeline_ = std::move(pipeline);
    queue_ = std::move(queue);
    RCLCPP_INFO(get_logger(), "CAM_A NV12 %dx%d @ %.6g; path=%s; tensor=BGR FP32 [1,3,320,512]",
      width_, height_, fps, format_.c_str());
    if (device->getUsbSpeed() == dai::UsbSpeed::HIGH)
      RCLCPP_WARN(get_logger(), "USB2 connection detected; measure actual FPS on this connection");
  }

  void close_camera() {
    if (pipeline_) {
      pipeline_->stop();
      pipeline_->wait();
      queue_.reset();
      pipeline_.reset();
    }
  }

  std::string apply_preview_input(const PreviewPanel::Edit & edit) {
    if (edit.field == PreviewPanel::fps_field) return request_camera_fps(edit.value);
    if (edit.field == PreviewPanel::roi_field) {
      if (!edit.value || *edit.value < 1 || *edit.value > 100 ||
          std::trunc(*edit.value) != *edit.value)
        return "Enter a whole ROI percent from 1 to 100";
      roi_height_percent_ = static_cast<int>(*edit.value);
      return {};
    }
    if (!edit.value || *edit.value < 0 || *edit.value > 1)
      return "Enter a threshold from 0 to 1";
    thresholds_[edit.field - PreviewPanel::first_threshold_field] = static_cast<float>(*edit.value);
    return {};
  }

  std::string request_camera_fps(std::optional<double> fps) {
    if (max_fps_ <= 0) return "FPS range unavailable";
    if (!fps || *fps <= 0 || *fps < min_fps_ || *fps > max_fps_)
      return cv::format("Enter %.6g to %.6g FPS", std::max(0.0, min_fps_), max_fps_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (requested_fps_) return "Camera FPS change in progress";
    if (*fps != fps_) {
      requested_fps_ = fps;
      fps_status_ = cv::format("Changing to %.6g FPS...", *fps);
    }
    return {};
  }

  std::shared_ptr<dai::ImgFrame> read_camera_frame() {
    if (!pipeline_->isRunning()) throw std::runtime_error("OAK pipeline stopped");
    auto packet = queue_->tryGet<dai::ImgFrame>();
    if (packet && (packet->getType() != dai::ImgFrame::Type::NV12 ||
      int(packet->getWidth()) != width_ || int(packet->getHeight()) != height_))
      throw std::runtime_error("Unexpected OAK frame type or dimensions");
    return packet;
  }

  std::shared_ptr<dai::ImgFrame> reopen_camera(double fps) {
    close_camera();
    start_camera(fps, std::make_shared<dai::Device>(dai::DeviceInfo(camera_id_), dai::UsbSpeed::SUPER));
    const auto started = Clock::now();
    while (!stop_ && rclcpp::ok()) {
      if (auto packet = read_camera_frame()) return packet;
      if (std::chrono::duration<double>(Clock::now() - started).count() > timeout_)
        throw std::runtime_error("No camera frames at the requested FPS");
      std::this_thread::sleep_for(1ms);
    }
    return nullptr;
  }

  void change_camera_fps(double requested, double previous) {
    auto applied = requested;
    std::string status;
    std::shared_ptr<dai::ImgFrame> packet;
    try {
      packet = reopen_camera(requested);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Cannot stream at %.6g FPS: %s; restoring %.6g FPS",
        requested, e.what(), previous);
      if (stop_ || !rclcpp::ok()) return;
      applied = previous;
      status = "Failed. Previous FPS restored.";
      packet = reopen_camera(previous);
    }
    if (!packet) return;
    std::lock_guard<std::mutex> lock(mutex_);
    fps_ = applied;
    fps_status_ = std::move(status);
    requested_fps_.reset();
    last_frame_ = first_frame_ = Clock::now();
    latest_ = std::move(packet);
    received_ = 1;
    ++camera_session_;
  }

  void capture_loop() {
    try {
      while (!stop_ && rclcpp::ok()) {
        std::optional<double> requested;
        double fps;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          requested = requested_fps_;
          fps = fps_;
        }
        if (requested) {
          change_camera_fps(*requested, fps);
          continue;
        }
        auto packet = read_camera_frame();
        if (!packet) {std::this_thread::sleep_for(100us); continue;}
        std::lock_guard<std::mutex> lock(mutex_);
        last_frame_ = Clock::now();
        if (received_ == 0) first_frame_ = last_frame_;
        latest_ = std::move(packet); ++received_;
      }
    } catch (const std::exception & e) {std::lock_guard<std::mutex> lock(mutex_); error_ = e.what();}
  }
  const std::array<const char *,4> names_{{"con","truck","sedan","traffic_light"}};
  std::array<float,4> thresholds_{};
  int width_, height_, roi_height_percent_;
  bool undistort_;
  double fps_, timeout_;
  static constexpr int sensor_width_ = 1280;
  static constexpr int sensor_height_ = 800;
  double min_fps_{0}, max_fps_{0};
  std::string camera_id_, format_;
  std::unique_ptr<YoloxDetector> detector_;
  std::unique_ptr<dai::Pipeline> pipeline_; std::shared_ptr<dai::MessageQueue> queue_;
  std::atomic<bool> stop_{false}; std::thread capture_; std::mutex mutex_;
  std::shared_ptr<dai::ImgFrame> latest_; std::uint64_t received_{0}; std::string error_;
  Clock::time_point first_frame_, last_frame_;
  std::optional<double> requested_fps_;
  std::string fps_status_;
  std::uint64_t camera_session_{0};
};
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {auto node = std::make_shared<multiclass_yolo::Preview>(); node->run();}
  catch (const std::exception & e) {RCLCPP_ERROR(rclcpp::get_logger("multiclass_yolo"), "%s", e.what()); status=1;}
  if (rclcpp::ok()) rclcpp::shutdown();
  return status;
}
