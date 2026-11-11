#include "multiclass_yolo/yolox_detector.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace multiclass_yolo {
class Preview : public rclcpp::Node {
 public:
  Preview() : Node("multiclass_yolo") {
    const auto share = ament_index_cpp::get_package_share_directory("multiclass_yolo");
    auto model = declare_parameter<std::string>("model_path", share + "/models/multiclass_yolox_tiny_512x320_final.onnx");
    auto backend = declare_parameter<std::string>("inference_backend", "TENSORRT");
    auto precision = declare_parameter<std::string>("engine_precision", "fp16");
    // User cache is writable even when ROS install/share is read-only.
    const char * home = std::getenv("HOME");
    auto cache = declare_parameter<std::string>("engine_cache_path", "");
    if (!std::filesystem::is_regular_file(model)) throw std::runtime_error("Model missing: " + model);
    if (cache.empty()) {
      const auto identity = std::filesystem::canonical(model).string() +
        std::to_string(std::filesystem::file_size(model)) +
        std::to_string(std::filesystem::last_write_time(model).time_since_epoch().count());
      cache = std::string(home ? home : "/tmp") + "/.cache/multiclass_yolo/" +
        std::to_string(std::hash<std::string>{}(identity)) + "." + precision + ".engine";
    }
    format_ = declare_parameter<std::string>("input_format", "nv12");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 800);
    fps_ = declare_parameter<double>("sensor_fps", 30.0);
    bool undistort = declare_parameter<bool>("undistort_enabled", true);
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
      sliders_[i] = static_cast<int>(std::lround(v*100));
    }
    if (!std::filesystem::is_regular_file(model)) throw std::runtime_error("Model missing: " + model);
    detector_ = std::make_unique<YoloxDetector>(model, backend, cache, precision,
      512, 320, 0.0f, static_cast<float>(nms), std::size_t(workspace)*1024*1024,
      [this](const std::string & s) {RCLCPP_INFO(get_logger(), "%s", s.c_str());});
    if (format_ == "nv12" && !detector_->supports_nv12_input())
      throw std::invalid_argument("input_format=nv12 requires TensorRT; CPU: use input_format=rgb engine_precision=fp32");
    auto device = std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
    pipeline_ = std::make_unique<dai::Pipeline>(device);
    pipeline_->setXLinkChunkSize(0);
    auto camera = pipeline_->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_A, std::make_pair(1280U, 800U), float(fps_));
    auto output = camera->requestOutput(std::make_pair(std::uint32_t(width_), std::uint32_t(height_)),
      dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, float(fps_), undistort);
    queue_ = output->createOutputQueue(1U, false);
    pipeline_->build();
    auto bridge = output->getXLinkBridge();
    if (!bridge || !bridge->xLinkOut) throw std::runtime_error("Missing XLink bridge");
    bridge->xLinkOut->input.setMaxSize(1);
    bridge->xLinkOut->input.setBlocking(false);
    pipeline_->start();
    RCLCPP_INFO(get_logger(), "CAM_A NV12 %dx%d @ %.1f; path=%s; tensor=BGR FP32 [1,3,320,512]", width_, height_, fps_, format_.c_str());
    if (device->getUsbSpeed() == dai::UsbSpeed::HIGH)
      RCLCPP_WARN(get_logger(), "USB2 connection detected; 30 FPS must be measured on this connection");
    last_frame_ = Clock::now();
    capture_ = std::thread([this] { capture_loop(); });
  }
  ~Preview() override {
    stop_ = true;
    if (capture_.joinable()) capture_.join();
    if (pipeline_) {try { pipeline_->stop(); pipeline_->wait(); } catch (...) {}}
  }
  void run() {
    const std::string window = "multiclass_yolo";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::resizeWindow(window, 1024, 780);
    for (int i=0; i<4; ++i) {
      cv::createTrackbar(names_[i], window, nullptr, 100);
      cv::setTrackbarPos(names_[i], window, sliders_[i]);
    }
    std::uint64_t previous = 0, outputs = 0, skipped = 0;
    auto last_log = Clock::now();
    while (rclcpp::ok() && !stop_) {
      rclcpp::spin_some(get_node_base_interface());
      std::shared_ptr<dai::ImgFrame> frame;
      Clock::time_point received, first;
      std::uint64_t count;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!error_.empty()) throw std::runtime_error(error_);
        count = received_; first = first_frame_; received = last_frame_;
        if (count != previous) frame = latest_;
      }
      if (std::chrono::duration<double>(Clock::now()-received).count() > timeout_)
        throw std::runtime_error("No camera frame within frame_timeout_sec; check OAK/USB and other camera processes");
      if (frame) {
        skipped += count > previous ? count - previous - 1 : 0;
        previous = count;
        std::array<float,4> thresholds;
        for (int i=0; i<4; ++i) thresholds[i] = cv::getTrackbarPos(names_[i], window)/100.f;
        detector_->set_thresholds(thresholds);
        const auto began = Clock::now();
        const auto & bytes = frame->getData();
        const std::size_t stride = frame->getStride() > 0 ? frame->getStride() : width_;
        if (stride < std::size_t(width_) || bytes.size() < stride*std::size_t(height_)*3/2)
          throw std::runtime_error("Invalid NV12 buffer/pitch");
        YoloxDetectionResult result;
        cv::Mat display;
        const auto conversion_started = Clock::now();
        double conversion_ms = 0;
        if (format_ == "nv12") {
          result = detector_->detect_nv12(bytes.data(), bytes.size(), stride, width_, height_, cv::Rect(0,0,width_,height_));
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
          result = detector_->detect(rgb, true);
          cv::cvtColor(rgb, display, cv::COLOR_RGB2BGR);
        }
        detector_->draw(display, result.detections);
        const double total_ms = std::chrono::duration<double,std::milli>(Clock::now()-began).count();
        // Common epoch: first host receipt, engine build and camera startup excluded.
        const double elapsed = std::chrono::duration<double>(Clock::now()-first).count();
        std::uint64_t current_received;
        {std::lock_guard<std::mutex> lock(mutex_); current_received = received_;}
        const double in_fps = elapsed > 0.5 ? double(current_received-1)/elapsed : 0;
        const double out_fps = elapsed > 0.5 ? double(outputs)/elapsed : 0;
        cv::Mat canvas;
        cv::copyMakeBorder(display, canvas, 72, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(20,20,20));
        cv::putText(canvas, cv::format("AVG input %.2f FPS | AVG output %.2f FPS | target %.0f", in_fps, out_fps, fps_),
          {10,25}, cv::FONT_HERSHEY_SIMPLEX, .65, {255,255,255}, 1, cv::LINE_AA);
        cv::putText(canvas, cv::format("%s | total %.1f ms | convert %.1f ms | skipped %llu | Q/ESC quit",
          format_.c_str(), total_ms, conversion_ms, static_cast<unsigned long long>(skipped)),
          {10,55}, cv::FONT_HERSHEY_SIMPLEX, .5, {200,220,200}, 1, cv::LINE_AA);
        cv::imshow(window, canvas);
        ++outputs;
        if (Clock::now()-last_log >= 2s) {
          RCLCPP_INFO(get_logger(), "AVG input %.2f / output %.2f FPS; total %.2f ms; preprocess %.2f ms; execute %.2f ms; skipped %llu",
            in_fps, out_fps, total_ms, result.timing.preprocessing_nanoseconds/1e6,
            result.timing.forward_nanoseconds/1e6, static_cast<unsigned long long>(skipped));
          last_log = Clock::now();
        }
      }
      int key = cv::waitKey(1) & 255;
      if (key == 27 || key == 'q' || key == 'Q' || cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) < 1) break;
      if (!frame) std::this_thread::sleep_for(1ms);
    }
    cv::destroyAllWindows();
  }
 private:
  void capture_loop() {
    try {
      while (!stop_ && rclcpp::ok()) {
        if (!pipeline_->isRunning()) throw std::runtime_error("OAK pipeline stopped");
        auto packet = queue_->tryGet<dai::ImgFrame>();
        if (!packet) {std::this_thread::sleep_for(100us); continue;}
        if (packet->getType() != dai::ImgFrame::Type::NV12 || int(packet->getWidth()) != width_ || int(packet->getHeight()) != height_)
          throw std::runtime_error("Unexpected OAK frame type or dimensions");
        std::lock_guard<std::mutex> lock(mutex_);
        last_frame_ = Clock::now();
        if (received_ == 0) first_frame_ = last_frame_;
        latest_ = std::move(packet); ++received_;
      }
    } catch (const std::exception & e) {std::lock_guard<std::mutex> lock(mutex_); error_ = e.what();}
  }
  const std::array<const char *,4> names_{{"con","truck","sedan","traffic_light"}};
  std::array<int,4> sliders_{};
  int width_, height_; double fps_, timeout_; std::string format_;
  std::unique_ptr<YoloxDetector> detector_;
  std::unique_ptr<dai::Pipeline> pipeline_; std::shared_ptr<dai::MessageQueue> queue_;
  std::atomic<bool> stop_{false}; std::thread capture_; std::mutex mutex_;
  std::shared_ptr<dai::ImgFrame> latest_; std::uint64_t received_{0}; std::string error_;
  Clock::time_point first_frame_, last_frame_;
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
