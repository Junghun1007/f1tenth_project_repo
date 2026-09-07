#include "traffic_detection_test/traffic_detection_test_node.hpp"

#include "traffic_detection_test/yolox_detector.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace traffic_detection_test
{

using namespace std::chrono_literals;

namespace
{

constexpr std::uint32_t kFullSensorWidth = 1280U;
constexpr std::uint32_t kFullSensorHeight = 800U;
constexpr char kModelFilename[] =
  "traffic_light_yolox_s_640_batch_1.onnx";

std::string uppercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {
      return static_cast<char>(std::toupper(character));
    });
  return value;
}

dai::CameraBoardSocket parse_camera_socket(const std::string & value)
{
  const auto normalized = uppercase(value);
  if (normalized == "CAM_A") {
    return dai::CameraBoardSocket::CAM_A;
  }
  if (normalized == "CAM_B") {
    return dai::CameraBoardSocket::CAM_B;
  }
  if (normalized == "CAM_C") {
    return dai::CameraBoardSocket::CAM_C;
  }
  if (normalized == "CAM_D") {
    return dai::CameraBoardSocket::CAM_D;
  }
  throw std::invalid_argument(
          "camera_socket must be CAM_A, CAM_B, CAM_C, or CAM_D");
}

bool uses_gray8_transport(const dai::CameraBoardSocket socket)
{
  return
    socket == dai::CameraBoardSocket::CAM_B ||
    socket == dai::CameraBoardSocket::CAM_C;
}

dai::ImgResizeMode parse_resize_mode(const std::string & value)
{
  const auto normalized = uppercase(value);
  if (normalized == "CROP") {
    return dai::ImgResizeMode::CROP;
  }
  if (normalized == "STRETCH") {
    return dai::ImgResizeMode::STRETCH;
  }
  if (normalized == "LETTERBOX") {
    return dai::ImgResizeMode::LETTERBOX;
  }
  throw std::invalid_argument(
          "resize_mode must be CROP, STRETCH, or LETTERBOX");
}

const char * usb_speed_name(const dai::UsbSpeed speed)
{
  switch (speed) {
    case dai::UsbSpeed::LOW:
      return "LOW";
    case dai::UsbSpeed::FULL:
      return "FULL";
    case dai::UsbSpeed::HIGH:
      return "HIGH";
    case dai::UsbSpeed::SUPER:
      return "SUPER";
    case dai::UsbSpeed::SUPER_PLUS:
      return "SUPER_PLUS";
    case dai::UsbSpeed::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

bool graphical_display_available()
{
#if defined(__linux__)
  return std::getenv("DISPLAY") != nullptr ||
         std::getenv("WAYLAND_DISPLAY") != nullptr;
#else
  return true;
#endif
}

std::string default_model_path()
{
  const auto package_share =
    ament_index_cpp::get_package_share_directory("traffic_detection_test");
  return (
    std::filesystem::path(package_share) / "models" / kModelFilename).string();
}

double nanoseconds_to_milliseconds(const std::uint64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1.0e6;
}

void update_maximum(
  std::atomic<std::uint64_t> & target,
  const std::uint64_t candidate)
{
  auto current = target.load(std::memory_order_relaxed);
  while (
    current < candidate &&
    !target.compare_exchange_weak(
      current, candidate,
      std::memory_order_relaxed,
      std::memory_order_relaxed))
  {
  }
}

struct DurationSummary
{
  std::uint64_t sample_count{0U};
  std::uint64_t total_nanoseconds{0U};
  std::uint64_t maximum_nanoseconds{0U};

  double average_milliseconds() const
  {
    return sample_count > 0U ?
      nanoseconds_to_milliseconds(total_nanoseconds) /
      static_cast<double>(sample_count) : 0.0;
  }

  double maximum_milliseconds() const
  {
    return nanoseconds_to_milliseconds(maximum_nanoseconds);
  }
};

class DurationAccumulator
{
public:
  void record(const std::chrono::steady_clock::duration duration)
  {
    const auto nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    constexpr std::int64_t maximum_valid_nanoseconds =
      60LL * 1000LL * 1000LL * 1000LL;
    if (nanoseconds < 0 || nanoseconds > maximum_valid_nanoseconds) {
      return;
    }
    record(static_cast<std::uint64_t>(nanoseconds));
  }

  void record(const std::uint64_t nanoseconds)
  {
    sample_count_.fetch_add(1U, std::memory_order_relaxed);
    total_nanoseconds_.fetch_add(nanoseconds, std::memory_order_relaxed);
    update_maximum(maximum_nanoseconds_, nanoseconds);
  }

  DurationSummary take_interval()
  {
    return DurationSummary{
      sample_count_.exchange(0U, std::memory_order_relaxed),
      total_nanoseconds_.exchange(0U, std::memory_order_relaxed),
      maximum_nanoseconds_.exchange(0U, std::memory_order_relaxed)};
  }

private:
  std::atomic<std::uint64_t> sample_count_{0U};
  std::atomic<std::uint64_t> total_nanoseconds_{0U};
  std::atomic<std::uint64_t> maximum_nanoseconds_{0U};
};

}  // namespace

class TrafficDetectionTestNode::Impl
{
public:
  explicit Impl(TrafficDetectionTestNode & node)
  : node_(node),
    started_at_(std::chrono::steady_clock::now()),
    last_status_at_(started_at_)
  {
    read_parameters();
    validate_parameters();

    if (!graphical_display_available()) {
      throw std::runtime_error(
              "traffic_detection_test is preview-only, but DISPLAY and "
              "WAYLAND_DISPLAY are unavailable");
    }

    if (!std::filesystem::is_regular_file(model_path_)) {
      throw std::runtime_error("ONNX model not found: " + model_path_);
    }
    detector_ = std::make_unique<YoloxDetector>(
      model_path_, inference_backend_, engine_cache_path_, model_input_width_,
      model_input_height_, score_threshold_, nms_threshold_,
      static_cast<std::size_t>(tensorrt_workspace_size_mb_) * 1024U * 1024U);

    try {
      start_pipeline();
      started_at_ = std::chrono::steady_clock::now();
      last_status_at_ = started_at_;
      status_timer_ = node_.create_wall_timer(
        std::chrono::duration<double>(status_log_interval_sec_),
        std::bind(&Impl::report_status, this));
      capture_thread_ = std::thread(&Impl::capture_loop, this);
      preview_thread_ = std::thread(&Impl::preview_loop, this);
    } catch (...) {
      stop();
      throw;
    }
  }

  ~Impl()
  {
    stop();
  }

private:
  struct FrameSnapshot
  {
    std::shared_ptr<dai::ImgFrame> packet;
    std::chrono::steady_clock::time_point sensor_at;
    std::chrono::steady_clock::time_point received_at;
    std::uint64_t generation;
    std::int64_t device_sequence;
  };

  void read_parameters()
  {
    camera_socket_name_ =
      node_.declare_parameter<std::string>("camera_socket", "CAM_A");
    width_ = node_.declare_parameter<int>("width", 640);
    height_ = node_.declare_parameter<int>("height", 400);
    sensor_fps_ = node_.declare_parameter<double>("sensor_fps", 80.0);
    resize_mode_name_ =
      node_.declare_parameter<std::string>("resize_mode", "CROP");
    undistort_enabled_ =
      node_.declare_parameter<bool>("undistort_enabled", true);

    model_path_ = node_.declare_parameter<std::string>(
      "model_path", default_model_path());
    inference_backend_ = node_.declare_parameter<std::string>(
      "inference_backend", "TENSORRT");
    engine_cache_path_ = node_.declare_parameter<std::string>(
      "engine_cache_path", "");
    tensorrt_workspace_size_mb_ = node_.declare_parameter<int>(
      "tensorrt_workspace_size_mb", 1024);
    model_input_width_ =
      node_.declare_parameter<int>("model_input_width", 640);
    model_input_height_ =
      node_.declare_parameter<int>("model_input_height", 640);
    score_threshold_ = static_cast<float>(
      node_.declare_parameter<double>("score_threshold", 0.25));
    nms_threshold_ = static_cast<float>(
      node_.declare_parameter<double>("nms_threshold", 0.65));

    preview_fps_ = node_.declare_parameter<double>("preview_fps", 60.0);
    preview_window_name_ = node_.declare_parameter<std::string>(
      "preview_window_name", "Traffic light detection test");
    preview_max_width_ =
      node_.declare_parameter<int>("preview_max_width", 1280);
    preview_max_height_ =
      node_.declare_parameter<int>("preview_max_height", 800);
    startup_timeout_sec_ =
      node_.declare_parameter<double>("startup_timeout_sec", 5.0);
    status_log_interval_sec_ =
      node_.declare_parameter<double>("status_log_interval_sec", 1.0);

    camera_socket_ = parse_camera_socket(camera_socket_name_);
    gray8_transport_ = uses_gray8_transport(camera_socket_);
    resize_mode_ = parse_resize_mode(resize_mode_name_);
  }

  void validate_parameters() const
  {
    if (width_ <= 0 || height_ <= 0 || width_ % 2 != 0 || height_ % 2 != 0) {
      throw std::invalid_argument(
              "width and height must be positive even numbers");
    }
    if (width_ != 640 || height_ != 400) {
      throw std::invalid_argument(
              "this test package requires a 640x400 camera frame");
    }
    if (model_input_width_ != 640 || model_input_height_ != 640) {
      throw std::invalid_argument(
              "the bundled YOLOX model requires a 640x640 tensor");
    }
    if (
      tensorrt_workspace_size_mb_ <= 0 ||
      tensorrt_workspace_size_mb_ > 16384)
    {
      throw std::invalid_argument(
              "tensorrt_workspace_size_mb must be in [1, 16384]");
    }
    if (!std::isfinite(sensor_fps_) || sensor_fps_ <= 0.0) {
      throw std::invalid_argument("sensor_fps must be positive");
    }
    if (!std::isfinite(preview_fps_) || preview_fps_ <= 0.0) {
      throw std::invalid_argument("preview_fps must be positive");
    }
    if (preview_window_name_.empty()) {
      throw std::invalid_argument("preview_window_name must not be empty");
    }
    if (preview_max_width_ < 0 || preview_max_height_ < 0) {
      throw std::invalid_argument(
              "preview maximum dimensions must not be negative");
    }
    if (!std::isfinite(startup_timeout_sec_) || startup_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("startup_timeout_sec must be positive");
    }
    if (
      !std::isfinite(status_log_interval_sec_) ||
      status_log_interval_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "status_log_interval_sec must be positive");
    }
  }

  void start_pipeline()
  {
    auto device = std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
    pipeline_ = std::make_unique<dai::Pipeline>(device);
    pipeline_->setXLinkChunkSize(0);

    auto camera = pipeline_->create<dai::node::Camera>()->build(
      camera_socket_,
      std::make_pair(kFullSensorWidth, kFullSensorHeight),
      static_cast<float>(sensor_fps_));
    const auto transport_type = gray8_transport_ ?
      dai::ImgFrame::Type::GRAY8 : dai::ImgFrame::Type::NV12;
    auto * output = camera->requestOutput(
      std::make_pair(
        static_cast<std::uint32_t>(width_),
        static_cast<std::uint32_t>(height_)),
      transport_type,
      resize_mode_,
      static_cast<float>(sensor_fps_),
      undistort_enabled_);

    output_queue_ = output->createOutputQueue(1U, false);

    pipeline_->build();
    const auto xlink_bridge = output->getXLinkBridge();
    if (!xlink_bridge || !xlink_bridge->xLinkOut) {
      throw std::runtime_error(
              "DepthAI did not create the camera XLink output bridge");
    }
    xlink_bridge->xLinkOut->input.setMaxSize(1);
    xlink_bridge->xLinkOut->input.setBlocking(false);
    pipeline_->start();

    RCLCPP_INFO(
      node_.get_logger(),
      "OAK preview input: %dx%d @ %.1f FPS on %s, USB=%s, transport=%s, "
      "undistort=%s, host/device queues=1/non-blocking",
      width_, height_, sensor_fps_, camera_socket_name_.c_str(),
      usb_speed_name(device->getUsbSpeed()),
      gray8_transport_ ? "GRAY8" : "NV12",
      undistort_enabled_ ? "on" : "off");
    RCLCPP_INFO(
      node_.get_logger(),
      "YOLOX FP32 preview: model=%s, input=%dx%d, score=%.2f, NMS=%.2f, "
      "backend=%s",
      model_path_.c_str(), model_input_width_, model_input_height_,
      static_cast<double>(score_threshold_),
      static_cast<double>(nms_threshold_),
      detector_->backend_name().c_str());
    if (gray8_transport_) {
      RCLCPP_WARN(
        node_.get_logger(),
        "The selected camera is monochrome; it will be expanded to BGR, "
        "but the model was trained with color CAM_A images.");
    }
  }

  void capture_loop()
  {
    while (!stop_requested_.load(std::memory_order_relaxed)) {
      try {
        if (!pipeline_ || !pipeline_->isRunning()) {
          break;
        }

        auto packet = output_queue_->tryGet<dai::ImgFrame>();
        if (!packet) {
          std::this_thread::sleep_for(100us);
          continue;
        }
        const auto received_at = std::chrono::steady_clock::now();
        const auto sensor_at = packet->getTimestamp(
          dai::CameraExposureOffset::MIDDLE);
        sensor_to_host_stats_.record(received_at - sensor_at);

        const auto expected_type = gray8_transport_ ?
          dai::ImgFrame::Type::GRAY8 : dai::ImgFrame::Type::NV12;
        if (
          packet->getType() != expected_type ||
          static_cast<int>(packet->getWidth()) != width_ ||
          static_cast<int>(packet->getHeight()) != height_)
        {
          invalid_frames_total_.fetch_add(1U, std::memory_order_relaxed);
          RCLCPP_ERROR_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000,
            "Expected a %dx%d %s frame from DepthAI.",
            width_, height_, gray8_transport_ ? "GRAY8" : "NV12");
          continue;
        }

        const auto device_sequence = packet->getSequenceNum();
        if (
          last_device_sequence_.has_value() &&
          device_sequence > *last_device_sequence_ + 1)
        {
          device_drops_total_.fetch_add(
            static_cast<std::uint64_t>(
              device_sequence - *last_device_sequence_ - 1),
            std::memory_order_relaxed);
        }
        last_device_sequence_ = device_sequence;

        const auto generation =
          received_total_.fetch_add(1U, std::memory_order_relaxed) + 1U;
        received_interval_.fetch_add(1U, std::memory_order_relaxed);
        std::shared_ptr<const FrameSnapshot> snapshot =
          std::make_shared<FrameSnapshot>(
          FrameSnapshot{
            packet,
            sensor_at,
            received_at,
            generation,
            device_sequence});
        std::atomic_store_explicit(
          &latest_frame_, std::move(snapshot), std::memory_order_release);
        first_frame_received_.store(true, std::memory_order_relaxed);
        frame_available_.notify_all();
      } catch (const std::exception & exception) {
        capture_errors_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 1000,
          "Camera capture error: %s", exception.what());
        std::this_thread::sleep_for(1ms);
      }
    }
  }

  void resize_preview_window(const cv::Mat & frame)
  {
    if (preview_window_sized_) {
      return;
    }

    double scale = 1.0;
    if (preview_max_width_ > 0) {
      scale = std::min(
        scale,
        static_cast<double>(preview_max_width_) /
        static_cast<double>(frame.cols));
    }
    if (preview_max_height_ > 0) {
      scale = std::min(
        scale,
        static_cast<double>(preview_max_height_) /
        static_cast<double>(frame.rows));
    }
    cv::resizeWindow(
      preview_window_name_,
      std::max(1, static_cast<int>(static_cast<double>(frame.cols) * scale)),
      std::max(1, static_cast<int>(static_cast<double>(frame.rows) * scale)));
    preview_window_sized_ = true;
  }

  void draw_status_overlay(
    cv::Mat & frame,
    const std::vector<TrafficLightDetection> & detections,
    const double forward_ms,
    const double detector_total_ms) const
  {
    float best_score = 0.0F;
    for (const auto & detection : detections) {
      best_score = std::max(best_score, detection.score);
    }

    const int banner_top = std::max(0, frame.rows - 42);
    cv::rectangle(
      frame,
      cv::Point(0, banner_top),
      cv::Point(frame.cols - 1, frame.rows - 1),
      cv::Scalar(0, 0, 0),
      cv::FILLED);
    const std::string state = detections.empty() ?
      "NOT DETECTED" : "TRAFFIC LIGHT DETECTED";
    const cv::Scalar state_color = detections.empty() ?
      cv::Scalar(180, 180, 180) : cv::Scalar(255, 80, 180);
    cv::putText(
      frame, state, cv::Point(8, banner_top + 18), cv::FONT_HERSHEY_SIMPLEX,
      0.52, state_color, 1, cv::LINE_AA);

    const std::string details = detections.empty() ?
      cv::format(
      "execute %.1f ms | total %.1f ms | Q/ESC: quit",
      forward_ms, detector_total_ms) :
      cv::format(
      "count %zu | best %.2f | execute %.1f ms | total %.1f ms",
      detections.size(), static_cast<double>(best_score),
      forward_ms, detector_total_ms);
    cv::putText(
      frame, details, cv::Point(8, banner_top + 35),
      cv::FONT_HERSHEY_SIMPLEX,
      0.40, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
  }

  bool quit_requested_from_window(bool & window_was_visible)
  {
    const int key = cv::waitKey(1) & 0xff;
    const double visible = cv::getWindowProperty(
      preview_window_name_, cv::WND_PROP_VISIBLE);
    if (visible >= 1.0) {
      window_was_visible = true;
    }
    return
      key == 'q' || key == 'Q' || key == 27 ||
      (window_was_visible && visible < 1.0);
  }

  void preview_loop()
  {
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
    } catch (const std::exception & exception) {
      preview_failed(exception.what());
      return;
    }

    const auto minimum_period = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / preview_fps_));
    auto next_allowed_at = std::chrono::steady_clock::now();
    std::uint64_t processed_generation = 0U;
    bool window_was_visible = false;

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        frame_available_.wait_for(
          lock, 100ms,
          [this, processed_generation]() {
            if (stop_requested_.load(std::memory_order_relaxed)) {
              return true;
            }
            const auto latest = std::atomic_load_explicit(
              &latest_frame_, std::memory_order_acquire);
            return latest && latest->generation != processed_generation;
          });
      }
      if (stop_requested_.load(std::memory_order_relaxed)) {
        break;
      }

      auto now = std::chrono::steady_clock::now();
      if (now < next_allowed_at) {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        frame_available_.wait_until(
          lock, next_allowed_at,
          [this]() {
            return stop_requested_.load(std::memory_order_relaxed);
          });
        if (stop_requested_.load(std::memory_order_relaxed)) {
          break;
        }
      }

      auto snapshot = std::atomic_load_explicit(
        &latest_frame_, std::memory_order_acquire);
      if (!snapshot || snapshot->generation == processed_generation) {
        try {
          if (quit_requested_from_window(window_was_visible)) {
            request_shutdown();
            break;
          }
        } catch (const std::exception & exception) {
          preview_failed(exception.what());
          break;
        }
        continue;
      }

      if (processed_generation > 0U &&
        snapshot->generation > processed_generation + 1U)
      {
        inference_skips_total_.fetch_add(
          snapshot->generation - processed_generation - 1U,
          std::memory_order_relaxed);
      }
      processed_generation = snapshot->generation;

      try {
        const auto conversion_started_at = std::chrono::steady_clock::now();
        cv::Mat frame = snapshot->packet->getCvFrame();
        if (frame.empty()) {
          throw std::runtime_error("DepthAI returned an empty preview frame");
        }
        if (frame.type() == CV_8UC1) {
          cv::Mat bgr;
          cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
          frame = std::move(bgr);
        } else if (frame.type() != CV_8UC3) {
          throw std::runtime_error(
                  "DepthAI returned an unsupported preview frame type");
        }
        const auto conversion_finished_at = std::chrono::steady_clock::now();
        conversion_stats_.record(
          conversion_finished_at - conversion_started_at);

        const auto result = detector_->detect(frame);
        preprocessing_stats_.record(
          result.timing.preprocessing_nanoseconds);
        input_transfer_stats_.record(
          result.timing.input_transfer_nanoseconds);
        forward_stats_.record(result.timing.forward_nanoseconds);
        output_transfer_stats_.record(
          result.timing.output_transfer_nanoseconds);
        postprocessing_stats_.record(
          result.timing.postprocessing_nanoseconds);
        const std::uint64_t detector_total_nanoseconds =
          result.timing.preprocessing_nanoseconds +
          result.timing.input_transfer_nanoseconds +
          result.timing.forward_nanoseconds +
          result.timing.output_transfer_nanoseconds +
          result.timing.postprocessing_nanoseconds;
        detector_total_stats_.record(detector_total_nanoseconds);

        const auto drawing_started_at = std::chrono::steady_clock::now();
        detector_->draw(frame, result.detections);
        draw_status_overlay(
          frame,
          result.detections,
          nanoseconds_to_milliseconds(result.timing.forward_nanoseconds),
          nanoseconds_to_milliseconds(detector_total_nanoseconds));
        const auto drawing_finished_at = std::chrono::steady_clock::now();
        drawing_stats_.record(drawing_finished_at - drawing_started_at);

        resize_preview_window(frame);
        cv::imshow(preview_window_name_, frame);

        inferred_total_.fetch_add(1U, std::memory_order_relaxed);
        inferred_interval_.fetch_add(1U, std::memory_order_relaxed);
        detection_total_.fetch_add(
          static_cast<std::uint64_t>(result.detections.size()),
          std::memory_order_relaxed);
        if (!result.detections.empty()) {
          detected_frames_total_.fetch_add(1U, std::memory_order_relaxed);
        }

        const auto displayed_at = std::chrono::steady_clock::now();
        host_to_display_stats_.record(displayed_at - snapshot->received_at);
        sensor_to_display_stats_.record(displayed_at - snapshot->sensor_at);

        if (quit_requested_from_window(window_was_visible)) {
          request_shutdown();
          break;
        }

        now = std::chrono::steady_clock::now();
        next_allowed_at += minimum_period;
        if (next_allowed_at < now - minimum_period) {
          next_allowed_at = now;
        }
      } catch (const std::exception & exception) {
        preview_failed(exception.what());
        break;
      }
    }

    try {
      cv::destroyWindow(preview_window_name_);
      cv::waitKey(1);
    } catch (const std::exception &) {
    }
  }

  void preview_failed(const char * message)
  {
    preview_errors_total_.fetch_add(1U, std::memory_order_relaxed);
    RCLCPP_ERROR(node_.get_logger(), "Detection preview stopped: %s", message);
    request_shutdown();
  }

  void request_shutdown()
  {
    stop_requested_.store(true, std::memory_order_relaxed);
    frame_available_.notify_all();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void report_status()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - last_status_at_).count();
    last_status_at_ = now;

    const auto captured =
      received_interval_.exchange(0U, std::memory_order_relaxed);
    const auto inferred =
      inferred_interval_.exchange(0U, std::memory_order_relaxed);
    const auto sensor_to_host = sensor_to_host_stats_.take_interval();
    const auto conversion = conversion_stats_.take_interval();
    const auto preprocessing = preprocessing_stats_.take_interval();
    const auto input_transfer = input_transfer_stats_.take_interval();
    const auto forward = forward_stats_.take_interval();
    const auto output_transfer = output_transfer_stats_.take_interval();
    const auto postprocessing = postprocessing_stats_.take_interval();
    const auto detector_total = detector_total_stats_.take_interval();
    const auto drawing = drawing_stats_.take_interval();
    const auto host_to_display = host_to_display_stats_.take_interval();
    const auto sensor_to_display = sensor_to_display_stats_.take_interval();
    const double capture_hz = elapsed > 0.0 ?
      static_cast<double>(captured) / elapsed : 0.0;
    const double inference_hz = elapsed > 0.0 ?
      static_cast<double>(inferred) / elapsed : 0.0;

    RCLCPP_INFO(
      node_.get_logger(),
      "FPS: capture=%.1f/%.1f, inference=%.1f | AVG ms: "
      "sensor->host=%.2f, NV12->BGR=%.2f, preprocess=%.2f, "
      "H2D=%.2f, execute=%.2f, D2H=%.2f, postprocess=%.2f, draw=%.2f, "
      "detector-total=%.2f",
      capture_hz, sensor_fps_, inference_hz,
      sensor_to_host.average_milliseconds(),
      conversion.average_milliseconds(),
      preprocessing.average_milliseconds(),
      input_transfer.average_milliseconds(),
      forward.average_milliseconds(),
      output_transfer.average_milliseconds(),
      postprocessing.average_milliseconds(),
      drawing.average_milliseconds(),
      detector_total.average_milliseconds());
    RCLCPP_INFO(
      node_.get_logger(),
      "MAX ms: sensor->host=%.2f, NV12->BGR=%.2f, preprocess=%.2f, "
      "H2D=%.2f, execute=%.2f, D2H=%.2f, postprocess=%.2f, draw=%.2f | "
      "AVG/MAX host->display=%.2f/%.2f, sensor->display=%.2f/%.2f | "
      "skipped=%lu, "
      "detected-frames=%lu, detections=%lu, errors=%lu/%lu, "
      "device-drops=%lu",
      sensor_to_host.maximum_milliseconds(),
      conversion.maximum_milliseconds(),
      preprocessing.maximum_milliseconds(),
      input_transfer.maximum_milliseconds(),
      forward.maximum_milliseconds(),
      output_transfer.maximum_milliseconds(),
      postprocessing.maximum_milliseconds(),
      drawing.maximum_milliseconds(),
      host_to_display.average_milliseconds(),
      host_to_display.maximum_milliseconds(),
      sensor_to_display.average_milliseconds(),
      sensor_to_display.maximum_milliseconds(),
      static_cast<unsigned long>(
        inference_skips_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        detected_frames_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        detection_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        capture_errors_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        preview_errors_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        device_drops_total_.load(std::memory_order_relaxed)));

    const double running_for =
      std::chrono::duration<double>(now - started_at_).count();
    if (
      !first_frame_received_.load(std::memory_order_relaxed) &&
      running_for >= startup_timeout_sec_ &&
      !startup_timeout_reported_.exchange(true, std::memory_order_relaxed))
    {
      RCLCPP_ERROR(
        node_.get_logger(),
        "No 640x400 camera frame was received within %.1f seconds.",
        startup_timeout_sec_);
    }
  }

  void stop()
  {
    if (shutdown_started_.exchange(true, std::memory_order_relaxed)) {
      return;
    }
    stop_requested_.store(true, std::memory_order_relaxed);
    frame_available_.notify_all();

    join_thread(preview_thread_);
    join_thread(capture_thread_);

    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }
    output_queue_.reset();
    if (pipeline_) {
      try {
        if (pipeline_->isRunning()) {
          pipeline_->stop();
          pipeline_->wait();
        }
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          node_.get_logger(),
          "DepthAI pipeline shutdown reported an error: %s",
          exception.what());
      }
      pipeline_.reset();
    }
  }

  static void join_thread(std::thread & thread)
  {
    if (thread.joinable() && thread.get_id() != std::this_thread::get_id()) {
      thread.join();
    }
  }

  TrafficDetectionTestNode & node_;

  std::string camera_socket_name_;
  int width_{640};
  int height_{400};
  double sensor_fps_{80.0};
  std::string resize_mode_name_;
  bool undistort_enabled_{true};
  std::string model_path_;
  std::string inference_backend_{"TENSORRT"};
  std::string engine_cache_path_;
  int tensorrt_workspace_size_mb_{1024};
  int model_input_width_{640};
  int model_input_height_{640};
  float score_threshold_{0.25F};
  float nms_threshold_{0.65F};
  double preview_fps_{60.0};
  std::string preview_window_name_;
  int preview_max_width_{1280};
  int preview_max_height_{800};
  double startup_timeout_sec_{5.0};
  double status_log_interval_sec_{1.0};

  dai::CameraBoardSocket camera_socket_{dai::CameraBoardSocket::CAM_A};
  bool gray8_transport_{false};
  dai::ImgResizeMode resize_mode_{dai::ImgResizeMode::CROP};
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> output_queue_;
  std::unique_ptr<YoloxDetector> detector_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::thread capture_thread_;
  std::thread preview_thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutdown_started_{false};
  std::mutex wait_mutex_;
  std::condition_variable frame_available_;
  std::shared_ptr<const FrameSnapshot> latest_frame_;
  std::optional<std::int64_t> last_device_sequence_;
  bool preview_window_sized_{false};

  std::atomic<bool> first_frame_received_{false};
  std::atomic<bool> startup_timeout_reported_{false};
  std::atomic<std::uint64_t> received_total_{0U};
  std::atomic<std::uint64_t> received_interval_{0U};
  std::atomic<std::uint64_t> inferred_total_{0U};
  std::atomic<std::uint64_t> inferred_interval_{0U};
  std::atomic<std::uint64_t> inference_skips_total_{0U};
  std::atomic<std::uint64_t> detected_frames_total_{0U};
  std::atomic<std::uint64_t> detection_total_{0U};
  std::atomic<std::uint64_t> device_drops_total_{0U};
  std::atomic<std::uint64_t> invalid_frames_total_{0U};
  std::atomic<std::uint64_t> capture_errors_total_{0U};
  std::atomic<std::uint64_t> preview_errors_total_{0U};
  DurationAccumulator sensor_to_host_stats_;
  DurationAccumulator conversion_stats_;
  DurationAccumulator preprocessing_stats_;
  DurationAccumulator input_transfer_stats_;
  DurationAccumulator forward_stats_;
  DurationAccumulator output_transfer_stats_;
  DurationAccumulator postprocessing_stats_;
  DurationAccumulator detector_total_stats_;
  DurationAccumulator drawing_stats_;
  DurationAccumulator host_to_display_stats_;
  DurationAccumulator sensor_to_display_stats_;

  std::chrono::steady_clock::time_point started_at_;
  std::chrono::steady_clock::time_point last_status_at_;
};

TrafficDetectionTestNode::TrafficDetectionTestNode(
  const rclcpp::NodeOptions & options)
: Node("traffic_detection_test", options),
  impl_(std::make_unique<Impl>(*this))
{
}

TrafficDetectionTestNode::~TrafficDetectionTestNode() = default;

}  // namespace traffic_detection_test

RCLCPP_COMPONENTS_REGISTER_NODE(
  traffic_detection_test::TrafficDetectionTestNode)
