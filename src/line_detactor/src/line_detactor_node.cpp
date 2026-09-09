#include "line_detactor/line_detactor_node.hpp"

#include "line_detactor/tensorrt_lane_backend.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace line_detactor
{
namespace
{

constexpr int kDefaultInputWidth = 120;
constexpr int kDefaultInputHeight = 300;
constexpr int kBannerHeight = 86;
constexpr char kModelFilename[] =
  "fast_scnn_highres_120x300_batch_1.onnx";

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
    ament_index_cpp::get_package_share_directory("line_detactor");
  return (
    std::filesystem::path(package_share) / "models" / kModelFilename).string();
}

double nanoseconds_to_milliseconds(const std::uint64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1.0e6;
}

struct StageStats
{
  std::uint64_t count{0U};
  std::uint64_t total_nanoseconds{0U};
  std::uint64_t maximum_nanoseconds{0U};

  void record(const std::uint64_t nanoseconds)
  {
    ++count;
    total_nanoseconds += nanoseconds;
    maximum_nanoseconds = std::max(maximum_nanoseconds, nanoseconds);
  }

  double average_milliseconds() const
  {
    return count == 0U ? 0.0 :
      nanoseconds_to_milliseconds(total_nanoseconds) /
      static_cast<double>(count);
  }

  double maximum_milliseconds() const
  {
    return nanoseconds_to_milliseconds(maximum_nanoseconds);
  }

  void reset()
  {
    count = 0U;
    total_nanoseconds = 0U;
    maximum_nanoseconds = 0U;
  }
};

}  // namespace

class LineDetactorNode::Impl
{
public:
  explicit Impl(LineDetactorNode & node)
  : node_(node)
  {
    read_parameters();
    validate_parameters();

    if (preview_enabled_ && !graphical_display_available()) {
      throw std::runtime_error(
              "Preview is enabled but DISPLAY/WAYLAND_DISPLAY is unavailable");
    }
    if (!std::filesystem::is_regular_file(model_path_)) {
      throw std::runtime_error("ONNX model not found: " + model_path_);
    }

    backend_ = std::make_unique<TensorRtLaneBackend>(
      model_path_, engine_cache_path_, model_input_width_, model_input_height_,
      static_cast<std::size_t>(tensorrt_workspace_size_mb_) * 1024U * 1024U,
      mask_threshold_, overlay_alpha_, smoothing_);
    warm_up();

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();
    subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
      input_topic_, image_qos,
      std::bind(&Impl::on_image, this, std::placeholders::_1));

    try {
      worker_ = std::thread(&Impl::worker_loop, this);
    } catch (...) {
      stop();
      throw;
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "Line detector ready: input=%s (bgr8 %dx%d), model=%s, backend="
      "TensorRT FP32, threshold=%.3f, preview=%s @ %.1f FPS",
      input_topic_.c_str(), model_input_width_, model_input_height_,
      model_path_.c_str(), static_cast<double>(mask_threshold_),
      preview_enabled_ ? "on" : "off", preview_fps_);
    RCLCPP_INFO(
      node_.get_logger(),
      "GPU path: pinned BGR8 H2D -> CUDA RGB FP32 NCHW -> TensorRT -> "
      "CUDA threshold/overlay -> BGR8 D2H; engine cache=%s; smoothing=%s "
      "(row D2H + CPU spline + correction H2D), limit=%s %.2f px",
      backend_->engine_cache_path().c_str(), smoothing_.enabled ? "on" : "off",
      smoothing_.correction_limit_enabled ? "on" : "off", smoothing_.max_correction_px);
  }

  ~Impl()
  {
    stop();
  }

private:
  using Image = sensor_msgs::msg::Image;

  void read_parameters()
  {
    input_topic_ = node_.declare_parameter<std::string>(
      "input_topic", "/camera/image_bev");
    model_path_ = node_.declare_parameter<std::string>(
      "model_path", default_model_path());
    engine_cache_path_ = node_.declare_parameter<std::string>(
      "engine_cache_path", "");
    tensorrt_workspace_size_mb_ = node_.declare_parameter<int>(
      "tensorrt_workspace_size_mb", 1024);
    model_input_width_ = node_.declare_parameter<int>(
      "model_input_width", kDefaultInputWidth);
    model_input_height_ = node_.declare_parameter<int>(
      "model_input_height", kDefaultInputHeight);
    mask_threshold_ = static_cast<float>(
      node_.declare_parameter<double>("mask_threshold", 0.5));
    overlay_alpha_ = static_cast<float>(
      node_.declare_parameter<double>("overlay_alpha", 0.75));
    smoothing_.enabled = node_.declare_parameter<bool>("smoothing_enabled", false);
    smoothing_.strength = node_.declare_parameter<double>("smoothing_strength", 8.0);
    smoothing_.correction_limit_enabled = node_.declare_parameter<bool>(
      "smoothing_correction_limit_enabled", true);
    smoothing_.max_correction_px = node_.declare_parameter<double>(
      "smoothing_max_correction_px", 2.0);
    smoothing_.max_row_jump_px = node_.declare_parameter<double>(
      "smoothing_max_row_jump_px", 4.0);
    smoothing_.min_segment_rows = node_.declare_parameter<int>(
      "smoothing_min_segment_rows", 12);
    warmup_iterations_ = node_.declare_parameter<int>(
      "warmup_iterations", 10);
    preview_enabled_ = node_.declare_parameter<bool>(
      "preview_enabled", true);
    preview_fps_ = node_.declare_parameter<double>(
      "preview_fps", 30.0);
    preview_scale_ = node_.declare_parameter<double>(
      "preview_scale", 2.0);
    preview_window_name_ = node_.declare_parameter<std::string>(
      "preview_window_name", "BEV lane TensorRT preview");
    status_log_interval_sec_ = node_.declare_parameter<double>(
      "status_log_interval_sec", 1.0);
  }

  void validate_parameters() const
  {
    validate_lane_smoothing(smoothing_);
    if (input_topic_.empty()) {
      throw std::invalid_argument("input_topic must not be empty");
    }
    if (model_path_.empty()) {
      throw std::invalid_argument("model_path must not be empty");
    }
    if (model_input_width_ <= 0 || model_input_height_ <= 0) {
      throw std::invalid_argument("model input dimensions must be positive");
    }
    if (tensorrt_workspace_size_mb_ <= 0 ||
      tensorrt_workspace_size_mb_ > 16384)
    {
      throw std::invalid_argument(
              "tensorrt_workspace_size_mb must be in [1,16384]");
    }
    if (!std::isfinite(mask_threshold_) ||
      mask_threshold_ < 0.0F || mask_threshold_ > 1.0F)
    {
      throw std::invalid_argument("mask_threshold must be in [0,1]");
    }
    if (!std::isfinite(overlay_alpha_) ||
      overlay_alpha_ < 0.0F || overlay_alpha_ > 1.0F)
    {
      throw std::invalid_argument("overlay_alpha must be in [0,1]");
    }
    if (warmup_iterations_ < 0 || warmup_iterations_ > 1000) {
      throw std::invalid_argument("warmup_iterations must be in [0,1000]");
    }
    if (!std::isfinite(preview_fps_) || preview_fps_ <= 0.0) {
      throw std::invalid_argument("preview_fps must be finite and positive");
    }
    if (!std::isfinite(preview_scale_) || preview_scale_ <= 0.0) {
      throw std::invalid_argument("preview_scale must be finite and positive");
    }
    if (preview_window_name_.empty()) {
      throw std::invalid_argument("preview_window_name must not be empty");
    }
    if (!std::isfinite(status_log_interval_sec_) ||
      status_log_interval_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "status_log_interval_sec must be finite and positive");
    }
  }

  void warm_up()
  {
    if (warmup_iterations_ == 0) {
      return;
    }
    const auto bytes =
      static_cast<std::size_t>(model_input_width_) *
      static_cast<std::size_t>(model_input_height_) * 3U;
    std::vector<std::uint8_t> black(bytes, 0U);
    const auto stride = static_cast<std::size_t>(model_input_width_) * 3U;
    for (int iteration = 0; iteration < warmup_iterations_; ++iteration) {
      static_cast<void>(backend_->infer_bgr(black.data(), bytes, stride));
    }
    RCLCPP_INFO(
      node_.get_logger(), "TensorRT warmup completed: %d iterations",
      warmup_iterations_);
  }

  void on_image(const Image::ConstSharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_message_ = message;
      ++latest_generation_;
      ++received_interval_;
      ++received_total_;
    }
    frame_condition_.notify_one();
  }

  void validate_image(const Image & message) const
  {
    if (message.encoding != "bgr8") {
      throw std::invalid_argument(
              "Expected bgr8, received " + message.encoding);
    }
    if (message.width != static_cast<std::uint32_t>(model_input_width_) ||
      message.height != static_cast<std::uint32_t>(model_input_height_))
    {
      throw std::invalid_argument(
              "Expected " + std::to_string(model_input_width_) + "x" +
              std::to_string(model_input_height_) + " BEV, received " +
              std::to_string(message.width) + "x" +
              std::to_string(message.height));
    }
    const std::size_t minimum_step =
      static_cast<std::size_t>(model_input_width_) * 3U;
    const std::size_t required_size =
      static_cast<std::size_t>(message.step) *
      static_cast<std::size_t>(model_input_height_);
    if (message.step < minimum_step || message.data.size() < required_size) {
      throw std::invalid_argument("BEV image step or data size is invalid");
    }
  }

  cv::Mat preview_canvas(
    const cv::Mat & overlay,
    const double inference_milliseconds,
    const double correction_milliseconds,
    const double preview_fps) const
  {
    cv::Mat banner = cv::Mat::zeros(
      kBannerHeight, model_input_width_, CV_8UC3);
    const double model_fps = inference_milliseconds > 0.0 ?
      1000.0 / inference_milliseconds : 0.0;
    const std::vector<std::pair<std::string, cv::Scalar>> lines{
      {"left:B right:R", cv::Scalar(220, 220, 220)},
      {cv::format("infer %.2f ms", inference_milliseconds),
        cv::Scalar(0, 255, 255)},
      {cv::format("model %.1f FPS", model_fps), cv::Scalar(0, 255, 255)},
      {smoothing_.enabled ? cv::format("correct %.2f ms", correction_milliseconds) :
        "correct OFF", cv::Scalar(0, 255, 0)},
      {smoothing_.correction_limit_enabled ?
        cv::format("limit %.1f px", smoothing_.max_correction_px) : "limit OFF",
        cv::Scalar(220, 220, 220)},
      {cv::format("view %.1f FPS", preview_fps), cv::Scalar(255, 255, 255)}};
    for (std::size_t index = 0; index < lines.size(); ++index) {
      cv::putText(
        banner, lines[index].first,
        cv::Point(3, 12 + static_cast<int>(index) * 14),
        cv::FONT_HERSHEY_SIMPLEX, 0.28, lines[index].second, 1,
        cv::LINE_AA);
    }
    cv::Mat canvas;
    cv::vconcat(overlay, banner, canvas);
    return canvas;
  }

  bool window_quit_requested(bool & window_seen) const
  {
    const int key = cv::waitKey(1) & 0xff;
    const double visible = cv::getWindowProperty(
      preview_window_name_, cv::WND_PROP_VISIBLE);
    if (visible >= 1.0) {
      window_seen = true;
    }
    return key == 'q' || key == 'Q' || key == 27 ||
           (window_seen && visible < 1.0);
  }

  void request_shutdown()
  {
    stop_requested_.store(true, std::memory_order_release);
    frame_condition_.notify_all();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void worker_loop()
  {
    bool window_seen = false;
    if (preview_enabled_) {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        preview_window_name_,
        static_cast<int>(model_input_width_ * preview_scale_),
        static_cast<int>((model_input_height_ + kBannerHeight) * preview_scale_));
    }

    const auto minimum_period = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / preview_fps_));
    auto next_allowed_at = std::chrono::steady_clock::now();
    auto report_started_at = next_allowed_at;
    auto last_error_at = next_allowed_at - std::chrono::seconds(5);
    std::uint64_t processed_generation = 0U;
    std::uint64_t preview_interval = 0U;
    double latest_preview_fps = 0.0;
    StageStats preprocessing;
    StageStats execution;
    StageStats correction;
    StageStats postprocessing;

    try {
      while (!stop_requested_.load(std::memory_order_acquire)) {
        Image::ConstSharedPtr message;
        std::uint64_t generation = 0U;
        {
          std::unique_lock<std::mutex> lock(frame_mutex_);
          frame_condition_.wait_for(
            lock, std::chrono::milliseconds(100), [this, processed_generation]() {
              return stop_requested_.load(std::memory_order_acquire) ||
                     latest_generation_ != processed_generation;
            });
          if (stop_requested_.load(std::memory_order_acquire)) {
            break;
          }
          message = latest_message_;
          generation = latest_generation_;
        }

        auto now = std::chrono::steady_clock::now();
        if (now < next_allowed_at) {
          std::unique_lock<std::mutex> lock(frame_mutex_);
          frame_condition_.wait_until(
            lock, next_allowed_at, [this]() {
              return stop_requested_.load(std::memory_order_acquire);
            });
          if (stop_requested_.load(std::memory_order_acquire)) {
            break;
          }
          message = latest_message_;
          generation = latest_generation_;
        }

        if (!message || generation == processed_generation) {
          if (preview_enabled_ && window_quit_requested(window_seen)) {
            request_shutdown();
          }
          continue;
        }
        if (processed_generation > 0U && generation > processed_generation + 1U) {
          skipped_total_ += generation - processed_generation - 1U;
        }
        processed_generation = generation;

        LaneInferenceTiming timing;
        try {
          validate_image(*message);
          timing = backend_->infer_bgr(
            message->data.data(), message->data.size(), message->step);
        } catch (const std::exception & exception) {
          now = std::chrono::steady_clock::now();
          if (now - last_error_at >= std::chrono::seconds(5)) {
            RCLCPP_ERROR(
              node_.get_logger(), "BEV inference frame rejected: %s",
              exception.what());
            last_error_at = now;
          }
          continue;
        }

        ++processed_total_;
        ++preview_interval;
        preprocessing.record(timing.preprocessing_nanoseconds);
        execution.record(timing.execution_nanoseconds);
        correction.record(timing.correction_nanoseconds);
        postprocessing.record(timing.postprocessing_nanoseconds);

        if (preview_enabled_) {
          cv::Mat overlay(
            model_input_height_, model_input_width_, CV_8UC3,
            const_cast<std::uint8_t *>(backend_->preview_bgr_data()));
          const cv::Mat canvas = preview_canvas(
            overlay,
            nanoseconds_to_milliseconds(timing.execution_nanoseconds),
            nanoseconds_to_milliseconds(timing.correction_nanoseconds),
            latest_preview_fps);
          cv::imshow(preview_window_name_, canvas);
          if (window_quit_requested(window_seen)) {
            request_shutdown();
            break;
          }
        }

        now = std::chrono::steady_clock::now();
        const double report_elapsed =
          std::chrono::duration<double>(now - report_started_at).count();
        if (report_elapsed >= status_log_interval_sec_) {
          std::uint64_t received = 0U;
          {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            received = received_interval_;
            received_interval_ = 0U;
          }
          latest_preview_fps =
            static_cast<double>(preview_interval) / report_elapsed;
          RCLCPP_INFO(
            node_.get_logger(),
            "FPS: input=%.1f, preview=%.1f/%.1f | AVG/MAX ms: "
            "H2D+preprocess=%.3f/%.3f, pure-inference=%.3f/%.3f, "
            "correction=%.3f/%.3f, postprocess+D2H=%.3f/%.3f | skipped=%llu, processed=%llu",
            static_cast<double>(received) / report_elapsed,
            latest_preview_fps, preview_fps_,
            preprocessing.average_milliseconds(),
            preprocessing.maximum_milliseconds(),
            execution.average_milliseconds(), execution.maximum_milliseconds(),
            correction.average_milliseconds(), correction.maximum_milliseconds(),
            postprocessing.average_milliseconds(),
            postprocessing.maximum_milliseconds(),
            static_cast<unsigned long long>(skipped_total_),
            static_cast<unsigned long long>(processed_total_));
          preprocessing.reset();
          execution.reset();
          correction.reset();
          postprocessing.reset();
          preview_interval = 0U;
          report_started_at = now;
        }

        next_allowed_at += minimum_period;
        if (next_allowed_at < now - minimum_period) {
          next_allowed_at = now + minimum_period;
        }
      }
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(
        node_.get_logger(), "Lane preview worker failed: %s", exception.what());
      request_shutdown();
    }

    if (preview_enabled_) {
      try {
        cv::destroyWindow(preview_window_name_);
      } catch (const cv::Exception &) {
      }
    }
  }

  void stop()
  {
    stop_requested_.store(true, std::memory_order_release);
    frame_condition_.notify_all();
    if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) {
      worker_.join();
    }
  }

  LineDetactorNode & node_;
  std::string input_topic_;
  std::string model_path_;
  std::string engine_cache_path_;
  int tensorrt_workspace_size_mb_{1024};
  int model_input_width_{kDefaultInputWidth};
  int model_input_height_{kDefaultInputHeight};
  float mask_threshold_{0.5F};
  float overlay_alpha_{0.75F};
  LaneSmoothingConfig smoothing_;
  int warmup_iterations_{10};
  bool preview_enabled_{true};
  double preview_fps_{30.0};
  double preview_scale_{2.0};
  std::string preview_window_name_;
  double status_log_interval_sec_{1.0};

  std::unique_ptr<TensorRtLaneBackend> backend_;
  rclcpp::Subscription<Image>::SharedPtr subscription_;
  std::mutex frame_mutex_;
  std::condition_variable frame_condition_;
  Image::ConstSharedPtr latest_message_;
  std::uint64_t latest_generation_{0U};
  std::uint64_t received_interval_{0U};
  std::uint64_t received_total_{0U};
  std::uint64_t processed_total_{0U};
  std::uint64_t skipped_total_{0U};
  std::atomic<bool> stop_requested_{false};
  std::thread worker_;
};

LineDetactorNode::LineDetactorNode(const rclcpp::NodeOptions & options)
: Node("line_detactor", options), impl_(std::make_unique<Impl>(*this))
{
}

LineDetactorNode::~LineDetactorNode() = default;

}  // namespace line_detactor

RCLCPP_COMPONENTS_REGISTER_NODE(line_detactor::LineDetactorNode)
