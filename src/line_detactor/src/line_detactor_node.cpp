#include "line_detactor/line_detactor_node.hpp"

#include "line_detactor/tensorrt_lane_backend.hpp"
#include "line_detactor/lane_connector.hpp"
#include "line_detactor/msg/lane_result.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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
constexpr int kBannerHeight = 100;
constexpr char kModelFilename[] =
  "fast_scnn_stop_line_120x300_batch_1.onnx";

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
      throw std::runtime_error("Three-channel ONNX model not found: " + model_path_ +
        ". Export the trained checkpoint with tools/export_stop_line_onnx.py first.");
    }

    backend_ = std::make_unique<TensorRtLaneBackend>(
      model_path_, engine_cache_path_, engine_precision_, model_input_width_, model_input_height_,
      static_cast<std::size_t>(tensorrt_workspace_size_mb_) * 1024U * 1024U,
      mask_threshold_, overlay_alpha_, connection_.enabled);
    warm_up();

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();
    if (connection_.enabled && result_publish_enabled_) {
      result_publisher_ = node_.create_publisher<line_detactor::msg::LaneResult>(
        result_topic_, image_qos);
      result_image_publisher_ = node_.create_publisher<Image>(result_image_topic_, image_qos);
    }
    subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
      input_topic_, image_qos,
      std::bind(&Impl::on_image, this, std::placeholders::_1));

    try {
      worker_ = std::thread(&Impl::worker_loop, this);
      if (preview_enabled_) {
        preview_worker_ = std::thread(&Impl::preview_loop, this);
      }
    } catch (...) {
      stop();
      throw;
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "Line detector ready: input=%s (bgr8 %dx%d), model=%s, backend="
      "TensorRT %s, threshold=%.3f, preview=%s @ %.1f FPS",
      input_topic_.c_str(), model_input_width_, model_input_height_,
      model_path_.c_str(), engine_precision_.c_str(), static_cast<double>(mask_threshold_),
      preview_enabled_ ? "on" : "off", preview_fps_);
    RCLCPP_INFO(
      node_.get_logger(),
      "GPU path: pinned BGR8 H2D -> CUDA RGB FP32 NCHW -> TensorRT -> "
      "%s; engine cache=%s",
      connection_.enabled ? "CUDA labels D2H (raw overlay skipped)" : "CUDA overlay BGR8 D2H",
      backend_->engine_cache_path().c_str());
    RCLCPP_INFO(node_.get_logger(), "Preview content=%s",
      connection_.enabled && preview_result_only_enabled_ ? "results only" : "camera overlay");
    RCLCPP_INFO(node_.get_logger(),
      "Border interpolation=%s, result=%dx%d (padding=%d each side), publish=%s, topic=%s",
      connection_.enabled ? "on" : "off", result_width(), model_input_height_,
      connection_.padding_px, result_publisher_ ? "on" : "off", result_topic_.c_str());
    RCLCPP_INFO(node_.get_logger(),
      "Yellow centerline=%s, lane width=%.2fm, BEV=%.2fx%.2fm, smoothing=%s strength=%.2f window=%.2fm",
      centerline_.enabled && connection_.enabled ? "on" : "off", centerline_.lane_width_m,
      centerline_.bev_width_m, centerline_.bev_height_m, centerline_.smoothing_enabled ? "on" : "off",
      centerline_.smoothing_strength, centerline_.smoothing_window_m);
    RCLCPP_INFO(node_.get_logger(),
      "Corner outer reference=%s weight=%.2f turn window=%.2fm min support=%.2fm "
      "outward offset=%.3fm entry distance=%.2fm",
      centerline_.corner_outer_enabled ? "on" : "off", centerline_.corner_outer_weight,
      centerline_.corner_outer_window_m, centerline_.corner_outer_min_length_m,
      centerline_.corner_outward_offset_m, centerline_.corner_entry_distance_m);
  }

  ~Impl()
  {
    stop();
  }

private:
  using Image = sensor_msgs::msg::Image;
  using SteadyClock = std::chrono::steady_clock;

  struct PreviewFrame
  {
    Image::ConstSharedPtr input;
    LaneConnectionResult result;
    cv::Mat raw;
    LaneInferenceTiming timing;
    std::uint64_t connection_nanoseconds{0U};
    std::uint64_t generation{0U};
  };

  void read_parameters()
  {
    input_topic_ = node_.declare_parameter<std::string>(
      "input_topic", "/camera/image_bev");
    model_path_ = node_.declare_parameter<std::string>(
      "model_path", default_model_path());
    engine_cache_path_ = node_.declare_parameter<std::string>(
      "engine_cache_path", "");
    engine_precision_ = node_.declare_parameter<std::string>(
      "engine_precision", "fp32");
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
    connection_.enabled = node_.declare_parameter<bool>(
      "connection_enabled", true);
    connection_.padding_px = node_.declare_parameter<int>(
      "result_padding_px", 30);
    connection_.min_component_area_px = node_.declare_parameter<int>(
      "connection_min_component_area_px", 8);
    connection_.skeleton_downsample_factor = node_.declare_parameter<int>(
      "connection_skeleton_downsample_factor", 1);
    connection_.min_fragment_length_px = node_.declare_parameter<double>(
      "connection_min_fragment_length_px", 8.0);
    connection_.max_fragments = node_.declare_parameter<int>(
      "connection_max_fragments", 24);
    connection_.tangent_window_px = node_.declare_parameter<double>(
      "connection_tangent_window_px", 8.0);
    connection_.max_gap_px = node_.declare_parameter<double>(
      "connection_max_gap_px", 80.0);
    connection_.corridor_half_width_px = node_.declare_parameter<double>(
      "connection_corridor_half_width_px", 4.0);
    connection_.direction_tolerance_deg = node_.declare_parameter<double>(
      "connection_direction_tolerance_deg", 20.0);
    connection_.max_turn_deg = node_.declare_parameter<double>(
      "connection_max_turn_deg", 180.0);
    connection_.max_curvature_per_px = node_.declare_parameter<double>(
      "connection_max_curvature_per_px", 0.12);
    connection_.max_arc_ratio = node_.declare_parameter<double>(
      "connection_max_arc_ratio", 1.8);
    connection_.border_endpoint_distance_px = node_.declare_parameter<double>(
      "connection_border_endpoint_distance_px", 6.0);
    connection_.line_width_px = node_.declare_parameter<int>(
      "result_line_width_px", 2);
    centerline_.enabled = node_.declare_parameter<bool>("centerline_enabled", true);
    centerline_.lane_width_m = node_.declare_parameter<double>("centerline_lane_width_m", 0.65);
    centerline_.bev_width_m = node_.declare_parameter<double>("centerline_bev_width_m", 1.2);
    centerline_.bev_height_m = node_.declare_parameter<double>("centerline_bev_height_m", 3.0);
    centerline_.sample_spacing_m = node_.declare_parameter<double>(
      "centerline_sample_spacing_m", 0.015);
    centerline_.output_spacing_m = node_.declare_parameter<double>(
      "centerline_output_spacing_m", 0.01);
    centerline_.clearance_check_spacing_m = node_.declare_parameter<double>(
      "centerline_clearance_check_spacing_m", 0.005);
    centerline_.min_fragment_length_m = node_.declare_parameter<double>(
      "centerline_min_fragment_length_m", 0.08);
    centerline_.tangent_window_m = node_.declare_parameter<double>(
      "centerline_tangent_window_m", 0.06);
    centerline_.width_tolerance_m = node_.declare_parameter<double>(
      "centerline_width_tolerance_m", 0.12);
    centerline_.pair_along_tolerance_m = node_.declare_parameter<double>(
      "centerline_pair_along_tolerance_m", 0.055);
    centerline_.pair_heading_tolerance_deg = node_.declare_parameter<double>(
      "centerline_pair_heading_tolerance_deg", 40.0);
    centerline_.max_gap_m = node_.declare_parameter<double>("centerline_max_gap_m", 0.12);
    centerline_.max_start_distance_m = node_.declare_parameter<double>(
      "centerline_max_start_distance_m", 0.65);
    centerline_.min_clearance_m = node_.declare_parameter<double>(
      "centerline_min_clearance_m", 0.16);
    centerline_.outside_margin_m = node_.declare_parameter<double>(
      "centerline_outside_margin_m", 0.12);
    centerline_.max_samples = node_.declare_parameter<int>("centerline_max_samples", 2000);
    centerline_.line_width_px = node_.declare_parameter<int>("centerline_line_width_px", 2);
    centerline_.corner_outer_enabled = node_.declare_parameter<bool>(
      "centerline_corner_outer_enabled", true);
    centerline_.corner_outer_weight = node_.declare_parameter<double>(
      "centerline_corner_outer_weight", 0.85);
    centerline_.corner_outward_offset_m = node_.declare_parameter<double>(
      "centerline_corner_outward_offset_m", 0.05);
    centerline_.corner_entry_distance_m = node_.declare_parameter<double>(
      "centerline_corner_entry_distance_m", 0.40);
    centerline_.corner_outer_window_m = node_.declare_parameter<double>(
      "centerline_corner_outer_window_m", 0.60);
    centerline_.corner_outer_tangent_window_m = node_.declare_parameter<double>(
      "centerline_corner_outer_tangent_window_m", 0.15);
    centerline_.corner_outer_min_length_m = node_.declare_parameter<double>(
      "centerline_corner_outer_min_length_m", 0.30);
    centerline_.corner_outer_min_turn_deg = node_.declare_parameter<double>(
      "centerline_corner_outer_min_turn_deg", 8.0);
    centerline_.corner_outer_full_turn_deg = node_.declare_parameter<double>(
      "centerline_corner_outer_full_turn_deg", 25.0);
    centerline_.smoothing_enabled = node_.declare_parameter<bool>(
      "centerline_smoothing_enabled", true);
    centerline_.smoothing_sigma_m = node_.declare_parameter<double>(
      "centerline_smoothing_sigma_m", 0.04);
    centerline_.smoothing_window_m = node_.declare_parameter<double>(
      "centerline_smoothing_window_m", 0.65);
    centerline_.smoothing_max_shift_m = node_.declare_parameter<double>(
      "centerline_smoothing_max_shift_m", 0.03);
    centerline_.smoothing_strength = node_.declare_parameter<double>(
      "centerline_smoothing_strength", 1.0);
    centerline_.straight_turn_deg = node_.declare_parameter<double>(
      "centerline_straight_turn_deg", 12.0);
    centerline_.corner_turn_deg = node_.declare_parameter<double>(
      "centerline_corner_turn_deg", 35.0);
    centerline_.turn_window_m = node_.declare_parameter<double>("centerline_turn_window_m", 0.30);
    result_publish_enabled_ = node_.declare_parameter<bool>("result_publish_enabled", true);
    result_topic_ = node_.declare_parameter<std::string>("result_topic", "/line_detactor/result");
    result_image_topic_ = node_.declare_parameter<std::string>(
      "result_image_topic", "/line_detactor/result_image");
    warmup_iterations_ = node_.declare_parameter<int>(
      "warmup_iterations", 10);
    preview_enabled_ = node_.declare_parameter<bool>(
      "preview_enabled", true);
    preview_result_only_enabled_ = node_.declare_parameter<bool>(
      "preview_result_only_enabled", true);
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
    validate_lane_connection(connection_);
    validate_centerline(centerline_);
    if (centerline_.enabled && !connection_.enabled) {
      RCLCPP_WARN(node_.get_logger(),
        "centerline_enabled requires connection_enabled; raw preview mode has no centerline");
    }
    if (result_topic_.empty() || result_image_topic_.empty() || result_topic_ == result_image_topic_ ||
      result_topic_ == input_topic_ || result_image_topic_ == input_topic_)
    {
      throw std::invalid_argument("Result topics must be nonempty, distinct and different from input");
    }
    if (input_topic_.empty()) {
      throw std::invalid_argument("input_topic must not be empty");
    }
    if (model_path_.empty()) {
      throw std::invalid_argument("model_path must not be empty");
    }
    if (engine_precision_ != "fp32" && engine_precision_ != "fp16" &&
      engine_precision_ != "int8")
    {
      throw std::invalid_argument("engine_precision must be fp32, fp16 or int8");
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
    const auto received_at = SteadyClock::now();
    const auto received_stamp =
      static_cast<builtin_interfaces::msg::Time>(node_.get_clock()->now());
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_message_ = message;
      latest_input_received_at_ = received_at;
      latest_input_received_stamp_ = received_stamp;
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

  int result_width() const
  {
    return model_input_width_ + 2 * connection_.padding_px;
  }

  Image image_message(const cv::Mat & image, const Image & input, const std::string & encoding) const
  {
    Image message;
    message.header = input.header;
    message.height = static_cast<std::uint32_t>(image.rows);
    message.width = static_cast<std::uint32_t>(image.cols);
    message.encoding = encoding;
    message.is_bigendian = false;
    message.step = static_cast<std::uint32_t>(image.cols * image.elemSize());
    message.data.resize(static_cast<std::size_t>(message.step) * image.rows);
    for (int y = 0; y < image.rows; ++y) {
      std::memcpy(message.data.data() + static_cast<std::size_t>(y) * message.step,
        image.ptr(y), message.step);
    }
    return message;
  }

  void publish_result(
    const LaneConnectionResult & result,
    const Image & input,
    const builtin_interfaces::msg::Time & input_received_stamp,
    const SteadyClock::time_point input_received_at,
    const SteadyClock::time_point processing_started_at,
    const LaneInferenceTiming & timing,
    const std::uint64_t lane_geometry_nanoseconds,
    const std::uint64_t detector_sequence)
  {
    if (!result_publisher_) {return;}
    const auto message_build_started_at = SteadyClock::now();
    line_detactor::msg::LaneResult message;
    message.header = input.header;
    message.detector_input_received_stamp = input_received_stamp;
    message.engine_precision = engine_precision_;
    message.detector_sequence = detector_sequence;
    message.detector_queue_nanoseconds = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        processing_started_at - input_received_at).count());
    message.h2d_preprocess_nanoseconds = timing.preprocessing_nanoseconds;
    message.pure_inference_nanoseconds = timing.execution_nanoseconds;
    message.label_export_nanoseconds = timing.label_export_nanoseconds;
    message.backend_postprocess_nanoseconds = timing.postprocessing_nanoseconds;
    message.lane_geometry_nanoseconds = lane_geometry_nanoseconds;
    message.state = result.state;
    message.processing_mode = line_detactor::msg::LaneResult::BORDER_ONLY;
    message.source_width = input.width;
    message.source_height = input.height;
    message.padding_left = connection_.padding_px;
    message.padding_right = connection_.padding_px;
    message.centerline_valid = result.centerline.points.size() >= 2U;
    message.centerline_sample_limit_reached = result.centerline.sample_limit_reached;
    message.centerline_support = result.centerline.support;
    message.centerline_bev_width_m = static_cast<float>(centerline_.bev_width_m);
    message.centerline_bev_height_m = static_cast<float>(centerline_.bev_height_m);
    for (const auto & point : result.centerline.points) {
      geometry_msgs::msg::Point32 output;
      output.x = point.x;
      output.y = point.y;
      output.z = 0.0F;
      message.centerline_points.push_back(output);
    }
    message.stop_line_present = result.stop_line_present;
    const auto result_ready_at = SteadyClock::now();
    message.result_message_build_nanoseconds = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        result_ready_at - message_build_started_at).count());
    message.detector_total_compute_nanoseconds = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        result_ready_at - processing_started_at).count());
    message.detector_result_ready_stamp =
      static_cast<builtin_interfaces::msg::Time>(node_.get_clock()->now());
    result_publisher_->publish(std::move(message));
    if (!result.image.empty() &&
      (result_image_publisher_->get_subscription_count() > 0U ||
      result_image_publisher_->get_intra_process_subscription_count() > 0U))
    {
      result_image_publisher_->publish(image_message(result.image, input, "bgr8"));
    }
  }

  cv::Mat result_overlay(const LaneConnectionResult & result, const Image & input) const
  {
    cv::Mat source(model_input_height_, model_input_width_, CV_8UC3,
      const_cast<std::uint8_t *>(input.data.data()), input.step);
    cv::Mat overlay;
    cv::copyMakeBorder(source, overlay, 0, 0, connection_.padding_px, connection_.padding_px,
      cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::Mat blended;
    cv::addWeighted(overlay, 1.0 - overlay_alpha_, result.image, overlay_alpha_, 0.0, blended);
    cv::Mat visible;
    cv::bitwise_or(result.labels, result.stop_line_mask, visible);
    blended.copyTo(overlay, visible);
    // Solid yellow centerline stays visible regardless of lane overlay alpha.
    overlay.setTo(cv::Scalar(0, 255, 255), result.centerline.mask);
    return overlay;
  }

  cv::Mat preview_canvas(
    const cv::Mat & overlay,
    const double inference_milliseconds,
    const double correction_milliseconds,
    const double connection_milliseconds,
    const std::uint8_t state,
    const double preview_fps) const
  {
    cv::Mat banner = cv::Mat::zeros(
      kBannerHeight, overlay.cols, CV_8UC3);
    const double model_fps = inference_milliseconds > 0.0 ?
      1000.0 / inference_milliseconds : 0.0;
    const std::vector<std::pair<std::string, cv::Scalar>> lines{
      {"left:B right:R stop:G", cv::Scalar(220, 220, 220)},
      {cv::format("infer %.2f ms", inference_milliseconds),
        cv::Scalar(0, 255, 255)},
      {cv::format("model %.1f FPS", model_fps), cv::Scalar(0, 255, 255)},
      {connection_.enabled ?
        cv::format("correct %.2f ms", correction_milliseconds) :
        "correct OFF", cv::Scalar(0, 255, 0)},
      {connection_.enabled ? cv::format("connect %.2f ms", connection_milliseconds) :
        "connect OFF", cv::Scalar(0, 255, 0)},
      {connection_.enabled ? std::string("lanes ") +
        (state == 3U ? "BOTH" : state == 1U ? "LEFT" : state == 2U ? "RIGHT" : "NONE") :
        "lanes RAW", cv::Scalar(220, 220, 220)},
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
    {
      // Synchronize the stop predicate with both waits so shutdown cannot miss a wakeup.
      std::scoped_lock lock(frame_mutex_, preview_mutex_);
      stop_requested_.store(true, std::memory_order_release);
    }
    frame_condition_.notify_all();
    preview_condition_.notify_all();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void worker_loop()
  {
    auto report_started_at = std::chrono::steady_clock::now();
    auto last_error_at = report_started_at - std::chrono::seconds(5);
    std::uint64_t processed_generation = 0U;
    std::uint64_t processed_interval = 0U;
    StageStats preprocessing;
    StageStats execution;
    StageStats correction;
    StageStats connection;
    StageStats postprocessing;

    try {
      while (!stop_requested_.load(std::memory_order_acquire)) {
        Image::ConstSharedPtr message;
        std::uint64_t generation = 0U;
        SteadyClock::time_point input_received_at;
        builtin_interfaces::msg::Time input_received_stamp;
        {
          std::unique_lock<std::mutex> lock(frame_mutex_);
          frame_condition_.wait(
            lock, [this, processed_generation]() {
              return stop_requested_.load(std::memory_order_acquire) ||
                     latest_generation_ != processed_generation;
            });
          if (stop_requested_.load(std::memory_order_acquire)) {
            break;
          }
          message = latest_message_;
          generation = latest_generation_;
          input_received_at = latest_input_received_at_;
          input_received_stamp = latest_input_received_stamp_;
        }

        auto now = std::chrono::steady_clock::now();
        if (!message || generation == processed_generation) {
          continue;
        }
        if (processed_generation > 0U && generation > processed_generation + 1U) {
          skipped_total_ += generation - processed_generation - 1U;
        }
        processed_generation = generation;
        const auto processing_started_at = SteadyClock::now();

        LaneInferenceTiming timing;
        LaneConnectionResult result;
        std::uint64_t connection_nanoseconds = 0U;
        try {
          validate_image(*message);
          timing = backend_->infer_bgr(
            message->data.data(), message->data.size(), message->step);
          if (connection_.enabled) {
            const auto started = std::chrono::steady_clock::now();
            const bool render_result = preview_enabled_ ||
              (result_image_publisher_ &&
              (result_image_publisher_->get_subscription_count() > 0U ||
              result_image_publisher_->get_intra_process_subscription_count() > 0U));
            cv::Mat labels(model_input_height_, model_input_width_, CV_8UC1,
              const_cast<std::uint8_t *>(backend_->label_data()));
            result = connect_lane_fragments(labels, connection_, render_result);
            // Stop lines never enter the left/right connector. Retain overlaps
            // in independent masks, painting green only in the display image.
            cv::Mat stop_mask(model_input_height_, model_input_width_, CV_8UC1,
              const_cast<std::uint8_t *>(backend_->stop_line_mask_data()));
            result.stop_line_present = cv::countNonZero(stop_mask) > 0;
            if (render_result) {
              cv::copyMakeBorder(stop_mask, result.stop_line_mask, 0, 0,
                connection_.padding_px, connection_.padding_px,
                cv::BORDER_CONSTANT, cv::Scalar(0));
              result.image.setTo(cv::Scalar(0, 255, 0), result.stop_line_mask);
            }
            result.centerline = generate_centerline(
              result.labels, result.observed_paths,
              model_input_width_, connection_.padding_px, centerline_, render_result);
            if (render_result) {
              result.image.setTo(cv::Scalar(0, 255, 255), result.centerline.mask);
            }
            if (result.centerline.sample_limit_reached) {
              RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                "Centerline sample budget exceeded; publishing an empty centerline for this frame");
            }
            connection_nanoseconds = static_cast<std::uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - started).count());
            timing.correction_nanoseconds += timing.label_export_nanoseconds + connection_nanoseconds;
            publish_result(
              result, *message, input_received_stamp, input_received_at,
              processing_started_at, timing, connection_nanoseconds,
              generation);
          }
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
        ++processed_interval;
        preprocessing.record(timing.preprocessing_nanoseconds);
        execution.record(timing.execution_nanoseconds);
        correction.record(timing.correction_nanoseconds);
        connection.record(connection_nanoseconds);
        postprocessing.record(timing.postprocessing_nanoseconds);

        if (preview_enabled_) {
          auto snapshot = std::make_shared<PreviewFrame>();
          if (connection_.enabled && !preview_result_only_enabled_) {
            snapshot->input = message;
          }
          snapshot->result = std::move(result);
          snapshot->timing = timing;
          snapshot->connection_nanoseconds = connection_nanoseconds;
          snapshot->generation = generation;
          if (!connection_.enabled) {
            // Backend buffers are reused by the next inference. The GUI owns this copy.
            const cv::Mat raw(model_input_height_, model_input_width_, CV_8UC3,
              const_cast<std::uint8_t *>(backend_->preview_bgr_data()));
            snapshot->raw = raw.clone();
          }
          {
            std::lock_guard<std::mutex> lock(preview_mutex_);
            latest_preview_ = std::move(snapshot);
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
          const double preview_fps = static_cast<double>(
            previewed_interval_.exchange(0U, std::memory_order_relaxed)) / report_elapsed;
          RCLCPP_INFO(
            node_.get_logger(),
            "FPS: input=%.1f, processed=%.1f, preview=%.1f/%.1f | AVG/MAX ms: "
            "H2D+preprocess=%.3f/%.3f, pure-inference=%.3f/%.3f, "
            "correction=%.3f/%.3f, connect=%.3f/%.3f, postprocess+D2H=%.3f/%.3f | "
            "skipped=%llu, processed=%llu",
            static_cast<double>(received) / report_elapsed,
            static_cast<double>(processed_interval) / report_elapsed,
            preview_fps, preview_fps_,
            preprocessing.average_milliseconds(),
            preprocessing.maximum_milliseconds(),
            execution.average_milliseconds(), execution.maximum_milliseconds(),
            correction.average_milliseconds(), correction.maximum_milliseconds(),
            connection.average_milliseconds(), connection.maximum_milliseconds(),
            postprocessing.average_milliseconds(),
            postprocessing.maximum_milliseconds(),
            static_cast<unsigned long long>(skipped_total_),
            static_cast<unsigned long long>(processed_total_));
          preprocessing.reset();
          execution.reset();
          correction.reset();
          connection.reset();
          postprocessing.reset();
          processed_interval = 0U;
          report_started_at = now;
        }
      }
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(
        node_.get_logger(), "Lane inference worker failed: %s", exception.what());
      request_shutdown();
    }
  }

  // All HighGUI calls live here. Slow rendering never blocks inference or result publication.
  void preview_loop()
  {
    bool window_seen = false;
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        preview_window_name_,
        static_cast<int>(result_width() * preview_scale_),
        static_cast<int>((model_input_height_ + kBannerHeight) * preview_scale_));
      const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / preview_fps_));
      auto next_preview_at = std::chrono::steady_clock::now();
      auto report_started_at = next_preview_at;
      std::uint64_t displayed_generation = 0U;
      std::uint64_t displayed_count = 0U;
      double display_fps = 0.0;
      while (!stop_requested_.load(std::memory_order_acquire)) {
        std::shared_ptr<const PreviewFrame> frame;
        {
          std::unique_lock<std::mutex> lock(preview_mutex_);
          preview_condition_.wait_until(lock, next_preview_at, [this]() {
            return stop_requested_.load(std::memory_order_acquire);
          });
          if (stop_requested_.load(std::memory_order_acquire)) {break;}
          frame = latest_preview_;
        }
        if (frame && frame->generation != displayed_generation) {
          cv::Mat overlay;
          if (connection_.enabled) {
            overlay = preview_result_only_enabled_ ? frame->result.image :
              result_overlay(frame->result, *frame->input);
          } else {
            cv::copyMakeBorder(frame->raw, overlay, 0, 0,
              connection_.padding_px, connection_.padding_px,
              cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
          }
          const cv::Mat canvas = preview_canvas(
            overlay,
            nanoseconds_to_milliseconds(frame->timing.execution_nanoseconds),
            nanoseconds_to_milliseconds(frame->timing.correction_nanoseconds),
            nanoseconds_to_milliseconds(frame->connection_nanoseconds),
            frame->result.state, display_fps);
          cv::imshow(preview_window_name_, canvas);
          displayed_generation = frame->generation;
          ++displayed_count;
          previewed_interval_.fetch_add(1U, std::memory_order_relaxed);
        }
        if (window_quit_requested(window_seen)) {
          request_shutdown();
          break;
        }
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - report_started_at).count();
        if (elapsed >= status_log_interval_sec_) {
          display_fps = static_cast<double>(displayed_count) / elapsed;
          displayed_count = 0U;
          report_started_at = now;
        }
        next_preview_at += period;
        if (next_preview_at < now) {next_preview_at = now + period;}
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(node_.get_logger(), "Lane GUI failed: %s", exception.what());
      request_shutdown();
    }
    try {
      cv::destroyWindow(preview_window_name_);
    } catch (const cv::Exception &) {
    }
  }

  void stop()
  {
    {
      // Synchronize the stop predicate with both waits so shutdown cannot miss a wakeup.
      std::scoped_lock lock(frame_mutex_, preview_mutex_);
      stop_requested_.store(true, std::memory_order_release);
    }
    frame_condition_.notify_all();
    preview_condition_.notify_all();
    if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) {
      worker_.join();
    }
    if (preview_worker_.joinable() && preview_worker_.get_id() != std::this_thread::get_id()) {
      preview_worker_.join();
    }
  }

  LineDetactorNode & node_;
  std::string input_topic_;
  std::string model_path_;
  std::string engine_cache_path_;
  std::string engine_precision_{"fp32"};
  int tensorrt_workspace_size_mb_{1024};
  int model_input_width_{kDefaultInputWidth};
  int model_input_height_{kDefaultInputHeight};
  float mask_threshold_{0.5F};
  float overlay_alpha_{0.75F};
  LaneConnectionConfig connection_;
  CenterlineConfig centerline_;
  bool result_publish_enabled_{true};
  std::string result_topic_;
  std::string result_image_topic_;
  rclcpp::Publisher<line_detactor::msg::LaneResult>::SharedPtr result_publisher_;
  rclcpp::Publisher<Image>::SharedPtr result_image_publisher_;
  int warmup_iterations_{10};
  bool preview_enabled_{true};
  bool preview_result_only_enabled_{true};
  double preview_fps_{30.0};
  double preview_scale_{2.0};
  std::string preview_window_name_;
  double status_log_interval_sec_{1.0};

  std::unique_ptr<TensorRtLaneBackend> backend_;
  rclcpp::Subscription<Image>::SharedPtr subscription_;
  std::mutex frame_mutex_;
  std::condition_variable frame_condition_;
  Image::ConstSharedPtr latest_message_;
  SteadyClock::time_point latest_input_received_at_{};
  builtin_interfaces::msg::Time latest_input_received_stamp_;
  std::uint64_t latest_generation_{0U};
  std::uint64_t received_interval_{0U};
  std::uint64_t received_total_{0U};
  std::uint64_t processed_total_{0U};
  std::uint64_t skipped_total_{0U};
  std::atomic<bool> stop_requested_{false};
  std::thread worker_;
  std::mutex preview_mutex_;
  std::condition_variable preview_condition_;
  std::shared_ptr<const PreviewFrame> latest_preview_;
  std::atomic<std::uint64_t> previewed_interval_{0U};
  std::thread preview_worker_;
};

LineDetactorNode::LineDetactorNode(const rclcpp::NodeOptions & options)
: Node("line_detactor", options), impl_(std::make_unique<Impl>(*this))
{
}

LineDetactorNode::~LineDetactorNode() = default;

}  // namespace line_detactor

RCLCPP_COMPONENTS_REGISTER_NODE(line_detactor::LineDetactorNode)
