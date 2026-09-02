#include "ir_camera_driver/cuda_center_reprojector.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "depthai/depthai.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ir_camera_driver
{

using namespace std::chrono_literals;

namespace
{

constexpr std::uint32_t kOv9282FullWidth = 1280U;
constexpr std::uint32_t kOv9282FullHeight = 800U;
constexpr double kOv9282MaximumFullResolutionFps = 129.0;
constexpr double kRvc2MaximumFastStereoFullResolutionFps = 50.0;

std::string uppercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {
      return static_cast<char>(std::toupper(character));
    });
  return value;
}

bool graphicalDisplayAvailable()
{
#if defined(__linux__)
  return std::getenv("DISPLAY") != nullptr ||
         std::getenv("WAYLAND_DISPLAY") != nullptr;
#else
  return true;
#endif
}

const char * usbSpeedName(const dai::UsbSpeed speed)
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

}  // namespace

class IrCameraDriverNode : public rclcpp::Node
{
public:
  IrCameraDriverNode()
  : Node("ir_camera_driver")
  {
    readParameters();
    if (!graphicalDisplayAvailable()) {
      throw std::runtime_error(
              "IR preview requires DISPLAY or WAYLAND_DISPLAY");
    }

    try {
      startOak();
      started_at_ = std::chrono::steady_clock::now();
      last_status_at_ = started_at_;
      status_timer_ = create_wall_timer(
        std::chrono::duration<double>(status_log_interval_sec_),
        std::bind(&IrCameraDriverNode::reportStatus, this));
      capture_thread_ = std::thread(&IrCameraDriverNode::captureLoop, this);
      preview_thread_ = std::thread(&IrCameraDriverNode::previewLoop, this);
    } catch (...) {
      stop();
      throw;
    }
  }

  ~IrCameraDriverNode() override
  {
    stop();
  }

private:
  enum class SelectedCamera
  {
    LEFT,
    RIGHT
  };

  struct StereoSnapshot
  {
    std::shared_ptr<dai::ImgFrame> left;
    std::shared_ptr<dai::ImgFrame> right;
    std::shared_ptr<dai::ImgFrame> disparity;
    std::uint64_t generation;
    std::int64_t sequence;
  };

  struct SingleSnapshot
  {
    std::shared_ptr<dai::ImgFrame> frame;
    std::uint64_t generation;
    std::int64_t sequence;
  };

  static SelectedCamera parseSelectedCamera(const std::string & value)
  {
    const auto normalized = uppercase(value);
    if (normalized == "LEFT" || normalized == "CAM_B") {
      return SelectedCamera::LEFT;
    }
    if (normalized == "RIGHT" || normalized == "CAM_C") {
      return SelectedCamera::RIGHT;
    }
    throw std::invalid_argument(
            "selected_camera must be LEFT/CAM_B or RIGHT/CAM_C");
  }

  dai::CameraBoardSocket selectedSocket() const
  {
    return selected_camera_ == SelectedCamera::LEFT ?
           dai::CameraBoardSocket::CAM_B : dai::CameraBoardSocket::CAM_C;
  }

  const char * selectedCameraLabel() const
  {
    return selected_camera_ == SelectedCamera::LEFT ?
           "LEFT/CAM_B" : "RIGHT/CAM_C";
  }

  double requestedFps() const
  {
    return reprojection_enabled_ ? reprojection_fps_ : single_camera_fps_;
  }

  void readParameters()
  {
    reprojection_enabled_ =
      declare_parameter<bool>("reprojection_enabled", true);
    selected_camera_name_ =
      declare_parameter<std::string>("selected_camera", "LEFT");
    virtual_camera_position_ratio_ = declare_parameter<double>(
      "virtual_camera_position_ratio", 0.5);

    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 800);
    reprojection_fps_ =
      declare_parameter<double>("reprojection_fps", 50.0);
    single_camera_fps_ =
      declare_parameter<double>("single_camera_fps", 129.0);
    undistort_single_camera_ =
      declare_parameter<bool>("undistort_single_camera", true);
    sync_threshold_ms_ =
      declare_parameter<double>("sync_threshold_ms", 2.0);

    ir_enabled_at_start_ = declare_parameter<bool>("ir_enabled", true);
    ir_dot_projector_intensity_ = declare_parameter<double>(
      "ir_dot_projector_intensity", 1.0);
    ir_flood_light_intensity_ = declare_parameter<double>(
      "ir_flood_light_intensity", 0.0);

    manual_exposure_enabled_ =
      declare_parameter<bool>("manual_exposure_enabled", false);
    manual_exposure_us_ =
      declare_parameter<int>("manual_exposure_us", 5000);
    manual_sensitivity_iso_ =
      declare_parameter<int>("manual_sensitivity_iso", 800);

    preview_window_name_ = declare_parameter<std::string>(
      "preview_window_name", "OAK IR center preview");
    preview_max_width_ =
      declare_parameter<int>("preview_max_width", 1280);
    preview_max_height_ =
      declare_parameter<int>("preview_max_height", 800);
    capture_directory_ =
      declare_parameter<std::string>("capture_directory", ".");
    status_log_interval_sec_ =
      declare_parameter<double>("status_log_interval_sec", 1.0);

    selected_camera_ = parseSelectedCamera(selected_camera_name_);
    const double maximum_exposure_us = 1.0e6 / requestedFps();
    if (
      width_ != static_cast<int>(kOv9282FullWidth) ||
      height_ != static_cast<int>(kOv9282FullHeight) ||
      !std::isfinite(reprojection_fps_) || reprojection_fps_ <= 0.0 ||
      reprojection_fps_ > kRvc2MaximumFastStereoFullResolutionFps ||
      !std::isfinite(single_camera_fps_) || single_camera_fps_ <= 0.0 ||
      single_camera_fps_ > kOv9282MaximumFullResolutionFps ||
      !std::isfinite(virtual_camera_position_ratio_) ||
      virtual_camera_position_ratio_ < 0.0 ||
      virtual_camera_position_ratio_ > 1.0 ||
      !std::isfinite(sync_threshold_ms_) || sync_threshold_ms_ <= 0.0 ||
      !std::isfinite(ir_dot_projector_intensity_) ||
      ir_dot_projector_intensity_ < 0.0 ||
      ir_dot_projector_intensity_ > 1.0 ||
      !std::isfinite(ir_flood_light_intensity_) ||
      ir_flood_light_intensity_ < 0.0 ||
      ir_flood_light_intensity_ > 1.0 ||
      (manual_exposure_enabled_ &&
      (manual_exposure_us_ < 10 ||
      static_cast<double>(manual_exposure_us_) > maximum_exposure_us ||
      manual_sensitivity_iso_ < 100 || manual_sensitivity_iso_ > 1600)) ||
      preview_window_name_.empty() ||
      preview_max_width_ < 0 || preview_max_height_ < 0 ||
      capture_directory_.empty() ||
      !std::isfinite(status_log_interval_sec_) ||
      status_log_interval_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "invalid ir_camera_driver parameter configuration");
    }

    if (
      ir_enabled_at_start_ && ir_dot_projector_intensity_ <= 0.0 &&
      ir_flood_light_intensity_ <= 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "IR is enabled but both configured emitter intensities are zero.");
    }
  }

  void configureExposure(const std::shared_ptr<dai::node::Camera> & camera)
  {
    if (manual_exposure_enabled_) {
      camera->initialControl.setManualExposure(
        static_cast<std::uint32_t>(manual_exposure_us_),
        static_cast<std::uint32_t>(manual_sensitivity_iso_));
    }
  }

  void startOak()
  {
    device_ = std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
    pipeline_ = std::make_unique<dai::Pipeline>(device_);
    // DepthAI 3.6 enables startup auto-calibration by default. This preview
    // package must never mutate the device EEPROM as a side effect.
    pipeline_->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
    pipeline_->setXLinkChunkSize(0);

    dai::Node::Output * host_output = nullptr;
    if (reprojection_enabled_) {
      auto left_camera = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_B,
        std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
        static_cast<float>(reprojection_fps_));
      auto right_camera = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_C,
        std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
        static_cast<float>(reprojection_fps_));
      configureExposure(left_camera);
      configureExposure(right_camera);

      auto * left_output = left_camera->requestOutput(
        std::make_pair(
          static_cast<std::uint32_t>(width_),
          static_cast<std::uint32_t>(height_)),
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        static_cast<float>(reprojection_fps_));
      auto * right_output = right_camera->requestOutput(
        std::make_pair(
          static_cast<std::uint32_t>(width_),
          static_cast<std::uint32_t>(height_)),
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        static_cast<float>(reprojection_fps_));

      auto stereo = pipeline_->create<dai::node::StereoDepth>();
      stereo->build(
        *left_output,
        *right_output,
        dai::node::StereoDepth::PresetMode::FAST_DENSITY);
      stereo->setRectification(true);
      stereo->enableDistortionCorrection(true);
      stereo->setOutputSize(width_, height_);
      stereo->setOutputKeepAspectRatio(true);
      stereo->setDepthAlign(
        dai::StereoDepthConfig::AlgorithmControl::DepthAlign::CENTER);
      stereo->initialConfig->algorithmControl.centerAlignmentShiftFactor =
        static_cast<float>(virtual_camera_position_ratio_);
      // CENTER alignment requires LR-check. Keep subpixel and post filters
      // disabled so the 800P disparity path remains as light as possible.
      stereo->setSubpixel(false);
      stereo->setExtendedDisparity(false);
      stereo->setLeftRightCheck(true);
      stereo->initialConfig->postProcessing.median =
        dai::node::StereoDepth::MedianFilter::MEDIAN_OFF;
      stereo->initialConfig->postProcessing.holeFilling.enable = false;
      stereo->initialConfig->postProcessing.adaptiveMedianFilter.enable =
        false;
      stereo->initialConfig->postProcessing.spatialFilter.enable = false;
      stereo->initialConfig->postProcessing.temporalFilter.enable = false;
      stereo->initialConfig->postProcessing.speckleFilter.enable = false;
      stereo->setRectifyEdgeFillColor(0);
      stereo->setFrameSync(true);

      auto sync = pipeline_->create<dai::node::Sync>();
      sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double, std::milli>(sync_threshold_ms_)));
      sync->setSyncAttempts(-1);
      stereo->rectifiedLeft.link(sync->inputs["left"]);
      stereo->rectifiedRight.link(sync->inputs["right"]);
      stereo->disparity.link(sync->inputs["disparity"]);
      host_output = &sync->out;
      center_reprojector_ = std::make_unique<CudaCenterReprojector>();
    } else {
      auto camera = pipeline_->create<dai::node::Camera>()->build(
        selectedSocket(),
        std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
        static_cast<float>(single_camera_fps_));
      configureExposure(camera);
      host_output = camera->requestOutput(
        std::make_pair(
          static_cast<std::uint32_t>(width_),
          static_cast<std::uint32_t>(height_)),
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        static_cast<float>(single_camera_fps_),
        undistort_single_camera_);
    }

    output_queue_ = host_output->createOutputQueue(1U, false);
    pipeline_->build();
    const auto xlink_bridge = host_output->getXLinkBridge();
    if (!xlink_bridge || !xlink_bridge->xLinkOut) {
      throw std::runtime_error(
              "DepthAI did not create the IR preview XLink bridge");
    }
    xlink_bridge->xLinkOut->input.setMaxSize(1);
    xlink_bridge->xLinkOut->input.setBlocking(false);

    pipeline_->start();
    setIrState(ir_enabled_at_start_, true);

    RCLCPP_INFO(
      get_logger(),
      "OAK IR preview: mode=%s, %dx%d@%.1f FPS requested, USB=%s, "
      "queue=1/non-blocking, XLink chunks=off, exposure=%s",
      reprojection_enabled_ ? "CUDA stereo-midpoint reprojection" :
      selectedCameraLabel(),
      width_, height_, requestedFps(), usbSpeedName(device_->getUsbSpeed()),
      manual_exposure_enabled_ ? "manual" : "auto");
    if (reprojection_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "Virtual camera ratio=%.3f (0=LEFT/CAM_B, 1=RIGHT/CAM_C); "
        "invalid disparity falls back to %s.",
        virtual_camera_position_ratio_, selectedCameraLabel());
    }
    RCLCPP_INFO(
      get_logger(),
      "Keys: I=toggle configured IR emitters, B=save PNG, Q/Esc=quit.");
  }

  void setIrState(const bool enabled, const bool fail_hard)
  {
    if (!device_) {
      return;
    }

    const float dot_intensity = enabled ?
      static_cast<float>(ir_dot_projector_intensity_) : 0.0F;
    const float flood_intensity = enabled ?
      static_cast<float>(ir_flood_light_intensity_) : 0.0F;
    const bool dot_success =
      device_->setIrLaserDotProjectorIntensity(dot_intensity);
    const bool flood_success =
      device_->setIrFloodLightIntensity(flood_intensity);
    const bool dot_required = dot_intensity > 0.0F;
    const bool flood_required = flood_intensity > 0.0F;
    const bool success =
      (!dot_required || dot_success) && (!flood_required || flood_success);

    if (!success) {
      // Avoid leaving one emitter on after a partial update failure.
      device_->setIrLaserDotProjectorIntensity(0.0F);
      device_->setIrFloodLightIntensity(0.0F);
      ir_active_.store(false, std::memory_order_relaxed);
      const std::string message =
        "failed to set the requested OAK IR emitter intensity; a Pro-series "
        "device is required";
      if (fail_hard) {
        throw std::runtime_error(message);
      }
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      return;
    }

    const bool actually_active = enabled && (dot_required || flood_required);
    ir_active_.store(actually_active, std::memory_order_relaxed);
    RCLCPP_INFO(
      get_logger(),
      "IR emitters %s: dot=%.2f, flood=%.2f",
      actually_active ? "ON" : "OFF",
      dot_intensity,
      flood_intensity);
  }

  bool validFrameSize(const dai::ImgFrame & frame) const
  {
    return
      static_cast<int>(frame.getWidth()) == width_ &&
      static_cast<int>(frame.getHeight()) == height_;
  }

  void recordSequence(const std::int64_t sequence)
  {
    if (last_sequence_.has_value() && sequence > *last_sequence_ + 1) {
      skipped_sequences_total_.fetch_add(
        static_cast<std::uint64_t>(sequence - *last_sequence_ - 1),
        std::memory_order_relaxed);
    }
    last_sequence_ = sequence;
  }

  void captureLoop()
  {
    while (!stop_requested_.load(std::memory_order_relaxed)) {
      try {
        if (!pipeline_ || !pipeline_->isRunning()) {
          break;
        }

        if (reprojection_enabled_) {
          auto group = output_queue_->tryGet<dai::MessageGroup>();
          if (!group) {
            std::this_thread::sleep_for(100us);
            continue;
          }
          auto left = group->get<dai::ImgFrame>("left");
          auto right = group->get<dai::ImgFrame>("right");
          auto disparity = group->get<dai::ImgFrame>("disparity");
          if (
            !left || !right || !disparity ||
            !validFrameSize(*left) || !validFrameSize(*right) ||
            !validFrameSize(*disparity))
          {
            invalid_frames_total_.fetch_add(1U, std::memory_order_relaxed);
            continue;
          }

          const auto sequence = disparity->getSequenceNum();
          recordSequence(sequence);
          const auto generation =
            captured_total_.fetch_add(1U, std::memory_order_relaxed) + 1U;
          capture_interval_.fetch_add(1U, std::memory_order_relaxed);
          last_exposure_us_.store(
            left->getExposureTime().count(), std::memory_order_relaxed);
          last_sensitivity_iso_.store(
            left->getSensitivity(), std::memory_order_relaxed);
          std::shared_ptr<const StereoSnapshot> snapshot =
            std::make_shared<StereoSnapshot>(
            StereoSnapshot{
              std::move(left), std::move(right), std::move(disparity),
              generation, sequence});
          std::atomic_store_explicit(
            &latest_stereo_, std::move(snapshot), std::memory_order_release);
        } else {
          auto frame = output_queue_->tryGet<dai::ImgFrame>();
          if (!frame) {
            std::this_thread::sleep_for(100us);
            continue;
          }
          if (!validFrameSize(*frame)) {
            invalid_frames_total_.fetch_add(1U, std::memory_order_relaxed);
            continue;
          }

          const auto sequence = frame->getSequenceNum();
          recordSequence(sequence);
          const auto generation =
            captured_total_.fetch_add(1U, std::memory_order_relaxed) + 1U;
          capture_interval_.fetch_add(1U, std::memory_order_relaxed);
          last_exposure_us_.store(
            frame->getExposureTime().count(), std::memory_order_relaxed);
          last_sensitivity_iso_.store(
            frame->getSensitivity(), std::memory_order_relaxed);
          std::shared_ptr<const SingleSnapshot> snapshot =
            std::make_shared<SingleSnapshot>(
            SingleSnapshot{std::move(frame), generation, sequence});
          std::atomic_store_explicit(
            &latest_single_, std::move(snapshot), std::memory_order_release);
        }
        frame_available_.notify_one();
      } catch (const std::exception & exception) {
        capture_errors_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "IR camera capture error: %s", exception.what());
        std::this_thread::sleep_for(1ms);
      }
    }
  }

  void resizePreviewWindow(const cv::Mat & frame)
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
      std::max(1, static_cast<int>(frame.cols * scale)),
      std::max(1, static_cast<int>(frame.rows * scale)));
    preview_window_sized_ = true;
  }

  void saveFrame(const cv::Mat & frame)
  {
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "No IR preview frame is available to save.");
      return;
    }
    try {
      const std::filesystem::path directory(capture_directory_);
      std::error_code error;
      std::filesystem::create_directories(directory, error);
      if (error) {
        RCLCPP_ERROR(
          get_logger(), "Could not create '%s': %s",
          directory.string().c_str(), error.message().c_str());
        return;
      }
      const std::string mode = reprojection_enabled_ ?
        "center" : (selected_camera_ == SelectedCamera::LEFT ?
        "left" : "right");
      const std::string ir_state =
        ir_active_.load(std::memory_order_relaxed) ? "ir_on" : "ir_off";
      const auto filename = directory /
        ("ir_" + mode + "_" + ir_state + "_" +
        std::to_string(get_clock()->now().nanoseconds()) + ".png");
      if (!cv::imwrite(filename.string(), frame)) {
        throw std::runtime_error("OpenCV imwrite returned false");
      }
      const auto absolute_path = std::filesystem::absolute(filename, error);
      RCLCPP_INFO(
        get_logger(), "IR preview saved: %s",
        (error ? filename : absolute_path).string().c_str());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "Could not save IR preview: %s", exception.what());
    }
  }

  void previewLoop()
  {
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(
        get_logger(), "Could not create IR preview window: %s",
        exception.what());
      rclcpp::shutdown();
      return;
    }

    std::uint64_t previewed_generation = 0U;
    cv::Mat current_frame;
    std::shared_ptr<const StereoSnapshot> displayed_stereo;
    std::shared_ptr<const SingleSnapshot> displayed_single;
    bool window_was_visible = false;

    while (!stop_requested_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        frame_available_.wait_for(lock, 2ms);
      }
      if (stop_requested_.load(std::memory_order_relaxed)) {
        break;
      }

      try {
        bool frame_updated = false;
        if (reprojection_enabled_) {
          auto snapshot = std::atomic_load_explicit(
            &latest_stereo_, std::memory_order_acquire);
          if (snapshot && snapshot->generation != previewed_generation) {
            const auto reprojection_started_at =
              std::chrono::steady_clock::now();
            const cv::Mat left = snapshot->left->getFrame(false);
            const cv::Mat right = snapshot->right->getFrame(false);
            const cv::Mat disparity = snapshot->disparity->getFrame(false);
            current_frame = center_reprojector_->process(
              left,
              right,
              disparity,
              virtual_camera_position_ratio_,
              selected_camera_ == SelectedCamera::LEFT);
            const auto elapsed = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - reprojection_started_at);
            latest_reprojection_ms_.store(
              elapsed.count(), std::memory_order_relaxed);
            displayed_stereo = std::move(snapshot);
            displayed_single.reset();
            previewed_generation = displayed_stereo->generation;
            frame_updated = true;
          }
        } else {
          auto snapshot = std::atomic_load_explicit(
            &latest_single_, std::memory_order_acquire);
          if (snapshot && snapshot->generation != previewed_generation) {
            current_frame = snapshot->frame->getFrame(false);
            displayed_single = std::move(snapshot);
            displayed_stereo.reset();
            previewed_generation = displayed_single->generation;
            frame_updated = true;
          }
        }

        if (frame_updated && !current_frame.empty()) {
          resizePreviewWindow(current_frame);
          cv::imshow(preview_window_name_, current_frame);
          previewed_total_.fetch_add(1U, std::memory_order_relaxed);
          preview_interval_.fetch_add(1U, std::memory_order_relaxed);
        }

        const int key = cv::waitKey(1) & 0xff;
        if (key == 'i' || key == 'I') {
          setIrState(
            !ir_active_.load(std::memory_order_relaxed), false);
        } else if (key == 'b' || key == 'B') {
          saveFrame(current_frame);
        }

        const double visible = cv::getWindowProperty(
          preview_window_name_, cv::WND_PROP_VISIBLE);
        if (visible >= 1.0) {
          window_was_visible = true;
        }
        if (
          key == 'q' || key == 'Q' || key == 27 ||
          (window_was_visible && visible < 1.0))
        {
          RCLCPP_INFO(get_logger(), "IR preview closed.");
          stop_requested_.store(true, std::memory_order_relaxed);
          frame_available_.notify_all();
          rclcpp::shutdown();
          break;
        }
      } catch (const std::exception & exception) {
        preview_errors_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "IR preview error: %s", exception.what());
      }
    }

    try {
      cv::destroyWindow(preview_window_name_);
    } catch (...) {
    }
  }

  void reportStatus()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - last_status_at_).count();
    last_status_at_ = now;
    if (elapsed <= 0.0) {
      return;
    }

    const auto capture_count =
      capture_interval_.exchange(0U, std::memory_order_relaxed);
    const auto preview_count =
      preview_interval_.exchange(0U, std::memory_order_relaxed);
    RCLCPP_INFO(
      get_logger(),
      "[IR_CAMERA] capture=%.1fHz preview=%.1fHz requested=%.1fHz, "
      "exposure=%ldus ISO=%d, IR=%s, reproject=%.2fms, "
      "skipped=%lu invalid=%lu capture_errors=%lu preview_errors=%lu",
      static_cast<double>(capture_count) / elapsed,
      static_cast<double>(preview_count) / elapsed,
      requestedFps(),
      static_cast<long>(last_exposure_us_.load(std::memory_order_relaxed)),
      last_sensitivity_iso_.load(std::memory_order_relaxed),
      ir_active_.load(std::memory_order_relaxed) ? "on" : "off",
      latest_reprojection_ms_.load(std::memory_order_relaxed),
      static_cast<unsigned long>(
        skipped_sequences_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        invalid_frames_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        capture_errors_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        preview_errors_total_.load(std::memory_order_relaxed)));
  }

  void stop()
  {
    if (stopped_.exchange(true, std::memory_order_relaxed)) {
      return;
    }
    stop_requested_.store(true, std::memory_order_relaxed);
    frame_available_.notify_all();

    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (
      preview_thread_.joinable() &&
      preview_thread_.get_id() != std::this_thread::get_id())
    {
      preview_thread_.join();
    }
    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }

    if (device_) {
      try {
        device_->setIrLaserDotProjectorIntensity(0.0F);
        device_->setIrFloodLightIntensity(0.0F);
        ir_active_.store(false, std::memory_order_relaxed);
      } catch (...) {
      }
    }
    output_queue_.reset();
    if (pipeline_) {
      try {
        if (pipeline_->isRunning()) {
          pipeline_->stop();
          pipeline_->wait();
        }
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          get_logger(), "OAK shutdown error: %s", exception.what());
      }
      pipeline_.reset();
    }
    device_.reset();
    center_reprojector_.reset();
  }

  bool reprojection_enabled_{true};
  std::string selected_camera_name_{"LEFT"};
  SelectedCamera selected_camera_{SelectedCamera::LEFT};
  double virtual_camera_position_ratio_{0.5};
  int width_{1280};
  int height_{800};
  double reprojection_fps_{50.0};
  double single_camera_fps_{129.0};
  bool undistort_single_camera_{true};
  double sync_threshold_ms_{2.0};

  bool ir_enabled_at_start_{true};
  double ir_dot_projector_intensity_{1.0};
  double ir_flood_light_intensity_{0.0};
  bool manual_exposure_enabled_{false};
  int manual_exposure_us_{5000};
  int manual_sensitivity_iso_{800};

  std::string preview_window_name_{"OAK IR center preview"};
  int preview_max_width_{1280};
  int preview_max_height_{800};
  std::string capture_directory_{"."};
  double status_log_interval_sec_{1.0};
  bool preview_window_sized_{false};

  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> output_queue_;
  std::unique_ptr<CudaCenterReprojector> center_reprojector_;

  std::shared_ptr<const StereoSnapshot> latest_stereo_;
  std::shared_ptr<const SingleSnapshot> latest_single_;
  std::mutex wait_mutex_;
  std::condition_variable frame_available_;
  std::thread capture_thread_;
  std::thread preview_thread_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> stopped_{false};
  std::atomic<bool> ir_active_{false};
  std::atomic<std::uint64_t> captured_total_{0U};
  std::atomic<std::uint64_t> capture_interval_{0U};
  std::atomic<std::uint64_t> previewed_total_{0U};
  std::atomic<std::uint64_t> preview_interval_{0U};
  std::atomic<std::uint64_t> skipped_sequences_total_{0U};
  std::atomic<std::uint64_t> invalid_frames_total_{0U};
  std::atomic<std::uint64_t> capture_errors_total_{0U};
  std::atomic<std::uint64_t> preview_errors_total_{0U};
  std::atomic<std::int64_t> last_exposure_us_{0};
  std::atomic<int> last_sensitivity_iso_{0};
  std::atomic<double> latest_reprojection_ms_{0.0};
  std::optional<std::int64_t> last_sequence_;
  std::chrono::steady_clock::time_point started_at_{};
  std::chrono::steady_clock::time_point last_status_at_{};
};

}  // namespace ir_camera_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<ir_camera_driver::IrCameraDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ir_camera_driver"),
      "IR camera driver terminated: %s", exception.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
