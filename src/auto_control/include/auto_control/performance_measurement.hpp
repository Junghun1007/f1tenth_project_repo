#ifndef AUTO_CONTROL__PERFORMANCE_MEASUREMENT_HPP_
#define AUTO_CONTROL__PERFORMANCE_MEASUREMENT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace auto_control
{

struct PerformanceFrame
{
  double measurement_elapsed_sec{0.0};
  std::uint64_t detector_sequence{0U};
  std::string actual_engine_precision;
  bool valid_centerline{false};
  bool servo_position_calculated{false};
  std::size_t centerline_point_count{0U};
  double steering_angle_rad{0.0};
  double servo_position{0.0};
  std::optional<double> h2d_preprocess_ms;
  std::optional<double> pure_inference_ms;
  std::optional<double> label_export_ms;
  std::optional<double> backend_postprocess_ms;
  std::optional<double> lane_geometry_ms;
  std::optional<double> result_message_build_ms;
  std::optional<double> lane_postprocess_total_ms;
  std::optional<double> detector_queue_ms;
  std::optional<double> detector_total_compute_ms;
  std::optional<double> lane_result_transport_ms;
  std::optional<double> source_to_detector_input_ms;
  std::optional<double> auto_control_compute_ms;
  std::optional<double> compute_only_total_ms;
  std::optional<double> detector_input_to_control_complete_ms;
  std::optional<double> source_capture_to_control_complete_ms;
};

class PerformanceMeasurement
{
public:
  using SteadyClock = std::chrono::steady_clock;

  PerformanceMeasurement(
    double duration_sec, double startup_timeout_sec,
    std::string log_directory, std::string engine_precision,
    std::string model_path);

  bool active() const;
  void start(SteadyClock::time_point now);
  double elapsed_sec(SteadyClock::time_point now = SteadyClock::now()) const;
  bool due_to_finish() const;
  bool startup_timed_out() const;
  void sample_power();
  void add_frame(PerformanceFrame frame);
  std::size_t frame_count() const;
  const std::string & power_description() const;
  std::filesystem::path write(const std::string & status);

private:
  struct PowerSource
  {
    std::string rail_name;
    std::string description;
    std::filesystem::path voltage_path;
    std::filesystem::path current_path;
    std::filesystem::path power_path;
    double power_divisor{1.0};
  };
  struct PowerSample {double elapsed_sec; double watts;};

  static std::optional<PowerSource> discover_power_source();
  static double read_number(const std::filesystem::path & path);
  static std::string safe_name(const std::string & value);

  double duration_sec_;
  double startup_timeout_sec_;
  std::filesystem::path log_directory_;
  std::string engine_precision_;
  std::string model_path_;
  std::chrono::system_clock::time_point created_system_;
  SteadyClock::time_point created_steady_;
  std::optional<std::chrono::system_clock::time_point> started_system_;
  std::optional<SteadyClock::time_point> started_steady_;
  std::vector<PerformanceFrame> frames_;
  std::optional<PowerSource> power_source_;
  std::string power_description_;
  std::vector<PowerSample> power_samples_;
  std::size_t power_read_errors_{0U};
};

}  // namespace auto_control

#endif  // AUTO_CONTROL__PERFORMANCE_MEASUREMENT_HPP_
