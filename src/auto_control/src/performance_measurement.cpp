#include "auto_control/performance_measurement.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace auto_control
{
namespace
{
using MetricMember = std::optional<double> PerformanceFrame::*;
const std::array<std::pair<const char *, MetricMember>, 15> kMetrics{{
  {"h2d_preprocess_ms", &PerformanceFrame::h2d_preprocess_ms},
  {"pure_inference_ms", &PerformanceFrame::pure_inference_ms},
  {"label_export_ms", &PerformanceFrame::label_export_ms},
  {"backend_postprocess_ms", &PerformanceFrame::backend_postprocess_ms},
  {"lane_geometry_ms", &PerformanceFrame::lane_geometry_ms},
  {"result_message_build_ms", &PerformanceFrame::result_message_build_ms},
  {"lane_postprocess_total_ms", &PerformanceFrame::lane_postprocess_total_ms},
  {"detector_queue_ms", &PerformanceFrame::detector_queue_ms},
  {"detector_total_compute_ms", &PerformanceFrame::detector_total_compute_ms},
  {"lane_result_transport_ms", &PerformanceFrame::lane_result_transport_ms},
  {"source_to_detector_input_ms", &PerformanceFrame::source_to_detector_input_ms},
  {"auto_control_compute_ms", &PerformanceFrame::auto_control_compute_ms},
  {"compute_only_total_ms", &PerformanceFrame::compute_only_total_ms},
  {"detector_input_to_control_complete_ms", &PerformanceFrame::detector_input_to_control_complete_ms},
  {"source_capture_to_control_complete_ms", &PerformanceFrame::source_capture_to_control_complete_ms},
}};

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return {};}
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '\"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<int>(character) << std::dec;
        } else {
          output << character;
        }
    }
  }
  return output.str();
}

std::string iso_time(std::chrono::system_clock::time_point value)
{
  const auto whole_seconds = std::chrono::time_point_cast<std::chrono::seconds>(value);
  const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
    value - whole_seconds).count();
  const std::time_t raw = std::chrono::system_clock::to_time_t(value);
  std::tm local{};
  localtime_r(&raw, &local);
  std::ostringstream output;
  output << std::put_time(&local, "%Y-%m-%dT%H:%M:%S") << '.' <<
    std::setw(6) << std::setfill('0') << microseconds << std::put_time(&local, "%z");
  return output.str();
}

std::string file_timestamp(std::chrono::system_clock::time_point value)
{
  const auto whole_seconds = std::chrono::time_point_cast<std::chrono::seconds>(value);
  const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
    value - whole_seconds).count();
  const std::time_t raw = std::chrono::system_clock::to_time_t(value);
  std::tm local{};
  localtime_r(&raw, &local);
  std::ostringstream output;
  output << std::put_time(&local, "%Y%m%d_%H%M%S_") <<
    std::setw(6) << std::setfill('0') << microseconds << std::put_time(&local, "%z");
  return output.str();
}

double percentile(const std::vector<double> & ordered, double requested)
{
  if (ordered.empty()) {return 0.0;}
  const double position = (ordered.size() - 1U) * requested / 100.0;
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction;
}

void write_optional(std::ostream & output, const std::optional<double> & value)
{
  if (value && std::isfinite(*value) && *value >= 0.0) {output << *value;}
  else {output << "null";}
}
}  // namespace

PerformanceMeasurement::PerformanceMeasurement(
  double duration_sec, double startup_timeout_sec,
  std::string log_directory, std::string engine_precision,
  std::string model_path)
: duration_sec_(duration_sec), startup_timeout_sec_(startup_timeout_sec),
  log_directory_(std::filesystem::absolute(std::filesystem::path(std::move(log_directory)))),
  engine_precision_(safe_name(engine_precision)), model_path_(std::move(model_path)),
  created_system_(std::chrono::system_clock::now()), created_steady_(SteadyClock::now()),
  power_source_(discover_power_source())
{
  frames_.reserve(static_cast<std::size_t>(std::ceil(duration_sec_ * 120.0)));
  power_samples_.reserve(static_cast<std::size_t>(std::ceil(duration_sec_ * 20.0)));
  power_description_ = power_source_ ?
    power_source_->rail_name + " (" + power_source_->description + ")" :
    "unavailable (timing will still be recorded)";
}

bool PerformanceMeasurement::active() const {return started_steady_.has_value();}

void PerformanceMeasurement::start(SteadyClock::time_point now)
{
  if (active()) {return;}
  started_steady_ = now;
  started_system_ = std::chrono::system_clock::now();
}

double PerformanceMeasurement::elapsed_sec(SteadyClock::time_point now) const
{
  return started_steady_ ? std::max(0.0, std::chrono::duration<double>(now - *started_steady_).count()) : 0.0;
}

bool PerformanceMeasurement::due_to_finish() const
{
  return active() && elapsed_sec() >= duration_sec_;
}

bool PerformanceMeasurement::startup_timed_out() const
{
  return !active() &&
    std::chrono::duration<double>(SteadyClock::now() - created_steady_).count() >= startup_timeout_sec_;
}

double PerformanceMeasurement::read_number(const std::filesystem::path & path)
{
  std::ifstream input(path);
  double value = 0.0;
  if (!(input >> value)) {throw std::runtime_error("cannot read " + path.string());}
  return value;
}

std::optional<PerformanceMeasurement::PowerSource>
PerformanceMeasurement::discover_power_source()
{
  namespace fs = std::filesystem;
  const std::map<std::string, int> priority{
    {"VDD_IN", 0}, {"5V_IN", 1}, {"POM_5V_IN", 2}, {"VIN_SYS_5V0", 3}};
  std::vector<std::pair<int, PowerSource>> candidates;
  const fs::path drivers("/sys/bus/i2c/drivers");
  std::error_code error;
  if (!fs::exists(drivers, error)) {return std::nullopt;}
  try {
    for (const auto & root : fs::directory_iterator(drivers)) {
      if (root.path().filename().string().rfind("ina3221", 0U) != 0U) {continue;}
      for (const auto & device : fs::directory_iterator(root.path())) {
        const auto hwmon_root = device.path() / "hwmon";
        if (fs::is_directory(hwmon_root)) {
          for (const auto & hwmon : fs::directory_iterator(hwmon_root)) {
            if (!hwmon.is_directory()) {continue;}
            for (const auto & entry : fs::directory_iterator(hwmon.path())) {
              const std::string name = entry.path().filename().string();
              std::smatch match;
              if (!std::regex_match(name, match, std::regex("in([0-9]+)_label"))) {continue;}
              std::ifstream label_input(entry.path());
              std::string rail;
              std::getline(label_input, rail);
              rail = trim(rail);
              const auto found = priority.find(rail);
              if (found == priority.end()) {continue;}
              const std::string channel = match[1].str();
              const auto voltage = hwmon.path() / ("in" + channel + "_input");
              const auto current = hwmon.path() / ("curr" + channel + "_input");
              if (fs::exists(voltage) && fs::exists(current)) {
                candidates.push_back({found->second, {rail,
                  voltage.string() + " * " + current.string(), voltage, current, {}, 1.0}});
              }
            }
          }
        }
        if (!fs::is_directory(device.path())) {continue;}
        for (const auto & child : fs::directory_iterator(device.path())) {
          if (child.path().filename().string().rfind("iio:device", 0U) != 0U ||
            !child.is_directory()) {continue;}
          for (const auto & entry : fs::directory_iterator(child.path())) {
            const std::string name = entry.path().filename().string();
            std::smatch match;
            if (!std::regex_match(name, match, std::regex("rail_name_([0-9]+)"))) {continue;}
            std::ifstream label_input(entry.path());
            std::string rail;
            std::getline(label_input, rail);
            rail = trim(rail);
            const auto found = priority.find(rail);
            if (found == priority.end()) {continue;}
            const auto power = child.path() / ("in_power" + match[1].str() + "_input");
            if (fs::exists(power)) {
              candidates.push_back({found->second, {rail, power.string(), {}, {}, power, 1000.0}});
            }
          }
        }
      }
    }
  } catch (const fs::filesystem_error &) {
    // Timing remains available when board power sysfs cannot be traversed.
  }
  if (candidates.empty()) {return std::nullopt;}
  return std::min_element(candidates.begin(), candidates.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;})->second;
}

void PerformanceMeasurement::sample_power()
{
  if (!active() || !power_source_) {return;}
  try {
    const double watts = power_source_->power_path.empty() ?
      read_number(power_source_->voltage_path) * read_number(power_source_->current_path) / 1.0e6 :
      read_number(power_source_->power_path) / power_source_->power_divisor;
    if (!std::isfinite(watts) || watts < 0.0) {throw std::runtime_error("invalid power sample");}
    power_samples_.push_back({elapsed_sec(), watts});
  } catch (const std::exception &) {
    ++power_read_errors_;
  }
}

void PerformanceMeasurement::add_frame(PerformanceFrame frame)
{
  frames_.push_back(std::move(frame));
}

std::size_t PerformanceMeasurement::frame_count() const {return frames_.size();}
const std::string & PerformanceMeasurement::power_description() const {return power_description_;}

std::string PerformanceMeasurement::safe_name(const std::string & value)
{
  std::string output;
  for (const unsigned char character : value) {
    output.push_back(std::isalnum(character) || character == '_' || character == '.' || character == '-' ?
      static_cast<char>(std::tolower(character)) : '_');
  }
  while (!output.empty() && output.back() == '_') {output.pop_back();}
  return output.empty() ? "unknown" : output;
}

std::filesystem::path PerformanceMeasurement::write(const std::string & status)
{
  const auto completed_system = std::chrono::system_clock::now();
  const double actual_duration = elapsed_sec();
  std::filesystem::create_directories(log_directory_);
  const std::string stem = "auto_drive_benchmark_" + engine_precision_ + "_" +
    file_timestamp(created_system_);
  std::filesystem::path output_path = log_directory_ / (stem + ".json");
  for (int suffix = 1; std::filesystem::exists(output_path); ++suffix) {
    output_path = log_directory_ / (stem + "_" + std::to_string(suffix) + ".json");
  }
  std::ofstream output(output_path);
  if (!output) {throw std::runtime_error("cannot create " + output_path.string());}
  output << std::setprecision(15);
  const auto quote = [&](const std::string & value) {output << '\"' << json_escape(value) << '\"';};
  std::set<std::string> observed;
  std::size_t valid_count = 0U;
  std::size_t control_count = 0U;
  for (const auto & frame : frames_) {
    observed.insert(frame.actual_engine_precision);
    valid_count += frame.valid_centerline ? 1U : 0U;
    control_count += frame.servo_position_calculated ? 1U : 0U;
  }
  output << "{\n  \"schema_version\": 2,\n  \"status\": "; quote(status);
  output << ",\n  \"title\": "; quote("auto_drive_" + engine_precision_ + "_" + iso_time(created_system_));
  output << ",\n  \"controller_implementation\": \"rclcpp_cpp\",\n  \"engine_precision\": "; quote(engine_precision_);
  output << ",\n  \"observed_engine_precisions\": [";
  bool first = true;
  for (const auto & precision : observed) {if (!first) {output << ',';} quote(precision); first = false;}
  output << "],\n  \"model_path\": "; quote(model_path_);
  output << ",\n  \"safety_mode\": \"monitor_only (no duty/brake/servo actuator output)\",\n";
  output << "  \"throughput_definition\": \"completed results divided by measurement wall time; "
    "fixed source pipeline latency is not added to every frame interval\",\n";
  output << "  \"detector_input_to_control_complete_definition\": \"per-frame latency from "
    "detector receipt through queue, detector compute, DDS delivery, and C++ servo calculation\",\n";
  output << "  \"created_at\": "; quote(iso_time(created_system_));
  output << ",\n  \"measurement_started_at\": ";
  if (started_system_) {quote(iso_time(*started_system_));} else {output << "null";}
  output << ",\n  \"completed_at\": "; quote(iso_time(completed_system));
  output << ",\n  \"target_duration_sec\": " << duration_sec_ <<
    ",\n  \"actual_duration_sec\": " << actual_duration <<
    ",\n  \"startup_timeout_sec\": " << startup_timeout_sec_ <<
    ",\n  \"frame_count\": " << frames_.size() <<
    ",\n  \"valid_centerline_count\": " << valid_count <<
    ",\n  \"servo_position_calculated_count\": " << control_count <<
    ",\n  \"valid_centerline_ratio\": " << (frames_.empty() ? 0.0 :
      static_cast<double>(valid_count) / frames_.size()) <<
    ",\n  \"lane_result_throughput_fps_excluding_source_transport_delay\": " <<
      (actual_duration > 0.0 ? frames_.size() / actual_duration : 0.0) <<
    ",\n  \"servo_position_calculation_throughput_fps\": " <<
      (actual_duration > 0.0 ? control_count / actual_duration : 0.0) << ",\n";
  output << "  \"metrics\": {\n";
  for (std::size_t metric_index = 0; metric_index < kMetrics.size(); ++metric_index) {
    const auto [name, member] = kMetrics[metric_index];
    std::vector<double> values;
    values.reserve(frames_.size());
    for (const auto & frame : frames_) {
      const auto & value = frame.*member;
      if (value && std::isfinite(*value) && *value >= 0.0) {values.push_back(*value);}
    }
    std::sort(values.begin(), values.end());
    const double average = values.empty() ? 0.0 :
      std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    output << "    \"" << name << "\": {\"count\": " << values.size() <<
      ", \"average_ms\": " << average <<
      ", \"minimum_ms\": " << (values.empty() ? 0.0 : values.front()) <<
      ", \"p50_ms\": " << percentile(values, 50.0) <<
      ", \"p95_ms\": " << percentile(values, 95.0) <<
      ", \"maximum_ms\": " << (values.empty() ? 0.0 : values.back()) <<
      ", \"equivalent_fps_from_average\": " << (average > 0.0 ? 1000.0 / average : 0.0) << '}';
    output << (metric_index + 1U == kMetrics.size() ? "\n" : ",\n");
  }
  output << "  },\n  \"power\": {\n    \"available\": " << (power_source_ ? "true" : "false") <<
    ",\n    \"rail_name\": ";
  if (power_source_) {quote(power_source_->rail_name);} else {output << "null";}
  output << ",\n    \"source\": ";
  if (power_source_) {quote(power_source_->description);} else {output << "null";}
  double power_sum = 0.0;
  double power_min = std::numeric_limits<double>::infinity();
  double power_max = 0.0;
  for (const auto & sample : power_samples_) {
    power_sum += sample.watts;
    power_min = std::min(power_min, sample.watts);
    power_max = std::max(power_max, sample.watts);
  }
  const std::optional<double> power_average = power_samples_.empty() ? std::nullopt :
    std::optional<double>(power_sum / power_samples_.size());
  const std::optional<double> power_minimum = power_samples_.empty() ? std::nullopt :
    std::optional<double>(power_min);
  const std::optional<double> power_maximum = power_samples_.empty() ? std::nullopt :
    std::optional<double>(power_max);
  output << ",\n    \"sample_count\": " << power_samples_.size() <<
    ",\n    \"read_error_count\": " << power_read_errors_ <<
    ",\n    \"average_watts\": "; write_optional(output, power_average);
  output << ",\n    \"minimum_watts\": ";
  write_optional(output, power_minimum);
  output << ",\n    \"maximum_watts\": ";
  write_optional(output, power_maximum);
  output << ",\n    \"estimated_energy_wh\": ";
  write_optional(output, power_average ? std::optional<double>(*power_average * actual_duration / 3600.0) : std::nullopt);
  output << ",\n    \"samples\": [";
  for (std::size_t index = 0; index < power_samples_.size(); ++index) {
    const auto & sample = power_samples_[index];
    if (index) {output << ',';}
    output << "{\"elapsed_sec\":" << sample.elapsed_sec << ",\"watts\":" << sample.watts << '}';
  }
  output << "]\n  },\n  \"frames\": [\n";
  for (std::size_t index = 0; index < frames_.size(); ++index) {
    const auto & frame = frames_[index];
    output << "    {\"measurement_elapsed_sec\":" << frame.measurement_elapsed_sec <<
      ",\"detector_sequence\":" << frame.detector_sequence <<
      ",\"actual_engine_precision\":"; quote(frame.actual_engine_precision);
    output << ",\"valid_centerline\":" << (frame.valid_centerline ? "true" : "false") <<
      ",\"servo_position_calculated\":" << (frame.servo_position_calculated ? "true" : "false") <<
      ",\"centerline_point_count\":" << frame.centerline_point_count <<
      ",\"steering_angle_rad\":" << frame.steering_angle_rad <<
      ",\"servo_position\":" << frame.servo_position;
    for (const auto & [name, member] : kMetrics) {
      output << ",\"" << name << "\":";
      write_optional(output, frame.*member);
    }
    output << '}' << (index + 1U == frames_.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
  if (!output) {throw std::runtime_error("failed while writing " + output_path.string());}
  return output_path;
}

}  // namespace auto_control
