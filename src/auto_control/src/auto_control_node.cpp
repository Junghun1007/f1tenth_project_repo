#include "auto_control/control_core.hpp"
#include "auto_control/performance_measurement.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "line_detactor/msg/lane_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

namespace auto_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double radians(double degrees) {return degrees * kPi / 180.0;}
double seconds(std::int64_t nanoseconds) {return nanoseconds / 1.0e9;}
std::optional<std::int64_t> stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec == 0 && stamp.nanosec == 0U) {return std::nullopt;}
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
std::optional<double> latency_ms(
  const std::optional<std::int64_t> & started, const std::optional<std::int64_t> & finished)
{
  if (!started || !finished || *finished < *started) {return std::nullopt;}
  return (*finished - *started) / 1.0e6;
}
}  // namespace

class AutoControlNode : public rclcpp::Node
{
public:
  AutoControlNode()
  : Node("auto_control")
  {
    declare_and_read_parameters();
    validate_parameters();
    speed_pid_ = std::make_unique<SpeedPid>(
      speed_pid_kp_, speed_pid_ki_, speed_pid_kd_, speed_pid_integral_limit_,
      minimum_duty_, maximum_duty_);
    brake_profile_ = std::make_unique<AutomaticBrakeProfile>(BrakeConfig{
      brake_entry_speed_error_mps_, brake_exit_speed_error_mps_,
      brake_minimum_vehicle_speed_mps_, brake_minimum_current_amps_,
      brake_maximum_current_amps_, brake_current_gain_amps_per_mps_,
      brake_current_rise_amps_per_sec_, brake_current_fall_amps_per_sec_});

    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto connection_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    duty_pub_ = create_publisher<std_msgs::msg::Float32>(duty_topic_, command_qos);
    brake_pub_ = create_publisher<std_msgs::msg::Float32>(brake_current_topic_, command_qos);
    servo_pub_ = create_publisher<std_msgs::msg::Float32>(servo_position_topic_, command_qos);
    command_duty_pub_ = create_publisher<std_msgs::msg::Float32>(command_duty_topic_, command_qos);
    command_brake_pub_ = create_publisher<std_msgs::msg::Float32>(command_brake_current_topic_, command_qos);
    target_speed_pub_ = create_publisher<std_msgs::msg::Float32>(target_speed_topic_, command_qos);
    current_speed_pub_ = create_publisher<std_msgs::msg::Float32>(current_speed_topic_, command_qos);
    curvature_pub_ = create_publisher<std_msgs::msg::Float32>(curvature_topic_, command_qos);
    steering_pub_ = create_publisher<std_msgs::msg::Float32>(steering_angle_topic_, command_qos);
    cross_track_pub_ = create_publisher<std_msgs::msg::Float32>(cross_track_error_topic_, command_qos);
    heading_pub_ = create_publisher<std_msgs::msg::Float32>(heading_error_topic_, command_qos);
    raw_steering_pub_ = create_publisher<std_msgs::msg::Float32>(raw_steering_angle_topic_, command_qos);
    command_servo_pub_ = create_publisher<std_msgs::msg::Float32>(command_servo_position_topic_, command_qos);
    lane_sub_ = create_subscription<line_detactor::msg::LaneResult>(
      lane_result_topic_, sensor_qos,
      std::bind(&AutoControlNode::on_lane_result, this, std::placeholders::_1));
    erpm_sub_ = create_subscription<std_msgs::msg::Int32>(
      measured_erpm_topic_, sensor_qos,
      std::bind(&AutoControlNode::on_measured_erpm, this, std::placeholders::_1));
    connection_sub_ = create_subscription<std_msgs::msg::Bool>(
      connection_status_topic_, connection_qos,
      std::bind(&AutoControlNode::on_connection_status, this, std::placeholders::_1));
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, command_qos,
      std::bind(&AutoControlNode::on_enable, this, std::placeholders::_1));

    latest_servo_position_ = servo_center_;
    last_control_ns_ = now().nanoseconds();
    watchdog_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&AutoControlNode::on_watchdog, this));
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_log_rate_hz_),
      std::bind(&AutoControlNode::log_status, this));

    if (performance_measurement_enabled_) {
      enabled_ = true;
      control_mode_ = "monitor_only";
      measurement_ = std::make_unique<PerformanceMeasurement>(
        performance_measurement_duration_sec_, performance_measurement_startup_timeout_sec_,
        performance_measurement_log_directory_, performance_measurement_engine_precision_,
        performance_measurement_model_path_);
      performance_timer_ = create_wall_timer(
        std::chrono::duration<double>(performance_measurement_power_sample_interval_sec_),
        std::bind(&AutoControlNode::on_performance_timer, this));
      RCLCPP_WARN(get_logger(),
        "Performance measurement enabled: C++ monitor_only safety mode, duration=%.1fs "
        "after the first valid centerline, startup timeout=%.1fs, power=%s",
        performance_measurement_duration_sec_, performance_measurement_startup_timeout_sec_,
        measurement_->power_description().c_str());
    }
    if (enabled_ && control_mode_ == "drive") {
      RCLCPP_WARN(get_logger(),
        "Automatic control is armed at launch. The vehicle will move when VESC telemetry "
        "and a valid ML centerline are both available.");
    } else if (enabled_) {
      RCLCPP_WARN(get_logger(),
        "Automatic control starts in %s mode; suppressed actuator topics will not be published.",
        control_mode_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Automatic control starts disabled.");
    }
    RCLCPP_INFO(get_logger(),
      "Auto control C++ ready: mode=%s, lane=%s, speed=%.2f..%.2fm/s, "
      "duty=%.3f..%.3f, auto_brake=%s/%.1fA, control=on_lane_result, watchdog=%.1fHz",
      control_mode_.c_str(), lane_result_topic_.c_str(), minimum_speed_mps_, maximum_speed_mps_,
      minimum_duty_, maximum_duty_, electrical_brake_enabled_ ? "on" : "off",
      brake_maximum_current_amps_, control_rate_hz_);
  }

  ~AutoControlNode() override
  {
    if (measurement_ && !performance_finished_) {
      performance_finished_ = true;
      try {
        const auto path = measurement_->write("interrupted");
        RCLCPP_WARN(get_logger(), "Interrupted performance measurement written to %s", path.c_str());
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(get_logger(), "Failed to write interrupted measurement: %s", exception.what());
      }
    }
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & default_value)
  {
    return declare_parameter<T>(name, default_value);
  }

  void declare_and_read_parameters()
  {
    enabled_ = parameter("enabled", true);
    control_mode_ = parameter<std::string>("control_mode", "drive");
    enable_topic_ = parameter<std::string>("enable_topic", "/auto/enabled");
    lane_result_topic_ = parameter<std::string>("lane_result_topic", "/line_detactor/result");
    lane_result_frame_id_ = parameter<std::string>("lane_result_frame_id", "front_axle_bev");
    path_maximum_gap_m_ = parameter("path_maximum_gap_m", 0.15);
    measured_erpm_topic_ = parameter<std::string>("measured_erpm_topic", "/vesc/measured_erpm");
    connection_status_topic_ = parameter<std::string>("connection_status_topic", "/vesc/connected");
    duty_topic_ = parameter<std::string>("duty_topic", "/vesc/duty");
    brake_current_topic_ = parameter<std::string>("brake_current_topic", "/vesc/brake_current");
    servo_position_topic_ = parameter<std::string>("servo_position_topic", "/vesc/servo_position");
    command_duty_topic_ = parameter<std::string>("command_duty_topic", "/auto/current_duty");
    command_brake_current_topic_ = parameter<std::string>(
      "command_brake_current_topic", "/auto/current_brake_current");
    target_speed_topic_ = parameter<std::string>("target_speed_topic", "/auto/target_speed");
    current_speed_topic_ = parameter<std::string>("current_speed_topic", "/auto/current_speed");
    curvature_topic_ = parameter<std::string>("curvature_topic", "/auto/path_curvature");
    steering_angle_topic_ = parameter<std::string>("steering_angle_topic", "/auto/steering_angle_rad");
    cross_track_error_topic_ = parameter<std::string>("cross_track_error_topic", "/auto/cross_track_error_m");
    heading_error_topic_ = parameter<std::string>("heading_error_topic", "/auto/heading_error_rad");
    raw_steering_angle_topic_ = parameter<std::string>(
      "raw_steering_angle_topic", "/auto/raw_steering_angle_rad");
    command_servo_position_topic_ = parameter<std::string>(
      "command_servo_position_topic", "/auto/current_servo_position");

    control_rate_hz_ = parameter("control_rate_hz", 80.0);
    status_log_rate_hz_ = parameter("status_log_rate_hz", 2.0);
    path_timeout_sec_ = parameter("path_timeout_sec", 0.15);
    path_capture_maximum_age_sec_ = parameter("path_capture_maximum_age_sec", 0.20);
    erpm_timeout_sec_ = parameter("erpm_timeout_sec", 0.15);
    bev_x_max_m_ = parameter("bev_x_max_m", 3.0);
    bev_y_max_m_ = parameter("bev_y_max_m", 0.60);
    bev_meter_per_pixel_ = parameter("bev_meter_per_pixel", 0.01);
    path_minimum_x_m_ = parameter("path_minimum_x_m", 0.05);
    path_maximum_x_m_ = parameter("path_maximum_x_m", 2.20);
    path_minimum_points_ = parameter("path_minimum_points", 8);
    path_minimum_span_m_ = parameter("path_minimum_span_m", 0.12);
    path_geometry_window_m_ = parameter("path_geometry_window_m", 0.14);
    stanley_gain_ = parameter("stanley_gain", 1.40);
    stanley_softening_speed_mps_ = parameter("stanley_softening_speed_mps", 0.40);
    stanley_heading_lookahead_m_ = parameter("stanley_heading_lookahead_m", 0.15);
    stanley_corner_heading_threshold_rad_ = radians(parameter(
      "stanley_corner_heading_threshold_deg", 4.0));
    stanley_corner_opposing_correction_ratio_ = parameter(
      "stanley_corner_opposing_correction_ratio", 0.45);
    maximum_steering_angle_rad_ = radians(parameter("maximum_steering_angle_deg", 30.0));
    steering_current_weight_ = parameter("steering_current_weight", 0.47);
    steering_rate_limit_rad_per_sec_ = radians(parameter("steering_rate_limit_deg_per_sec", 240.0));
    servo_left_ = parameter("servo_left", 1.0);
    servo_center_ = parameter("servo_center", 0.46);
    servo_right_ = parameter("servo_right", 0.0);
    steering_servo_inverted_ = parameter("steering_servo_inverted", true);
    minimum_speed_mps_ = parameter("minimum_speed_mps", 0.80);
    maximum_speed_mps_ = parameter("maximum_speed_mps", 1.8);
    maximum_lateral_acceleration_mps2_ = parameter("maximum_lateral_acceleration_mps2", 0.6);
    curvature_lookahead_minimum_x_m_ = parameter("curvature_lookahead_minimum_x_m", 0.50);
    curvature_lookahead_maximum_x_m_ = parameter("curvature_lookahead_maximum_x_m", 1.60);
    curvature_percentile_ = parameter("curvature_percentile", 90.0);
    minimum_duty_ = parameter("minimum_duty", 0.070);
    maximum_duty_ = parameter("maximum_duty", 0.090);
    duty_rise_rate_per_sec_ = parameter("duty_rise_rate_per_sec", 0.04);
    duty_fall_rate_per_sec_ = parameter("duty_fall_rate_per_sec", 0.08);
    speed_pid_kp_ = parameter("speed_pid_kp", 0.012);
    speed_pid_ki_ = parameter("speed_pid_ki", 0.004);
    speed_pid_kd_ = parameter("speed_pid_kd", 0.0);
    speed_pid_integral_limit_ = parameter("speed_pid_integral_limit", 1.0);
    speed_filter_time_constant_sec_ = parameter("speed_filter_time_constant_sec", 0.05);
    electrical_brake_enabled_ = parameter("electrical_brake_enabled", true);
    brake_entry_speed_error_mps_ = parameter("brake_entry_speed_error_mps", 0.10);
    brake_exit_speed_error_mps_ = parameter("brake_exit_speed_error_mps", 0.03);
    brake_minimum_vehicle_speed_mps_ = parameter("brake_minimum_vehicle_speed_mps", 0.20);
    brake_minimum_current_amps_ = parameter("brake_minimum_current_amps", 0.5);
    brake_maximum_current_amps_ = parameter("brake_maximum_current_amps", 2.5);
    brake_current_gain_amps_per_mps_ = parameter("brake_current_gain_amps_per_mps", 6.0);
    brake_current_rise_amps_per_sec_ = parameter("brake_current_rise_amps_per_sec", 6.0);
    brake_current_fall_amps_per_sec_ = parameter("brake_current_fall_amps_per_sec", 16.0);
    wheel_diameter_m_ = parameter("wheel_diameter_m", 0.1095);
    motor_pole_pairs_ = parameter("motor_pole_pairs", 2);
    motor_pinion_teeth_ = parameter("motor_pinion_teeth", 13);
    spur_gear_teeth_ = parameter("spur_gear_teeth", 54);
    differential_pinion_teeth_ = parameter("differential_pinion_teeth", 13);
    differential_ring_teeth_ = parameter("differential_ring_teeth", 37);
    erpm_direction_sign_ = parameter("erpm_direction_sign", 1.0);
    speed_scale_correction_ = parameter("speed_scale_correction", 1.0);
    performance_measurement_enabled_ = parameter("performance_measurement_enabled", false);
    performance_measurement_duration_sec_ = parameter("performance_measurement_duration_sec", 30.0);
    performance_measurement_startup_timeout_sec_ = parameter(
      "performance_measurement_startup_timeout_sec", 300.0);
    performance_measurement_power_sample_interval_sec_ = parameter(
      "performance_measurement_power_sample_interval_sec", 0.1);
    performance_measurement_log_directory_ = parameter<std::string>(
      "performance_measurement_log_directory", "performance_logs");
    performance_measurement_engine_precision_ = parameter<std::string>(
      "performance_measurement_engine_precision", "unknown");
    performance_measurement_model_path_ = parameter<std::string>(
      "performance_measurement_model_path", "");
  }

  void validate_parameters() const
  {
    const auto finite_positive = [](std::initializer_list<double> values) {
        return std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value) && value > 0.0;
          });
      };
    const auto finite = [](std::initializer_list<double> values) {
        return std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value);
          });
      };
    if (control_mode_ != "drive" && control_mode_ != "steering_only" && control_mode_ != "monitor_only") {
      throw std::invalid_argument("control_mode must be drive, steering_only, or monitor_only");
    }
    if (lane_result_topic_.empty() || lane_result_frame_id_.empty()) {
      throw std::invalid_argument("lane result topic and frame ID must not be empty");
    }
    if (!finite_positive({
        control_rate_hz_, status_log_rate_hz_, path_timeout_sec_,
        path_capture_maximum_age_sec_, erpm_timeout_sec_, path_maximum_gap_m_,
        bev_x_max_m_, bev_y_max_m_, bev_meter_per_pixel_, path_minimum_span_m_,
        path_geometry_window_m_, maximum_steering_angle_rad_,
        steering_rate_limit_rad_per_sec_, minimum_speed_mps_,
        maximum_lateral_acceleration_mps2_, duty_rise_rate_per_sec_,
        duty_fall_rate_per_sec_, speed_filter_time_constant_sec_, wheel_diameter_m_,
        speed_scale_correction_, performance_measurement_duration_sec_,
        performance_measurement_startup_timeout_sec_,
        performance_measurement_power_sample_interval_sec_}))
    {throw std::invalid_argument("positive auto-control parameter is not positive");}
    if (path_minimum_points_ < 3 || path_minimum_x_m_ >= path_maximum_x_m_) {
      throw std::invalid_argument("invalid path acceptance parameters");
    }
    if (!(minimum_duty_ > 0.0 && minimum_duty_ <= maximum_duty_ && maximum_duty_ <= 1.0)) {
      throw std::invalid_argument("duty limits must satisfy 0 < minimum <= maximum <= 1");
    }
    if (!finite({maximum_speed_mps_, steering_current_weight_, curvature_percentile_,
        speed_pid_integral_limit_, stanley_corner_heading_threshold_rad_,
        stanley_corner_opposing_correction_ratio_}) || maximum_speed_mps_ < minimum_speed_mps_ ||
      steering_current_weight_ <= 0.0 || steering_current_weight_ > 1.0 ||
      curvature_percentile_ < 0.0 || curvature_percentile_ > 100.0 ||
      speed_pid_integral_limit_ < 0.0 || stanley_corner_heading_threshold_rad_ < 0.0 ||
      stanley_corner_opposing_correction_ratio_ < 0.0 ||
      stanley_corner_opposing_correction_ratio_ >= 1.0)
    {throw std::invalid_argument("invalid speed, steering weight, or curvature percentile");}
    if (!finite({brake_entry_speed_error_mps_, brake_exit_speed_error_mps_,
        brake_minimum_vehicle_speed_mps_, brake_minimum_current_amps_,
        brake_maximum_current_amps_, brake_current_gain_amps_per_mps_,
        brake_current_rise_amps_per_sec_, brake_current_fall_amps_per_sec_}) ||
      brake_entry_speed_error_mps_ <= 0.0 || brake_exit_speed_error_mps_ < 0.0 ||
      brake_exit_speed_error_mps_ >= brake_entry_speed_error_mps_ ||
      brake_minimum_vehicle_speed_mps_ < 0.0 || brake_minimum_current_amps_ < 0.0 ||
      brake_maximum_current_amps_ < brake_minimum_current_amps_ ||
      brake_maximum_current_amps_ <= 0.0 || brake_current_gain_amps_per_mps_ <= 0.0 ||
      brake_current_rise_amps_per_sec_ <= 0.0 || brake_current_fall_amps_per_sec_ <= 0.0)
    {throw std::invalid_argument("invalid automatic brake parameters");}
    if (std::min({motor_pole_pairs_, motor_pinion_teeth_, spur_gear_teeth_,
        differential_pinion_teeth_, differential_ring_teeth_}) <= 0)
    {throw std::invalid_argument("motor and gear parameters must be positive");}
    if (performance_measurement_log_directory_.empty()) {
      throw std::invalid_argument("performance measurement log directory must not be empty");
    }
  }

  void on_lane_result(const line_detactor::msg::LaneResult::ConstSharedPtr message)
  {
    const auto callback_started = PerformanceMeasurement::SteadyClock::now();
    const auto received_ros = now();
    ++lane_result_count_;
    lane_result_points_ = message->centerline_points.size();
    lane_capture_age_sec_.reset();
    path_.reset();
    last_path_received_ns_ = received_ros.nanoseconds();
    try {
      const auto capture_ns = stamp_nanoseconds(message->header.stamp);
      if (!capture_ns) {throw std::invalid_argument("missing source capture timestamp");}
      const double age = seconds(received_ros.nanoseconds() - *capture_ns);
      lane_capture_age_sec_ = age;
      if (age < -0.05 || age > path_capture_maximum_age_sec_) {
        throw std::invalid_argument("source capture timestamp is stale or in the future");
      }
      if (path_capture_ns_ && *capture_ns <= *path_capture_ns_) {
        throw std::invalid_argument("duplicate or out-of-order lane result");
      }
      path_capture_ns_ = capture_ns;
      if (message->centerline_sample_limit_reached) {
        lane_result_status_ = "centerline_sample_limit";
      } else if (!message->centerline_valid) {
        lane_result_status_ = "no_centerline(state=" + std::to_string(message->state) + ")";
      } else {
        validate_and_build_path(*message);
      }
    } catch (const std::exception & exception) {
      path_.reset();
      lane_result_status_ = exception.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Rejected ML centerline: %s", exception.what());
    }
    const bool servo_calculated = update_control_from_path();
    const auto callback_finished = PerformanceMeasurement::SteadyClock::now();
    if (performance_measurement_enabled_) {
      record_performance_frame(
        *message, received_ros.nanoseconds(), callback_started, callback_finished,
        path_.has_value(), servo_calculated);
    }
  }

  void validate_and_build_path(const line_detactor::msg::LaneResult & message)
  {
    if (message.state != line_detactor::msg::LaneResult::LEFT_ONLY &&
      message.state != line_detactor::msg::LaneResult::RIGHT_ONLY &&
      message.state != line_detactor::msg::LaneResult::BOTH)
    {throw std::invalid_argument("centerline has no observed lane support");}
    if (message.header.frame_id != lane_result_frame_id_) {
      throw std::invalid_argument("unexpected lane result frame: " + message.header.frame_id);
    }
    const int width = static_cast<int>(message.source_width);
    const int height = static_cast<int>(message.source_height);
    const int padding_left = static_cast<int>(message.padding_left);
    const int padding_right = static_cast<int>(message.padding_right);
    if (width <= 0 || height <= 0) {throw std::invalid_argument("invalid source geometry");}
    const double scale_x = message.centerline_bev_width_m / width;
    const double scale_y = message.centerline_bev_height_m / height;
    const auto close = [](double a, double b) {
        return std::abs(a - b) <= 1.0e-4 * std::max(std::abs(a), std::abs(b));};
    if (!std::isfinite(scale_x) || !std::isfinite(scale_y) || scale_x <= 0.0 || scale_y <= 0.0 ||
      !close(scale_x, bev_meter_per_pixel_) || !close(scale_y, bev_meter_per_pixel_) ||
      !close(width * scale_x, 2.0 * bev_y_max_m_) || !close(height * scale_y, bev_x_max_m_))
    {throw std::invalid_argument("lane result scale disagrees with configured symmetric BEV");}
    const auto count = message.centerline_points.size();
    if (count < 2U || count > 10000U || message.centerline_support.size() != count) {
      throw std::invalid_argument("invalid centerline point/support lengths");
    }
    std::vector<Point> metric_points;
    metric_points.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      const auto & point = message.centerline_points[index];
      const auto support = message.centerline_support[index];
      if (support < line_detactor::msg::LaneResult::CENTER_SINGLE ||
        support > line_detactor::msg::LaneResult::CENTER_OUTER)
      {throw std::invalid_argument("unknown centerline support value");}
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        std::abs(point.z) > 1.0e-6 || point.x < 0.0F ||
        point.x >= width + padding_left + padding_right || point.y < 0.0F || point.y >= height)
      {throw std::invalid_argument("centerline points are outside their result canvas");}
      metric_points.push_back({
        bev_x_max_m_ - (point.y + 0.5) * scale_y,
        bev_y_max_m_ - (point.x - padding_left + 0.5) * scale_x});
    }
    path_ = build_ordered_path(
      metric_points, path_minimum_points_, path_minimum_span_m_, path_minimum_x_m_,
      path_maximum_x_m_, path_maximum_gap_m_, path_geometry_window_m_);
    lane_result_status_ = path_ ? "accepted" : "insufficient_contiguous_path";
  }

  std::optional<std::string> stop_reason(std::int64_t now_ns) const
  {
    if (!enabled_) {return "disabled";}
    if (control_mode_ != "monitor_only") {
      if (!vesc_connected_) {return "vesc_disconnected";}
      if (!last_erpm_ns_) {return "waiting_for_erpm";}
      if (seconds(now_ns - *last_erpm_ns_) > erpm_timeout_sec_) {return "erpm_timeout";}
    }
    if (!path_ || !last_path_received_ns_) {return "centerline_missing";}
    if (seconds(now_ns - *last_path_received_ns_) > path_timeout_sec_) {return "centerline_timeout";}
    if (path_capture_ns_ && seconds(now_ns - *path_capture_ns_) > path_capture_maximum_age_sec_) {
      return "centerline_stale";
    }
    return std::nullopt;
  }

  bool update_control_from_path()
  {
    const std::int64_t now_ns = now().nanoseconds();
    const double dt = clamp(seconds(now_ns - last_control_ns_), 1.0e-6,
      std::min({path_timeout_sec_, path_capture_maximum_age_sec_, erpm_timeout_sec_}));
    last_control_ns_ = now_ns;
    if (const auto reason = stop_reason(now_ns)) {
      stop_control(*reason);
      publish_commands(0.0, 0.0, servo_center_, "stop");
      return false;
    }
    const double curvature = representative_curvature(
      *path_, curvature_lookahead_minimum_x_m_, curvature_lookahead_maximum_x_m_,
      curvature_percentile_);
    const double target_speed = curvature_target_speed(
      curvature, maximum_lateral_acceleration_mps2_, minimum_speed_mps_, maximum_speed_mps_);
    const auto stanley = stanley_control(
      *path_, current_speed_mps_, stanley_gain_, stanley_softening_speed_mps_,
      stanley_heading_lookahead_m_, maximum_steering_angle_rad_,
      stanley_corner_heading_threshold_rad_, stanley_corner_opposing_correction_ratio_);
    const bool corner_reset =
      std::abs(stanley.heading_error_rad) >= stanley_corner_heading_threshold_rad_ &&
      stanley.steering_angle_rad * steering_angle_rad_ < 0.0;
    double filtered = stanley.steering_angle_rad;
    if (corner_reset) {
      steering_angle_rad_ = 0.0;
    } else {
      const double current_weight = 1.0 - std::pow(
        1.0 - steering_current_weight_, dt * control_rate_hz_);
      filtered = current_weight * stanley.steering_angle_rad +
        (1.0 - current_weight) * steering_angle_rad_;
    }
    steering_angle_rad_ = move_toward(
      steering_angle_rad_, filtered, steering_rate_limit_rad_per_sec_ * dt);
    const double servo = steering_angle_to_servo(
      steering_angle_rad_, maximum_steering_angle_rad_, servo_left_, servo_center_,
      servo_right_, steering_servo_inverted_);
    std::string motor_mode;
    if (control_mode_ != "drive") {
      command_duty_ = 0.0;
      command_brake_current_ = 0.0;
      brake_mode_active_ = false;
      speed_pid_->reset();
      brake_profile_->reset();
      motor_mode = "suppressed";
    } else {
      command_brake_current_ = electrical_brake_enabled_ ?
        brake_profile_->update(target_speed, current_speed_mps_, dt) : 0.0;
      if (!electrical_brake_enabled_) {brake_profile_->reset();}
      if (command_brake_current_ > 0.0) {
        command_duty_ = 0.0;
        speed_pid_->reset();
        motor_mode = "brake";
        brake_mode_active_ = true;
      } else if (brake_mode_active_) {
        command_duty_ = 0.0;
        speed_pid_->reset();
        motor_mode = "brake_release";
        brake_mode_active_ = false;
      } else {
        const double feedforward = speed_feedforward_duty(
          target_speed, minimum_speed_mps_, maximum_speed_mps_, minimum_duty_, maximum_duty_);
        const double desired = speed_pid_->update(target_speed, current_speed_mps_, feedforward, dt);
        if (command_duty_ < minimum_duty_) {
          command_duty_ = minimum_duty_;
        } else {
          const double rate = desired >= command_duty_ ? duty_rise_rate_per_sec_ : duty_fall_rate_per_sec_;
          command_duty_ = move_toward(command_duty_, desired, rate * dt);
        }
        motor_mode = "duty";
      }
    }
    last_stop_reason_ = "running";
    latest_target_speed_mps_ = target_speed;
    latest_curvature_per_m_ = curvature;
    latest_cross_track_error_m_ = stanley.cross_track_error_m;
    latest_heading_error_rad_ = stanley.heading_error_rad;
    latest_raw_steering_angle_rad_ = stanley.steering_angle_rad;
    latest_direction_guard_used_ = stanley.direction_guard_used || corner_reset;
    publish_commands(command_duty_, command_brake_current_, servo, motor_mode);
    return true;
  }

  void stop_control(const std::string & reason)
  {
    last_control_ns_ = now().nanoseconds();
    command_duty_ = 0.0;
    command_brake_current_ = 0.0;
    brake_mode_active_ = false;
    steering_angle_rad_ = 0.0;
    latest_target_speed_mps_ = 0.0;
    latest_curvature_per_m_ = 0.0;
    latest_cross_track_error_m_ = 0.0;
    latest_heading_error_rad_ = 0.0;
    latest_raw_steering_angle_rad_ = 0.0;
    latest_servo_position_ = servo_center_;
    latest_direction_guard_used_ = false;
    speed_pid_->reset();
    brake_profile_->reset();
    if (reason != last_stop_reason_) {
      RCLCPP_WARN(get_logger(), "Automatic drive stopped: %s. Releasing motor command.", reason.c_str());
    }
    last_stop_reason_ = reason;
  }

  void publish_float(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher, double value)
  {
    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(value);
    publisher->publish(message);
  }

  void publish_diagnostic(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher, double value)
  {
    if (publisher->get_subscription_count() == 0U &&
      publisher->get_intra_process_subscription_count() == 0U)
    {
      return;
    }
    publish_float(publisher, value);
  }

  void publish_commands(
    double duty, double brake_current, double servo_position, const std::string & motor_mode)
  {
    if (control_mode_ == "drive") {
      if (motor_mode == "duty") {publish_float(duty_pub_, duty);}
      else if (motor_mode == "brake") {publish_float(brake_pub_, brake_current);}
      else if (motor_mode == "brake_release") {publish_float(brake_pub_, 0.0);}
      else if (motor_mode == "stop") {
        publish_float(brake_pub_, 0.0);
        publish_float(duty_pub_, 0.0);
      } else {throw std::invalid_argument("unsupported motor mode: " + motor_mode);}
    } else if (motor_mode != "suppressed" && motor_mode != "stop") {
      throw std::invalid_argument("unsupported passive motor mode: " + motor_mode);
    }
    last_motor_mode_ = motor_mode;
    latest_servo_position_ = servo_position;
    if (control_mode_ != "monitor_only") {publish_float(servo_pub_, servo_position);}
    publish_diagnostic(command_duty_pub_, duty);
    publish_diagnostic(command_brake_pub_, brake_current);
    publish_diagnostic(target_speed_pub_, latest_target_speed_mps_);
    publish_diagnostic(current_speed_pub_, current_speed_mps_);
    publish_diagnostic(curvature_pub_, latest_curvature_per_m_);
    publish_diagnostic(steering_pub_, steering_angle_rad_);
    publish_diagnostic(cross_track_pub_, latest_cross_track_error_m_);
    publish_diagnostic(heading_pub_, latest_heading_error_rad_);
    publish_diagnostic(raw_steering_pub_, latest_raw_steering_angle_rad_);
    publish_diagnostic(command_servo_pub_, latest_servo_position_);
  }

  void on_measured_erpm(const std_msgs::msg::Int32::ConstSharedPtr message)
  {
    const auto now_ns = now().nanoseconds();
    const double raw = erpm_to_speed_mps(
      message->data, wheel_diameter_m_, motor_pole_pairs_, motor_pinion_teeth_,
      spur_gear_teeth_, differential_pinion_teeth_, differential_ring_teeth_,
      erpm_direction_sign_, speed_scale_correction_);
    if (!last_erpm_ns_) {
      current_speed_mps_ = raw;
    } else {
      const double dt = seconds(now_ns - *last_erpm_ns_);
      if (dt <= 0.0 || dt > erpm_timeout_sec_) {current_speed_mps_ = raw;}
      else {
        const double gain = 1.0 - std::exp(-dt / std::max(1.0e-4, speed_filter_time_constant_sec_));
        current_speed_mps_ += gain * (raw - current_speed_mps_);
      }
    }
    last_erpm_ns_ = now_ns;
  }

  void on_connection_status(const std_msgs::msg::Bool::ConstSharedPtr message)
  {
    vesc_connected_ = message->data;
  }

  void on_enable(const std_msgs::msg::Bool::ConstSharedPtr message)
  {
    enabled_ = message->data;
    if (!enabled_) {
      stop_control("disabled");
      publish_commands(0.0, 0.0, servo_center_, "stop");
    }
    RCLCPP_WARN(get_logger(), "Automatic control %s.", enabled_ ? "enabled" : "disabled");
  }

  void on_watchdog()
  {
    if (const auto reason = stop_reason(now().nanoseconds())) {
      stop_control(*reason);
      publish_commands(0.0, 0.0, servo_center_, "stop");
    }
  }

  void record_performance_frame(
    const line_detactor::msg::LaneResult & message,
    std::int64_t callback_received_ros_ns,
    PerformanceMeasurement::SteadyClock::time_point callback_started,
    PerformanceMeasurement::SteadyClock::time_point callback_finished,
    bool valid_centerline, bool servo_position_calculated)
  {
    if (!measurement_ || performance_finished_) {return;}
    if (!measurement_->active()) {
      if (!valid_centerline || !servo_position_calculated) {return;}
      measurement_->start(callback_started);
      RCLCPP_INFO(get_logger(),
        "First valid ML centerline reached C++ servo-position calculation; "
        "starting %.1fs performance measurement.", performance_measurement_duration_sec_);
    }
    const std::int64_t callback_finished_ros_ns = now().nanoseconds();
    const auto source_ns = stamp_nanoseconds(message.header.stamp);
    const auto detector_input_ns = stamp_nanoseconds(message.detector_input_received_stamp);
    const auto result_ready_ns = stamp_nanoseconds(message.detector_result_ready_stamp);
    const double detector_compute = message.detector_total_compute_nanoseconds / 1.0e6;
    const double control_compute = std::chrono::duration<double, std::milli>(
      callback_finished - callback_started).count();
    PerformanceFrame frame;
    frame.measurement_elapsed_sec = measurement_->elapsed_sec(callback_finished);
    frame.detector_sequence = message.detector_sequence;
    frame.actual_engine_precision = message.engine_precision;
    frame.valid_centerline = valid_centerline;
    frame.servo_position_calculated = servo_position_calculated;
    frame.centerline_point_count = message.centerline_points.size();
    frame.steering_angle_rad = steering_angle_rad_;
    frame.servo_position = latest_servo_position_;
    frame.h2d_preprocess_ms = message.h2d_preprocess_nanoseconds / 1.0e6;
    frame.pure_inference_ms = message.pure_inference_nanoseconds / 1.0e6;
    frame.label_export_ms = message.label_export_nanoseconds / 1.0e6;
    frame.backend_postprocess_ms = message.backend_postprocess_nanoseconds / 1.0e6;
    frame.lane_geometry_ms = message.lane_geometry_nanoseconds / 1.0e6;
    frame.result_message_build_ms = message.result_message_build_nanoseconds / 1.0e6;
    frame.lane_postprocess_total_ms =
      (message.label_export_nanoseconds + message.backend_postprocess_nanoseconds +
      message.lane_geometry_nanoseconds + message.result_message_build_nanoseconds) / 1.0e6;
    frame.detector_queue_ms = message.detector_queue_nanoseconds / 1.0e6;
    frame.detector_total_compute_ms = detector_compute;
    frame.lane_result_transport_ms = latency_ms(result_ready_ns, callback_received_ros_ns);
    frame.source_to_detector_input_ms = latency_ms(source_ns, detector_input_ns);
    frame.auto_control_compute_ms = control_compute;
    frame.compute_only_total_ms = detector_compute + control_compute;
    frame.detector_input_to_control_complete_ms = latency_ms(
      detector_input_ns, callback_finished_ros_ns);
    frame.source_capture_to_control_complete_ms = latency_ms(
      source_ns, callback_finished_ros_ns);
    measurement_->add_frame(std::move(frame));
  }

  void on_performance_timer()
  {
    if (!measurement_ || performance_finished_) {return;}
    if (measurement_->startup_timed_out()) {finish_performance("startup_timeout"); return;}
    if (!measurement_->active()) {return;}
    measurement_->sample_power();
    if (measurement_->due_to_finish()) {finish_performance("complete");}
  }

  void finish_performance(const std::string & status)
  {
    if (!measurement_ || performance_finished_) {return;}
    performance_finished_ = true;
    try {
      const auto path = measurement_->write(status);
      RCLCPP_INFO(get_logger(), "PERFORMANCE_MEASUREMENT_COMPLETE status=%s file=%s frames=%zu",
        status.c_str(), path.c_str(), measurement_->frame_count());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Failed to write performance measurement: %s", exception.what());
    }
    enabled_ = false;
    stop_control("shutdown");
    publish_commands(0.0, 0.0, servo_center_, "stop");
    watchdog_timer_->cancel();
    status_timer_->cancel();
    performance_timer_->cancel();
    rclcpp::shutdown();
  }

  void log_status()
  {
    const auto current_ns = now().nanoseconds();
    const std::string receive_age = last_path_received_ns_ ?
      std::to_string(seconds(current_ns - *last_path_received_ns_)) + "s" : "never";
    const std::string capture_age = lane_capture_age_sec_ ?
      std::to_string(*lane_capture_age_sec_) + "s" : "unknown";
    RCLCPP_INFO(get_logger(),
      "Auto status | state=%s | path_points=%zu | speed=%.2f/%.2fm/s | curvature=%.3f/m | "
      "cte=%+.3fm | heading=%+.1fdeg | guard=%s | raw/final_steering=%+.1f/%+.1fdeg | "
      "servo=%.3f | motor=%s | duty=%.4f | brake=%.2fA | lane_rx=%zu | lane_status=%s | "
      "source_points=%zu | last_rx_age=%s | capture_age_at_rx=%s",
      last_stop_reason_.c_str(), path_ ? path_->points.size() : 0U,
      current_speed_mps_, latest_target_speed_mps_, latest_curvature_per_m_,
      latest_cross_track_error_m_, latest_heading_error_rad_ * 180.0 / kPi,
      latest_direction_guard_used_ ? "on" : "off",
      latest_raw_steering_angle_rad_ * 180.0 / kPi, steering_angle_rad_ * 180.0 / kPi,
      latest_servo_position_, last_motor_mode_.c_str(), command_duty_, command_brake_current_,
      lane_result_count_, lane_result_status_.c_str(), lane_result_points_,
      receive_age.c_str(), capture_age.c_str());
  }

  bool enabled_{true};
  bool electrical_brake_enabled_{true};
  bool steering_servo_inverted_{true};
  bool performance_measurement_enabled_{false};
  std::string control_mode_, enable_topic_, lane_result_topic_, lane_result_frame_id_;
  std::string measured_erpm_topic_, connection_status_topic_, duty_topic_;
  std::string brake_current_topic_, servo_position_topic_, command_duty_topic_;
  std::string command_brake_current_topic_, target_speed_topic_, current_speed_topic_;
  std::string curvature_topic_, steering_angle_topic_, cross_track_error_topic_;
  std::string heading_error_topic_, raw_steering_angle_topic_, command_servo_position_topic_;
  std::string performance_measurement_log_directory_, performance_measurement_engine_precision_;
  std::string performance_measurement_model_path_;
  double control_rate_hz_, status_log_rate_hz_, path_timeout_sec_;
  double path_capture_maximum_age_sec_, erpm_timeout_sec_, bev_x_max_m_, bev_y_max_m_;
  double bev_meter_per_pixel_, path_minimum_x_m_, path_maximum_x_m_, path_minimum_span_m_;
  double path_maximum_gap_m_, path_geometry_window_m_, stanley_gain_;
  double stanley_softening_speed_mps_, stanley_heading_lookahead_m_;
  double stanley_corner_heading_threshold_rad_, stanley_corner_opposing_correction_ratio_;
  double maximum_steering_angle_rad_, steering_current_weight_, steering_rate_limit_rad_per_sec_;
  double servo_left_, servo_center_, servo_right_, minimum_speed_mps_, maximum_speed_mps_;
  double maximum_lateral_acceleration_mps2_, curvature_lookahead_minimum_x_m_;
  double curvature_lookahead_maximum_x_m_, curvature_percentile_, minimum_duty_, maximum_duty_;
  double duty_rise_rate_per_sec_, duty_fall_rate_per_sec_, speed_pid_kp_, speed_pid_ki_;
  double speed_pid_kd_, speed_pid_integral_limit_, speed_filter_time_constant_sec_;
  double brake_entry_speed_error_mps_, brake_exit_speed_error_mps_;
  double brake_minimum_vehicle_speed_mps_, brake_minimum_current_amps_;
  double brake_maximum_current_amps_, brake_current_gain_amps_per_mps_;
  double brake_current_rise_amps_per_sec_, brake_current_fall_amps_per_sec_;
  double wheel_diameter_m_, erpm_direction_sign_, speed_scale_correction_;
  double performance_measurement_duration_sec_, performance_measurement_startup_timeout_sec_;
  double performance_measurement_power_sample_interval_sec_;
  int path_minimum_points_, motor_pole_pairs_, motor_pinion_teeth_, spur_gear_teeth_;
  int differential_pinion_teeth_, differential_ring_teeth_;

  std::optional<OrderedPath> path_;
  std::optional<std::int64_t> last_path_received_ns_, path_capture_ns_, last_erpm_ns_;
  std::optional<double> lane_capture_age_sec_;
  std::int64_t last_control_ns_{0};
  std::size_t lane_result_count_{0U}, lane_result_points_{0U};
  std::string lane_result_status_{"waiting_for_lane_result"};
  bool vesc_connected_{false}, brake_mode_active_{false}, latest_direction_guard_used_{false};
  double current_speed_mps_{0.0}, command_duty_{0.0}, command_brake_current_{0.0};
  double steering_angle_rad_{0.0}, latest_target_speed_mps_{0.0}, latest_curvature_per_m_{0.0};
  double latest_cross_track_error_m_{0.0}, latest_heading_error_rad_{0.0};
  double latest_raw_steering_angle_rad_{0.0}, latest_servo_position_{0.0};
  std::string last_motor_mode_{"stop"}, last_stop_reason_{"startup"};
  std::unique_ptr<SpeedPid> speed_pid_;
  std::unique_ptr<AutomaticBrakeProfile> brake_profile_;
  std::unique_ptr<PerformanceMeasurement> measurement_;
  bool performance_finished_{false};

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_pub_, brake_pub_, servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr command_duty_pub_, command_brake_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_speed_pub_, current_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr curvature_pub_, steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cross_track_pub_, heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_steering_pub_, command_servo_pub_;
  rclcpp::Subscription<line_detactor::msg::LaneResult>::SharedPtr lane_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr erpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connection_sub_, enable_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_, status_timer_, performance_timer_;
};

}  // namespace auto_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<auto_control::AutoControlNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("auto_control"), "%s", exception.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
