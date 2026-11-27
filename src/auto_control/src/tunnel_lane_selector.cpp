// C++ composable replacement for scripts/tunnel_lane_selector (auto_control_tunnel/node.py).
// Loaded into the detector container, so RGB/IR LaneResult arrive intra-process and only the
// forwarded result crosses to the controller process.
#include <array>
#include <chrono>
#include <functional>
#include <optional>
#include <vector>
#include <memory>
#include <string>

#include "auto_control/tunnel_selection.hpp"
#include "bev_handoff/ir_camera_settings.hpp"
#include "line_detactor_msgs/msg/lane_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace auto_control
{

using line_detactor_msgs::msg::LaneResult;
using std_msgs::msg::String;
using ForceIr = std_srvs::srv::SetBool;
using namespace std::chrono_literals;

class TunnelLaneSelector : public rclcpp::Node
{
public:
  explicit TunnelLaneSelector(const rclcpp::NodeOptions & options)
  : rclcpp::Node("tunnel_lane_selector", options),
    policy_(make_policy())
  {
    const auto sensor = rclcpp::QoS(1).best_effort();
    // Humble intra-process cannot carry transient_local; latched status stays inter-process.
    rclcpp::PublisherOptions latched_options;
    latched_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    rgb_output_ = create_publisher<LaneResult>("/tunnel/lane_result/rgb", sensor);
    ir_output_ = create_publisher<LaneResult>("/tunnel/lane_result/ir", sensor);
    for (const char * key : {"state", "target_source", "selected_source", "compute_source"}) {
      status_.push_back(create_publisher<String>(std::string("/tunnel/") + key, latched, latched_options));
    }
    preview_pub_ = create_publisher<String>("/tunnel/preview_status", latched, latched_options);
    force_ir_pub_ = create_publisher<std_msgs::msg::Bool>("/tunnel/force_ir_enabled", latched, latched_options);
    tcn_state_override_pub_ = create_publisher<String>("/tunnel_tcn/state_override", latched, latched_options);
    force_ir_service_ = create_service<ForceIr>("/tunnel/set_force_ir",
      [this](std::shared_ptr<rmw_request_id_t> id, std::shared_ptr<ForceIr::Request> request) {
        set_force_ir(*id, request->data);
      });
    preview_timer_ = create_wall_timer(200ms, [this]() {publish_preview();});
    // 유효 경로 대기 상한·사전 준비 취소 (두 실험 기능이 꺼져 있으면 아무 일도 하지 않음)
    if (policy_.options.valid_handover || policy_.options.prearm_entry || policy_.options.prearm_exit) {
      transition_timer_ = create_wall_timer(20ms, [this]() {
        if (policy_.tick(now().nanoseconds())) {publish_status();}
      });
    }

    rclcpp::SubscriptionOptions inter_process;
    inter_process.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    tcn_sub_ = create_subscription<String>("/tunnel_tcn/result", sensor,
        std::bind(&TunnelLaneSelector::on_event, this, std::placeholders::_1), inter_process);
    rgb_sub_ = create_subscription<LaneResult>(rgb_topic_, sensor,
        [this](LaneResult::ConstSharedPtr message) {on_lane("RGB", message);});
    ir_sub_ = create_subscription<LaneResult>("/ir_bev_line_detection/result", sensor,
        [this](LaneResult::ConstSharedPtr message) {on_lane("IR", message);});
    set_tcn_override("reset_outside");
    publish_status();
    publish_force_ir();
  }

private:
  void publish_force_ir()
  {
    std_msgs::msg::Bool message;
    message.data = force_ir_;
    force_ir_pub_->publish(message);
  }

  void set_tcn_override(const char * state)
  {
    String message;
    message.data = state;
    tcn_state_override_pub_->publish(message);
  }

  void set_force_ir(rmw_request_id_t id, bool enable)
  {
    if (enable != force_ir_) {
      const auto now_ns = now().nanoseconds();
      force_ir_ = enable;
      policy_.request_source(enable ? "IR" : "RGB", now_ns);
      if (enable) {
        set_tcn_override("inside");
      } else {
        resume_tcn_outside(now_ns);
      }
      publish_status();
      publish_force_ir();
    }
    ForceIr::Response response;
    response.success = true;
    force_ir_service_->send_response(id, response);
  }

  void resume_tcn_outside(std::int64_t now_ns)
  {
    tcn_report_after_ns_ = now_ns;
    set_tcn_override("resume_outside");
  }

  void refresh_runtime_settings()
  {
    const auto settings = bev_handoff::tunnelRuntimeSettings();
    if (!settings || settings == runtime_settings_) {return;}
    runtime_settings_ = settings;
    policy_.validation_mode = settings->validation_mode;
  }

  bool apply_tcn_state(const std::string & state, std::int64_t stamp, std::int64_t now_ns)
  {
    const std::string requested = state == "inside" ? "IR" : "RGB";
    if (requested == policy_.requested) {return false;}
    return policy_.event(requested == "IR" ? "entry" : "exit", stamp, now_ns);
  }

  tunnel::LaneSelector make_policy()
  {
    tunnel::PathRules rules;
    rules.frame_id = declare_parameter<std::string>("lane_result_frame_id", "front_axle_bev");
    rules.minimum_points = declare_parameter<int>("path_minimum_points", 8);
    rules.minimum_span = declare_parameter<double>("path_minimum_span_m", .12);
    rules.minimum_x = declare_parameter<double>("path_minimum_x_m", .05);
    rules.maximum_x = declare_parameter<double>("path_maximum_x_m", 2.20);
    rules.maximum_gap = declare_parameter<double>("path_maximum_gap_m", .15);
    rules.x_max = declare_parameter<double>("bev_x_max_m", 3.);
    rules.y_max = declare_parameter<double>("bev_y_max_m", .6);
    rules.mpp = declare_parameter<double>("bev_meter_per_pixel", .01);
    rgb_topic_ = declare_parameter<std::string>("rgb_result_topic", "/line_detactor/result");
    // 실험 기능: launch 인자(tunnel_prearm, tunnel_valid_handover)로만 켠다. 기본은 모두 꺼짐 = 즉시 전환
    tunnel::TransitionOptions options;
    options.prearm_entry = declare_parameter<bool>("prearm_entry", false);
    options.prearm_exit = declare_parameter<bool>("prearm_exit", false);
    options.prearm_score = declare_parameter<double>("prearm_score", .3);
    options.prearm_hold_sec = declare_parameter<double>("prearm_hold_sec", 1.);
    options.valid_handover = declare_parameter<bool>("valid_handover", false);
    options.valid_handover_max_sec = declare_parameter<double>("valid_handover_max_sec", .5);
    RCLCPP_INFO(get_logger(), "Tunnel transition: prearm entry=%s exit=%s score>=%.2f hold=%.1fs | valid handover=%s max=%.2fs",
      options.prearm_entry ? "on" : "off", options.prearm_exit ? "on" : "off", options.prearm_score,
      options.prearm_hold_sec, options.valid_handover ? "on" : "off", options.valid_handover_max_sec);
    return tunnel::LaneSelector(declare_parameter<double>("max_age_sec", .2),
      declare_parameter<bool>("validation_mode", true), rules, options);
  }

  static double monotonic()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publish_status()
  {
    const std::array<std::string, 4> values{policy_.state, policy_.camera(), policy_.selected, policy_.compute()};
    if (published_ && *published_ == values) {return;}
    for (std::size_t i = 0; i < values.size(); ++i) {
      String message;
      message.data = values[i];
      status_[i]->publish(message);
    }
    published_ = values;
    RCLCPP_INFO(get_logger(), "Tunnel %s | selected=%s compute=%s camera=%s reason=%s",
      values[0].c_str(), values[2].c_str(), values[3].c_str(), values[1].c_str(), policy_.reason.c_str());
    if (!policy_.last_handover.empty() && !policy_.transitioning()) {
      RCLCPP_INFO(get_logger(), "Tunnel handover %s", policy_.last_handover.c_str());
      policy_.last_handover.clear();
    }
    publish_preview();
  }

  void publish_preview()
  {
    String message;
    message.data = preview_.text(policy_.state, monotonic());
    preview_pub_->publish(message);
  }

  void on_event(const String::ConstSharedPtr message)
  {
    try {
      refresh_runtime_settings();
      const auto report = tunnel::json_object(message->data);
      const auto stamp = tunnel::json_int(report, "source_stamp_ns");
      const std::int64_t now_ns = now().nanoseconds();
      // 강제 IR을 끈 뒤에는 그 이전에 만들어진 보고를 따르지 않는다 (끈 순간 RGB로 돌아간 것을 되돌리지 않게)
      if (!stamp || (tcn_report_after_ns_ && *stamp <= *tcn_report_after_ns_)) {return;}
      preview_.update(report, monotonic());
      // TCN 판정(state)을 그대로 따른다. 보고 나이·LIVE 여부·이벤트 종류·차선 유무는 보지 않는다 (전환 정책 제거)
      const std::string state = tunnel::json_string(report, "state").value_or("");
      if (state != "inside" && state != "outside") {return;}   // 시작 직후 unknown: 판정 없음
      if (force_ir_) {return;}   // 강제 IR 중에는 수동 지정이 우선
      bool changed = false;
      const auto scores = report.find("scores");
      if (scores != report.end() && !scores->second.empty() && scores->second.front() == '{') {   // 사전 준비 (실험)
        const auto values = tunnel::json_object(scores->second);
        const bool live = tunnel::json_string(report, "status").value_or("").rfind("LIVE", 0) == 0;
        changed = policy_.scores(tunnel::json_double(values, "entry"), tunnel::json_double(values, "exit"), live, now_ns);
      }
      changed = apply_tcn_state(state, *stamp, now_ns) || changed;
      if (changed) {publish_status();}
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "Ignoring malformed TCN result");
    }
  }

  void on_lane(const std::string & source, const LaneResult::ConstSharedPtr & message)
  {
    refresh_runtime_settings();
    // 선택된 출처의 결과만 넘긴다. TCN 상태·결과 신선도로 막지 않는다 (전환 정책 제거)
    const auto now_ns = now().nanoseconds();
    const bool forward = policy_.lane(source, *message, now_ns);
    publish_status();
    if (forward) {
      // Unmodified detector output; the controller learns the source from the topic.
      (source == "IR" ? ir_output_ : rgb_output_)->publish(*message);
    }
  }

  std::string rgb_topic_;
  tunnel::LaneSelector policy_;
  tunnel::PreviewTelemetry preview_;
  std::optional<std::array<std::string, 4>> published_;
  std::shared_ptr<const bev_handoff::TunnelRuntimeSettings> runtime_settings_;
  std::optional<std::int64_t> tcn_report_after_ns_;
  bool force_ir_{false};
  rclcpp::Publisher<LaneResult>::SharedPtr rgb_output_, ir_output_;
  std::vector<rclcpp::Publisher<String>::SharedPtr> status_;
  rclcpp::Publisher<String>::SharedPtr preview_pub_, tcn_state_override_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr force_ir_pub_;
  rclcpp::Service<ForceIr>::SharedPtr force_ir_service_;
  rclcpp::TimerBase::SharedPtr preview_timer_, transition_timer_;
  rclcpp::Subscription<String>::SharedPtr tcn_sub_;
  rclcpp::Subscription<LaneResult>::SharedPtr rgb_sub_, ir_sub_;
};

}  // namespace auto_control

RCLCPP_COMPONENTS_REGISTER_NODE(auto_control::TunnelLaneSelector)
