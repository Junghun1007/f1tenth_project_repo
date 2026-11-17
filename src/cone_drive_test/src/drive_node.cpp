// cone_drive_test 주행 프로세스: OAK → 안정화 → CUDA BEV → 차선 → YOLO → 콘 기억·회피 계획 → 제어 → 웹 프리뷰.
// 자동 명령은 /cone_drive/auto/*로만 낸다 (/vesc/*는 안전 프로세스가 유일하게 발행).
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include "cone_drive_test/core/control_loop.hpp"
#include "cone_drive_test/core/system_monitor.hpp"
#include "cone_drive_test/runtime/camera_pipeline.hpp"
#include "cone_drive_test/runtime/perception.hpp"
#include "cone_drive_test/runtime/web_server.hpp"

using namespace std::chrono_literals;
namespace cdt = cone_drive_test;

class DriveNode : public rclcpp::Node {
public:
  DriveNode() : Node("cone_drive_test") {
    cdt::register_params(registry_);
    for (const auto & p : registry_.list()) declare_parameter<double>(p.name, p.value);   // YAML 시작값
    std::map<std::string, double> start;
    for (const auto & p : registry_.list()) start[p.name] = get_parameter(p.name).as_double();
    const auto applied = registry_.apply(start);
    if (!applied.first) throw std::invalid_argument("parameter file: " + applied.second);

    const auto share = ament_index_cpp::get_package_share_directory("cone_drive_test");
    const auto mode = declare_parameter<std::string>("mode", "drive");
    if (mode != "detect" && mode != "drive") throw std::invalid_argument("mode must be detect or drive");
    rc_.wheelbase = declare_parameter<double>("vehicle.wheelbase_m", 0.33);
    rc_.wheel_diameter_m = declare_parameter<double>("vehicle.wheel_diameter_m", 0.1095);
    rc_.motor_pole_pairs = int(declare_parameter<int>("vehicle.motor_pole_pairs", 2));
    rc_.motor_pinion_teeth = int(declare_parameter<int>("vehicle.motor_pinion_teeth", 13));
    rc_.spur_gear_teeth = int(declare_parameter<int>("vehicle.spur_gear_teeth", 54));
    rc_.differential_pinion_teeth = int(declare_parameter<int>("vehicle.differential_pinion_teeth", 13));
    rc_.differential_ring_teeth = int(declare_parameter<int>("vehicle.differential_ring_teeth", 37));
    rc_.erpm_direction_sign = declare_parameter<double>("vehicle.erpm_direction_sign", 1.0);
    rc_.speed_scale_correction = declare_parameter<double>("vehicle.speed_scale_correction", 1.0);
    rc_.steering.max_steer_rad = declare_parameter<double>("vehicle.maximum_steering_deg", 30.0) * M_PI / 180.0;
    rc_.steering.servo_left = declare_parameter<double>("vehicle.servo_left", 1.0);
    rc_.steering.servo_center = declare_parameter<double>("vehicle.servo_center", 0.5);
    rc_.steering.servo_right = declare_parameter<double>("vehicle.servo_right", 0.0);
    rc_.steering.inverted = declare_parameter<bool>("vehicle.steering_servo_inverted", true);

    cam_cfg_.device_id = declare_parameter<std::string>("camera.device_id", "");
    cam_cfg_.fps = declare_parameter<double>("camera.fps", 70.0);
    // camera.fx/fy는 bev_config.yaml 값(1.25배 고정 확대 포함) 그대로 받고, 원본 영상 기준으로 되돌려 쓴다 (cx/cy는 확대 중심이라 그대로)
    const double fixed_view_zoom = declare_parameter<double>("camera.fixed_view_zoom", 1.25);
    cam_cfg_.fx = declare_parameter<double>("camera.fx", 701.751174926) / fixed_view_zoom;
    cam_cfg_.fy = declare_parameter<double>("camera.fy", 701.420440674) / fixed_view_zoom;
    cam_cfg_.cx = declare_parameter<double>("camera.cx", cam_cfg_.cx);
    cam_cfg_.cy = declare_parameter<double>("camera.cy", cam_cfg_.cy);
    cam_cfg_.stabilization_enabled = declare_parameter<bool>("camera.stabilization_enabled", true);
    cam_cfg_.dot_intensity = declare_parameter<double>("camera.ir_dot_intensity", 0.5);
    camera_x_ = declare_parameter<double>("camera.x_m", -0.09);
    camera_y_ = declare_parameter<double>("camera.y_m", 0.0);
    camera_yaw_deg_ = declare_parameter<double>("camera.yaw_deg", 0.0);
    measure_cfg_.manual_camera_height_enabled = declare_parameter<bool>("camera.manual_height_enabled", false);
    measure_cfg_.manual_camera_height_m = declare_parameter<double>("camera.manual_height_m", 0.20);
    measure_cfg_.timeout_sec = declare_parameter<double>("camera.measurement_timeout_sec", 45.0);
    rc_.camera.camera_x = camera_x_;
    rc_.camera.roi_height_ratio = registry_.get("yolo.roi_height_percent") / 100.0;

    // 차선 모델: 기존 line_detactor 기본값과 같이 INT8 Q/DQ 모델 + int8 엔진. fp16/fp32는 일반 FP32 ONNX
    models_.lane_precision = declare_parameter<std::string>("model.lane_precision", "int8");
    models_.lane_model = declare_parameter<std::string>("model.lane_path", share + (models_.lane_precision == "int8"
      ? "/models/fast_scnn_stop_line_120x300_batch_1.int8.qdq.onnx" : "/models/fast_scnn_stop_line_120x300_batch_1.onnx"));
    models_.yolo_model = declare_parameter<std::string>("model.yolo_path", share + "/models/multiclass_yolox_tiny_512x320_holdout10.onnx");
    const auto cache = declare_parameter<std::string>("model.engine_cache_dir", std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") + "/.cache/cone_drive_test");
    std::filesystem::create_directories(cache);
    models_.yolo_precision = declare_parameter<std::string>("model.yolo_precision", "fp16");
    models_.lane_engine_cache = cache + "/lane_" + models_.lane_precision + ".engine";
    models_.yolo_engine_cache = cache + "/yolo_" + models_.yolo_precision + ".engine";
    params_save_path_ = declare_parameter<std::string>("params_save_path", cache + "/cone_drive_test_saved.yaml");

    loop_ = std::make_unique<cdt::ControlLoop>(mode == "drive" ? cdt::DriveMode::Drive : cdt::DriveMode::Detect, rc_);
    apply_settings();

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    duty_pub_ = create_publisher<std_msgs::msg::Float32>(declare_parameter<std::string>("topics.auto_duty", "cone_drive/auto/duty"), command_qos);
    brake_pub_ = create_publisher<std_msgs::msg::Float32>(declare_parameter<std::string>("topics.auto_brake_current", "cone_drive/auto/brake_current"), command_qos);
    servo_pub_ = create_publisher<std_msgs::msg::Float32>(declare_parameter<std::string>("topics.auto_servo_position", "cone_drive/auto/servo_position"), command_qos);
    request_pub_ = create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("topics.handover_request", "cone_drive/handover/request"),
                                                           rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    erpm_sub_ = create_subscription<std_msgs::msg::Int32>(declare_parameter<std::string>("topics.measured_erpm", "vesc/measured_erpm"), command_qos,
      [this](std_msgs::msg::Int32::ConstSharedPtr m) { loop_->on_erpm(m->data, cdt::now_sec()); });
    connected_sub_ = create_subscription<std_msgs::msg::Bool>(declare_parameter<std::string>("topics.vesc_connected", "vesc/connected"), state_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { loop_->on_vesc_connected(m->data, cdt::now_sec()); });
    mode_sub_ = create_subscription<std_msgs::msg::String>(declare_parameter<std::string>("topics.handover_mode", "cone_drive/handover/mode"), state_qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) { loop_->on_handover_mode(m->data, cdt::now_sec()); });

    // 웹
    cdt::WebCallbacks cb;
    cb.start = [this] { return loop_->request_start(cdt::now_sec()); };
    cb.stop = [this] { loop_->request_stop("stop button", cdt::now_sec()); };
    cb.refit = [this] { return loop_->request_refit(cdt::now_sec()); };
    cb.mode = [this](const std::string & m) {
      if (m != "drive" && m != "detect") return cdt::Reply{false, "mode must be drive or detect"};
      return loop_->request_mode(m == "drive" ? cdt::DriveMode::Drive : cdt::DriveMode::Detect, cdt::now_sec());
    };
    cb.params = [this](const std::map<std::string, double> & c) {
      auto r = registry_.apply(c);
      if (r.first) apply_settings();
      return r;
    };
    cb.params_save = [this] {
      std::ofstream out(params_save_path_);
      out << registry_.to_yaml("cone_drive_test");
      return out ? cdt::Reply{true, params_save_path_} : cdt::Reply{false, "cannot write " + params_save_path_};
    };
    cb.heartbeat = [this](const std::string & id) { loop_->on_heartbeat(id, cdt::now_sec()); };
    web_ = std::make_unique<cdt::WebServer>(declare_parameter<std::string>("web.root", share + "/web"),
                                            declare_parameter<std::string>("web.bind", "0.0.0.0"),
                                            int(declare_parameter<int>("web.port", 8765)), registry_, cb);
    web_->start();
    RCLCPP_INFO(get_logger(), "Preview: http://<jetson-ip>:%d  mode=%s", web_->port(), mode.c_str());

    cdt::ImuImageStabilizerConfig stab;   // camera_driver camera_config.yaml 값
    stab.maximum_correction_deg = declare_parameter<double>("stabilization.maximum_correction_deg", 3.0);
    stab.maximum_frame_imu_wait_sec = declare_parameter<double>("stabilization.maximum_frame_imu_wait_sec", 0.005);
    stab.maximum_frame_imu_age_sec = declare_parameter<double>("stabilization.maximum_frame_imu_age_sec", 0.012);
    stab.maximum_frame_imu_prediction_sec = declare_parameter<double>("stabilization.maximum_prediction_sec", 0.015);
    camera_ = std::make_unique<cdt::CameraPipeline>(cam_cfg_, stab);
    camera_->on_yaw_rate = [this](double w, double t) { loop_->on_yaw_rate(w, t); };
    camera_->stationary_hint = [this]() -> std::optional<bool> {   // ERPM 속도로 정지 여부 (camera_driver는 ERPM 100 이하 1 s)
      const double v = loop_->speed();
      if (std::abs(v) < 0.03) return true;
      if (std::abs(v) > 0.10) return false;
      return std::nullopt;
    };
    lane_ = std::make_unique<cdt::LaneWorker>(*camera_, *loop_, [this](const cdt::StepOutput & o) { publish(o); });
    lane_->bev_wanted = [this] { return web_->stream_wanted("bev", cdt::now_sec()); };
    lane_->publish_bev_jpeg = [this](std::vector<std::uint8_t> j) { web_->publish_jpeg("bev", std::move(j)); };
    detect_ = std::make_unique<cdt::DetectWorker>(*camera_, *loop_);
    detect_->camera_wanted = [this] { return web_->stream_wanted("camera", cdt::now_sec()); };
    detect_->publish_camera_jpeg = [this](std::vector<std::uint8_t> j) { web_->publish_jpeg("camera", std::move(j)); };
    detect_->set_settings(loop_->settings());

    on_set_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {   // ros2 param set도 같은 경로
      std::map<std::string, double> c;
      for (const auto & p : ps) if (registry_.has(p.get_name())) c[p.get_name()] = p.as_double();
      rcl_interfaces::msg::SetParametersResult res;
      const auto r = registry_.apply(c);
      res.successful = r.first;
      res.reason = r.second;
      if (r.first && !c.empty()) apply_settings();
      return res;
    });

    watchdog_ = create_wall_timer(20ms, [this] {   // 차선 결과가 끊겨도 정지·제동이 이어지게
      if (cdt::now_sec() - lane_->last_frame_time() > 0.05) publish(loop_->tick(cdt::now_sec()));
      if (loop_->refit_pending() && !refitting_.exchange(true)) {
        if (refit_thread_.joinable()) refit_thread_.join();
        refit_thread_ = std::thread([this] { refit(); });
      }
    });
    telemetry_ = create_wall_timer(33ms, [this] { send_telemetry(); });
    inspect_thread_ = std::thread([this] { inspect(); });
  }

  ~DriveNode() override {
    stopping_ = true;
    if (inspect_thread_.joinable()) inspect_thread_.join();
    if (refit_thread_.joinable()) refit_thread_.join();
    lane_->stop();
    detect_->stop();
    camera_->close();
    web_->stop();
  }

private:
  void apply_settings() {
    const auto s = cdt::settings_from(registry_);
    loop_->apply_settings(s);
    if (detect_) detect_->set_settings(s);
    if (lane_) {
      cdt::LanePostConfig lc;
      lc.centerline.corner_outward_offset_m = s.corner_outward_offset_m;
      lc.centerline.corner_entry_distance_m = s.corner_entry_distance_m;
      lc.centerline.corner_entry_full_offset_distance_m = s.corner_entry_full_offset_m;
      lane_->set_post_config(lc);
    }
  }

  void publish(const cdt::StepOutput & o) {
    if (!o.handover_request.empty()) {
      std_msgs::msg::String m;
      m.data = o.handover_request;
      request_pub_->publish(m);
    }
    if (!o.publish) return;
    std_msgs::msg::Float32 v;
    if (o.brake_mode) { v.data = float(o.brake_current); brake_pub_->publish(v); }   // 제동 해제 주기는 0 A
    else { v.data = float(o.duty); duty_pub_->publish(v); }
    v.data = float(o.servo);
    servo_pub_->publish(v);
  }

  void apply_pose(const cdt::OakStartupMeasurement & m) {
    cdt::RectifiedCameraModel cam{cam_cfg_.fx, cam_cfg_.fy, cam_cfg_.cx, cam_cfg_.cy, cam_cfg_.width, cam_cfg_.height,
                                  cv::Vec3d(camera_x_, camera_y_, m.height_m),
                                  cdt::mountRotationVehicleFromCamera(cdt::degToRad(m.roll_deg), cdt::degToRad(m.pitch_down_deg), cdt::degToRad(camera_yaw_deg_))};
    lane_->set_geometry(cam, cdt::BevConfig{0.0, 3.0, -0.6, 0.6, 0.01, 120, 300});
    detect_->set_mount({camera_x_, camera_y_, m.height_m, cdt::degToRad(m.roll_deg), cdt::degToRad(m.pitch_down_deg), cdt::degToRad(camera_yaw_deg_)});
    auto geometry = rc_.camera;
    geometry.height = m.height_m;
    geometry.pitch_rad = cdt::degToRad(m.pitch_down_deg);
    geometry.roi_height_ratio = registry_.get("yolo.roi_height_percent") / 100.0;
    loop_->set_camera_geometry(geometry);
    loop_->set_attitude({true, m.height_m, m.roll_deg, m.pitch_down_deg, "PASS", ""});
  }

  bool measure_and_open() {
    loop_->set_attitude({false, 0, 0, 0, "MEASURING", "정지 상태 지면·IMU 측정 중"});
    try {
      const auto m = camera_->measure(measure_cfg_, [this] { return stopping_.load(); });
      apply_pose(m);
      camera_->open(m, camera_yaw_deg_);
      last_pose_ = m;
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "camera attitude measurement failed: %s", e.what());
      loop_->set_attitude({false, 0, 0, 0, "FAIL", e.what()});
      return false;
    }
  }

  void inspect() {   // 출발 전 점검: 카메라 자세 측정 → 모델 로드(엔진 생성·시험 추론)
    const bool cam_ok = measure_and_open();
    auto load = [this](int index, const std::string & name, const std::function<void(const std::function<void(const std::string &)> &)> & fn) {
      const double t0 = cdt::now_sec();
      try {
        fn([&](const std::string & stage) { loop_->set_model_status(index, {name, "LOADING", stage, "", cdt::now_sec() - t0}); });
        loop_->set_model_status(index, {name, "READY", "ready", "", cdt::now_sec() - t0});
        return true;
      } catch (const std::exception & e) {
        loop_->set_model_status(index, {name, "FAILED", "", e.what(), cdt::now_sec() - t0});
        RCLCPP_ERROR(get_logger(), "%s load failed: %s", name.c_str(), e.what());
        return false;
      }
    };
    const bool lane_ok = load(0, "차선 (Fast-SCNN)", [this](const auto & p) { lane_->load_model(models_, p); });
    const bool yolo_ok = load(1, "YOLOX-tiny 512x320", [this](const auto & p) { detect_->load_model(models_, p); });
    if (cam_ok) { lane_->start(); detect_->start(); }
    loop_->on_inspection_done(cam_ok && lane_ok && yolo_ok, cam_ok ? "model load failed" : "camera attitude failed", cdt::now_sec());
  }

  void refit() {   // 정차 확인 후 상태기계가 REFITTING일 때만 호출됨
    lane_->stop();
    detect_->stop();
    const bool ok = measure_and_open();
    if (!ok && last_pose_) {   // 실패 시 이전 보정으로 다시 연다
      try { apply_pose(*last_pose_); camera_->open(*last_pose_, camera_yaw_deg_); } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "camera reopen failed: %s", e.what());
      }
    }
    lane_->start();
    detect_->start();
    loop_->on_refit_done(ok, ok ? "" : "measurement failed", cdt::now_sec());
    refitting_ = false;
  }

  void send_telemetry() {
    const double now = cdt::now_sec();
    if (web_->client_count(now) == 0) return;   // 보는 사람이 없으면 아무것도 만들지 않음
    const bool full = web_->preview_wanted(now);
    if (!full && now - last_status_t_ < 0.5) return;   // 프리뷰 꺼짐: 상태만 초당 2회
    last_status_t_ = now;
    auto f = loop_->telemetry(full, now);
    if (now - last_stab_t_ >= 1.0) { camera_->stabilization_stats().summary(); last_stab_t_ = now; }
    f.stabilization = camera_->stabilization_stats().last();
    if (full) {
      if (now - last_usage_t_ >= 1.0) { usage_ = sysmon_.sample(now); last_usage_t_ = now; }
      f.usage = usage_;
      f.params_version = registry_.version();
    }
    const double t0 = cdt::now_sec();
    web_->publish_telemetry(cdt::telemetry_json(f));
    if (full) loop_->stats().record("web", t0, cdt::now_sec());
  }

  cdt::ParamRegistry registry_;
  cdt::RuntimeConfig rc_;
  cdt::CameraConfig cam_cfg_;
  cdt::OakStartupMeasurementConfig measure_cfg_;
  cdt::ModelConfig models_;
  double camera_x_{-0.09}, camera_y_{0}, camera_yaw_deg_{0};
  std::string params_save_path_;
  std::unique_ptr<cdt::ControlLoop> loop_;
  std::unique_ptr<cdt::WebServer> web_;
  std::unique_ptr<cdt::CameraPipeline> camera_;
  std::unique_ptr<cdt::LaneWorker> lane_;
  std::unique_ptr<cdt::DetectWorker> detect_;
  std::optional<cdt::OakStartupMeasurement> last_pose_;
  cdt::SystemMonitor sysmon_;
  cdt::SystemUsage usage_;
  double last_usage_t_{0}, last_status_t_{0}, last_stab_t_{0};
  std::atomic<bool> stopping_{false}, refitting_{false};
  std::thread inspect_thread_, refit_thread_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_pub_, brake_pub_, servo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr erpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_, telemetry_;
  OnSetParametersCallbackHandle::SharedPtr on_set_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
