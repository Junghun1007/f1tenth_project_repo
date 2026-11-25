// auto_control 주행 컴포넌트 (cone_drive_test/src/drive_node.cpp에서 시작).
// 카메라·BEV·차선 컨테이너(bev_processor_container)에 함께 로드해 영상·깊이·차선 결과를 복사 없이 받는다.
//   차선: line_detactor LaneResult (intra-process) → ControlLoop (콘 기억·회피·신호등·추종·duty_only·상태기계)
//   검출: camera_driver 직접 전달 NV12 → YOLOX → 신호등 색, CAM_A 정렬 깊이 → 콘·차량 위치
//   출력: <ns>/auto_drive/auto/{duty,brake_current,servo_position} → safety_node (vesc/*의 유일한 발행자)
//   웹: http://<jetson-ip>:8765 (SSE 텔레메트리, BEV·카메라 MJPEG, 명령·파라미터)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <line_detactor_msgs/msg/lane_result.hpp>
#include <bev_handoff/direct_bev_handoff.hpp>
#include <bev_handoff/direct_camera_handoff.hpp>
#include <bev_handoff/device_frame_handoff.hpp>
#include <bev_handoff/direct_obstacle_handoff.hpp>
#include <bev_handoff/ir_camera_settings.hpp>
#include <bev_handoff/lane_control_hints.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <optional>
#include <regex>
#include <sstream>
#include "auto_control/drive/core/control_loop.hpp"
#include "auto_control/drive/core/json_writer.hpp"
#include "auto_control/drive/core/lane_input.hpp"
#include "auto_control/drive/core/system_monitor.hpp"
#include "auto_control/drive/core/telemetry.hpp"
#include "auto_control/drive/runtime/detector.hpp"
#include "auto_control/drive/runtime/web_server.hpp"

using namespace std::chrono_literals;
namespace acd = auto_control::drive;

namespace auto_control {

namespace {
double steady_sec(std::chrono::steady_clock::time_point t) { return std::chrono::duration<double>(t.time_since_epoch()).count(); }
std::string home() { return std::getenv("HOME") ? std::getenv("HOME") : "/tmp"; }
std::string json_escape(const std::string & s) {
  std::string o;
  for (char c : s) { if (c == '"' || c == '\\') o += '\\'; if (static_cast<unsigned char>(c) >= 0x20) o += c; }
  return o;
}
bool is_tcn_param(const std::string & name) { return name.rfind("tcn.", 0) == 0; }

std::string camera_exposure_json() {
  acd::JsonWriter w;
  const auto now = std::chrono::steady_clock::now();
  auto exposure = [&](const std::shared_ptr<const bev_handoff::CameraExposureObservation> & obs) {
    w.key("ms");
    if (obs && obs->exposure_us > 0) w.value(obs->exposure_us / 1000.0, 3); else w.null();
    w.key("stale").value(obs && (now < obs->captured_at || now - obs->captured_at > 750ms));
  };
  auto stereo = [&](const char * key, const auto & obs) {
    w.key(key).begin_object();
    exposure(obs);
    w.key("iso");
    if (obs && obs->iso > 0) w.value(static_cast<long long>(obs->iso)); else w.null();
    w.end_object();
  };
  const auto rgb_mode = bev_handoff::latestRgbExposureControl();
  const char * mode = rgb_mode == bev_handoff::RgbExposureControl::restoring ? "manual" :
    rgb_mode == bev_handoff::RgbExposureControl::auto_pending ? "auto_pending" : "auto";
  w.begin_object().key("rgb").begin_object().key("mode").value(mode);
  exposure(bev_handoff::latestCameraExposure(false));
  w.end_object();
  stereo("ir_left", bev_handoff::latestCameraExposure(true));
  stereo("ir_right", bev_handoff::latestRightIrExposure());
  w.key("ir_flood");
  if (const auto intensity = bev_handoff::latestIrFloodIntensity()) w.value(*intensity, 2); else w.null();
  w.end_object();
  return w.str();
}
}  // namespace

class AutoDriveNode : public rclcpp::Node {
  using ForceIr = std_srvs::srv::SetBool;

public:
  static double as_number(const rclcpp::Parameter & p) {   // 실수·정수·bool 모두 숫자로 (YAML에 1, 1.0, true 어느 쪽으로 써도 됨)
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE: return p.as_double();
      case rclcpp::ParameterType::PARAMETER_INTEGER: return double(p.as_int());
      case rclcpp::ParameterType::PARAMETER_BOOL: return p.as_bool() ? 1.0 : 0.0;
      default: throw std::invalid_argument("parameter '" + p.get_name() + "' must be a number");
    }
  }

  explicit AutoDriveNode(const rclcpp::NodeOptions & options) : Node("auto_drive", options) {
    acd::register_params(registry_);
    rcl_interfaces::msg::ParameterDescriptor any_number;
    any_number.dynamic_typing = true;
    for (const auto & p : registry_.list()) declare_parameter(p.name, rclcpp::ParameterValue(p.value), any_number);
    std::map<std::string, double> start;
    for (const auto & p : registry_.list()) start[p.name] = as_number(get_parameter(p.name));
    const auto applied = registry_.apply(start);
    if (!applied.first) throw std::invalid_argument("parameter file: " + applied.second);

    const auto share = ament_index_cpp::get_package_share_directory("auto_control");
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
    camera_yaw_deg_ = declare_parameter<double>("camera.yaw_deg", 0.0);
    // camera_driver의 1.25배 확대는 BEV에서만 쓰는 가상 확대다. bev_processor가 공유하는 fx·fy는 확대 영상 기준이고
    // YOLO가 받는 원본 NV12는 확대 전이므로 fx·fy를 이 값으로 나눠 쓴다 (cx·cy는 확대 중심이라 그대로, cone_drive_test와 같음)
    fixed_view_zoom_ = declare_parameter<double>("camera.fixed_view_zoom", 1.25);
    if (!(fixed_view_zoom_ >= 1.0 && fixed_view_zoom_ <= 3.0)) throw std::invalid_argument("camera.fixed_view_zoom must be 1..3");
    rc_.camera.roi_height_ratio = registry_.get("yolo.roi_height_percent") / 100.0;

    models_.yolo_model = declare_parameter<std::string>("model.yolo_path", share + "/models/multiclass_yolox_tiny_512x320_holdout10.onnx");
    const auto cache = declare_parameter<std::string>("model.engine_cache_dir", home() + "/.cache/auto_control");
    std::filesystem::create_directories(cache);
    models_.yolo_precision = declare_parameter<std::string>("model.yolo_precision", "fp16");
    models_.yolo_backend = declare_parameter<std::string>("model.yolo_backend", "TENSORRT");   // CPU: 개발 PC 확인용
    models_.yolo_engine_cache = cache + "/yolo_" + models_.yolo_precision + ".engine";
    params_save_path_ = declare_parameter<std::string>("params_save_path", cache + "/auto_drive_saved.yaml");
    lane_model_node_ = declare_parameter<std::string>("inspection.lane_model_node", "line_detactor");
    tunnel_enabled_ = declare_parameter<bool>("tunnel.enabled", false);
    publish_tunnel_runtime_settings();
    ir_model_nodes_ = declare_parameter<std::vector<std::string>>("inspection.ir_model_nodes",
                                                                   std::vector<std::string>{"ir_line_detactor", "tunnel_tcn_test"});

    loop_ = std::make_unique<acd::ControlLoop>(mode == "drive" ? acd::DriveMode::Drive : acd::DriveMode::Detect, rc_);
    loop_->set_model_status(0, {"차선 (line_detactor)", "WAITING", "", "", 0});
    if (tunnel_enabled_) loop_->set_model_status(2, {"터널 IR 차선", "WAITING", "", "", 0});
    // 콘 관측 기록: <log.dir>/cone_obs_<시각>.csv, 주행 시작마다 새 파일 (분석: tools/analyze_cone_obs.py)
    cone_log_enabled_ = declare_parameter<bool>("log.cone_observations", true);
    log_dir_ = declare_parameter<std::string>("log.dir", "");
    if (log_dir_.empty()) log_dir_ = home() + "/auto_control_data";
    if (cone_log_enabled_)
      loop_->on_cone_log = [this](const std::vector<acd::ConeLogRow> & rows) {
        std::lock_guard<std::mutex> l(cone_log_m_);
        if (!cone_log_) return;
        auto & o = *cone_log_;
        for (const auto & r : rows)
          o << r.t_rgb << ',' << r.t_depth << ',' << r.t_pose << ',' << r.id << ',' << r.class_id << ',' << r.local_x << ',' << r.local_y << ','
            << r.front_x << ',' << r.front_y << ',' << r.range << ',' << r.bearing << ',' << r.pose_x << ',' << r.pose_y << ',' << r.pose_yaw << ','
            << r.speed << ',' << r.yaw_rate << ',' << r.gyro_bias << ',' << r.world_x << ',' << r.world_y << ',' << r.mem_x << ',' << r.mem_y << ','
            << r.hits << '\n';
        o.flush();
      };
    detect_ = std::make_unique<acd::DetectWorker>(*loop_);
    apply_settings();

    // ---- 출력 (safety_node로) ----
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    // 콜백 그룹: 멀티스레드 컨테이너에서 서로 기다리지 않게 나눈다
    //   imu      400 Hz 요레이트 (오도메트리 락만 씀 → 회피 계획 중에도 바로 처리)
    //   vehicle  ERPM·VESC·안전 전환·조이스틱 (제어 락을 잠깐 잡음)
    //   control  차선 결과 → 계획·제어, 감시 타이머 (순서 보장)
    //   status   시작 측정·모델 상태·터널 상태, 카메라 장착값 대기
    //   web      텔레메트리 JSON
    auto group = [this] { return create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); };
    imu_group_ = group(); vehicle_group_ = group(); control_group_ = group(); status_group_ = group(); web_group_ = group();
    auto in = [](const rclcpp::CallbackGroup::SharedPtr & g) { rclcpp::SubscriptionOptions o; o.callback_group = g; return o; };
    // 컨테이너가 intra-process를 켜도 래치(transient_local) 구독은 intra-process로 받을 수 없다 → 이 구독만 끈다
    auto latched_in = [&in](const rclcpp::CallbackGroup::SharedPtr & g) {
      auto o = in(g);
      o.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
      return o;
    };
    const auto vehicle = in(vehicle_group_), vehicle_latched = latched_in(vehicle_group_);
    const auto control = in(control_group_), status_latched = latched_in(status_group_);
    refit_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("topics.refit_request", "/bev_processor/refit_request"), rclcpp::QoS(1).reliable());
    duty_pub_ = create_publisher<std_msgs::msg::Float32>(declare_parameter<std::string>("topics.auto_duty", "auto_drive/auto/duty"), command_qos);
    brake_pub_ = create_publisher<std_msgs::msg::Float32>(declare_parameter<std::string>("topics.auto_brake_current", "auto_drive/auto/brake_current"), command_qos);
    servo_pub_ = create_publisher<std_msgs::msg::Float32>(declare_parameter<std::string>("topics.auto_servo_position", "auto_drive/auto/servo_position"), command_qos);
    request_pub_ = create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("topics.handover_request", "auto_drive/handover/request"),
                                                           rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    // ---- 차량 입력 ----
    erpm_sub_ = create_subscription<std_msgs::msg::Int32>(declare_parameter<std::string>("topics.measured_erpm", "vesc/measured_erpm"), command_qos,
      [this](std_msgs::msg::Int32::ConstSharedPtr m) { loop_->on_erpm(m->data, acd::now_sec()); }, vehicle);
    connected_sub_ = create_subscription<std_msgs::msg::Bool>(declare_parameter<std::string>("topics.vesc_connected", "vesc/connected"), state_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { loop_->on_vesc_connected(m->data, acd::now_sec()); }, vehicle_latched);
    mode_sub_ = create_subscription<std_msgs::msg::String>(declare_parameter<std::string>("topics.handover_mode", "auto_drive/handover/mode"), state_qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        loop_->on_handover_mode(m->data, acd::now_sec());
        std::lock_guard<std::mutex> l(status_m_);
        handover_mode_ = m->data;
      }, vehicle_latched);
    // 조이스틱 버튼 (8BitDo SDL 배치, vehicle_bringup/config/controller_keymap.yaml): A=주행 시작, B=Refit, X=정지.
    // X는 safety_node에서도 제동 → MANUAL 전환을 한다 (여기서는 주행 상태기계도 정지로)
    joy_start_button_ = int(declare_parameter<int>("joy.start_button", 0));
    joy_refit_button_ = int(declare_parameter<int>("joy.refit_button", 1));
    joy_stop_button_ = int(declare_parameter<int>("joy.stop_button", 2));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(declare_parameter<std::string>("topics.joy", "joy"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr m) { on_joy(*m); }, vehicle);
    // IMU (camera_driver IMU 브리지, 카메라 좌표 자이로) → 차량 요레이트 (오도메트리)
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(declare_parameter<std::string>("topics.imu", "/camera/imu"),
      rclcpp::SensorDataQoS().keep_last(20), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { on_imu(*m); }, in(imu_group_));

    // ---- 차선 ----
    const auto lane_topic = declare_parameter<std::string>("topics.lane_result", "/line_detactor/result");
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    if (tunnel_enabled_) {   // 터널 선택기가 카메라별 결과를 따로 낸다 (/rgb, /ir)
      lane_sub_ = create_subscription<line_detactor_msgs::msg::LaneResult>(lane_topic + "/rgb", sensor_qos,
        [this](line_detactor_msgs::msg::LaneResult::ConstSharedPtr m) { on_lane(*m, acd::LaneSource::RGB); }, control);
      ir_lane_sub_ = create_subscription<line_detactor_msgs::msg::LaneResult>(lane_topic + "/ir", sensor_qos,
        [this](line_detactor_msgs::msg::LaneResult::ConstSharedPtr m) { on_lane(*m, acd::LaneSource::IR); }, control);
      tunnel_sub_ = create_subscription<std_msgs::msg::String>(declare_parameter<std::string>("topics.tunnel_state", "/tunnel/state"), state_qos,
        [this](std_msgs::msg::String::ConstSharedPtr m) { on_tunnel_state(m->data); }, status_latched);
      auto tcn_options = in(web_group_);
      tcn_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
      tcn_sub_ = create_subscription<std_msgs::msg::String>("/tunnel_tcn/result", rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::String::ConstSharedPtr m) { tcn_telemetry_.update(m->data, acd::now_sec() - t_start_); }, tcn_options);
      rclcpp::PublisherOptions tcn_publisher_options;
      tcn_publisher_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
      tcn_tuning_pub_ = create_publisher<std_msgs::msg::String>("/tunnel/tcn_tuning", state_qos, tcn_publisher_options);
      publish_tcn_tuning();
      force_ir_sub_ = create_subscription<std_msgs::msg::Bool>("/tunnel/force_ir_enabled", state_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) { force_ir_enabled_.store(m->data); }, status_latched);
      force_ir_client_ = create_client<ForceIr>("/tunnel/set_force_ir");
      // 터널 안 웹 "BEV 원본": RGB BEV 대신 IR BEV (shared_ir_bev, mono8 120×300). 켰을 때만 1/15 s 복사
      ir_bev_sub_ = create_subscription<sensor_msgs::msg::Image>(declare_parameter<std::string>("topics.ir_bev_image", "/camera/image_bev_ir"),
        sensor_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_ir_bev(*m); }, in(status_group_));
    } else {
      lane_sub_ = create_subscription<line_detactor_msgs::msg::LaneResult>(lane_topic, sensor_qos,
        [this](line_detactor_msgs::msg::LaneResult::ConstSharedPtr m) { on_lane(*m, acd::LaneSource::RGB); }, control);
    }

    // ---- 출발 전 점검: 카메라 자세(bev_processor 시작 측정), 모델 상태 ----
    startup_sub_ = create_subscription<std_msgs::msg::String>(declare_parameter<std::string>("topics.startup_status", "/bev_processor/startup_status"),
      state_qos, [this](std_msgs::msg::String::ConstSharedPtr m) { on_startup_status(m->data); }, status_latched);
    lane_model_sub_ = create_subscription<std_msgs::msg::String>("/" + lane_model_node_ + "/model_status", state_qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) { on_model_status(0, "차선 (line_detactor)", m->data); }, status_latched);
    if (tunnel_enabled_)
      for (std::size_t i = 0; i < ir_model_nodes_.size(); ++i) {
        ir_model_status_.push_back("WAITING");
        ir_model_subs_.push_back(create_subscription<std_msgs::msg::String>("/" + ir_model_nodes_[i] + "/model_status", state_qos,
          [this, i](std_msgs::msg::String::ConstSharedPtr m) { on_ir_model_status(i, m->data); }, status_latched));
      }

    // ---- 웹 ----
    acd::WebCallbacks cb;
    cb.start = [this] { return start_drive(acd::now_sec()); };
    cb.force_ir = [this](bool enabled) { return request_force_ir(enabled); };
    cb.stop = [this] { loop_->request_stop("stop button", acd::now_sec()); };
    // Refit: 정지(주행 중이면 먼저 세움) → 실시간 깊이로 카메라 높이·기울기 다시 측정(bev_processor) → BEV·영상 안정화 기준 갱신,
    // 끝나면 콘·차량 기억을 모두 비운다 (실패해도 이전 보정 유지 + 기억 비움)
    cb.refit = [this] { return loop_->request_refit(acd::now_sec()); };
    cb.mode = [this](const std::string & m) {
      if (m != "drive" && m != "detect") return acd::Reply{false, "mode must be drive or detect"};
      return loop_->request_mode(m == "drive" ? acd::DriveMode::Drive : acd::DriveMode::Detect, acd::now_sec());
    };
    cb.params = [this](const std::map<std::string, double> & c) { return apply_tuning(c); };
    cb.params_save = [this] {
      std::ofstream out(params_save_path_);
      out << registry_.to_yaml("auto_drive");
      return out ? acd::Reply{true, params_save_path_} : acd::Reply{false, "cannot write " + params_save_path_};
    };
    cb.heartbeat = [this](const std::string & id) { loop_->on_heartbeat(id, acd::now_sec()); };
    cb.record_start = [this](const std::string & label) { return record_start(label); };
    cb.record_stop = [this] { return record_stop(); };
    loop_->on_record = [this](const acd::RecordFrame & f) { record_frame(f); };
    web_ = std::make_unique<acd::WebServer>(declare_parameter<std::string>("web.root", share + "/web"),
                                            declare_parameter<std::string>("web.bind", "0.0.0.0"),
                                            int(declare_parameter<int>("web.port", 8765)), registry_, cb);
    web_->start();
    RCLCPP_INFO(get_logger(), "auto_control web: http://<jetson-ip>:%d  mode=%s  tunnel=%s", web_->port(), mode.c_str(),
                tunnel_enabled_ ? "on" : "off");

    // ---- 같은 프로세스 영상·깊이 ----
    detect_->camera_wanted = [this] { return web_->stream_wanted("camera", acd::now_sec()); };
    detect_->publish_camera_jpeg = [this](std::vector<std::uint8_t> j) { web_->publish_jpeg("camera", std::move(j)); };
    camera_consumer_ = bev_handoff::registerDirectCameraConsumer([this](std::shared_ptr<const bev_handoff::DirectCameraFrame> f) {
      auto frame = std::make_shared<acd::Nv12Frame>();
      frame->nv12 = f->nv12; frame->size = f->size; frame->stride = f->stride;
      frame->width = f->width; frame->height = f->height; frame->t = steady_sec(f->captured_at);
      frame->owner = f;   // 카메라 버퍼를 붙잡는다 (최신 1장만)
      detect_->push_frame(std::move(frame));
    });
    // camera_driver 공유 업로드(gpu_shared_upload_enabled)가 켜져 있으면 GPU 원본으로 추론 (업로드 1회, BEV와 공유)
    device_consumer_ = bev_handoff::registerDeviceNv12Consumer([this](std::shared_ptr<const bev_handoff::DeviceNv12Frame> f) {
      auto frame = std::make_shared<acd::Nv12Frame>();
      frame->device = f->device; frame->device_pitch = f->pitch;
      frame->width = f->width; frame->height = f->height; frame->t = steady_sec(f->captured_at);
      frame->device_owner = f;
      detect_->push_device_frame(std::move(frame));
    });
    depth_thread_ = std::thread([this] { depth_loop(); });
    bev_handoff::setDirectBevPreviewTap([this](std::shared_ptr<const bev_handoff::DirectBevFrame> f) {   // 웹 "BEV 원본" (켰을 때만 1/15 s)
      const double t = acd::now_sec();
      if (!bev_wanted_.load() || t - last_bev_tap_t_ < 1.0 / 15.0) return;
      if (tunnel_view_.load() && t - last_ir_bev_t_.load() < 0.5) return;   // 터널 안: IR BEV를 보여 주는 중
      last_bev_tap_t_ = t;
      std::lock_guard<std::mutex> l(bev_m_);
      bev_pending_ = std::move(f);
      bev_cv_.notify_one();
    });
    bev_thread_ = std::thread([this] { bev_jpeg_loop(); });

    on_set_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {   // ros2 param set도 같은 경로
      std::map<std::string, double> c;
      for (const auto & p : ps) if (registry_.has(p.get_name())) c[p.get_name()] = as_number(p);
      rcl_interfaces::msg::SetParametersResult res;
      res.successful = true;
      if (c.empty()) return res;
      const auto r = apply_tuning(c);
      res.successful = r.first;
      res.reason = r.second;
      return res;
    });

    watchdog_ = create_wall_timer(20ms, [this] {   // 차선 결과가 끊겨도 정지·제동이 이어지게
      if (acd::now_sec() - last_lane_t_.load() > 0.05) publish(loop_->tick(acd::now_sec()));
      drive_refit(acd::now_sec());
      bev_wanted_ = web_->stream_wanted("bev", acd::now_sec());
    }, control_group_);
    telemetry_ = create_wall_timer(33ms, [this] { send_telemetry(); }, web_group_);
    setup_timer_ = create_wall_timer(100ms, [this] { connect_reference(); }, status_group_);
    // 터널 안 웹 "카메라": YOLO가 쉬는 동안 IR 카메라(CAM_B 왼쪽 흑백, 차선 입력 원본)를 보여 준다
    if (tunnel_enabled_) ir_camera_timer_ = create_wall_timer(66ms, [this] { publish_ir_camera(); }, web_group_);   // bev_processor 시작 측정이 끝나야 카메라 장착값이 생김
    inspect_thread_ = std::thread([this] { load_yolo(); });
  }

  ~AutoDriveNode() override {
    stopping_ = true;
    bev_handoff::setDirectBevPreviewTap({});
    if (camera_consumer_) bev_handoff::unregisterDirectCameraConsumer(camera_consumer_);
    if (device_consumer_) bev_handoff::unregisterDeviceNv12Consumer(device_consumer_);
    bev_handoff::publishLaneControlHints(nullptr);
    bev_cv_.notify_all();
    if (inspect_thread_.joinable()) inspect_thread_.join();
    if (depth_thread_.joinable()) depth_thread_.join();
    if (bev_thread_.joinable()) bev_thread_.join();
    detect_->stop();
    web_->stop();
  }

private:
  // ------------------------------------------------------------------ 입력
  double steady_from_stamp(const builtin_interfaces::msg::Time & stamp) {
    // camera_driver는 촬영 시각(steady)을 ROS 시각으로 바꿔 헤더에 넣는다 → 지금 두 시계 차이로 되돌린다
    const double ros_now = now().seconds();
    const double ros_t = stamp.sec + stamp.nanosec * 1e-9;
    if (ros_t <= 0) return acd::now_sec();
    return acd::now_sec() - std::max(0.0, ros_now - ros_t);
  }

  void on_lane(const line_detactor_msgs::msg::LaneResult & m, acd::LaneSource source) {
    const double t_in = acd::now_sec();
    const double capture = steady_from_stamp(m.header.stamp);
    acd::LanePixels px;
    px.source_width = int(m.source_width);
    px.padding_left = int(m.padding_left);
    px.bev_width_m = m.centerline_bev_width_m > 0 ? m.centerline_bev_width_m : 1.2;
    px.bev_height_m = m.centerline_bev_height_m > 0 ? m.centerline_bev_height_m : 3.0;
    px.centerline_valid = m.centerline_valid;
    px.stop_line_m = m.stop_line_distance_m;
    auto copy = [](const std::vector<geometry_msgs::msg::Point32> & in, std::vector<acd::LanePixels::P> & out) {
      out.reserve(in.size());
      for (const auto & p : in) out.push_back({p.x, p.y});
    };
    copy(m.centerline_points, px.centerline);
    copy(m.reference_centerline_points, px.reference);
    copy(m.left_points, px.left);
    copy(m.right_points, px.right);
    const auto lane = acd::lane_from_pixels(px, capture);
    const bool is_ir = source == acd::LaneSource::IR;
    const char * source_name = is_ir ? "IR" : "RGB";
    {
      std::lock_guard<std::mutex> l(status_m_);
      if (lane_source_ != source_name) {
        lane_source_ = source_name;
        lane_source_changed_s_ = t_in - t_start_;
        RCLCPP_INFO(get_logger(), "lane source: %s", source_name);
      }
      if (is_ir) {
        last_ir_lane_t_ = t_in;
        last_ir_lane_valid_ = lane.valid;
      }
    }
    auto & stats = loop_->stats();
    stats.record("lane_latency", capture, t_in);
    stats.record("lane_pre", t_in, t_in + m.h2d_preprocess_nanoseconds * 1e-9);
    stats.record("lane_infer", t_in, t_in + m.pure_inference_nanoseconds * 1e-9);
    stats.record("lane_geom", t_in, t_in + m.lane_geometry_nanoseconds * 1e-9);
    last_lane_t_ = acd::now_sec();
    publish(loop_->on_lane(lane, acd::now_sec(), source));
    publish_hints();   // 다음 프레임의 기준 중앙선 요청 (콘이 근처일 때만)
  }

  // Web and joystick cannot arm the car while a tunnel override request is in flight.
  bool force_ir_request_pending() {
    if (!force_ir_request_) return false;
    if (force_ir_request_->wait_for(0s) != std::future_status::ready) return true;
    const auto response = force_ir_request_->get();
    force_ir_request_.reset();
    if (response->success) force_ir_enabled_.store(force_ir_requested_value_);
    return false;
  }

  acd::Reply request_force_ir(bool enabled) {
    if (!force_ir_client_) return {false, "터널 기능이 꺼져 있습니다"};
    std::unique_lock<std::mutex> command(drive_command_m_, std::try_to_lock);
    if (!command || force_ir_request_pending()) return {false, "터널 전환 요청 처리 중"};
    if (!loop_->tunnel_override_allowed(acd::now_sec())) return {false, "정지 상태에서만 변경할 수 있습니다"};
    if (!force_ir_client_->service_is_ready()) return {false, "터널 차선 선택기가 준비 중입니다"};
    auto request = std::make_shared<ForceIr::Request>();
    request->data = enabled;
    force_ir_requested_value_ = enabled;
    force_ir_request_.emplace(force_ir_client_->async_send_request(request));
    if (force_ir_request_->wait_for(2s) != std::future_status::ready) {
      return {false, "터널 차선 선택기의 응답을 기다리는 중입니다"};
    }
    const auto response = force_ir_request_->get();
    force_ir_request_.reset();
    if (response->success) force_ir_enabled_.store(enabled);
    return {response->success, response->message};
  }

  acd::Reply start_drive(double now) {
    std::unique_lock<std::mutex> command(drive_command_m_, std::try_to_lock);
    if (!command || force_ir_request_pending()) return {false, "터널 전환 요청 처리 중"};
    if (force_ir_enabled_.load()) {
      std::lock_guard<std::mutex> l(status_m_);
      if (tunnel_state_ != "INSIDE" || lane_source_ != "IR") return {false, "터널 IR 차선 준비 중"};
    }
    auto result = loop_->request_start(now);
    if (result.first && !recording()) open_cone_log(log_dir_, "");
    return result;
  }

  void on_joy(const sensor_msgs::msg::Joy & m) {
    const double now = acd::now_sec();
    last_joy_t_ = now;
    auto rising = [&](int button, bool & was) {
      const bool pressed = button >= 0 && std::size_t(button) < m.buttons.size() && m.buttons[std::size_t(button)] != 0;
      const bool edge = pressed && !was;
      was = pressed;
      return edge;
    };
    if (rising(joy_stop_button_, joy_stop_was_)) {
      loop_->request_stop("joystick X", now);
      RCLCPP_INFO(get_logger(), "joystick X: stop");
    }
    if (rising(joy_start_button_, joy_start_was_)) {
      const auto r = start_drive(now);
      RCLCPP_INFO(get_logger(), "joystick A: start -> %s %s", r.first ? "ok" : "refused", r.second.c_str());
    }
    if (rising(joy_refit_button_, joy_refit_was_)) {
      const auto r = loop_->request_refit(now);
      RCLCPP_INFO(get_logger(), "joystick B: refit -> %s %s", r.first ? "ok" : "refused", r.second.c_str());
    }
  }

  void on_imu(const sensor_msgs::msg::Imu & m) {
    std::shared_ptr<const bev_handoff::ObstacleReference> ref;
    { std::lock_guard<std::mutex> l(status_m_); ref = reference_; }
    if (!ref) return;
    const cv::Vec3d gyro(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
    loop_->on_yaw_rate((ref->rotation_vehicle_from_camera * gyro)[2], steady_from_stamp(m.header.stamp));   // 차량 z축(위) 요레이트
  }

  void depth_loop() {   // CAM_A 정렬 깊이 → 검출 워커 (최근 8장)
    std::uint64_t revision = 0;
    while (!stopping_) {
      const auto d = bev_handoff::waitDirectDepth(revision, 100ms);
      if (!d || d->captured_at <= last_depth_) continue;
      last_depth_ = d->captured_at;
      auto img = std::make_shared<acd::DepthImage>();
      img->owner = d; img->data = d->data; img->stride = d->stride; img->width = d->width; img->height = d->height;
      img->t = steady_sec(d->captured_at);
      img->fx = d->intrinsics[0]; img->fy = d->intrinsics[1]; img->cx = d->intrinsics[2]; img->cy = d->intrinsics[3];
      loop_->stats().record("cam_depth", img->t, acd::now_sec());
      detect_->push_depth(std::move(img));
    }
  }

  void on_ir_bev(const sensor_msgs::msg::Image & m) {
    const double t = acd::now_sec();
    if (!tunnel_view_.load() || !bev_wanted_.load() || t - last_ir_bev_t_.load() < 1.0 / 15.0) return;
    if (m.encoding != "mono8" || m.width == 0 || m.height == 0 || m.step < m.width || m.data.size() < std::size_t(m.step) * m.height) return;
    last_ir_bev_t_ = t;
    cv::Mat img = cv::Mat(int(m.height), int(m.width), CV_8UC1, const_cast<std::uint8_t *>(m.data.data()), m.step).clone();
    std::lock_guard<std::mutex> l(bev_m_);
    ir_bev_pending_ = std::move(img);
    bev_cv_.notify_one();
  }

  void publish_ir_camera() {   // 웹 스레드: 최신 IR 한 장 → 640 폭 JPEG (켰을 때·터널 안일 때만)
    if (!tunnel_view_.load() || !web_->stream_wanted("camera", acd::now_sec())) return;
    const auto f = bev_handoff::latestDirectIr();
    if (!f || f->mono.empty() || f->captured_at == last_ir_camera_at_) return;
    last_ir_camera_at_ = f->captured_at;
    cv::Mat view;
    const double scale = 640.0 / f->mono.cols;
    cv::resize(f->mono, view, {}, scale, scale, cv::INTER_AREA);
    cv::cvtColor(view, view, cv::COLOR_GRAY2BGR);
    std::ostringstream label;
    label << "IR (CAM_B) " << f->mono.cols << 'x' << f->mono.rows << " | "
          << std::fixed << std::setprecision(2) << f->exposure_us / 1000.0 << " ms | ISO " << f->sensitivity_iso;
    cv::putText(view, label.str(), {8, 22}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
    std::vector<std::uint8_t> jpeg;
    if (cv::imencode(".jpg", view, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70})) web_->publish_jpeg("camera", std::move(jpeg));
  }

  void bev_jpeg_loop() {   // BEV 원본 다운로드·인코딩 (BEV·차선 스레드를 막지 않게 따로)
    while (!stopping_) {
      std::shared_ptr<const bev_handoff::DirectBevFrame> f;
      cv::Mat ir;
      {
        std::unique_lock<std::mutex> l(bev_m_);
        bev_cv_.wait_for(l, 200ms, [this] { return bev_pending_ != nullptr || !ir_bev_pending_.empty() || stopping_; });
        f = std::move(bev_pending_);
        bev_pending_.reset();
        ir = std::move(ir_bev_pending_);
        ir_bev_pending_ = cv::Mat();
      }
      if (!ir.empty()) {   // IR BEV 120×300 → 2배 확대
        cv::Mat big;
        cv::resize(ir, big, {}, 2.0, 2.0, cv::INTER_NEAREST);
        std::vector<std::uint8_t> jpeg;
        if (cv::imencode(".jpg", big, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80})) web_->publish_jpeg("bev", std::move(jpeg));
      }
      if (!f) continue;
      const cv::Mat bgr = !f->bgr.empty() ? f->bgr : (f->download_bgr ? f->download_bgr() : cv::Mat());
      f.reset();
      std::vector<std::uint8_t> jpeg;
      if (!bgr.empty() && cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70})) web_->publish_jpeg("bev", std::move(jpeg));
    }
  }

  // ------------------------------------------------------------------ 점검
  // 상태기계가 REFITTING이 되면(정차 확인 뒤) bev_processor에 다시 측정을 요청하고, 결과(~/startup_status)를 기다린다
  void drive_refit(double now) {
    if (!loop_->refit_pending()) { refit_sent_ = false; return; }
    if (!refit_sent_) {
      refit_sent_ = true;
      refit_t0_ = now;
      { std::lock_guard<std::mutex> l(status_m_); refit_failure_.clear(); }
      std_msgs::msg::String m;
      m.data = "refit";
      refit_pub_->publish(m);
      RCLCPP_INFO(get_logger(), "Refit requested: re-measuring camera pose from live depth, obstacle memory will be cleared");
    } else if (now - refit_t0_ > 25.0) {   // bev_processor 응답 없음
      loop_->on_refit_done(false, "camera re-measurement did not answer (25 s)", now);
    }
  }

  void on_startup_status(const std::string & s) {
    // bev_processor: "READY | height=0.201m roll=-0.4deg pitch_down=13.2deg source=..." / "FAILED | ..." / 측정 중
    //               Refit: "MEASURING | refit ..." → "READY | ..." 또는 "REFIT_FAILED | 이유" 뒤 이전 값으로 "READY | ..."
    std::lock_guard<std::mutex> l(status_m_);
    std::smatch mt;
    if (s.rfind("REFIT_FAILED", 0) == 0) {
      const auto bar = s.find(" | ");
      refit_failure_ = bar == std::string::npos ? "refit failed" : s.substr(bar + 3);
      return;
    }
    if (s.rfind("READY", 0) == 0 &&
        std::regex_search(s, mt, std::regex(R"(height=([-0-9.eE]+)m roll=([-0-9.eE]+)deg pitch_down=([-0-9.eE]+)deg)"))) {
      pose_ = Pose{std::stod(mt[1]), std::stod(mt[2]), std::stod(mt[3])};
      loop_->set_attitude({true, pose_->height, pose_->roll, pose_->pitch, "PASS", ""});
      if (auto ref = bev_handoff::obstacleReference()) reference_ = ref;   // Refit이면 카메라 회전이 바뀜
      update_mount();
      if (refit_sent_ && loop_->refit_pending()) {   // Refit 끝 (실패면 이전 보정을 다시 알린 것): 콘 기억도 여기서 비워짐
        loop_->on_refit_done(refit_failure_.empty(), refit_failure_, acd::now_sec());
        RCLCPP_INFO(get_logger(), "Refit %s: height %.3f m, roll %.2f deg, pitch %.2f deg%s", refit_failure_.empty() ? "done" : "failed",
                    pose_->height, pose_->roll, pose_->pitch, refit_failure_.empty() ? "" : (" (" + refit_failure_ + ")").c_str());
      }
    } else if (refit_sent_ && s.rfind("MEASURING", 0) == 0) {
      loop_->set_attitude({false, 0, 0, 0, "MEASURING", "Refit: 카메라 높이·기울기 다시 측정 중 — 차를 움직이지 마세요"});
    } else if (s.rfind("FAIL", 0) == 0) {
      const auto bar = s.find(" | ");
      loop_->set_attitude({false, 0, 0, 0, "FAIL",
                           "측정 실패 (평평한 바닥·정지 상태 확인 후 재실행): " + (bar == std::string::npos ? s : s.substr(bar + 3))});
    } else {
      loop_->set_attitude({false, 0, 0, 0, "MEASURING", "카메라 높이·기울기 측정 중 — 차를 평평한 바닥에 세우고 움직이지 마세요"});
    }
    check_inspection();
  }

  void connect_reference() {   // bev_processor가 공유하는 카메라 모델 (fx·fy·cx·cy, 차량 기준 위치·회전)
    auto ref = bev_handoff::obstacleReference();
    if (!ref) return;
    setup_timer_->cancel();
    std::lock_guard<std::mutex> l(status_m_);
    reference_ = ref;
    update_mount();
    RCLCPP_INFO(get_logger(), "camera reference: fx=%.1f fy=%.1f %dx%d position=(%.3f, %.3f, %.3f)", ref->fx, ref->fy,
                ref->source_width, ref->source_height, ref->position_vehicle_m[0], ref->position_vehicle_m[1], ref->position_vehicle_m[2]);
  }

  void update_mount() {   // status_m_ 잡은 채로
    if (!reference_ || !pose_) return;
    const auto & r = *reference_;
    const double fx = r.fx / fixed_view_zoom_, fy = r.fy / fixed_view_zoom_;   // 원본(확대 전) 영상 기준
    detect_->set_mount({r.position_vehicle_m[0], r.position_vehicle_m[1], pose_->height, pose_->roll * M_PI / 180.0,
                        pose_->pitch * M_PI / 180.0, camera_yaw_deg_ * M_PI / 180.0},
                       {fx, fy, r.cx, r.cy});
    auto g = loop_->camera_geometry();
    g.camera_x = r.position_vehicle_m[0];
    g.height = pose_->height;
    g.pitch_rad = pose_->pitch * M_PI / 180.0;
    acd::set_field_of_view(g, fx, fy, r.source_width, r.source_height);   // 검출 영상(원본, 확대 전) 화각
    g.roi_height_ratio = registry_.get("yolo.roi_height_percent") / 100.0;
    loop_->set_camera_geometry(g);
  }

  static acd::InspectionModel parse_model(const std::string & name, const std::string & s, double elapsed) {
    // bev_handoff::ModelLoadStatus: "READY | ..." / "LOADING | <단계> | elapsed=..." / "FAILED | <이유>"
    const auto bar = s.find(" | ");
    const std::string head = s.substr(0, bar);
    std::string rest = bar == std::string::npos ? "" : s.substr(bar + 3);
    rest = rest.substr(0, rest.find(" | "));
    if (head == "READY") return {name, "READY", "ready", "", elapsed};
    if (head == "FAILED") return {name, "FAILED", "", rest, elapsed};
    return {name, "LOADING", rest, "", elapsed};
  }

  void on_model_status(int index, const std::string & name, const std::string & s) {
    loop_->set_model_status(index, parse_model(name, s, acd::now_sec() - t_start_));
    std::lock_guard<std::mutex> l(status_m_);
    check_inspection();
  }

  void on_ir_model_status(std::size_t i, const std::string & s) {
    std::lock_guard<std::mutex> l(status_m_);
    ir_model_status_[i] = s.substr(0, s.find(" | "));
    std::string worst = "READY", detail;
    for (std::size_t k = 0; k < ir_model_status_.size(); ++k) {
      if (ir_model_status_[k] == "READY") continue;
      if (worst != "FAILED") worst = ir_model_status_[k] == "FAILED" ? "FAILED" : "LOADING";
      detail += (detail.empty() ? "" : ", ") + ir_model_nodes_[k];
    }
    loop_->set_model_status(2, {"터널 IR 차선", worst, worst == "READY" ? "ready" : detail, worst == "FAILED" ? detail : "",
                                acd::now_sec() - t_start_});
    check_inspection();
  }

  void check_inspection() {   // status_m_ 잡은 채로. 자세·YOLO가 준비되면 점검 단계 끝 (차선·VESC 등은 출발 때 상태기계가 확인)
    if (inspection_done_ || !pose_ || !yolo_ready_) return;
    inspection_done_ = true;
    detect_->start();
    loop_->on_inspection_done(true, "", acd::now_sec());
    RCLCPP_INFO(get_logger(), "inspection done: camera attitude PASS, YOLO READY");
  }

  void load_yolo() {
    const double t0 = acd::now_sec();
    const std::string name = "YOLOX-tiny 512x320";
    try {
      detect_->load_model(models_, [&](const std::string & stage) { loop_->set_model_status(1, {name, "LOADING", stage, "", acd::now_sec() - t0}); });
      loop_->set_model_status(1, {name, "READY", "ready", "", acd::now_sec() - t0});
      std::lock_guard<std::mutex> l(status_m_);
      yolo_ready_ = true;
      check_inspection();
    } catch (const std::exception & e) {
      loop_->set_model_status(1, {name, "FAILED", "", e.what(), acd::now_sec() - t0});
      loop_->on_inspection_done(false, std::string("YOLO load failed: ") + e.what(), acd::now_sec());
      RCLCPP_ERROR(get_logger(), "YOLO load failed: %s", e.what());
    }
  }

  // ------------------------------------------------------------------ 터널
  void on_tunnel_state(const std::string & s) {
    if (s != "OUTSIDE" && s != "ENTERING" && s != "INSIDE" && s != "EXITING") return;
    const bool inside = s != "OUTSIDE";
    {
      std::lock_guard<std::mutex> l(status_m_);
      tunnel_state_ = s;
      if (inside == tunnel_inside_) return;
      tunnel_inside_ = inside;
    }
    // 터널 안: YOLO를 쉬고 콘·차량 기억을 비우며 신호등·앞차 추종을 끈다 (차선만 따라감). 나오면 새 관측부터 다시
    detect_->set_paused(inside);
    tunnel_view_ = inside;
    loop_->set_obstacles_suspended(inside);
    apply_settings();
    RCLCPP_INFO(get_logger(), "tunnel %s: obstacles/traffic/lead %s", s.c_str(), inside ? "suspended" : "restored");
  }

  // ------------------------------------------------------------------ 설정·출력
  acd::Reply apply_tuning(const std::map<std::string, double> & changes) {
    std::lock_guard<std::mutex> lock(tuning_m_);
    const auto result = registry_.apply(changes);
    if (!result.first) return result;
    bool tcn_changed = false, tunnel_changed = false, drive_changed = false;
    for (const auto & change : changes) {
      if (is_tcn_param(change.first)) tcn_changed = true;
      else if (change.first.rfind("tunnel.", 0) == 0) tunnel_changed = true;
      else drive_changed = true;
    }
    if (drive_changed) apply_settings();
    if (tcn_changed) publish_tcn_tuning();
    if (tunnel_changed) publish_tunnel_runtime_settings();
    return result;
  }

  void publish_tunnel_runtime_settings() {
    if (!tunnel_enabled_) return;
    auto settings = std::make_shared<bev_handoff::TunnelRuntimeSettings>();
    settings->validation_mode = registry_.get("tunnel.validation_mode") == 1;
    settings->ir_length_exit_enabled = registry_.get("tunnel.ir_length_exit_enabled") == 1;
    settings->ir_length_exit_ratio = registry_.get("tunnel.ir_length_exit_ratio");
    settings->ir_length_exit_confirm_frames = static_cast<int>(registry_.get("tunnel.ir_length_exit_confirm_frames"));
    settings->ir_length_exit_minimum_reference_m = registry_.get("tunnel.ir_length_exit_minimum_reference_m");
    bev_handoff::publishTunnelRuntimeSettings(std::move(settings));
  }

  void publish_tcn_tuning() {
    if (!tcn_tuning_pub_) return;
    acd::JsonWriter w;
    w.begin_object();
    for (const auto & p : registry_.list()) {
      if (!is_tcn_param(p.name)) continue;
      const auto name = p.name.substr(4);
      w.key(name.c_str()).value(p.value, 9);
    }
    w.end_object();
    std_msgs::msg::String message;
    message.data = w.str();
    tcn_tuning_pub_->publish(message);
  }

  void apply_settings() {
    std::lock_guard<std::mutex> settings_lock(apply_settings_m_);
    auto s = acd::settings_from(registry_);
    bool inside;
    { std::lock_guard<std::mutex> l(status_m_); inside = tunnel_inside_; }
    if (inside) { s.traffic.enabled = false; s.lead.enabled = false; }
    loop_->apply_settings(s);
    detect_->set_settings(s);
    publish_ir_camera(s.ir_camera);
    {
      std::lock_guard<std::mutex> l(status_m_);
      hint_base_.rgb = s.rgb_lane;
      hint_base_.ir = s.ir_lane;
      hint_base_.rgb.side_tracking = s.lane_check_mode != 0 && s.rgb_lane.side_tracking;
      hint_base_.ir.side_tracking = s.lane_check_mode != 0 && s.ir_lane.side_tracking;
      update_mount();   // ROI 비율이 바뀌었을 수 있음
    }
    hint_version_++;
    publish_hints();
  }

  // 웹 "IR 카메라" 값 → camera_driver (같은 프로세스). 바뀐 경우에만 새로 내서 장치 명령을 줄인다
  void publish_ir_camera(const acd::Settings::IrCamera & c) {
    auto conv = [](const acd::Settings::IrCameraProfile & p) {
      return bev_handoff::IrCameraProfile{p.exposure_us, p.iso, p.dot, p.flood};
    };
    const auto cur = bev_handoff::irCameraSettings();
    const auto in = conv(c.inside), out = conv(c.outside);
    if (cur && cur->inside == in && cur->outside == out && cur->use_400p_ir_output == c.use_400p_ir_output) return;
    auto s = std::make_shared<bev_handoff::IrCameraSettings>();
    s->inside = in;
    s->outside = out;
    s->use_400p_ir_output = c.use_400p_ir_output;
    bev_handoff::publishIrCameraSettings(std::move(s));
    const std::string out_exposure = out.exposure_us > 0 ? std::to_string(out.exposure_us) + "us" : std::string("AE");
    RCLCPP_INFO(get_logger(), "IR camera: inside %dus ISO%d dot%.2f flood%.2f | outside %s ISO%d dot%.2f flood%.2f",
                in.exposure_us, in.iso, in.dot_intensity, in.flood_intensity,
                out_exposure.c_str(), out.iso, out.dot_intensity, out.flood_intensity);
  }

  void publish_hints() {   // 바뀐 것이 있을 때만 새로 만들어 교체 (매 프레임 할당 없음). 제어·웹·상태 스레드에서 호출
    std::lock_guard<std::mutex> hl(hint_m_);
    const bool want = loop_->cones_nearby();
    const unsigned v = hint_version_.load();
    if (want == last_hint_want_ && v == last_hint_version_) return;
    auto h = std::make_shared<bev_handoff::LaneControlHints>();
    { std::lock_guard<std::mutex> l(status_m_); *h = hint_base_; }
    h->want_reference = want;
    bev_handoff::publishLaneControlHints(std::move(h));
    last_hint_want_ = want;
    last_hint_version_ = v;
  }

  void publish(const acd::StepOutput & o) {
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

  // ------------------------------------------------------------------ 텔레메트리
  void send_telemetry() {
    const double now = acd::now_sec();
    if (web_->client_count(now) == 0) return;   // 보는 사람이 없으면 아무것도 만들지 않음
    const bool full = web_->preview_wanted(now);
    if (!full && now - last_status_t_ < 0.5) return;   // 프리뷰 꺼짐: 상태만 초당 2회
    last_status_t_ = now;
    auto f = loop_->telemetry(full, now);
    f.stabilization = "camera_driver";
    if (full) {
      if (now - last_usage_t_ >= 1.0) { usage_ = sysmon_.sample(now); last_usage_t_ = now; }
      f.usage = usage_;
      f.params_version = registry_.version();
    }
    const double t0 = acd::now_sec();
    std::string j = acd::telemetry_json(f);
    const bool can_force_ir = tunnel_enabled_ && loop_->tunnel_override_allowed(now);
    if (!j.empty() && j.back() == '}') {   // auto_control 추가 항목
      std::ostringstream x;
      const auto tunnel_settings = bev_handoff::tunnelRuntimeSettings();
      {
        std::lock_guard<std::mutex> l(status_m_);
        x << ",\"handover\":\"" << json_escape(handover_mode_) << "\",\"joystick\":" << (now - last_joy_t_.load() < 1.0 ? "true" : "false")
          << ",\"tunnel\":{\"enabled\":" << (tunnel_enabled_ ? "true" : "false") << ",\"source\":\"" << lane_source_
          << "\",\"state\":\"" << json_escape(tunnel_state_) << "\"";
        if (lane_source_changed_s_) {
          x << ",\"source_changed_s\":" << std::fixed << std::setprecision(3) << *lane_source_changed_s_
            << std::defaultfloat << std::setprecision(6);
        }
        if (lane_source_ == "IR") {
          const char * ir = now - last_ir_lane_t_ > 0.5 ? "stale" : last_ir_lane_valid_ ? "valid" : "invalid";
          x << ",\"ir_lane\":\"" << ir << "\"";
        }
        if (tunnel_settings) x << ",\"validation_mode\":" << (tunnel_settings->validation_mode ? "true" : "false");
        x << ",\"force_ir\":" << (force_ir_enabled_.load() ? "true" : "false")
          << ",\"force_ir_change_allowed\":" << (can_force_ir ? "true" : "false") << "}"
          << ",\"yolo\":{\"fps\":" << detect_->fps() << ",\"signal_score\":" << detect_->last_signal_score() << "}"
          << ",\"refit_supported\":true";
      }
      if (tunnel_enabled_) {
        x << ",\"tcn\":" << tcn_telemetry_.json(now - t_start_) << ",\"camera_exposure\":" << camera_exposure_json();
      }
      if (drive_t0_ > 0 && f.state == "DRIVING") x << ",\"drive_s\":" << (now - drive_t0_);
      x << ",\"record\":" << record_status_json() << "}";
      j.pop_back();
      j += x.str();
    }
    if (f.state == "DRIVING") { if (drive_t0_ <= 0) drive_t0_ = now; } else drive_t0_ = 0;
    web_->publish_telemetry(std::move(j));
    if (full) loop_->stats().record("web", t0, acd::now_sec());
  }

  // ------------------------------------------------------------------ 기록 (콘 관측·수동 주행 녹화)
  static std::string time_string(const char * fmt) {
    char b[64];
    const std::time_t now = std::time(nullptr);
    std::strftime(b, sizeof(b), fmt, std::localtime(&now));
    return b;
  }

  void open_cone_log(const std::string & dir, std::string name) {   // name이 비면 cone_obs_<시각>.csv
    if (!cone_log_enabled_) return;
    std::lock_guard<std::mutex> l(cone_log_m_);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (name.empty()) name = time_string("cone_obs_%Y-%m-%d_%H%M%S.csv");
    cone_log_ = std::make_unique<std::ofstream>(dir + "/" + name);
    if (!*cone_log_) { RCLCPP_WARN(get_logger(), "cannot write cone log in %s", dir.c_str()); cone_log_.reset(); return; }
    *cone_log_ << std::setprecision(10)
               << "t_rgb,t_depth,t_pose,id,class,local_x,local_y,front_x,front_y,range,bearing,pose_x,pose_y,pose_yaw,"
                  "speed,yaw_rate,gyro_bias,world_x,world_y,mem_x,mem_y,hits\n";
    RCLCPP_INFO(get_logger(), "cone observation log: %s/%s", dir.c_str(), name.c_str());
  }

  bool recording() { std::lock_guard<std::mutex> l(rec_m_); return bool(rec_); }

  acd::Reply record_start(std::string label) {   // <log.dir>/manual/<시각>_<상황>/ meta.json, params.yaml, frames.csv, cone_obs.csv
    for (auto & ch : label) if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '-' && ch != '_') ch = '_';
    if (label.empty()) label = "run";
    std::string dir;
    {
      std::lock_guard<std::mutex> l(rec_m_);
      if (rec_) return {false, "이미 녹화 중: " + rec_dir_};
      dir = log_dir_ + "/manual/" + time_string("%Y-%m-%d_%H%M%S") + "_" + label;
      std::error_code ec;
      std::filesystem::create_directories(dir, ec);
      rec_ = std::make_unique<std::ofstream>(dir + "/frames.csv");
      if (!*rec_) { rec_.reset(); return {false, "폴더를 만들 수 없음: " + dir}; }
      *rec_ << std::setprecision(10)
            << "t,pose_x,pose_y,pose_yaw,speed,yaw_rate,state,plan_mode,rollout_ok,obstacle_active,left,right,path,plan,cones,center\n";
      rec_dir_ = dir;
      rec_t0_ = acd::now_sec();
      rec_frames_ = 0;
    }
    double camera_x = 0;
    { std::lock_guard<std::mutex> l(status_m_); if (reference_) camera_x = reference_->position_vehicle_m[0]; }
    {
      std::ofstream y(dir + "/params.yaml");
      y << registry_.to_yaml("auto_drive");
      std::ofstream m(dir + "/meta.json");
      m << "{\"label\":\"" << label << "\",\"started\":\"" << time_string("%Y-%m-%d %H:%M:%S") << "\",\"camera_x\":" << camera_x
        << ",\"wheelbase\":" << rc_.wheelbase << ",\"vehicle_length\":" << registry_.get("plan.vehicle_length_m")
        << ",\"vehicle_width\":" << registry_.get("plan.vehicle_width_m") << ",\"rear_overhang\":" << registry_.get("plan.vehicle_rear_overhang_m")
        << ",\"cone_size\":" << registry_.get("object.cone_size_m") << ",\"memory_mode\":" << registry_.get("memory.mode") << "}\n";
    }
    open_cone_log(dir, "cone_obs.csv");
    RCLCPP_INFO(get_logger(), "manual recording start: %s", dir.c_str());
    return {true, dir};
  }

  acd::Reply record_stop() {
    std::string dir;
    int frames = 0;
    {
      std::lock_guard<std::mutex> l(rec_m_);
      if (!rec_) return {false, "녹화 중이 아님"};
      rec_->flush();
      rec_.reset();
      dir = rec_dir_;
      frames = rec_frames_;
      rec_last_ = dir;
    }
    { std::lock_guard<std::mutex> l(cone_log_m_); cone_log_.reset(); }
    RCLCPP_INFO(get_logger(), "manual recording saved: %s (%d frames)", dir.c_str(), frames);
    return {true, dir + " (" + std::to_string(frames) + " 프레임)"};
  }

  static void put_pts(std::ostream & o, const std::vector<acd::Vec2> & v, std::size_t max_n) {
    const std::size_t stride = std::max<std::size_t>(1, (v.size() + max_n - 1) / std::max<std::size_t>(1, max_n));
    for (std::size_t i = 0; i < v.size(); i += stride) o << (i ? ";" : "") << std::fixed << std::setprecision(3) << v[i].x << ' ' << v[i].y;
    o << std::defaultfloat << std::setprecision(10);
  }

  void record_frame(const acd::RecordFrame & f) {
    std::lock_guard<std::mutex> l(rec_m_);
    if (!rec_) return;
    auto & o = *rec_;
    o << f.t << ',' << f.pose.x << ',' << f.pose.y << ',' << f.pose.yaw << ',' << f.speed << ',' << f.yaw_rate << ',' << f.state << ','
      << f.plan_mode << ',' << int(f.rollout_ok) << ',' << int(f.obstacle_active) << ',';
    put_pts(o, f.left, 40); o << ',';
    put_pts(o, f.right, 40); o << ',';
    put_pts(o, f.path, 40); o << ',';
    put_pts(o, f.plan_path, 40); o << ',';
    put_pts(o, f.cones_world, 60); o << ',';
    put_pts(o, f.center, 40); o << '\n';
    if (++rec_frames_ % 30 == 0) o.flush();
  }

  std::string record_status_json() {
    std::lock_guard<std::mutex> l(rec_m_);
    std::ostringstream s;
    s << "{\"active\":" << (rec_ ? "true" : "false") << ",\"dir\":\"" << json_escape(rec_ ? rec_dir_ : rec_last_) << "\",\"seconds\":"
      << (rec_ ? acd::now_sec() - rec_t0_ : 0.0) << ",\"frames\":" << rec_frames_ << "}";
    return s.str();
  }

  struct Pose { double height, roll, pitch; };

  acd::ParamRegistry registry_;
  acd::RuntimeConfig rc_;
  acd::ModelConfig models_;
  double camera_yaw_deg_{0}, fixed_view_zoom_{1.25};
  std::string params_save_path_, lane_model_node_;
  std::vector<std::string> ir_model_nodes_;
  std::vector<std::string> ir_model_status_;
  bool tunnel_enabled_{false};
  std::unique_ptr<acd::ControlLoop> loop_;
  std::unique_ptr<acd::DetectWorker> detect_;
  std::unique_ptr<acd::WebServer> web_;
  acd::SystemMonitor sysmon_;
  acd::SystemUsage usage_;
  acd::TunnelTcnTelemetry tcn_telemetry_;   // web_group_에서만 갱신·조회
  double last_usage_t_{0}, last_status_t_{0}, drive_t0_{0};
  const double t_start_{acd::now_sec()};
  // 점검·상태 (status_m_)
  std::mutex status_m_;
  std::mutex apply_settings_m_;
  std::mutex tuning_m_;
  std::mutex drive_command_m_;
  std::optional<rclcpp::Client<ForceIr>::FutureAndRequestId> force_ir_request_;
  bool force_ir_requested_value_{false};
  std::atomic<bool> force_ir_enabled_{false};
  std::optional<Pose> pose_;
  std::shared_ptr<const bev_handoff::ObstacleReference> reference_;
  bool yolo_ready_{false}, inspection_done_{false}, tunnel_inside_{false};
  std::string handover_mode_{"-"}, lane_source_{"RGB"}, tunnel_state_{"OUTSIDE"};
  std::optional<double> lane_source_changed_s_;
  double last_ir_lane_t_{-1e9};
  bool last_ir_lane_valid_{false};
  bev_handoff::LaneControlHints hint_base_;
  std::atomic<unsigned> hint_version_{0};
  std::mutex hint_m_;   // last_hint_*
  unsigned last_hint_version_{~0u};
  bool last_hint_want_{false};
  std::atomic<double> last_lane_t_{0}, last_joy_t_{-1e9};
  int joy_start_button_{0}, joy_refit_button_{1}, joy_stop_button_{2};
  bool joy_start_was_{false}, joy_refit_was_{false}, joy_stop_was_{false};
  // 영상·깊이
  std::uint64_t camera_consumer_{0}, device_consumer_{0};
  std::chrono::steady_clock::time_point last_depth_{};
  std::atomic<bool> stopping_{false}, bev_wanted_{false};
  double last_bev_tap_t_{0};
  std::mutex bev_m_;
  std::condition_variable bev_cv_;
  std::shared_ptr<const bev_handoff::DirectBevFrame> bev_pending_;
  cv::Mat ir_bev_pending_;                                  // 터널 안 IR BEV (bev_m_)
  std::atomic<bool> tunnel_view_{false};                    // 터널 안: 웹 카메라·BEV 원본을 IR로
  std::atomic<double> last_ir_bev_t_{-1e9};
  std::chrono::steady_clock::time_point last_ir_camera_at_{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ir_bev_sub_;
  rclcpp::TimerBase::SharedPtr ir_camera_timer_;
  std::thread inspect_thread_, depth_thread_, bev_thread_;
  // 기록
  std::mutex rec_m_;
  std::unique_ptr<std::ofstream> rec_;
  std::string rec_dir_, rec_last_;
  double rec_t0_{0};
  int rec_frames_{0};
  bool cone_log_enabled_{true};
  std::string log_dir_;
  std::mutex cone_log_m_;
  std::unique_ptr<std::ofstream> cone_log_;
  // ROS
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_pub_, brake_pub_, servo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_, refit_pub_, tcn_tuning_pub_;
  std::atomic<bool> refit_sent_{false};   // 제어 스레드(watchdog)가 쓰고 상태 스레드가 읽음
  double refit_t0_{0};
  std::string refit_failure_;   // status_m_
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr erpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connected_sub_, force_ir_sub_;
  rclcpp::Client<ForceIr>::SharedPtr force_ir_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_, startup_sub_, lane_model_sub_, tunnel_sub_, tcn_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> ir_model_subs_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<line_detactor_msgs::msg::LaneResult>::SharedPtr lane_sub_, ir_lane_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_, telemetry_, setup_timer_;
  rclcpp::CallbackGroup::SharedPtr imu_group_, vehicle_group_, control_group_, status_group_, web_group_;
  OnSetParametersCallbackHandle::SharedPtr on_set_;
};

}  // namespace auto_control

RCLCPP_COMPONENTS_REGISTER_NODE(auto_control::AutoDriveNode)
