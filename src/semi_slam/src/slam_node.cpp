// semi_slam 주행 프로세스: OAK → (IMU 안정화 켜기/끄기) → CUDA BEV → 차선 → 일정 속도 차선 주행
//   + 바퀴 거리·IMU 위치 적분 → 정답 트랙 위 매핑 → 정지선 S로 바퀴 세기·위치 보정 → 목표 바퀴 완료 시 보정 계산·적용 → 기록
// 자동 명령은 semi_slam/auto/*로만 낸다 (vesc/*는 안전 프로세스가 유일하게 발행). 정지하면 수동 주행으로 넘어간다.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include "semi_slam/core/drive_loop.hpp"
#include "semi_slam/core/json_writer.hpp"
#include "semi_slam/core/lane_fit.hpp"
#include "semi_slam/core/mini_json.hpp"
#include "semi_slam/core/run_store.hpp"
#include "semi_slam/core/system_monitor.hpp"
#include "semi_slam/runtime/camera_pipeline.hpp"
#include "semi_slam/runtime/lane_worker.hpp"
#include "semi_slam/runtime/web_server.hpp"

using namespace std::chrono_literals;
namespace ss = semi_slam;

namespace {
void points_json(ss::JsonWriter & w, const std::vector<ss::Vec2> & pts, std::size_t stride = 1) {
  w.begin_array();
  for (std::size_t i = 0; i < pts.size(); i += stride) w.begin_array().value(pts[i].x, 3).value(pts[i].y, 3).end_array();
  w.end_array();
}
}  // namespace

class SlamNode : public rclcpp::Node {
public:
  static double as_number(const rclcpp::Parameter & p) {
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE: return p.as_double();
      case rclcpp::ParameterType::PARAMETER_INTEGER: return double(p.as_int());
      case rclcpp::ParameterType::PARAMETER_BOOL: return p.as_bool() ? 1.0 : 0.0;
      default: throw std::invalid_argument("parameter '" + p.get_name() + "' must be a number");
    }
  }

  SlamNode() : Node("semi_slam") {
    ss::register_params(registry_);
    rcl_interfaces::msg::ParameterDescriptor any_number;
    any_number.dynamic_typing = true;
    for (const auto & p : registry_.list()) declare_parameter(p.name, rclcpp::ParameterValue(p.value), any_number);
    std::map<std::string, double> start;
    for (const auto & p : registry_.list()) start[p.name] = as_number(get_parameter(p.name));
    const auto applied = registry_.apply(start);
    if (!applied.first) throw std::invalid_argument("parameter file: " + applied.second);

    const auto share = ament_index_cpp::get_package_share_directory("semi_slam");
    const std::string home = std::getenv("HOME") ? std::getenv("HOME") : "/tmp";
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
    front_overhang_ = declare_parameter<double>("vehicle.front_overhang_m", 0.05);

    cam_cfg_.device_id = declare_parameter<std::string>("camera.device_id", "");
    cam_cfg_.fps = declare_parameter<double>("camera.fps", 70.0);
    const double fixed_view_zoom = declare_parameter<double>("camera.fixed_view_zoom", 1.25);
    cam_cfg_.fx = declare_parameter<double>("camera.fx", 701.751174926) / fixed_view_zoom;
    cam_cfg_.fy = declare_parameter<double>("camera.fy", 701.420440674) / fixed_view_zoom;
    cam_cfg_.cx = declare_parameter<double>("camera.cx", cam_cfg_.cx);
    cam_cfg_.cy = declare_parameter<double>("camera.cy", cam_cfg_.cy);
    camera_x_ = declare_parameter<double>("camera.x_m", -0.09);
    camera_y_ = declare_parameter<double>("camera.y_m", 0.0);
    measure_cfg_.manual_camera_height_enabled = declare_parameter<bool>("camera.manual_height_enabled", false);
    measure_cfg_.manual_camera_height_m = declare_parameter<double>("camera.manual_height_m", 0.20);
    measure_cfg_.timeout_sec = declare_parameter<double>("camera.measurement_timeout_sec", 45.0);

    models_.lane_precision = declare_parameter<std::string>("model.lane_precision", "int8");
    models_.lane_model = declare_parameter<std::string>("model.lane_path", share + (models_.lane_precision == "int8"
      ? "/models/fast_scnn_stop_line_120x300_batch_1.int8.qdq.onnx" : "/models/fast_scnn_stop_line_120x300_batch_1.onnx"));
    const auto cache = declare_parameter<std::string>("model.engine_cache_dir", home + "/.cache/semi_slam");
    std::filesystem::create_directories(cache);
    models_.lane_engine_cache = cache + "/lane_" + models_.lane_precision + ".engine";
    params_save_path_ = declare_parameter<std::string>("params_save_path", cache + "/semi_slam_saved.yaml");

    data_dir_ = declare_parameter<std::string>("slam.data_dir", "");
    if (data_dir_.empty()) data_dir_ = home + "/semi_slam_data";
    auto map_path = declare_parameter<std::string>("slam.map_path", "");
    if (map_path.empty()) map_path = share + "/maps/track_4x4.csv";
    slam_ = std::make_unique<ss::SlamCore>(ss::TrackMap::load_csv(map_path));
    store_ = std::make_unique<ss::CalibStore>(data_dir_);
    {
      auto c = store_->load();
      if (c.version < ss::OdomCalib::kVersion) {   // 예전 방식(지도 맞춤 Nelder–Mead) 보정 값은 쓰지 않는다
        const auto backup = store_->reset();
        c = ss::OdomCalib{};
        RCLCPP_WARN(get_logger(), "old calibration format: moved to %s, starting from defaults", backup.c_str());
      }
      slam_->set_calib(c);
    }
    {   // 웹에서 바꾸거나 자동 측정한 카메라 yaw는 data_dir에 남겨 다음 실행에도 쓴다
      std::ifstream in(data_dir_ + "/camera_yaw.json");
      std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
      const auto j = ss::parse_json(text);
      if (j && j->get("yaw_deg") && j->get("yaw_deg")->kind == ss::JsonValue::Number) {
        registry_.apply({{"camera.yaw_deg", j->get("yaw_deg")->num}});
        RCLCPP_INFO(get_logger(), "camera yaw %.2f deg (from %s/camera_yaw.json)", j->get("yaw_deg")->num, data_dir_.c_str());
      }
      camera_yaw_deg_ = registry_.get("camera.yaw_deg");
    }
    RCLCPP_INFO(get_logger(), "track %s (lap %.2f m), data %s", map_path.c_str(), slam_->map().lap_length(), data_dir_.c_str());

    loop_ = std::make_unique<ss::DriveLoop>(rc_);

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    duty_pub_ = create_publisher<std_msgs::msg::Float32>(declare_parameter<std::string>("topics.auto_duty", "semi_slam/auto/duty"), command_qos);
    brake_pub_ = create_publisher<std_msgs::msg::Float32>(declare_parameter<std::string>("topics.auto_brake_current", "semi_slam/auto/brake_current"), command_qos);
    servo_pub_ = create_publisher<std_msgs::msg::Float32>(declare_parameter<std::string>("topics.auto_servo_position", "semi_slam/auto/servo_position"), command_qos);
    request_pub_ = create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("topics.handover_request", "semi_slam/handover/request"),
                                                           rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    erpm_sub_ = create_subscription<std_msgs::msg::Int32>(declare_parameter<std::string>("topics.measured_erpm", "vesc/measured_erpm"), command_qos,
      [this](std_msgs::msg::Int32::ConstSharedPtr m) {
        const double t = ss::now_sec();
        slam_->on_speed(loop_->on_erpm(m->data, t), t);
      });
    connected_sub_ = create_subscription<std_msgs::msg::Bool>(declare_parameter<std::string>("topics.vesc_connected", "vesc/connected"), state_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { loop_->on_vesc_connected(m->data); });
    mode_sub_ = create_subscription<std_msgs::msg::String>(declare_parameter<std::string>("topics.handover_mode", "semi_slam/handover/mode"), state_qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) { loop_->on_handover_mode(m->data, ss::now_sec()); });

    // 웹
    ss::WebCallbacks cb;
    cb.start = [this] { return start_run(); };
    cb.stop = [this] { loop_->request_stop("stop button", ss::now_sec()); };
    cb.params = [this](const std::map<std::string, double> & c) {
      auto r = registry_.apply(c);
      if (r.first) apply_settings();
      return r;
    };
    cb.params_save = [this] {
      std::ofstream out(params_save_path_);
      out << registry_.to_yaml("semi_slam");
      return out ? ss::Reply{true, params_save_path_} : ss::Reply{false, "cannot write " + params_save_path_};
    };
    cb.heartbeat = [this](const std::string & id) { loop_->on_heartbeat(id, ss::now_sec()); };
    cb.get_json["map"] = [this] { return map_json(); };
    cb.get_json["run"] = [this] { return run_json(); };
    cb.actions["reset"] = [this] {
      if (slam_->live().running || finishing_) return ss::Reply{false, "주행 중·기록 처리 중에는 초기화할 수 없음"};
      const auto backup = store_->reset();
      slam_->set_calib(ss::OdomCalib{});
      set_status("보정 초기화: 이전 값·이력은 " + backup + " 로 옮김");
      RCLCPP_INFO(get_logger(), "calibration reset, backup %s", backup.c_str());
      return ss::Reply{true, backup};
    };
    cb.actions["yaw_measure"] = [this] {
      if (slam_->live().running || loop_->state() == ss::DriveState::Driving) return ss::Reply{false, "주행 중에는 측정할 수 없음: 정지선 S 앞에 세우고 누르세요"};
      std::lock_guard<std::mutex> l(yaw_m_);
      yaw_ = YawMeasure{};
      yaw_.active = true;
      yaw_.t0 = ss::now_sec();
      yaw_.message = "측정 중 (2초): 차를 움직이지 마세요";
      return ss::Reply{true, ""};
    };
    cb.actions["revert"] = [this] {
      if (slam_->live().running) return ss::Reply{false, "주행 중에는 되돌릴 수 없음"};
      ss::OdomCalib c;
      if (!store_->revert(&c)) return ss::Reply{false, "되돌릴 보정이 없음"};
      slam_->set_calib(c);
      set_status("이전 보정 값으로 되돌림");
      return ss::Reply{true, ""};
    };
    web_ = std::make_unique<ss::WebServer>(declare_parameter<std::string>("web.root", share + "/web"),
                                           declare_parameter<std::string>("web.bind", "0.0.0.0"),
                                           int(declare_parameter<int>("web.port", 8766)), registry_, cb);
    web_->start();
    RCLCPP_INFO(get_logger(), "Preview: http://<jetson-ip>:%d", web_->port());

    ss::ImuImageStabilizerConfig stab;
    stab.maximum_correction_deg = declare_parameter<double>("stabilization.maximum_correction_deg", 3.0);
    stab.maximum_frame_imu_wait_sec = declare_parameter<double>("stabilization.maximum_frame_imu_wait_sec", 0.005);
    stab.maximum_frame_imu_age_sec = declare_parameter<double>("stabilization.maximum_frame_imu_age_sec", 0.012);
    stab.maximum_frame_imu_prediction_sec = declare_parameter<double>("stabilization.maximum_prediction_sec", 0.015);
    camera_ = std::make_unique<ss::CameraPipeline>(cam_cfg_, stab);
    camera_->stats = &loop_->stats();
    camera_->on_yaw_rate = [this](double w, double t) { slam_->on_gyro(w, t); };
    camera_->stationary_hint = [this]() -> std::optional<bool> {
      const double v = loop_->speed();
      if (std::abs(v) < 0.03) return true;
      if (std::abs(v) > 0.10) return false;
      return std::nullopt;
    };
    lane_ = std::make_unique<ss::LaneWorker>(*camera_, loop_->stats(), [this](const ss::LaneObservation & lane) {
      publish(loop_->on_lane(lane, ss::now_sec()));
      feed_yaw_measure(lane);
      if (slam_->on_lane(lane)) loop_->request_stop("목표 바퀴 완료", ss::now_sec());
    });
    lane_->bev_wanted = [this] { return web_->stream_wanted("bev", ss::now_sec()); };
    lane_->publish_bev_jpeg = [this](std::vector<std::uint8_t> j) { web_->publish_jpeg("bev", std::move(j)); };
    apply_settings();

    on_set_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
      std::map<std::string, double> c;
      for (const auto & p : ps) if (registry_.has(p.get_name())) c[p.get_name()] = as_number(p);
      rcl_interfaces::msg::SetParametersResult res;
      const auto r = registry_.apply(c);
      res.successful = r.first;
      res.reason = r.second;
      if (r.first && !c.empty()) apply_settings();
      return res;
    });

    watchdog_ = create_wall_timer(20ms, [this] {
      if (ss::now_sec() - lane_->last_frame_time() > 0.05) publish(loop_->tick(ss::now_sec()));
      check_run_end();
    });
    collector_ = create_wall_timer(100ms, [this] { collect(); });
    telemetry_ = create_wall_timer(66ms, [this] { send_telemetry(); });
    inspect_thread_ = std::thread([this] { inspect(); });
  }

  ~SlamNode() override {
    stopping_ = true;
    if (inspect_thread_.joinable()) inspect_thread_.join();
    if (finish_thread_.joinable()) finish_thread_.join();
    lane_->stop();
    camera_->close();
    web_->stop();
  }

private:
  void apply_settings() {
    const auto s = ss::settings_from(registry_);
    if (std::abs(s.camera_yaw_deg - camera_yaw_deg_) > 1e-6) {   // 카메라 yaw: BEV 변환을 다시 만든다 (바로 반영)
      camera_yaw_deg_ = s.camera_yaw_deg;
      rebuild_geometry();
      std::ofstream out(data_dir_ + "/camera_yaw.json");
      out << "{\"yaw_deg\":" << camera_yaw_deg_.load() << "}\n";
      RCLCPP_INFO(get_logger(), "camera yaw -> %.2f deg", camera_yaw_deg_.load());
    }
    loop_->apply_settings(s);
    slam_->set_config(s.slam);
    if (camera_) camera_->stabilization_on = s.stabilization;
    if (lane_) {
      ss::LanePostConfig lc;
      lc.centerline.corner_outward_offset_m = s.corner_outward_offset_m;
      lc.centerline.corner_entry_distance_m = s.corner_entry_distance_m;
      lc.centerline.corner_entry_full_offset_distance_m = s.corner_entry_full_offset_m;
      lc.side_tracking = s.lane_side_tracking;
      lc.centerline.lane_width_m = slam_->map().lane_width();   // 한쪽 선만 보일 때 중앙선 = 그 선 + 실제 반폭 (주행 경로)
      lane_->set_post_config(lc);
    }
  }

  void set_status(const std::string & s) { std::lock_guard<std::mutex> l(run_m_); status_ = s; }

  // 출발 위치: 앞범퍼를 아래 직선 시작점에 맞춘 도로 가운데 (반시계 = 왼쪽 아래에서 +x, 시계 = 오른쪽 아래에서 −x)
  ss::Pose start_pose(const ss::Settings & s) const {
    const auto & m = slam_->map();
    const ss::Vec2 T = m.straight_start(s.clockwise), d = m.travel_dir_at(T, s.clockwise), left{-d.y, d.x};
    const double along = s.start_offset_m - front_overhang_;   // 앞차축 = 앞범퍼 − 앞 오버행
    return {ss::now_sec(), T.x + d.x * along + left.x * s.start_lateral_m, T.y + d.y * along + left.y * s.start_lateral_m,
            std::atan2(d.y, d.x), 0};
  }

  ss::Reply start_run() {
    if (finishing_) return {false, "이전 주행 기록·보정 처리 중"};
    const auto res = loop_->request_start(ss::now_sec());
    if (!res.first) return res;
    const auto s = loop_->settings();
    {
      std::lock_guard<std::mutex> l(run_m_);
      run_dir_ = ss::make_run_dir(data_dir_, s.run_speed);
      calib_used_ = slam_->calib();
      track_.clear(); points_.clear(); sides_.clear(); after_track_.clear(); free_track_.clear();
      pending_track_.clear(); pending_points_.clear(); pending_sides_.clear(); pending_free_.clear();
      track_cursor_ = 0; free_cursor_ = 0;
      status_ = "주행 중";
    }
    slam_->begin_run(s.run_speed, s.run_laps, start_pose(s), s.clockwise);
    std::map<std::string, double> rp;
    for (const char * k : {"camera.yaw_deg", "camera.stabilization", "run.speed_mps", "run.laps", "slam.direction", "slam.stopline_correction",
                           "lane.corner_outward_offset_m", "lane.corner_entry_distance_m", "lane.side_tracking", "drive.stanley_gain",
                           "drive.heading_lookahead_m", "drive.longitudinal_mode"})
      if (registry_.has(k)) rp[k] = registry_.get(k);
    slam_->set_run_params(rp);
    RCLCPP_INFO(get_logger(), "run start: %.2f m/s, %d laps -> %s", s.run_speed, s.run_laps, run_dir_.c_str());
    return res;
  }

  void collect() {   // 화면용 궤적·매핑 점을 서버에 모아 둔다 (재접속·보정 후 다시 그리기용)
    std::vector<int> sides;
    auto pts = slam_->take_map_points(&sides);
    std::lock_guard<std::mutex> l(run_m_);
    auto tr = slam_->track_since(&track_cursor_);
    for (const auto & p : tr) { track_.push_back(p); pending_track_.push_back(p); }
    for (const auto & p : slam_->free_since(&free_cursor_)) { free_track_.push_back(p); pending_free_.push_back(p); }
    for (std::size_t i = 0; i < pts.size(); ++i) {
      if (points_.size() < 60000) { points_.push_back(pts[i]); sides_.push_back(sides[i]); }
      if (pending_points_.size() < 3000) { pending_points_.push_back(pts[i]); pending_sides_.push_back(sides[i]); }
    }
  }

  void check_run_end() {   // 차가 멈추면(수동 전환 또는 준비 상태) 주행을 끝내고 기록·보정
    const auto live = slam_->live();
    if (!live.running || finishing_) return;
    const auto st = loop_->state();
    if (st == ss::DriveState::Driving || st == ss::DriveState::Arming || st == ss::DriveState::Stopping) return;
    slam_->end_run();
    collect();
    finishing_ = true;
    if (finish_thread_.joinable()) finish_thread_.join();
    finish_thread_ = std::thread([this, live] { finish_run(live.finished); });
  }

  void finish_run(bool completed) {
    const auto run = slam_->run_data();
    std::string dir;
    std::vector<ss::Pose> track;
    ss::OdomCalib used;
    { std::lock_guard<std::mutex> l(run_m_); dir = run_dir_; track = track_; used = calib_used_; status_ = "기록 저장 중"; }
    try {
      ss::write_run(dir, run, used, slam_->lap_metrics(), slam_->corrections(), track);
      const auto s = loop_->settings();
      if (!completed) {
        set_status("중간 정지: 기록만 저장 (보정은 목표 바퀴를 채웠을 때만)");
      } else if (!s.auto_calibrate) {
        set_status("목표 바퀴 완료: 자동 보정 꺼짐 (기록만 저장)");
      } else {
        set_status("보정 계산 중");
        const auto r = ss::calibrate(run, used, slam_->map(), s.slam.max_range_m);
        ss::write_calibration_result(dir, r);
        store_->record(r, dir, used, run.target_speed, int(run.crossing_t.size()), r.applied);
        if (r.applied) { store_->save(r.calib); slam_->set_calib(r.calib); }
        const auto after = ss::integrate(run, r.calib, slam_->map(), run.stopline_correction);
        std::string msg = r.message;
        if (std::isfinite(r.est.bev_yaw_residual_deg)) {
          char b[160];
          std::snprintf(b, sizeof(b), " 주행 검산: BEV가 %+.1f° 더 기울어 보임 (직선 %d개)", r.est.bev_yaw_residual_deg, r.est.straights);
          msg += b;
        }
        std::lock_guard<std::mutex> l(run_m_);
        after_track_.clear();
        for (std::size_t i = 0; i < after.poses.size(); i += 20) after_track_.push_back(after.poses[i]);
        last_result_ = r;
        has_result_ = true;
        status_ = msg;
      }
      RCLCPP_INFO(get_logger(), "run saved: %s", dir.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "run finish failed: %s", e.what());
      set_status(std::string("기록·보정 실패: ") + e.what());
    }
    finishing_ = false;
  }

  void publish(const ss::StepOutput & o) {
    if (!o.handover_request.empty()) {
      std_msgs::msg::String m;
      m.data = o.handover_request;
      request_pub_->publish(m);
    }
    if (!o.publish) return;
    std_msgs::msg::Float32 v;
    if (o.brake_mode) { v.data = float(o.brake_current); brake_pub_->publish(v); }
    else { v.data = float(o.duty); duty_pub_->publish(v); }
    v.data = float(o.servo);
    servo_pub_->publish(v);
  }

  void inspect() {   // 출발 전 점검: 카메라 자세 측정 → 차선 모델 로드
    loop_->set_attitude({false, 0, 0, 0, "MEASURING", "정지 상태 지면·IMU 측정 중"});
    bool cam_ok = false;
    try {
      const auto m = camera_->measure(measure_cfg_, [this] { return stopping_.load(); });
      { std::lock_guard<std::mutex> l(geom_m_); att_ = {true, m.height_m, m.roll_deg, m.pitch_down_deg}; }
      rebuild_geometry();
      camera_->open(m, camera_yaw_deg_.load());
      loop_->set_attitude({true, m.height_m, m.roll_deg, m.pitch_down_deg, "PASS", ""});
      cam_ok = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "camera attitude measurement failed: %s", e.what());
      loop_->set_attitude({false, 0, 0, 0, "FAIL", e.what()});
    }
    const double t0 = ss::now_sec();
    bool lane_ok = false;
    try {
      lane_->load_model(models_, [&](const std::string & stage) { loop_->set_model_status({"차선 (Fast-SCNN)", "LOADING", stage, "", ss::now_sec() - t0}); });
      loop_->set_model_status({"차선 (Fast-SCNN)", "READY", "ready", "", ss::now_sec() - t0});
      lane_ok = true;
    } catch (const std::exception & e) {
      loop_->set_model_status({"차선 (Fast-SCNN)", "FAILED", "", e.what(), ss::now_sec() - t0});
      RCLCPP_ERROR(get_logger(), "lane model load failed: %s", e.what());
    }
    if (cam_ok) lane_->start();
    loop_->on_inspection_done(cam_ok && lane_ok, cam_ok ? "lane model load failed" : "camera attitude failed", ss::now_sec());
  }

  // 카메라 자세(측정값) + yaw(설정) → BEV 변환. yaw를 바꾸면 차선 모델이 보는 BEV 자체가 돌아간다 (주행·매핑 모두 반영)
  void rebuild_geometry() {
    std::lock_guard<std::mutex> l(geom_m_);
    if (!att_.ok || !lane_) return;
    ss::RectifiedCameraModel cam{cam_cfg_.fx, cam_cfg_.fy, cam_cfg_.cx, cam_cfg_.cy, cam_cfg_.width, cam_cfg_.height,
                                 cv::Vec3d(camera_x_, camera_y_, att_.height),
                                 ss::mountRotationVehicleFromCamera(ss::degToRad(att_.roll), ss::degToRad(att_.pitch), ss::degToRad(camera_yaw_deg_.load()))};
    lane_->set_geometry(cam, ss::BevConfig{0.0, 3.0, -0.6, 0.6, 0.01, 120, 300});
  }

  // 카메라 yaw 자동 측정: 차를 정지선 S 앞 0.3~1.0 m에 차선과 나란히 세우고 2초 동안
  // 양쪽 차선 기울기와 정지선 기울기를 잰다. 둘이 1° 안에서 맞으면 평균만큼 yaw를 고친다
  void feed_yaw_measure(const ss::LaneObservation & lane) {
    std::lock_guard<std::mutex> l(yaw_m_);
    if (!yaw_.active) return;
    const double now = ss::now_sec();
    if (std::abs(loop_->speed()) > 0.03) { yaw_.active = false; yaw_.message = "차가 움직여서 취소함"; return; }
    if (lane.valid) {
      const auto f = ss::fit_lane_frame(lane.left, lane.right, 0.15, 0.8, 0.3);
      if (f.both) { yaw_.lane_sum += f.angle; ++yaw_.lane_n; }
      if (std::isfinite(lane.stop_line_angle) && std::isfinite(lane.stop_line_m) && lane.stop_line_m > 0.2 && lane.stop_line_m < 1.2) {
        yaw_.stop_sum += lane.stop_line_angle; ++yaw_.stop_n;
      }
    }
    if (now - yaw_.t0 < 2.0) return;
    yaw_.active = false;
    const double deg = 180.0 / M_PI;
    yaw_.lane_deg = yaw_.lane_n ? yaw_.lane_sum / yaw_.lane_n * deg : std::nan("");
    yaw_.stop_deg = yaw_.stop_n ? yaw_.stop_sum / yaw_.stop_n * deg : std::nan("");
    char b[240];
    if (yaw_.lane_n < 30) { yaw_.message = "양쪽 차선이 잘 안 보여 실패: 직선에서 차선 가운데에 세우세요"; return; }
    if (yaw_.stop_n < 15) { yaw_.message = "정지선이 안 보여 실패: 정지선 S 앞 0.3~1.0 m에 세우세요"; return; }
    if (std::abs(yaw_.lane_deg - yaw_.stop_deg) > 1.0) {
      std::snprintf(b, sizeof(b), "차선 %+.2f° · 정지선 %+.2f°가 1° 넘게 달라 적용 안 함 (인식 확인)", yaw_.lane_deg, yaw_.stop_deg);
      yaw_.message = b;
      return;
    }
    const double tilt = 0.5 * (yaw_.lane_deg + yaw_.stop_deg);   // BEV가 왼쪽으로 기울어 보이는 각 (지금 yaw 적용 후)
    // BEV가 θ만큼 왼쪽(반시계)으로 돌아 보이면 실제 카메라는 지금 값보다 θ만큼 오른쪽(시계)을 보고 있는 것
    const double new_yaw = std::clamp(camera_yaw_deg_.load() - tilt, -20.0, 20.0);
    yaw_.applied_deg = new_yaw;
    std::snprintf(b, sizeof(b), "차선 %+.2f° · 정지선 %+.2f° → 카메라 yaw %+.2f° 적용 (이전 %+.2f°). 다시 재면 0° 근처여야 함",
                  yaw_.lane_deg, yaw_.stop_deg, new_yaw, camera_yaw_deg_.load());
    yaw_.message = b;
    yaw_.apply_pending = true;
  }

  std::string map_json() const {
    const auto & m = slam_->map();
    ss::JsonWriter w;
    w.begin_object();
    w.key("outer"); points_json(w, m.outer, 2);
    w.key("inner"); points_json(w, m.inner, 2);
    w.key("center"); points_json(w, m.center, 1);
    w.key("stop"); points_json(w, {m.stop_a, m.stop_b});
    w.key("start"); points_json(w, {m.start_a, m.start_b});
    for (const bool cw : {false, true}) {   // 출발 위치 (아래 직선 시작점)와 진행 방향 1 m 앞 점
      const ss::Vec2 a = m.straight_start(cw), d = m.travel_dir_at(a, cw);
      w.key(cw ? "start_cw" : "start_ccw"); points_json(w, {a, {a.x + d.x, a.y + d.y}});
    }
    w.key("lap_length").value(m.lap_length(), 3);
    w.key("lane_width").value(m.lane_width(), 3);
    w.end_object();
    return w.str();
  }

  std::string run_json() {   // 재접속·보정 후 다시 그리기: 이번 주행 궤적, 매핑 점, 보정 후 궤적
    std::lock_guard<std::mutex> l(run_m_);
    ss::JsonWriter w;
    w.begin_object();
    w.key("track").begin_array();
    for (const auto & p : track_) w.begin_array().value(p.x, 3).value(p.y, 3).end_array();
    w.end_array();
    w.key("after").begin_array();
    for (const auto & p : after_track_) w.begin_array().value(p.x, 3).value(p.y, 3).end_array();
    w.end_array();
    w.key("free").begin_array();
    for (std::size_t i = 0; i < free_track_.size(); i += 2) w.begin_array().value(free_track_[i].x, 3).value(free_track_[i].y, 3).end_array();
    w.end_array();
    w.key("points").begin_array();
    for (std::size_t i = 0; i < points_.size(); ++i) w.begin_array().value(points_[i].x, 3).value(points_[i].y, 3).value((long long)sides_[i]).end_array();
    w.end_array();
    w.end_object();
    return w.str();
  }

  void send_telemetry() {
    const double now = ss::now_sec();
    if (web_->client_count(now) == 0) return;
    const bool full = web_->preview_wanted(now);
    const auto snap = loop_->snapshot(now);
    const auto live = slam_->live();
    const auto calib = slam_->calib();
    if (now - last_stab_t_ >= 1.0) { camera_->stabilization_stats().summary(); last_stab_t_ = now; }
    ss::JsonWriter w;
    w.begin_object();
    w.key("t").value(now, 3).key("state").value(snap.state).key("reason").value(snap.reason).key("full").value(full);
    const auto & r = snap.readiness;
    w.key("readiness").begin_object().key("attitude_ok").value(r.attitude_ok).key("lane_model_ready").value(r.lane_model_ready)
     .key("vesc_connected").value(r.vesc_connected).key("erpm_fresh").value(r.erpm_fresh).key("lane_fresh").value(r.lane_fresh)
     .key("client_alive").value(r.client_alive).end_object();
    w.key("inspection").begin_object().key("attitude").begin_object().key("status").value(snap.attitude.status)
     .key("height_m").value(snap.attitude.height_m, 3).key("roll_deg").value(snap.attitude.roll_deg, 2)
     .key("pitch_down_deg").value(snap.attitude.pitch_down_deg, 2).key("message").value(snap.attitude.message).end_object()
     .key("model").begin_object().key("status").value(snap.model.status).key("stage").value(snap.model.stage)
     .key("message").value(snap.model.message).key("elapsed_s").value(snap.model.elapsed_s, 1).end_object()
     .key("stabilization").value(camera_->stabilization_stats().last()).key("stabilization_on").value(camera_->stabilization_on.load()).end_object();
    const auto & v = snap.vehicle;
    w.key("vehicle").begin_object().key("speed").value(v.speed, 3).key("target").value(v.target, 3).key("steer_deg").value(v.steer_deg, 2)
     .key("cte").value(v.cte, 3).key("duty").value(v.duty, 4).key("brake").value(v.brake, 2).key("phase").value(v.phase)
     .key("lon_mode").value((long long)v.lon_mode).end_object();
    w.key("run").begin_object().key("running").value(live.running).key("finished").value(live.finished).key("lap").value((long long)live.lap)
     .key("laps").value((long long)live.laps_target).key("target_speed").value(live.target_speed, 2).key("lap_dist").value(live.lap_dist, 3)
     .key("lap_length").value(live.lap_length, 3).key("bias0").value(live.bias0, 5)
     .key("waiting_s").value(live.waiting_s).key("cw").value(live.cw).key("lane_width").value(live.lane_width, 3).key("stop_seen").value(live.stop_seen_m, 3).key("stop_near").value(live.stop_near_s)
     .key("rejected_far").value((long long)live.rejected_far).key("rejected_lap").value((long long)live.rejected_lap)
     .key("pose").begin_array().value(live.pose.x, 3).value(live.pose.y, 3).value(live.pose.yaw, 4).end_array()
     .key("free_pose").begin_array().value(live.free_pose.x, 3).value(live.free_pose.y, 3).value(live.free_pose.yaw, 4).end_array()
     .key("s_arc").value(live.s_arc, 3).key("lat").value(live.lat_valid ? live.lat : std::nan(""), 3);
    {
      std::lock_guard<std::mutex> l(run_m_);
      w.key("status").value(status_).key("dir").value(std::filesystem::path(run_dir_).filename().string()).key("finishing").value(finishing_.load());
    }
    w.end_object();
    w.key("laps").begin_array();
    for (const auto & m : slam_->lap_metrics())
      w.begin_object().key("lap").value((long long)m.lap).key("dist").value(m.dist_m, 3).key("s_err").value(m.s_err_m, 4)
       .key("turn").value(m.turn_deg, 2).key("lat").value(m.lat_mean_m, 3).key("free_along").value(m.along_err_m, 4)
       .key("free_heading").value(m.heading_err_deg, 2).key("rms").value(m.rms_m, 4).end_object();
    w.end_array();
    w.key("corrections").begin_array();
    for (const auto & [lap, sh] : slam_->corrections()) w.begin_array().value((long long)lap).value(sh, 4).end_array();
    w.end_array();
    w.key("calib").begin_object().key("gyro_scale").value(calib.gyro_scale, 5).key("gyro_laps").value((long long)calib.gyro_laps)
     .key("latency").value(calib.latency, 3).key("camera_yaw").value(camera_yaw_deg_.load(), 2).key("lap_ref").value(slam_->map().center_length(), 3)
     .key("wheel").begin_array();
    for (const auto & [k, sc] : calib.wheel_scale) {
      const auto it = calib.wheel_laps.find(k);
      w.begin_array().value(k / 100.0, 2).value(sc, 5).value((long long)(it == calib.wheel_laps.end() ? 0 : it->second)).end_array();
    }
    w.end_array().end_object();
    {
      std::lock_guard<std::mutex> l(run_m_);
      if (has_result_) {
        const auto & e = last_result_.est;
        w.key("result").begin_object().key("applied").value(last_result_.applied).key("message").value(last_result_.message)
         .key("laps").value((long long)e.laps).key("gyro_est").value(e.gyro_scale, 5).key("wheel_est").value(e.wheel_scale, 5)
         .key("turn_per_lap").value(e.raw_turn_deg_per_lap, 2).key("dist_per_lap").value(e.raw_dist_per_lap, 3)
         .key("yaw_residual").value(e.bev_yaw_residual_deg, 2).key("straight_angle").value(e.straight_angle_deg, 2)
         .key("straight_slip").value(e.straight_slip_deg, 2).key("straights").value((long long)e.straights)
         .key("rms_after").value(last_result_.after.rms_m, 4).end_object();
      }
    }
    {
      bool apply = false;
      double yaw_new = 0;
      {
        std::lock_guard<std::mutex> l(yaw_m_);
        w.key("yaw_measure").begin_object().key("active").value(yaw_.active).key("message").value(yaw_.message)
         .key("lane").value(yaw_.lane_deg, 2).key("stop").value(yaw_.stop_deg, 2).end_object();
        if (yaw_.apply_pending) { apply = true; yaw_new = yaw_.applied_deg; yaw_.apply_pending = false; }
      }
      if (apply) {   // 차선 스레드가 아니라 여기서 적용 (BEV 변환을 다시 만듦)
        registry_.apply({{"camera.yaw_deg", yaw_new}});
        apply_settings();
      }
    }
    w.key("history").begin_array();
    for (const auto & h : store_->history(8))
      w.begin_object().key("time").value(h.time).key("action").value(h.action).key("speed").value(h.speed, 2).key("laps").value((long long)h.laps)
       .key("gyro").value(h.gyro_est, 5).key("wheel").value(h.wheel_est, 5).key("yaw_res").value(h.yaw_residual, 2)
       .key("before").value(h.rms_before, 4).key("after").value(h.rms_after, 4).end_object();
    w.end_array();
    {   // 새 궤적·매핑 점 (보내고 비움)
      std::lock_guard<std::mutex> l(run_m_);
      w.key("new_track").begin_array();
      for (const auto & p : pending_track_) w.begin_array().value(p.x, 3).value(p.y, 3).end_array();
      w.end_array();
      w.key("new_free").begin_array();
      for (const auto & p : pending_free_) w.begin_array().value(p.x, 3).value(p.y, 3).end_array();
      w.end_array();
      w.key("new_points").begin_array();
      for (std::size_t i = 0; i < pending_points_.size(); ++i) w.begin_array().value(pending_points_[i].x, 3).value(pending_points_[i].y, 3).value((long long)pending_sides_[i]).end_array();
      w.end_array();
      pending_track_.clear(); pending_points_.clear(); pending_sides_.clear(); pending_free_.clear();
    }
    if (full) {
      w.key("lane").begin_object();
      w.key("left"); points_json(w, snap.lane.left, 2);
      w.key("right"); points_json(w, snap.lane.right, 2);
      w.key("path"); points_json(w, snap.lane.path, 2);
      w.key("stop").value(snap.lane.stop_line_m, 3);
      w.key("stop_angle").value(snap.lane.stop_line_angle * 180.0 / M_PI, 2);
      w.end_object();
      w.key("stages").begin_array();
      for (const auto & row : loop_->stats().snapshot(ss::now_sec()))
        w.begin_object().key("name").value(row.stage).key("ms").value(row.avg_ms, 2).key("fps").value(row.fps, 1).end_object();
      w.end_array();
      if (now - last_usage_t_ >= 1.0) { usage_ = sysmon_.sample(now); last_usage_t_ = now; }
      w.key("usage").begin_object().key("cpu").value(usage_.cpu_percent, 1).key("proc_cpu").value(usage_.process_cpu_percent, 1)
       .key("mem").value(usage_.mem_percent, 1).key("gpu").value(usage_.gpu_percent.value_or(std::nan("")), 1).end_object();
      w.key("params_version").value((long long)registry_.version());
    }
    w.end_object();
    web_->publish_telemetry(w.str());
  }

  ss::ParamRegistry registry_;
  struct Attitude { bool ok{false}; double height{0}, roll{0}, pitch{0}; };
  std::mutex geom_m_;
  Attitude att_;
  struct YawMeasure {
    bool active{false}, apply_pending{false};
    double t0{0}, lane_sum{0}, stop_sum{0}, applied_deg{0};
    int lane_n{0}, stop_n{0};
    double lane_deg{std::nan("")}, stop_deg{std::nan("")};
    std::string message;
  };
  std::mutex yaw_m_;
  YawMeasure yaw_;
  ss::RuntimeConfig rc_;
  ss::CameraConfig cam_cfg_;
  ss::OakStartupMeasurementConfig measure_cfg_;
  ss::ModelConfig models_;
  double camera_x_{-0.09}, camera_y_{0}, front_overhang_{0.05};
  std::atomic<double> camera_yaw_deg_{0};   // 차선 스레드(yaw 측정)도 읽는다
  std::string params_save_path_, data_dir_;
  std::unique_ptr<ss::SlamCore> slam_;
  std::unique_ptr<ss::CalibStore> store_;
  std::unique_ptr<ss::DriveLoop> loop_;
  std::unique_ptr<ss::WebServer> web_;
  std::unique_ptr<ss::CameraPipeline> camera_;
  std::unique_ptr<ss::LaneWorker> lane_;
  ss::SystemMonitor sysmon_;
  ss::SystemUsage usage_;
  double last_usage_t_{0}, last_stab_t_{0};
  std::atomic<bool> stopping_{false}, finishing_{false};
  std::thread inspect_thread_, finish_thread_;
  // 이번 주행 (run_m_ 보호)
  std::mutex run_m_;
  std::string run_dir_, status_{"대기"};
  ss::OdomCalib calib_used_;
  std::size_t track_cursor_{0}, free_cursor_{0};
  std::vector<ss::Pose> track_, pending_track_, after_track_, free_track_, pending_free_;
  std::vector<ss::Vec2> points_, pending_points_;
  std::vector<int> sides_, pending_sides_;
  ss::CalibResult last_result_;
  bool has_result_{false};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_pub_, brake_pub_, servo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr erpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_, collector_, telemetry_;
  OnSetParametersCallbackHandle::SharedPtr on_set_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
