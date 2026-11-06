#include "roi_vfh_test/planner.hpp"
#include "roi_vfh_test/motion.hpp"
#include "roi_vfh_test/render.hpp"
#include "roi_vfh_test/drive.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <line_detactor/msg/lane_result.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <deque>
#include <memory>
#include <cstring>
namespace roi_vfh_test {
class Node:public rclcpp::Node {
  Config config_; Planner planner_; Plan result_;
  bool drive_enabled_{false},vesc_connected_{false},servo_commanded_{false};
  double drive_max_age_{.20};
  std::unique_ptr<DriveController> drive_;
  DriveCommand drive_command_;
  std::chrono::steady_clock::time_point last_drive_tick_{std::chrono::steady_clock::now()};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_pub_,brake_pub_,servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_log_pub_,brake_log_pub_,servo_log_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connection_sub_;
  MotionHistory motion_;
  std::uint64_t motion_generation_{0};
  double measured_yaw_rate_{0},measured_speed_{0},speed_at_{-1e9},imu_at_{-1e9};
  bool motion_ok_{false};
  MotionPose previous_pose_;bool previous_pose_valid_{false};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  bool gui_{true},requested_{false},locked_{false},window_{false};
  bool visible_supported_{false},autosize_supported_{false};
  double max_age_{.30},max_sync_{.10},plan_ms_{0};
  std::deque<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids_;
  line_detactor::msg::LaneResult::ConstSharedPtr lane_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr last_grid_;
  std::vector<Point> lane_points_; Grid display_;
  std::string status_{"STOPPED: configure ROI, then Start"};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<line_detactor::msg::LaneResult>::SharedPtr lane_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr running_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_,steer_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  static double stamp(const builtin_interfaces::msg::Time & t) {return t.sec+t.nanosec*1e-9;}
  bool fresh(const builtin_interfaces::msg::Time & t) {const double age=now().seconds()-stamp(t);return age>=-.02 && age<=max_age_;}
  void publishLock(bool value) {std_msgs::msg::Bool m;m.data=value;running_pub_->publish(m);}
  void stop(const std::string& reason) {
    requested_=false;locked_=false;planner_.reset();previous_pose_valid_=false;result_=Plan{};last_grid_.reset();status_=reason;publishLock(false);applyDrive(false);
  }
  void start() {
    requested_=true;locked_=false;planner_.reset();previous_pose_valid_=false;result_=Plan{};last_grid_.reset();
    status_="WAIT: ROI parameter lock acknowledgement";publishLock(true);
  }
  static void mouse(int event,int x,int y,int,void * ptr) {
    if (event!=cv::EVENT_LBUTTONDOWN) return;
    auto & self=*static_cast<Node*>(ptr);
    if (startButton().contains(cv::Point(x,y)) && !self.requested_) self.start();
    if (stopButton().contains(cv::Point(x,y))) {self.stop("STOPPED: parameters unlocked");self.publish();}
  }
  bool inputs() {
    if (!lane_ || !fresh(lane_->header.stamp) ||
      (!planner_.guidance() && (!lane_->centerline_valid || lane_->centerline_points.size()<2)) || lane_->centerline_points.size()>1000) {status_="STOP: missing/stale lane";return false;}
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr match;
    double delta=max_sync_;
    for (const auto & g:grids_) {
      const double d=std::abs(stamp(g->header.stamp)-stamp(lane_->header.stamp));
      if (fresh(g->header.stamp) && d<=delta) {match=g;delta=d;}
    }
    if (!match) {status_="STOP: no synchronized fresh terrain";return false;}
    if (match->header.frame_id!="front_axle_bev" || lane_->header.frame_id!=match->header.frame_id ||
      std::abs(match->info.origin.orientation.x)>1e-6 || std::abs(match->info.origin.orientation.y)>1e-6 ||
      std::abs(match->info.origin.orientation.z)>1e-6 || std::abs(match->info.origin.orientation.w-1)>1e-6) {
      status_="STOP: coordinate/frame mismatch";return false;
    }
    if (!lane_->source_width || !lane_->source_height || lane_->source_width>4096 || lane_->source_height>4096 || !std::isfinite(lane_->centerline_bev_width_m) ||
      !std::isfinite(lane_->centerline_bev_height_m) || lane_->centerline_bev_width_m<=0 || lane_->centerline_bev_height_m<=0) {
      status_="STOP: invalid lane geometry";return false;
    }
    Grid grid;grid.width=match->info.width;grid.height=match->info.height;grid.resolution=match->info.resolution;
    grid.x0=match->info.origin.position.x;grid.y0=match->info.origin.position.y;grid.cells=match->data;
    grid.validate();
    std::vector<Point> points;
    if (lane_->centerline_valid) for (const auto & p:lane_->centerline_points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x<0 || p.x>=double(lane_->source_width)+lane_->padding_left+lane_->padding_right || p.y<0 || p.y>=lane_->source_height) {
        status_="STOP: invalid lane point";return false;
      }
      Point q{lane_->centerline_bev_height_m-(p.y+.5)*lane_->centerline_bev_height_m/lane_->source_height,
        lane_->centerline_bev_width_m/2-(p.x-lane_->padding_left+.5)*lane_->centerline_bev_width_m/lane_->source_width};
      if (!points.empty() && std::hypot(q.x-points.back().x,q.y-points.back().y)>.20) break;
      points.push_back(q);
    }
    if (points.size()<2 && !planner_.guidance()) {status_="STOP: disconnected lane";return false;}
    lane_points_=std::move(points);display_=std::move(grid);
    if(requested_ && locked_ && now().seconds()-speed_at_>.15){status_="STOP: missing/stale measured speed (check VESC ERPM topic/bridge)";return false;}
    if(requested_ && locked_ && now().seconds()-imu_at_>.15){status_="STOP: missing/stale vehicle-frame IMU (/roi_lidar/imu)";return false;}
    MotionPose pose;
    if(requested_ && locked_) {
      MotionPose lane_pose;
      if(!motion_ok_ || !motion_.at(stamp(match->header.stamp),pose) || !motion_.at(stamp(lane_->header.stamp),lane_pose)) {
        status_="STOP: motion history cannot align sensor captures";return false;
      }
      for(auto& point:lane_points_)point=reproject(point,lane_pose,pose);
    }
    if (requested_ && locked_ && match!=last_grid_) {
      if(!motion_ok_ || now().seconds()-speed_at_>.15 || !motion_.at(stamp(match->header.stamp),pose)) {
        status_="STOP: fresh VESC speed + IMU required";return false;
      }
      if(motion_generation_!=motion_.generation) {previous_pose_valid_=false;motion_generation_=motion_.generation;}
      if(previous_pose_valid_)planner_.rotateReference(wrap(pose.yaw-previous_pose_.yaw));
      previous_pose_=pose;previous_pose_valid_=true;
      const auto begin=std::chrono::steady_clock::now();
      auto working=config_;
      working.delay=std::max(config_.delay,std::max(0.0,now().seconds()-stamp(match->header.stamp))+.067);
      working.current_steer=drive_enabled_ && servo_commanded_ ? drive_command_.steer :
        std::clamp(std::atan(config_.wheelbase*measured_yaw_rate_/std::max(.1,std::abs(measured_speed_))),-config_.max_steer,config_.max_steer);
      result_=planner_.plan(display_,lane_points_,working);
      plan_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      last_grid_=match;status_=result_.status;
    }
    return true;
  }
  static void publishFloat(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& publisher,double value) {
    if(!publisher)return;
    std_msgs::msg::Float32 msg;msg.data=static_cast<float>(value);publisher->publish(msg);
  }
  void applyDrive(bool active) {
    if(!drive_enabled_ || !drive_)return;
    const auto clock=std::chrono::steady_clock::now();
    const double dt=std::chrono::duration<double>(clock-last_drive_tick_).count();last_drive_tick_=clock;
    drive_command_=drive_->update(active,result_.speed,measured_speed_,result_.steer,dt);
    // Publish one motor mode per cycle: duty and brake messages replace each
    // other in the bridge. Zero propulsion during braking is recorded below.
    if(drive_command_.brake_mode)publishFloat(brake_pub_,drive_command_.brake);
    else publishFloat(duty_pub_,drive_command_.duty);
    if(active && result_.speed>0)servo_commanded_=true;
    if(servo_commanded_)publishFloat(servo_pub_,drive_command_.servo);
    publishFloat(duty_log_pub_,drive_command_.duty);
    publishFloat(brake_log_pub_,drive_command_.brake);
    publishFloat(servo_log_pub_,drive_command_.servo);
  }
  std::string driveGuard() {
    if(!vesc_connected_)return "STOP: VESC not connected";
    if(count_publishers(duty_pub_->get_topic_name())>1 || count_publishers(brake_pub_->get_topic_name())>1 ||
      count_publishers(servo_pub_->get_topic_name())>1)return "STOP: another actuator publisher (stop auto/manual controller)";
    if(locked_) {
      const double clock=now().seconds();
      if(clock-speed_at_>.15 || clock<speed_at_-.02 || !std::isfinite(measured_speed_))
        return "STOP: drive speed telemetry expired";
      if(clock-imu_at_>.15 || clock<imu_at_-.02 || !motion_ok_)
        return "STOP: drive IMU/motion history expired";
      if(!last_grid_)return "STOP: no drive plan input";
      const double age=now().seconds()-stamp(last_grid_->header.stamp);
      if(age<-.02 || age>drive_max_age_)return "STOP: drive path capture expired";
    }
    return {};
  }
  void publish() {
    if(drive_enabled_ && requested_) {
      const auto reason=driveGuard();if(!reason.empty())stop(reason);
    }
    nav_msgs::msg::Path path;path.header.stamp=now();path.header.frame_id="front_axle_bev";
    const bool active=requested_ && locked_;
    applyDrive(active);
    if(active && last_grid_)path.header=last_grid_->header;
    if (active) for (const auto & p:result_.path) {
      geometry_msgs::msg::PoseStamped pose;pose.header=path.header;pose.pose.position.x=p.x;pose.pose.position.y=p.y;
      pose.pose.orientation.z=std::sin(p.yaw/2);pose.pose.orientation.w=std::cos(p.yaw/2);path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    std_msgs::msg::Float32 speed,steer;speed.data=active?result_.speed:0;steer.data=active?result_.steer:0;
    speed_pub_->publish(speed);steer_pub_->publish(steer);
    std_msgs::msg::String message;
    message.data=status_+cv::format(" | target v=%.2fm/s steer=%.1fdeg plan=%.2fms candidates=%zu | rear unverified",
      speed.data,steer.data*180/pi,plan_ms_,result_.candidates.size());
    if(drive_enabled_)message.data+=cv::format(" | DRIVE duty=%.3f brake=%.2fA command_steer=%.1fdeg",drive_command_.duty,drive_command_.brake,drive_command_.steer*180/pi);
    status_pub_->publish(message);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"%s",message.data.c_str());
  }
  bool windowClosed() {
    // Older GTK HighGUI returns -1 for unsupported VISIBLE, even for an open window.
    // AUTOSIZE is available on GTK. Observe support before treating a missing
    // property/window as closure, so unsupported queries never terminate startup.
    try {
      const double visible=cv::getWindowProperty("ROI VFH+ test",cv::WND_PROP_VISIBLE);
      if (visible>0) {visible_supported_=true;return false;}
      if (visible==0 && visible_supported_) return true;
      const double autosize=cv::getWindowProperty("ROI VFH+ test",cv::WND_PROP_AUTOSIZE);
      if (autosize>=0) {autosize_supported_=true;return false;}
      return visible_supported_ || autosize_supported_;
    } catch (const cv::Exception&) {
      // Some backends throw after the last native window has been destroyed.
      return visible_supported_ || autosize_supported_;
    }
  }
  void draw() {
    if(!gui_ && !image_pub_->get_subscription_count())return;
    const auto canvas=renderPreview(display_,lane_points_,result_,config_,requested_,locked_,status_,plan_ms_,drive_enabled_,drive_command_.duty,drive_command_.brake);
    if (image_pub_->get_subscription_count()) {
      sensor_msgs::msg::Image msg;msg.header.stamp=now();msg.header.frame_id="front_axle_bev";
      msg.width=canvas.cols;msg.height=canvas.rows;msg.encoding="bgr8";msg.step=canvas.cols*3;
      msg.data.assign(canvas.data,canvas.data+canvas.total()*3);image_pub_->publish(msg);
    }
    if (gui_) {
      if (!window_) {cv::namedWindow("ROI VFH+ test",cv::WINDOW_AUTOSIZE);cv::setMouseCallback("ROI VFH+ test",mouse,this);window_=true;}
      cv::imshow("ROI VFH+ test",canvas);
      const int key=cv::waitKey(1);if (key==' ' || key=='s'){stop("STOPPED: parameters unlocked");publish();}
      if (key=='q' || key==27 || windowClosed()) {
        stop("STOPPED: preview closed");publish();
        RCLCPP_INFO(get_logger(),"VFH window closed: stopping and unlocking ROI parameters");
        rclcpp::shutdown();
      }
    }
  }
  void tick() {
    try {
      const bool ok=inputs();
      if (requested_ && !locked_)publishLock(true);
      if (!ok && requested_ && locked_) {const auto reason=status_;stop(reason);}
      if (!requested_)result_=Plan{};
      publish();
    } catch (const std::exception& e) {stop(std::string("STOP: ")+e.what());publish();}
    try {draw();}catch(const cv::Exception& e){stop("STOP: preview GUI error");publish();gui_=false;RCLCPP_ERROR(get_logger(),"%s",e.what());}
  }
public:
  Node():rclcpp::Node("roi_vfh_test") {
    rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only=true;
    gui_=declare_parameter("gui",true,fixed);
    drive_enabled_=declare_parameter("drive_enabled",false,fixed);
#define PARAM(name,field) config_.field=declare_parameter(name,config_.field,fixed)
    PARAM("nominal_speed_mps",speed);PARAM("horizon_m",horizon);PARAM("sample_step_m",step);
    PARAM("wheelbase_m",wheelbase);PARAM("vehicle_width_m",width);PARAM("rear_extent_m",rear);PARAM("front_overhang_m",front);
    PARAM("safety_margin_m",margin);PARAM("lane_width_m",lane_width);PARAM("lane_departure_m",departure);
    PARAM("lane_lookahead_m",lane_lookahead);
    PARAM("deceleration_mps2",deceleration);PARAM("response_delay_sec",delay);PARAM("reference_distance_m",reference_distance);
#undef PARAM
    config_.max_steer=declare_parameter("max_steering_deg",30.,fixed)*pi/180;
    config_.steer_rate=declare_parameter("steering_rate_deg_per_sec",240.,fixed)*pi/180;
    config_.hist_low=declare_parameter("histogram_low",1.,fixed);config_.hist_high=declare_parameter("histogram_high",2.,fixed);
    config_.validate();motion_.front_axle_offset=config_.wheelbase;
    max_age_=declare_parameter("max_age_sec",.30,fixed);max_sync_=declare_parameter("max_sync_sec",.10,fixed);
    if(!std::isfinite(max_age_) || !std::isfinite(max_sync_) || max_age_<.05 || max_age_>.5 || max_sync_<.001 || max_sync_>.15)throw std::invalid_argument("invalid synchronization limits");
    DriveOptions drive_options;drive_options.max_steer=config_.max_steer;
#define DRIVE_PARAM(name,field) drive_options.field=declare_parameter("drive." name,drive_options.field,fixed)
    DRIVE_PARAM("maximum_duty",maximum_duty);DRIVE_PARAM("longitudinal_start_duty",start_duty);
    DRIVE_PARAM("duty_rise_rate_per_sec",rise);DRIVE_PARAM("duty_fall_rate_per_sec",fall);
    DRIVE_PARAM("longitudinal_pid_kp",kp);DRIVE_PARAM("longitudinal_pid_ki",ki);
    DRIVE_PARAM("longitudinal_pid_kd",kd);DRIVE_PARAM("longitudinal_pid_integral_limit",integral_limit);
    DRIVE_PARAM("longitudinal_feedforward_offset_duty",ff_offset);
    DRIVE_PARAM("longitudinal_feedforward_duty_per_mps",ff_slope);
    DRIVE_PARAM("longitudinal_feedforward_fade_speed_mps",ff_fade_speed);
    DRIVE_PARAM("brake_maximum_current_amps",brake_max);DRIVE_PARAM("longitudinal_brake_speed_gain",brake_speed_gain);
    DRIVE_PARAM("traffic_brake_speed_hysteresis_mps",brake_hysteresis);
    DRIVE_PARAM("brake_current_rise_amps_per_sec",brake_rise);DRIVE_PARAM("brake_current_fall_amps_per_sec",brake_fall);
    DRIVE_PARAM("servo_left",servo_left);DRIVE_PARAM("servo_center",servo_center);DRIVE_PARAM("servo_right",servo_right);
    DRIVE_PARAM("steering_servo_inverted",inverted);
#undef DRIVE_PARAM
    drive_=std::make_unique<DriveController>(drive_options);
    drive_max_age_=declare_parameter("drive.max_command_age_sec",.20,fixed);
    if(!std::isfinite(drive_max_age_) || drive_max_age_<.05 || drive_max_age_>.3)throw std::invalid_argument("invalid drive command age");
    const auto sensor=rclcpp::SensorDataQoS().keep_last(1);
    const auto latch=rclcpp::QoS(1).reliable().transient_local();
    const auto duty_topic=declare_parameter<std::string>("duty_topic","/vesc/duty",fixed);
    const auto brake_topic=declare_parameter<std::string>("brake_current_topic","/vesc/brake_current",fixed);
    const auto servo_topic=declare_parameter<std::string>("servo_position_topic","/vesc/servo_position",fixed);
    const auto connection_topic=declare_parameter<std::string>("vesc_connection_topic","/vesc/connected",fixed);
    if(drive_enabled_) {
      const auto command=rclcpp::QoS(1).reliable().durability_volatile();
      duty_pub_=create_publisher<std_msgs::msg::Float32>(duty_topic,command);
      brake_pub_=create_publisher<std_msgs::msg::Float32>(brake_topic,command);
      servo_pub_=create_publisher<std_msgs::msg::Float32>(servo_topic,command);
      duty_log_pub_=create_publisher<std_msgs::msg::Float32>("~/command_duty",1);
      brake_log_pub_=create_publisher<std_msgs::msg::Float32>("~/command_brake_current",1);
      servo_log_pub_=create_publisher<std_msgs::msg::Float32>("~/command_servo_position",1);
      connection_sub_=create_subscription<std_msgs::msg::Bool>(connection_topic,latch,
        [this](std_msgs::msg::Bool::ConstSharedPtr p){
          vesc_connected_=p->data;
          if(!vesc_connected_ && requested_){stop("STOP: VESC disconnected");publish();}
        });
      RCLCPP_INFO(get_logger(),"DRIVE output after Start: duty=%s brake=%s servo=%s connection=%s",duty_topic.c_str(),brake_topic.c_str(),servo_topic.c_str(),connection_topic.c_str());
    }
    running_pub_=create_publisher<std_msgs::msg::Bool>("~/running",latch);
    path_pub_=create_publisher<nav_msgs::msg::Path>("~/path",1);
    speed_pub_=create_publisher<std_msgs::msg::Float32>("~/target_speed",1);
    steer_pub_=create_publisher<std_msgs::msg::Float32>("~/steering_angle_rad",1);
    status_pub_=create_publisher<std_msgs::msg::String>("~/status",1);
    image_pub_=create_publisher<sensor_msgs::msg::Image>("~/preview",sensor);
    grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/roi_lidar/terrain_for_planning",sensor,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr p){grids_.push_back(p);while(grids_.size()>16)grids_.pop_front();});
    const auto speed_topic=declare_parameter<std::string>("measured_speed_topic","/vehicle/dynamics/speed_mps",fixed);
    speed_sub_=create_subscription<std_msgs::msg::Float32>(speed_topic,sensor,
      [this](std_msgs::msg::Float32::ConstSharedPtr p){
        if(!std::isfinite(p->data)){speed_at_=-1e9;motion_.clear();motion_ok_=false;return;}
        measured_speed_=p->data;speed_at_=now().seconds();
      });
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("/roi_lidar/imu",sensor,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr p){
        if(p->header.frame_id!="front_axle_bev" || !fresh(p->header.stamp) || !std::isfinite(p->angular_velocity.z)) {
          motion_.clear();motion_ok_=false;return;
        }
        imu_at_=stamp(p->header.stamp);
        measured_yaw_rate_=p->angular_velocity.z;
        if(now().seconds()-speed_at_>.15){motion_.clear();motion_ok_=false;return;}
        motion_.add(stamp(p->header.stamp),measured_speed_,measured_yaw_rate_);motion_ok_=true;
      });
    lane_sub_=create_subscription<line_detactor::msg::LaneResult>("/line_detactor/result",sensor,
      [this](line_detactor::msg::LaneResult::ConstSharedPtr p){lane_=p;});
    lock_sub_=create_subscription<std_msgs::msg::Bool>("/roi_lidar/parameters_locked",latch,
      [this](std_msgs::msg::Bool::ConstSharedPtr p){
        if(requested_ && locked_ && !p->data){stop("STOP: ROI parameter lock lost");publish();return;}
        locked_=requested_ && p->data;
      });
    RCLCPP_INFO(get_logger(),"VFH telemetry: speed=%s (Float32 m/s), IMU=/roi_lidar/imu (vehicle-frame Z rad/s)",speed_sub_->get_topic_name());
    publishLock(false);
    timer_=create_wall_timer(std::chrono::milliseconds(67),[this]{tick();});
  }
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<roi_vfh_test::Node>());if(rclcpp::ok())rclcpp::shutdown();}
