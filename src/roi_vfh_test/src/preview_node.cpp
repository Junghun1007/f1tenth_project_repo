#include "roi_vfh_test/planner.hpp"
#include "roi_vfh_test/motion.hpp"
#include "roi_vfh_test/render.hpp"
#include "roi_vfh_test/drive.hpp"
#include "roi_vfh_test/controls.hpp"
#include <auto_control/driving_log.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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
#include <optional>
#include <cstring>
namespace roi_vfh_test {
class Node:public rclcpp::Node {
  Config config_; Planner planner_; Plan result_;
  PlannerControls controls_;
  double planner_deceleration_ceiling_{.5},speed_ceiling_{1.8};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr planner_parameters_callback_;
  bool drive_enabled_{false},vesc_connected_{false},servo_commanded_{false};
  double drive_max_age_{.30};
  std::unique_ptr<DriveController> drive_;
  DriveCommand drive_command_;
  std::chrono::steady_clock::time_point last_drive_tick_{std::chrono::steady_clock::now()};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_pub_,brake_pub_,servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_log_pub_,brake_log_pub_,servo_log_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connection_sub_;
  MotionHistory motion_;
  std::uint64_t motion_generation_{0};
  double imu_received_at_{-1e9};
  std::string imu_issue_{"no messages received"};
  double measured_yaw_rate_{0},measured_speed_{0},speed_at_{-1e9},imu_at_{-1e9};
  bool motion_ok_{false};
  MotionPose previous_pose_;bool previous_pose_valid_{false};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  bool gui_{true},requested_{false},locked_{false},window_{false};
  bool visible_supported_{false},autosize_supported_{false};
  bool suspended_{false},input_recoverable_{true},waiting_for_pair_{false};
  double resume_stable_sec_{.5};
  std::optional<std::chrono::steady_clock::time_point> healthy_since_,last_recovery_check_;
  double planned_lane_stamp_{-1e9};
  line_detactor::msg::LaneResult::ConstSharedPtr planned_lane_;
  double max_age_{.30},max_sync_{.06},plan_ms_{0};
  double lane_receipt_timeout_{.30},erpm_timeout_{.15},imu_timeout_{.10},lane_received_at_{-1e9};
  double planning_rate_{20},status_rate_{2};
  int path_minimum_points_{8},max_lane_points_{2000};
  double path_minimum_x_{.01},path_maximum_x_{2.98},path_maximum_gap_{.15};
  std::string frame_id_{"front_axle_bev"},lane_topic_{"/line_detactor/result"};
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
  rclcpp::TimerBase::SharedPtr timer_,actuator_timer_;
  std::unique_ptr<auto_control::DrivingLog> driving_log_;
  double driving_log_rate_{30},last_log_at_{-1e9},last_avoidance_at_{-1e9},last_target_{0},last_plan_at_{-1e9};
  double obstacle_speed_cap_{1},obstacle_clear_sec_{1},recovery_acceleration_{.8};
  bool obstacle_slowdown_{true};
  std::string last_log_state_;
  std::chrono::steady_clock::time_point log_started_{std::chrono::steady_clock::now()};
  static double stamp(const builtin_interfaces::msg::Time & t) {return t.sec+t.nanosec*1e-9;}
  bool fresh(const builtin_interfaces::msg::Time & t) {const double age=now().seconds()-stamp(t);return age>=-.02 && age<=max_age_;}
  void publishLock(bool value) {std_msgs::msg::Bool m;m.data=value;running_pub_->publish(m);}
  void stop(const std::string& reason) {
    controls_.blur();controls_.message("Stopped. ROI and VFH parameters unlocked.");
    requested_=false;locked_=false;suspended_=false;healthy_since_.reset();last_recovery_check_.reset();planned_lane_.reset();planner_.reset();previous_pose_valid_=false;result_=Plan{};last_grid_.reset();last_target_=0;last_plan_at_=-1e9;last_avoidance_at_=-1e9;status_=reason;publishLock(false);applyDrive(false);
  }
  void start() {
    if(!applyPlannerEdits())return;
    controls_.show(false);
    requested_=true;locked_=false;suspended_=false;healthy_since_.reset();last_recovery_check_.reset();planned_lane_.reset();planner_.reset();previous_pose_valid_=false;result_=Plan{};last_grid_.reset();last_target_=0;last_plan_at_=-1e9;last_avoidance_at_=-1e9;
    status_="WAIT: ROI parameter lock acknowledgement";publishLock(true);
  }
  bool active()const {return requested_ && locked_ && !suspended_;}
  void pauseForInput(const std::string& reason) {
    if(!requested_ || !locked_)return;
    if(!suspended_) {
      result_=Plan{};last_grid_.reset();planned_lane_.reset();planner_.reset();previous_pose_valid_=false;
      last_target_=0;last_plan_at_=-1e9;last_avoidance_at_=-1e9;
    }
    suspended_=true;healthy_since_.reset();last_recovery_check_.reset();
    status_="WAIT input: "+reason;
    controls_.message("Input wait: parameters locked. Stop cancels automatic restart.");
    applyDrive(false);
  }
  void updateRecovery(bool inputs_ok) {
    if(!suspended_ || !requested_ || !locked_)return;
    if(!inputs_ok || !last_grid_ || result_.speed<=0) {
      healthy_since_.reset();last_recovery_check_.reset();return;
    }
    const auto clock=std::chrono::steady_clock::now();
    if(last_recovery_check_ && std::chrono::duration<double>(clock-*last_recovery_check_).count()>std::max(.1,2/planning_rate_))
      healthy_since_.reset(); // A stalled event loop is not continuous healthy input.
    last_recovery_check_=clock;
    if(!healthy_since_)healthy_since_=clock;
    const double elapsed=std::chrono::duration<double>(clock-*healthy_since_).count();
    if(elapsed>=resume_stable_sec_) {
      suspended_=false;healthy_since_.reset();last_recovery_check_.reset();status_=result_.status;
      controls_.message("Inputs recovered. Driving resumed; parameters remain locked.");
    }else status_=cv::format("WAIT: inputs stable %.2f / %.2fs (braking)",elapsed,resume_stable_sec_);
  }
  static void mouse(int event,int x,int y,int,void * ptr) {
    if (event!=cv::EVENT_LBUTTONDOWN) return;
    auto & self=*static_cast<Node*>(ptr);
    if (stopButton().contains(cv::Point(x,y))) {self.stop("STOPPED: parameters unlocked");self.publish();return;}
    if (PlannerControls::toggleButton().contains(cv::Point(x,y))) {self.controls_.toggle();return;}
    if (startButton().contains(cv::Point(x,y)) && !self.requested_) {self.start();return;}
    self.controlAction(self.controls_.click(x,y,self.requested_));
  }
  Config plannerParameters()const {
    auto result=config_;
    for(const auto& field:plannerFields())result.*(field.member)=get_parameter(field.name).as_double();
    return result;
  }
  void syncPlannerParameters() {
    const auto next=plannerParameters();bool changed=false;
    for(const auto& field:plannerFields())if(next.*(field.member)!=config_.*(field.member))changed=true;
    if(!changed)return;
    validatePlannerEdits(next,planner_deceleration_ceiling_,speed_ceiling_);
    config_=next;planner_.reset();result_=Plan{};last_grid_.reset();previous_pose_valid_=false;
    controls_.sync(config_);
  }
  bool applyPlannerEdits() {
    if(requested_){controls_.message("Locked: press Stop before editing VFH parameters.");return false;}
    try {
      syncPlannerParameters();
      std::vector<rclcpp::Parameter> edits;
      for(const auto& change:controls_.changes())edits.emplace_back(change.first,change.second);
      if(!edits.empty()) {
        const auto result=set_parameters_atomically(edits);
        if(!result.successful)throw std::invalid_argument(result.reason);
      }
      syncPlannerParameters();controls_.reload(config_);controls_.message("Applied. Start uses these values.");return true;
    }catch(const std::exception& e) {
      controls_.show(true);controls_.message(std::string("REJECTED: ")+e.what());
      status_="STOPPED: invalid VFH parameters; correct highlighted edits";
      RCLCPP_WARN(get_logger(),"VFH settings rejected: %s",e.what());return false;
    }
  }
  void controlAction(PlannerControls::Action action) {
    if(requested_)return;
    if(action==PlannerControls::Action::Apply)applyPlannerEdits();
    if(action==PlannerControls::Action::Reload){controls_.reload(config_);controls_.message("Reloaded applied values. Draft edits discarded.");}
  }
  bool inputs() {
    input_recoverable_=true;waiting_for_pair_=false;
    if (!lane_ || now().seconds()-lane_received_at_>lane_receipt_timeout_ || !fresh(lane_->header.stamp)) {waiting_for_pair_=true;status_="STOP: missing/stale lane";return false;}
    if((!planner_.guidance() && (!lane_->centerline_valid || lane_->centerline_points.size()<2)) ||
      lane_->centerline_points.size()>static_cast<std::size_t>(max_lane_points_)) {status_="STOP: invalid/missing lane reference";return false;}
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr match;
    double delta=max_sync_;
    for (const auto & g:grids_) {
      const double d=std::abs(stamp(g->header.stamp)-stamp(lane_->header.stamp));
      if (fresh(g->header.stamp) && d<=delta) {match=g;delta=d;}
    }
    if (!match) {waiting_for_pair_=true;status_="STOP: no synchronized fresh terrain";return false;}
    if (match->header.frame_id!=frame_id_ || lane_->header.frame_id!=match->header.frame_id ||
      std::abs(match->info.origin.orientation.x)>1e-6 || std::abs(match->info.origin.orientation.y)>1e-6 ||
      std::abs(match->info.origin.orientation.z)>1e-6 || std::abs(match->info.origin.orientation.w-1)>1e-6) {
      input_recoverable_=false;status_="STOP: coordinate/frame mismatch";return false;
    }
    if (!lane_->source_width || !lane_->source_height || lane_->source_width>4096 || lane_->source_height>4096 || !std::isfinite(lane_->centerline_bev_width_m) ||
      !std::isfinite(lane_->centerline_bev_height_m) || lane_->centerline_bev_width_m<=0 || lane_->centerline_bev_height_m<=0) {
      input_recoverable_=false;status_="STOP: invalid lane geometry";return false;
    }
    Grid grid;grid.width=match->info.width;grid.height=match->info.height;grid.resolution=match->info.resolution;
    grid.x0=match->info.origin.position.x;grid.y0=match->info.origin.position.y;grid.cells=match->data;
    grid.validate();
    std::vector<Point> points;
    if (lane_->centerline_valid) for (const auto & p:lane_->centerline_points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x<0 || p.x>=double(lane_->source_width)+lane_->padding_left+lane_->padding_right || p.y<0 || p.y>=lane_->source_height) {
        input_recoverable_=false;status_="STOP: invalid lane point";return false;
      }
      Point q{lane_->centerline_bev_height_m-(p.y+.5)*lane_->centerline_bev_height_m/lane_->source_height,
        lane_->centerline_bev_width_m/2-(p.x-lane_->padding_left+.5)*lane_->centerline_bev_width_m/lane_->source_width};
      if (!points.empty() && std::hypot(q.x-points.back().x,q.y-points.back().y)>path_maximum_gap_) break;
      points.push_back(q);
    }
    if(points.size()>=2) {
      std::vector<auto_control::Point> input;input.reserve(points.size());
      for(auto p:points)input.push_back({p.x,p.y});
      const auto ordered=auto_control::build_ordered_path(input,path_minimum_points_,config_.minimum_path_length,
        path_minimum_x_,path_maximum_x_,path_maximum_gap_,config_.geometry_window);
      points.clear();if(ordered)for(auto p:ordered->points)points.push_back({p.x,p.y});
    }
    if (points.size()<2 && !planner_.guidance()) {status_="STOP: disconnected lane";return false;}
    lane_points_=std::move(points);display_=std::move(grid);
    if(requested_ && locked_ && now().seconds()-speed_at_>erpm_timeout_){status_="STOP: missing/stale measured speed (check VESC ERPM topic/bridge)";return false;}
    if(requested_ && locked_ && now().seconds()-imu_at_>imu_timeout_){status_="STOP: missing/stale vehicle-frame IMU (/roi_lidar/imu)";return false;}
    MotionPose pose;
    if(requested_ && locked_) {
      MotionPose lane_pose;
      if(!motion_ok_ || !motion_.at(stamp(match->header.stamp),pose) || !motion_.at(stamp(lane_->header.stamp),lane_pose)) {
        status_="STOP: motion history cannot align sensor captures";return false;
      }
      for(auto& point:lane_points_)point=reproject(point,lane_pose,pose);
    }
    if (requested_ && locked_ && (match!=last_grid_ || lane_!=planned_lane_)) {
      if(!motion_ok_ || now().seconds()-speed_at_>erpm_timeout_ || !motion_.at(stamp(match->header.stamp),pose)) {
        status_="STOP: fresh VESC speed + IMU required";return false;
      }
      if(motion_generation_!=motion_.generation) {previous_pose_valid_=false;motion_generation_=motion_.generation;}
      if(previous_pose_valid_)planner_.rotateReference(wrap(pose.yaw-previous_pose_.yaw));
      previous_pose_=pose;previous_pose_valid_=true;
      const auto begin=std::chrono::steady_clock::now();
      auto working=config_;
      working.delay=std::max(config_.delay,std::max(0.0,now().seconds()-stamp(match->header.stamp))+1.0/planning_rate_);
      working.current_speed=measured_speed_;
      working.current_steer=drive_enabled_ && servo_commanded_ ? drive_command_.steer :
        std::clamp(std::atan(config_.wheelbase*measured_yaw_rate_/std::max(.1,std::abs(measured_speed_))),-config_.max_steer,config_.max_steer);
      result_=planner_.plan(display_,lane_points_,working);
      const double planned_at=now().seconds();
      if(result_.reference_blocked || result_.guidance)last_avoidance_at_=planned_at;
      if(obstacle_slowdown_ && planned_at-last_avoidance_at_<=obstacle_clear_sec_)result_.speed=std::min(result_.speed,obstacle_speed_cap_);
      if(last_target_>0 && std::abs(measured_speed_)>=.05 && planned_at>last_plan_at_)
        result_.speed=std::min(result_.speed,last_target_+recovery_acceleration_*(planned_at-last_plan_at_));
      last_target_=result_.speed;last_plan_at_=planned_at;
      plan_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      last_grid_=match;planned_lane_=lane_;planned_lane_stamp_=stamp(lane_->header.stamp);status_=result_.status;
    }
    return true;
  }
  static void publishFloat(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& publisher,double value) {
    if(!publisher)return;
    std_msgs::msg::Float32 msg;msg.data=static_cast<float>(value);publisher->publish(msg);
  }
  void logDrive(bool active) {
    if(!driving_log_)return;
    const double clock=now().seconds();
    const std::string state=status_+(active?" active":" stopped")+(drive_command_.brake_mode?" brake":" duty");
    if(state==last_log_state_ && clock-last_log_at_<1/driving_log_rate_)return;
    last_log_state_=state;last_log_at_=clock;
    auto_control::DrivingLogFrame frame;frame.ros_ns=now().nanoseconds();
    frame.elapsed_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-log_started_).count();
    frame.labels={state};
    frame.values={active?result_.speed:0,measured_speed_,drive_command_.steer,drive_command_.duty,drive_command_.brake,
      requested_?1.:0.,locked_?1.:0.,result_.length,double(result_.candidates.size()),result_.used_vfh?1.:0.,result_.guidance?1.:0.};
    for(const auto& p:result_.path)frame.path.push_back({p.x,p.y});
    driving_log_->enqueue(std::move(frame));
  }
  void applyDrive(bool active) {
    if(!drive_enabled_ || !drive_)return;
    const auto clock=std::chrono::steady_clock::now();
    const double dt=std::chrono::duration<double>(clock-last_drive_tick_).count();last_drive_tick_=clock;
    drive_command_=drive_->update(active,result_.speed,measured_speed_,result_.steer,dt,result_.used_vfh || result_.guidance);
    // Publish one motor mode per cycle: duty and brake messages replace each
    // other in the bridge. Zero propulsion during braking is recorded below.
    if(drive_command_.brake_mode)publishFloat(brake_pub_,drive_command_.brake);
    else publishFloat(duty_pub_,drive_command_.duty);
    if(active && result_.speed>0)servo_commanded_=true;
    if(servo_commanded_)publishFloat(servo_pub_,drive_command_.servo);
    publishFloat(duty_log_pub_,drive_command_.duty);
    publishFloat(brake_log_pub_,drive_command_.brake);
    publishFloat(servo_log_pub_,drive_command_.servo);
    logDrive(active);
  }
  std::string driveGuard(bool* recoverable=nullptr) {
    if(recoverable)*recoverable=false;
    if(drive_enabled_ && !vesc_connected_)return "STOP: VESC not connected";
    if(drive_enabled_ && (count_publishers(duty_pub_->get_topic_name())>1 || count_publishers(brake_pub_->get_topic_name())>1 ||
      count_publishers(servo_pub_->get_topic_name())>1))return "STOP: another actuator publisher (stop auto/manual controller)";
    if(recoverable)*recoverable=true;
    if(locked_) {
      if(!lane_ || now().seconds()-lane_received_at_>lane_receipt_timeout_ || !fresh(lane_->header.stamp))return "STOP: drive lane input expired";
      const double clock=now().seconds();
      if(clock-speed_at_>erpm_timeout_ || clock<speed_at_-.02 || !std::isfinite(measured_speed_))
        return "STOP: drive speed telemetry expired";
      if(imu_received_at_<0)return "STOP: no IMU received on /roi_lidar/imu";
      if(clock-imu_received_at_>imu_timeout_)return cv::format("STOP: IMU receive timeout %.3fs",clock-imu_received_at_);
      if(clock-imu_at_>imu_timeout_ || clock<imu_at_-.02)
        return cv::format("STOP: IMU capture age %.3fs; %s",clock-imu_at_,imu_issue_.c_str());
      if(!motion_ok_)return "STOP: IMU motion history unavailable; "+imu_issue_;
      if(!last_grid_)return "STOP: no drive plan input";
      const double age=clock-stamp(last_grid_->header.stamp),lane_age=clock-planned_lane_stamp_;
      if(age<-.02 || age>drive_max_age_ || lane_age<-.02 || lane_age>drive_max_age_)
        return cv::format("STOP: capture expired terrain %.3fs lane %.3fs (limit %.2fs)",age,lane_age,drive_max_age_);
    }
    return {};
  }
  bool checkDriveGuard() {
    if(!requested_ || !locked_)return false;
    bool recoverable=false;const auto reason=driveGuard(&recoverable);
    if(reason.empty())return true;
    if(recoverable)pauseForInput(reason);else stop(reason);
    return false;
  }
  void publish() {
    checkDriveGuard();
    nav_msgs::msg::Path path;path.header.stamp=now();path.header.frame_id=frame_id_;
    const bool active=this->active();
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
    if(!drive_enabled_)logDrive(active);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),static_cast<int64_t>(1000/status_rate_),"%s",message.data.c_str());
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
    auto canvas=renderPreview(display_,lane_points_,result_,config_,requested_,locked_ && !suspended_,status_,plan_ms_,drive_enabled_,drive_command_.duty,drive_command_.brake);
    controls_.sync(config_);controls_.draw(canvas,requested_,planner_deceleration_ceiling_,speed_ceiling_);
    if (image_pub_->get_subscription_count()) {
      sensor_msgs::msg::Image msg;msg.header.stamp=now();msg.header.frame_id=frame_id_;
      msg.width=canvas.cols;msg.height=canvas.rows;msg.encoding="bgr8";msg.step=canvas.cols*3;
      msg.data.assign(canvas.data,canvas.data+canvas.total()*3);image_pub_->publish(msg);
    }
    if (gui_) {
      if (!window_) {cv::namedWindow("ROI VFH+ test",cv::WINDOW_AUTOSIZE);cv::setMouseCallback("ROI VFH+ test",mouse,this);window_=true;}
      cv::imshow("ROI VFH+ test",canvas);
      const int key=cv::waitKeyEx(1);const bool editing=controls_.editing();
      if (key==' ' || key=='s' || key=='S'){stop("STOPPED: parameters unlocked");publish();}
      else if (key=='q' || (key==27 && !editing) || windowClosed()) {
        stop("STOPPED: window closed");publish();
        RCLCPP_INFO(get_logger(),"VFH window closed: stopping and unlocking ROI parameters");
        rclcpp::shutdown();
      }else {
        if(key=='p' || key=='P')controls_.toggle();
        controlAction(controls_.key(key,requested_));
      }
    }
  }
  void tick() {
    try {
      syncPlannerParameters();
      const bool ok=inputs();
      if (requested_ && !locked_)publishLock(true);
      bool held=false;
      if(!ok && requested_ && locked_) {
        bool recoverable=false;
        const bool cached_ok=last_grid_ && driveGuard(&recoverable).empty();
        if(input_recoverable_ && waiting_for_pair_ && !suspended_ && cached_ok) {
          held=true;status_="HOLD: previous valid path; waiting for synchronized input";
        }else if(input_recoverable_)pauseForInput(status_);
        else {const auto reason=status_;stop(reason);}
      }
      const bool guarded=checkDriveGuard();
      updateRecovery(ok && guarded && !held);
      if (!requested_)result_=Plan{};
      publish();
    } catch (const std::exception& e) {stop(std::string("STOP: ")+e.what());publish();}
    try {draw();}catch(const cv::Exception& e){stop("STOP: preview GUI error");publish();gui_=false;RCLCPP_ERROR(get_logger(),"%s",e.what());}
  }
public:
  Node():rclcpp::Node("roi_vfh_test") {
    rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only=true;
    rcl_interfaces::msg::ParameterDescriptor editable;editable.description="Editable only before Start or after Stop";
    gui_=declare_parameter("gui",true,fixed);
    drive_enabled_=declare_parameter("drive_enabled",false,fixed);
#define PARAM(name,field) config_.field=declare_parameter(name,config_.field,plannerField(name)?editable:fixed)
    PARAM("nominal_speed_mps",speed);PARAM("horizon_m",horizon);PARAM("sample_step_m",step);
    PARAM("wheelbase_m",wheelbase);PARAM("vehicle_width_m",width);PARAM("rear_extent_m",rear);PARAM("front_overhang_m",front);
    PARAM("safety_margin_m",margin);PARAM("lane_width_m",lane_width);PARAM("lane_departure_m",departure);
    PARAM("lane_lookahead_m",lane_lookahead);
    PARAM("minimum_path_length_m",minimum_path_length);PARAM("obstacle_uncertainty_m",uncertainty);
    PARAM("max_curvature_per_m",max_curvature);PARAM("lane_lateral_acceleration_mps2",lane_lateral_acceleration);
    PARAM("avoidance_lateral_acceleration_mps2",avoidance_lateral_acceleration);
    PARAM("stanley_gain",stanley_gain);PARAM("stanley_softening_speed_mps",stanley_softening);
    PARAM("stanley_corner_opposing_correction_ratio",stanley_opposing);
    PARAM("path_geometry_window_m",geometry_window);PARAM("steering_current_weight",steering_weight);
    PARAM("control_rate_hz",control_rate);
    config_.curvature_speed_control=declare_parameter("curvature_speed_control_enabled",true,fixed);
    PARAM("deceleration_mps2",deceleration);PARAM("response_delay_sec",delay);PARAM("reference_distance_m",reference_distance);
#undef PARAM
    config_.stanley_corner_heading=declare_parameter("stanley_corner_heading_threshold_deg",60.,fixed)*pi/180;
    config_.max_steer=declare_parameter("max_steering_deg",30.,fixed)*pi/180;
    config_.steer_rate=declare_parameter("steering_rate_deg_per_sec",240.,fixed)*pi/180;
    config_.hist_low=declare_parameter("histogram_low",1.,editable);config_.hist_high=declare_parameter("histogram_high",2.,editable);
    config_.validate();motion_.front_axle_offset=config_.wheelbase;
    planner_deceleration_ceiling_=config_.deceleration;
    speed_ceiling_=declare_parameter("speed_ceiling_mps",1.8,fixed);
    if(!std::isfinite(speed_ceiling_) || speed_ceiling_<.05 || speed_ceiling_>3)throw std::invalid_argument("invalid configured speed ceiling");
    validatePlannerEdits(config_,planner_deceleration_ceiling_,speed_ceiling_);controls_.reload(config_);
    max_age_=declare_parameter("max_age_sec",.30,fixed);max_sync_=declare_parameter("max_sync_sec",.10,fixed);
    if(!std::isfinite(max_age_) || !std::isfinite(max_sync_) || max_age_<.05 || max_age_>.5 || max_sync_<.001 || max_sync_>.15)throw std::invalid_argument("invalid synchronization limits");
    frame_id_=declare_parameter<std::string>("frame_id","front_axle_bev",fixed);
    lane_topic_=declare_parameter<std::string>("lane_result_topic","/line_detactor/result",fixed);
    lane_receipt_timeout_=declare_parameter("lane_receipt_timeout_sec",.30,fixed);
    erpm_timeout_=declare_parameter("erpm_timeout_sec",.15,fixed);imu_timeout_=declare_parameter("imu_timeout_sec",.10,fixed);
    planning_rate_=declare_parameter("planning_rate_hz",20.,fixed);status_rate_=declare_parameter("status_log_rate_hz",2.,fixed);
    path_minimum_points_=declare_parameter("path_minimum_points",8,fixed);max_lane_points_=declare_parameter("max_lane_points",2000,fixed);
    path_minimum_x_=declare_parameter("path_minimum_x_m",.01,fixed);path_maximum_x_=declare_parameter("path_maximum_x_m",2.98,fixed);
    path_maximum_gap_=declare_parameter("path_maximum_gap_m",.15,fixed);
    for(double v:{lane_receipt_timeout_,erpm_timeout_,imu_timeout_,planning_rate_,status_rate_,path_minimum_x_,path_maximum_x_,path_maximum_gap_})
      if(!std::isfinite(v) || v<=0)throw std::invalid_argument("invalid source controller input/watchdog settings");
    if(frame_id_.empty() || lane_receipt_timeout_>1 || erpm_timeout_>1 || imu_timeout_>1 || planning_rate_>100 || status_rate_>20 ||
      path_minimum_points_<2 || max_lane_points_<path_minimum_points_ || max_lane_points_>2000 ||
      path_minimum_x_>=path_maximum_x_ || path_maximum_gap_>1)throw std::invalid_argument("source controller input/watchdog limits exceeded");
    obstacle_slowdown_=declare_parameter("obstacle_slowdown_enabled",true,fixed);
    obstacle_speed_cap_=declare_parameter("obstacle_slowdown_speed_mps",1.,fixed);
    obstacle_clear_sec_=declare_parameter("obstacle_slowdown_clear_sec",1.,fixed);
    recovery_acceleration_=declare_parameter("target_recovery_acceleration_mps2",.8,fixed);
    if(!std::isfinite(obstacle_speed_cap_) || obstacle_speed_cap_<=0 || !std::isfinite(obstacle_clear_sec_) || obstacle_clear_sec_<0 ||
      !std::isfinite(recovery_acceleration_) || recovery_acceleration_<=0)throw std::invalid_argument("invalid source slowdown/recovery settings");
    DriveOptions drive_options;drive_options.max_steer=config_.max_steer;
    drive_options.steering_weight=config_.steering_weight;drive_options.control_rate=config_.control_rate;
    drive_options.steer_rate=config_.steer_rate;drive_options.planning_deceleration=config_.deceleration;
#define DRIVE_PARAM(name,field) drive_options.field=declare_parameter("drive." name,drive_options.field,fixed)
    DRIVE_PARAM("maximum_duty",maximum_duty);DRIVE_PARAM("longitudinal_start_duty",start_duty);
    DRIVE_PARAM("duty_rise_rate_per_sec",rise);DRIVE_PARAM("duty_fall_rate_per_sec",fall);
    DRIVE_PARAM("longitudinal_pid_kp",kp);DRIVE_PARAM("longitudinal_pid_ki",ki);
    DRIVE_PARAM("longitudinal_pid_kd",kd);DRIVE_PARAM("longitudinal_pid_integral_limit",integral_limit);
    DRIVE_PARAM("longitudinal_feedforward_offset_duty",ff_offset);
    DRIVE_PARAM("longitudinal_feedforward_duty_per_mps",ff_slope);
    DRIVE_PARAM("longitudinal_feedforward_fade_speed_mps",ff_fade_speed);
    DRIVE_PARAM("brake_maximum_current_amps",brake_max);DRIVE_PARAM("longitudinal_brake_speed_gain",brake_speed_gain);

    DRIVE_PARAM("brake_current_rise_amps_per_sec",brake_rise);DRIVE_PARAM("brake_current_fall_amps_per_sec",brake_fall);
    DRIVE_PARAM("servo_left",servo_left);DRIVE_PARAM("servo_center",servo_center);DRIVE_PARAM("servo_right",servo_right);
    DRIVE_PARAM("steering_servo_inverted",inverted);
    DRIVE_PARAM("longitudinal_recovery_duty_rise_per_sec",recovery_rise);
    DRIVE_PARAM("longitudinal_recovery_start_ramp_sec",recovery_ramp);
    DRIVE_PARAM("electrical_brake_enabled",electrical_brake);
    DRIVE_PARAM("longitudinal_deceleration_filter_sec",deceleration_filter);
    DRIVE_PARAM("longitudinal_brake_deceleration_gain",brake_deceleration_gain);
    DRIVE_PARAM("brake_entry_speed_error_mps",normal_entry);DRIVE_PARAM("brake_exit_speed_error_mps",normal_exit);
    DRIVE_PARAM("obstacle_brake_entry_margin_mps",obstacle_entry);DRIVE_PARAM("obstacle_brake_exit_margin_mps",obstacle_exit);
    DRIVE_PARAM("obstacle_coast_probe_sec",obstacle_probe);DRIVE_PARAM("corner_coast_probe_sec",normal_probe);
    DRIVE_PARAM("obstacle_brake_deceleration_margin_mps2",obstacle_deceleration_margin);
    DRIVE_PARAM("corner_brake_deceleration_margin_mps2",normal_deceleration_margin);
#undef DRIVE_PARAM
    drive_=std::make_unique<DriveController>(drive_options);
    drive_max_age_=declare_parameter("drive.max_command_age_sec",.30,fixed);
    if(!std::isfinite(drive_max_age_) || drive_max_age_<.05 || drive_max_age_>.3)throw std::invalid_argument("invalid drive command age");
    resume_stable_sec_=declare_parameter("input_resume_stable_sec",.5,fixed);
    if(!std::isfinite(resume_stable_sec_) || resume_stable_sec_<.1 || resume_stable_sec_>5)
      throw std::invalid_argument("invalid input resume stability period");
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
    // Keep a bounded capture history across short GUI/planner stalls; grids
    // still use latest-only QoS. Never refresh an old IMU sample's timestamp.
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("/roi_lidar/imu",rclcpp::SensorDataQoS().keep_last(200),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr p){
        imu_received_at_=now().seconds();
        if(p->header.frame_id!=frame_id_ || !std::isfinite(p->angular_velocity.z)) {
          imu_issue_="invalid frame or yaw rate";motion_.clear();motion_ok_=false;return;
        }
        const double capture=stamp(p->header.stamp),age=imu_received_at_-capture;
        if(age<-.02 || age>max_age_) {imu_issue_="stale/future IMU sample discarded";return;}
        if(capture<=imu_at_)return; // A duplicate must not erase the integration history.
        imu_at_=capture;measured_yaw_rate_=p->angular_velocity.z;
        if(imu_received_at_-speed_at_>erpm_timeout_) {
          imu_issue_="waiting for fresh VESC speed";motion_.clear();motion_ok_=false;return;
        }
        motion_.add(capture,measured_speed_,measured_yaw_rate_);
        MotionPose pose;motion_ok_=motion_.at(capture,pose);
        imu_issue_=motion_ok_?"ok":"motion sample rejected (speed/yaw/time)";
      });
    lane_sub_=create_subscription<line_detactor::msg::LaneResult>(lane_topic_,sensor,
      [this](line_detactor::msg::LaneResult::ConstSharedPtr p){lane_=p;lane_received_at_=now().seconds();});
    lock_sub_=create_subscription<std_msgs::msg::Bool>("/roi_lidar/parameters_locked",latch,
      [this](std_msgs::msg::Bool::ConstSharedPtr p){
        if(requested_ && locked_ && !p->data){stop("STOP: ROI parameter lock lost");publish();return;}
        locked_=requested_ && p->data;
      });
    RCLCPP_INFO(get_logger(),"VFH telemetry: speed=%s (Float32 m/s), IMU=/roi_lidar/imu (vehicle-frame Z rad/s)",speed_sub_->get_topic_name());
    const bool log_enabled=declare_parameter("driving_log_enabled",true,fixed);
    driving_log_rate_=declare_parameter("driving_log_rate_hz",30.,fixed);
    const auto log_directory=declare_parameter<std::string>("driving_log_directory","driving_logs",fixed);
    declare_parameter<std::string>("reference_profile","",fixed);
    if(!std::isfinite(driving_log_rate_) || driving_log_rate_<=0 || driving_log_rate_>200)throw std::invalid_argument("invalid driving log rate");
    planner_parameters_callback_=add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      if(requested_){result.reason="VFH is locked: press Stop before changing parameters";return result;}
      try {
        auto next=plannerParameters();
        for(const auto& parameter:parameters) {
          const auto* field=plannerField(parameter.get_name());
          if(!field)throw std::invalid_argument("Only VFH planner fields are runtime-editable");
          if(parameter.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
            throw std::invalid_argument(parameter.get_name()+": expected a floating-point number");
          next.*(field->member)=parameter.as_double();
        }
        validatePlannerEdits(next,planner_deceleration_ceiling_,speed_ceiling_);result.successful=true;
      }catch(const std::exception& e){result.reason=e.what();}
      return result;
    });
    if(log_enabled) {
      std::string parameters;
      for(const auto& name:list_parameters({},10).names)parameters+=name+"="+get_parameter(name).value_to_string()+"\n";
      driving_log_=std::make_unique<auto_control::DrivingLog>(log_directory,
        "ros_ns,elapsed_sec,dropped,state,target_speed_mps,speed_mps,steer_rad,duty,brake_amps,requested,locked,path_length_m,candidates,vfh,guidance,path",parameters);
      RCLCPP_INFO(get_logger(),"VFH driving CSV: %s",driving_log_->path().c_str());
    }
    publishLock(false);
    RCLCPP_INFO(get_logger(),"Source geometry: width=%.3f front=%.3f rear=%.3f margin=%.3f min_path=%.3f; Stanley lookahead=%.3f, nominal=%.2f ceiling=%.2f",
      config_.width,config_.front,config_.rear,config_.margin,config_.minimum_path_length,config_.lane_lookahead,config_.speed,speed_ceiling_);
    timer_=create_wall_timer(std::chrono::duration<double>(1/planning_rate_),[this]{tick();});
    if(drive_enabled_)actuator_timer_=create_wall_timer(std::chrono::duration<double>(1/config_.control_rate),[this]{
      try {
        checkDriveGuard();
        applyDrive(active() && bool(last_grid_));
      }catch(const std::exception& e){stop(std::string("STOP: actuator loop ")+e.what());publish();}
    });
  }
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<roi_vfh_test::Node>());if(rclcpp::ok())rclcpp::shutdown();}
