#include "roi_vfh_test/planner.hpp"
#include "roi_vfh_test/memory.hpp"
#include "roi_vfh_test/render.hpp"
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
  MotionHistory motion_;TerrainMemory memory_;
  std::uint64_t motion_generation_{0};
  double measured_yaw_rate_{0},measured_speed_{0},speed_at_{-1e9},imu_at_{-1e9},memory_ttl_{.5},memory_distance_{.35};
  bool motion_ok_{false};
  MotionPose previous_pose_;bool previous_pose_valid_{false};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  bool gui_{true},requested_{false},locked_{false},window_{false};
  double max_age_{.30},max_sync_{.10},plan_ms_{0};
  std::deque<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids_;
  line_detactor::msg::LaneResult::ConstSharedPtr lane_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr last_grid_;
  std::vector<Point> lane_points_; Grid display_, observed_display_, fused_display_;
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
    requested_=false;locked_=false;planner_.reset();memory_.clear();previous_pose_valid_=false;result_=Plan{};last_grid_.reset();status_=reason;publishLock(false);
  }
  void start() {
    requested_=true;locked_=false;planner_.reset();memory_.clear();previous_pose_valid_=false;result_=Plan{};last_grid_.reset();
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
    lane_points_=std::move(points);observed_display_=grid;display_=std::move(grid);
    if(requested_ && locked_ && (now().seconds()-imu_at_>.15 || now().seconds()-speed_at_>.15)){status_="STOP: motion telemetry stale";return false;}
    MotionPose pose;
    if(requested_ && locked_) {
      MotionPose lane_pose;
      if(!motion_ok_ || !motion_.at(stamp(match->header.stamp),pose) || !motion_.at(stamp(lane_->header.stamp),lane_pose)) {
        memory_.clear();status_="STOP: motion history cannot align sensor captures";return false;
      }
      for(auto& point:lane_points_)point=reproject(point,lane_pose,pose);
      if(match==last_grid_)display_=fused_display_;
    }
    if (requested_ && locked_ && match!=last_grid_) {
      if(!motion_ok_ || now().seconds()-speed_at_>.15 || !motion_.at(stamp(match->header.stamp),pose)) {
        memory_.clear();status_="STOP: fresh VESC speed + IMU required";return false;
      }
      if(motion_generation_!=motion_.generation) {memory_.clear();previous_pose_valid_=false;motion_generation_=motion_.generation;}
      const Grid observed=display_;
      display_=memory_.update(observed,pose,memory_ttl_,memory_distance_);fused_display_=display_;
      if(previous_pose_valid_)planner_.rotateReference(wrap(pose.yaw-previous_pose_.yaw));
      previous_pose_=pose;previous_pose_valid_=true;
      const auto begin=std::chrono::steady_clock::now();
      auto working=config_;
      working.delay=std::max(config_.delay,std::max(0.0,now().seconds()-stamp(match->header.stamp))+.067);
      working.current_steer=std::clamp(std::atan(config_.wheelbase*measured_yaw_rate_/std::max(.1,std::abs(measured_speed_))),-config_.max_steer,config_.max_steer);
      result_=planner_.plan(display_,lane_points_,working,&observed);
      plan_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      last_grid_=match;status_=result_.status;
    }
    return true;
  }
  void publish() {
    nav_msgs::msg::Path path;path.header.stamp=now();path.header.frame_id="front_axle_bev";
    const bool active=requested_ && locked_;
    if(active && last_grid_)path.header=last_grid_->header;
    if (active) for (const auto & p:result_.path) {
      geometry_msgs::msg::PoseStamped pose;pose.header=path.header;pose.pose.position.x=p.x;pose.pose.position.y=p.y;
      pose.pose.orientation.z=std::sin(p.yaw/2);pose.pose.orientation.w=std::cos(p.yaw/2);path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    std_msgs::msg::Float32 speed,steer;speed.data=active?result_.speed:0;steer.data=active?result_.steer:0;
    speed_pub_->publish(speed);steer_pub_->publish(steer);
    std_msgs::msg::String message;
    message.data=status_+cv::format(" | preview v=%.2fm/s steer=%.1fdeg plan=%.2fms candidates=%zu | rear unverified",
      speed.data,steer.data*180/pi,plan_ms_,result_.candidates.size());
    status_pub_->publish(message);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"%s",message.data.c_str());
  }
  void draw() {
    if(!gui_ && !image_pub_->get_subscription_count())return;
    const auto canvas=renderPreview(display_,observed_display_,lane_points_,result_,config_,requested_,locked_,status_,plan_ms_);
    if (image_pub_->get_subscription_count()) {
      sensor_msgs::msg::Image msg;msg.header.stamp=now();msg.header.frame_id="front_axle_bev";
      msg.width=canvas.cols;msg.height=canvas.rows;msg.encoding="bgr8";msg.step=canvas.cols*3;
      msg.data.assign(canvas.data,canvas.data+canvas.total()*3);image_pub_->publish(msg);
    }
    if (gui_) {
      if (!window_) {cv::namedWindow("ROI VFH+ test",cv::WINDOW_AUTOSIZE);cv::setMouseCallback("ROI VFH+ test",mouse,this);window_=true;}
      cv::imshow("ROI VFH+ test",canvas);
      const int key=cv::waitKey(1);if (key==' ' || key=='s'){stop("STOPPED: parameters unlocked");publish();}
      if (key=='q' || key==27 || cv::getWindowProperty("ROI VFH+ test",cv::WND_PROP_VISIBLE)<1) {stop("STOPPED: preview closed");publish();rclcpp::shutdown();}
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
#define PARAM(name,field) config_.field=declare_parameter(name,config_.field,fixed)
    PARAM("nominal_speed_mps",speed);PARAM("horizon_m",horizon);PARAM("sample_step_m",step);
    PARAM("wheelbase_m",wheelbase);PARAM("vehicle_width_m",width);PARAM("rear_extent_m",rear);PARAM("front_overhang_m",front);
    PARAM("safety_margin_m",margin);PARAM("lane_width_m",lane_width);PARAM("lane_departure_m",departure);
    PARAM("deceleration_mps2",deceleration);PARAM("response_delay_sec",delay);PARAM("reference_distance_m",reference_distance);
#undef PARAM
    config_.max_steer=declare_parameter("max_steering_deg",30.,fixed)*pi/180;
    config_.steer_rate=declare_parameter("steering_rate_deg_per_sec",240.,fixed)*pi/180;
    config_.hist_low=declare_parameter("histogram_low",1.,fixed);config_.hist_high=declare_parameter("histogram_high",2.,fixed);
    config_.validate();motion_.front_axle_offset=config_.wheelbase;
    memory_ttl_=declare_parameter("memory_time_sec",.5,fixed);memory_distance_=declare_parameter("memory_distance_m",.35,fixed);
    if(!std::isfinite(memory_ttl_)||!std::isfinite(memory_distance_)||memory_ttl_<=0||memory_ttl_>2||memory_distance_<=0||memory_distance_>.5)throw std::invalid_argument("invalid memory limits");
    max_age_=declare_parameter("max_age_sec",.30,fixed);max_sync_=declare_parameter("max_sync_sec",.10,fixed);
    if(!std::isfinite(max_age_) || !std::isfinite(max_sync_) || max_age_<.05 || max_age_>.5 || max_sync_<.001 || max_sync_>.15)throw std::invalid_argument("invalid synchronization limits");
    const auto sensor=rclcpp::SensorDataQoS().keep_last(1);
    const auto latch=rclcpp::QoS(1).reliable().transient_local();
    running_pub_=create_publisher<std_msgs::msg::Bool>("~/running",latch);
    path_pub_=create_publisher<nav_msgs::msg::Path>("~/path",1);
    speed_pub_=create_publisher<std_msgs::msg::Float32>("~/target_speed",1);
    steer_pub_=create_publisher<std_msgs::msg::Float32>("~/steering_angle_rad",1);
    status_pub_=create_publisher<std_msgs::msg::String>("~/status",1);
    image_pub_=create_publisher<sensor_msgs::msg::Image>("~/preview",sensor);
    grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/roi_lidar/terrain_for_planning",sensor,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr p){grids_.push_back(p);while(grids_.size()>16)grids_.pop_front();});
    speed_sub_=create_subscription<std_msgs::msg::Float32>(declare_parameter<std::string>("measured_speed_topic","/vehicle/dynamics/speed_mps",fixed),sensor,
      [this](std_msgs::msg::Float32::ConstSharedPtr p){measured_speed_=p->data;speed_at_=now().seconds();});
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("/roi_lidar/imu",sensor,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr p){
        if(p->header.frame_id!="front_axle_bev" || !fresh(p->header.stamp) || now().seconds()-speed_at_>.15) {
          motion_.clear();motion_ok_=false;return;
        }
        imu_at_=stamp(p->header.stamp);
        measured_yaw_rate_=p->angular_velocity.z;
        motion_.add(stamp(p->header.stamp),measured_speed_,measured_yaw_rate_);motion_ok_=true;
      });
    lane_sub_=create_subscription<line_detactor::msg::LaneResult>("/line_detactor/result",sensor,
      [this](line_detactor::msg::LaneResult::ConstSharedPtr p){lane_=p;});
    lock_sub_=create_subscription<std_msgs::msg::Bool>("/roi_lidar/parameters_locked",latch,
      [this](std_msgs::msg::Bool::ConstSharedPtr p){locked_=requested_ && p->data;});
    publishLock(false);
    timer_=create_wall_timer(std::chrono::milliseconds(67),[this]{tick();});
  }
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<roi_vfh_test::Node>());if(rclcpp::ok())rclcpp::shutdown();}
