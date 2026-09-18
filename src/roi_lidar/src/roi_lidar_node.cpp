#include "roi_lidar/config.hpp"
#include "roi_lidar/preview.hpp"
#include "roi_lidar/freshness.hpp"
#include <point_cloud/startup_parameters.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;
namespace roi_lidar
{
using Clock=std::chrono::steady_clock;
static double seconds(Clock::time_point t) {return std::chrono::duration<double>(t.time_since_epoch()).count();}
static point_cloud::Intrinsics intrinsics(dai::ImgFrame & frame)
{
  const auto & metadata=frame.getTransformation();
  if (!metadata.isValid()) {throw std::runtime_error("Missing frame intrinsics");}
  const auto k=metadata.getIntrinsicMatrix();
  return {k[0][0],k[1][1],k[0][2],k[1][2]};
}
static std::vector<double> geometryKey(dai::ImgFrame & frame, const point_cloud::RigidTransform & t)
{
  const auto k=intrinsics(frame);
  std::vector<double> key{double(frame.getWidth()),double(frame.getHeight()),k.fx,k.fy,k.cx,k.cy};
  key.insert(key.end(),t.rotation.begin(),t.rotation.end());
  key.insert(key.end(),t.translation.begin(),t.translation.end()); return key;
}
static cv::Mat frameView(dai::ImgFrame & frame, int cv_type)
{
  const auto type=frame.getType();
  if ((cv_type==CV_16UC1 && type!=dai::ImgFrame::Type::RAW16) ||
    (cv_type==CV_8UC3 && type!=dai::ImgFrame::Type::BGR888i) ||
    (cv_type==CV_8UC1 && type!=dai::ImgFrame::Type::RAW8 &&
     type!=dai::ImgFrame::Type::GRAY8 && type!=dai::ImgFrame::Type::YUV400p)) {
    throw std::runtime_error("Unexpected preview pixel format");
  }
  const auto width=frame.getWidth(), height=frame.getHeight();
  const std::size_t row=width*CV_ELEM_SIZE(cv_type);
  const std::size_t stride=frame.getStride() ? frame.getStride() : row;
  auto && bytes=frame.getData();
  if (!width || !height || stride<row || bytes.size()<row ||
    (height>1 && stride>(bytes.size()-row)/(height-1))) {throw std::runtime_error("Truncated image frame");}
  return cv::Mat(height,width,cv_type,bytes.data(),stride);
}
static void apply(Config & c,const rclcpp::Parameter & p)
{
  if (p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER &&
    (p.as_int()<std::numeric_limits<int>::min() || p.as_int()>std::numeric_limits<int>::max())) {
    throw std::invalid_argument("Integer out of range");
  }
#define APPLY(name,field,getter) if (p.get_name()==name) {c.field=p.getter(); return;}
  ROI_PARAMETERS(APPLY)
#undef APPLY
}
class Node : public rclcpp::Node
{
  struct Snapshot
  {
    Scan scan;
    std::shared_ptr<dai::ImgFrame> depth, right, rgb;
    point_cloud::RigidTransform vehicle_from_rgb;
    std::uint64_t revision{0};
    double capture{0}, fps{0}, host_ms{0}, age_ms{0};
    std::string state{"MEASURING GROUND: keep stationary"};
  };
  Config config_;
  Snapshot snapshot_;
  std::mutex mutex_;
  std::atomic_bool stop_{false};
  std::atomic<std::uint64_t> camera_revision_{0};
  std::uint64_t revision_{0};
  std::thread worker_;
  oak_startup::OakStartupMeasurementConfig startup_;
  std::string frame_id_,device_id_;
  double camera_x_,camera_y_,camera_yaw_;
  std::vector<std::string> names_;
  std::vector<rclcpp::Parameter> last_parameters_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_pub_, roi_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr sync_timer_, preview_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
  BevProjector bev_;
  std::vector<double> rgb_key_;
  std::uint64_t preview_revision_{std::numeric_limits<std::uint64_t>::max()};
  double last_preview_{0};
  bool windows_{false}, gui_failed_{false}, dragging_{false};
  int roi_width_{0},roi_height_{0};
  cv::Point drag_start_;
  bool stopping() const {return stop_ || !rclcpp::ok();}
  void state(const std::string & text)
  {
    {std::lock_guard<std::mutex> lock(mutex_); snapshot_.state=text;}
    std_msgs::msg::String msg; msg.data=text; status_pub_->publish(msg);
  }
  Config readParameters()
  {
    Config c; for (const auto & p:get_parameters(names_)) {apply(c,p);} validate(c); return c;
  }
  void syncParameters()
  {
    const auto params=get_parameters(names_);
    bool changed=false,reopen=false;
    for (std::size_t i=0;i<params.size();++i) {
      if (params[i].to_parameter_msg()==last_parameters_[i].to_parameter_msg()) {continue;}
      changed=true;
      const auto & n=names_[i];
      reopen|=n.rfind("camera.",0)==0 || n.rfind("depth.",0)==0 ||
        n=="preview.rgb_fps" || n=="preview.camera_image" || n=="preview.gui" || n=="preview.publish";
    }
    if (!changed) {return;}
    auto c=readParameters();
    {std::lock_guard<std::mutex> lock(mutex_); config_=c; ++revision_;}
    if (reopen) {++camera_revision_;}
    last_parameters_=params;
  }
  void publishScan(const Scan & scan,const Config & c,const rclcpp::Time & stamp,double period)
  {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp=stamp; msg.header.frame_id=frame_id_;
    msg.angle_min=c.scan.angle_min*radians; msg.angle_max=c.scan.angle_max*radians;
    msg.angle_increment=(msg.angle_max-msg.angle_min)/(c.scan.bins-1);
    msg.range_min=c.scan.min_range; msg.range_max=c.scan.max_range;
    msg.scan_time=std::max(0.0,period); msg.time_increment=0; msg.ranges=scan.ranges;
    scan_pub_->publish(msg);
  }
  void invalidate(const Config & c,const std::string & text)
  {
    {std::lock_guard<std::mutex> lock(mutex_); snapshot_=Snapshot{}; snapshot_.state=text;}
    publishScan(emptyScan(c.scan),c,now(),0); state(text);
  }
  void run()
  {
    point_cloud::RigidTransform vehicle_from_camera;
    std::string selected=device_id_;
    try {
      state("MEASURING GROUND: keep stationary on a flat floor");
      const auto pose=oak_startup::measureOakStartupExtrinsics(startup_,[this](){return stopping();});
      selected=pose.device_id;
      vehicle_from_camera=point_cloud::vehicleFromRgb(camera_x_,camera_y_,pose.height_m,
        pose.roll_deg,pose.pitch_down_deg,camera_yaw_);
      RCLCPP_INFO(get_logger(),"Ground fixed: height=%.3fm roll=%.2f pitch=%.2f",pose.height_m,pose.roll_deg,pose.pitch_down_deg);
    } catch (const std::exception & e) {
      state(std::string("STARTUP FAILED: restart after fixing ground ROI: ")+e.what()); return;
    }
    while (!stopping()) {
      Config startup_config;
      {std::lock_guard<std::mutex> lock(mutex_); startup_config=config_;}
      const auto camera_revision=camera_revision_.load();
      try {
        invalidate(startup_config,"OPENING CAMERA");
        point_cloud::DepthSource source(startup_config.camera,selected,
          startup_config.camera_image && (startup_config.gui || startup_config.publish_preview),
          (startup_config.gui || startup_config.publish_preview) ? startup_config.rgb_fps : 0.0);
        Projector projector;
        std::vector<double> key;
        std::uint64_t configured_revision=std::numeric_limits<std::uint64_t>::max();
        double last_capture=0,last_rx=seconds(Clock::now()),report=last_rx;
        std::size_t frames=0; double work_ms=0,fps=0,host_ms=0;
        bool stale=false;
        while (!stopping() && camera_revision==camera_revision_) {
          Config c; std::uint64_t version;
          {std::lock_guard<std::mutex> lock(mutex_); c=config_; version=revision_;}
          const auto host=Clock::now(); const double clock=seconds(host);
          if (auto rgb=source.tryGetRgb()) {
            const auto transform=point_cloud::compose(vehicle_from_camera,source.rgbFromFrame(*rgb));
            std::lock_guard<std::mutex> lock(mutex_); snapshot_.rgb=std::move(rgb); snapshot_.vehicle_from_rgb=transform;
          }
          if (auto right=source.tryGetRight()) {
            std::lock_guard<std::mutex> lock(mutex_); snapshot_.right=std::move(right);
          }
          if (clock-last_rx>c.max_age && !stale) {invalidate(c,"STALE: no fresh depth"); stale=true;}
          auto depth=source.tryGet();
          if (!depth) {std::this_thread::sleep_for(1ms); continue;}
          const double capture=seconds(depth->getTimestamp()),age=clock-capture;
          if (age<-.01 || age>c.max_age || capture<=last_capture) {continue;}
          if (depth->getType()!=dai::ImgFrame::Type::RAW16) {throw std::runtime_error("Expected RAW16 depth");}
          const auto transform=point_cloud::compose(vehicle_from_camera,source.rgbFromFrame(*depth));
          const auto next_key=geometryKey(*depth,transform);
          if (key!=next_key || configured_revision!=version) {
            projector.configure(depth->getWidth(),depth->getHeight(),intrinsics(*depth),transform,c.scan);
            key=next_key; configured_revision=version;
          }
          const auto & bytes=depth->getData();
          auto scan=projector.project(bytes.data(),bytes.size(),depth->getStride() ? depth->getStride() : depth->getWidth()*2);
          auto stamp=now();
          const auto delay=rclcpp::Duration::from_seconds(std::max(0.0,age));
          if (stamp.nanoseconds()>=delay.nanoseconds()) {stamp=stamp-delay;}
          // Never publish an in-flight result after settings were replaced.
          {std::lock_guard<std::mutex> lock(mutex_); if (version!=revision_) {continue;}}
          publishScan(scan,c,stamp,last_capture>0 ? capture-last_capture : 0);
          ++frames; work_ms+=std::chrono::duration<double,std::milli>(Clock::now()-host).count();
          if (clock-report>=1) {
            fps=frames/(clock-report); host_ms=work_ms/frames;
            RCLCPP_INFO(get_logger(),"requested=%.1f depth_fps=%.1f host=%.2fms age=%.1fms ROI_rays=%zu depth=%zu ground_removed=%zu height_removed=%zu accepted=%zu bins=%zu range_limit=%.1fm",
              c.camera.fps,fps,host_ms,age*1000,projector.rayCount(),scan.depth_points,scan.ground_removed,scan.height_removed,scan.accepted_points,scan.valid_bins,c.scan.max_range);
            report=clock; frames=0; work_ms=0;
          }
          {std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.scan=std::move(scan); snapshot_.depth=std::move(depth); snapshot_.capture=capture;
            snapshot_.revision=version; snapshot_.fps=fps; snapshot_.host_ms=host_ms; snapshot_.age_ms=age*1000;
            snapshot_.state="STREAMING";
          }
          if (last_capture==0 || stale) {state("STREAMING");}
          last_capture=capture; last_rx=clock; stale=false;
        }
      } catch (const std::exception & e) {
        invalidate(startup_config,std::string("CAMERA ERROR: ")+e.what());
        RCLCPP_ERROR(get_logger(),"%s",e.what());
        for (int i=0;i<20 && !stopping() && camera_revision==camera_revision_;++i) {std::this_thread::sleep_for(100ms);}
      }
    }
  }
  sensor_msgs::msg::Image imageMessage(const cv::Mat & image,const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::Image msg; msg.header.stamp=stamp; msg.header.frame_id=frame_id_;
    msg.width=image.cols; msg.height=image.rows; msg.encoding="bgr8"; msg.step=image.cols*3;
    msg.data.resize(msg.step*msg.height);
    for (int row=0;row<image.rows;++row) {std::memcpy(msg.data.data()+row*msg.step,image.ptr(row),msg.step);}
    return msg;
  }
  static void mouse(int event,int x,int y,int,void * context)
  {
    auto & self=*static_cast<Node *>(context);
    if (event==cv::EVENT_LBUTTONDOWN) {self.dragging_=true; self.drag_start_={x,y};}
    if (event!=cv::EVENT_LBUTTONUP || !self.dragging_) {return;}
    self.dragging_=false;
    if (self.roi_width_<1 || self.roi_height_<1) {return;}
    const int x0=std::clamp(std::min(x,self.drag_start_.x),0,self.roi_width_-1);
    const int y0=std::clamp(std::min(y,self.drag_start_.y),0,self.roi_height_-1);
    const int x1=std::clamp(std::max(x,self.drag_start_.x),x0+1,self.roi_width_);
    const int y1=std::clamp(std::max(y,self.drag_start_.y),y0+1,self.roi_height_);
    if (x1-x0<4 || y1-y0<4) {return;}
    const auto result=self.set_parameters_atomically({
      rclcpp::Parameter("roi.x",double(x0)/self.roi_width_),rclcpp::Parameter("roi.y",double(y0)/self.roi_height_),
      rclcpp::Parameter("roi.width",double(x1-x0)/self.roi_width_),rclcpp::Parameter("roi.height",double(y1-y0)/self.roi_height_)});
    if (!result.successful) {RCLCPP_WARN(self.get_logger(),"ROI: %s",result.reason.c_str());}
  }
  void preview()
  {
    Config c; Snapshot s; std::uint64_t version;
    {std::lock_guard<std::mutex> lock(mutex_); c=config_; s=snapshot_; version=revision_;}
    const double clock=seconds(Clock::now());
    if (clock-last_preview_<1.0/c.preview_fps) {return;} last_preview_=clock;
    if (preview_revision_!=version) {rgb_key_.clear(); preview_revision_=version;}
    if (!c.gui && windows_) {
      try {cv::destroyAllWindows();} catch (const cv::Exception &) {}
      windows_=false;
    }
    if (!c.gui && !c.publish_preview) {return;}
    try {
      const bool fresh=s.depth && s.revision==version && freshCapture(s.capture,clock,c.max_age);
      const bool rgb_fresh=s.rgb && freshCapture(seconds(s.rgb->getTimestamp()),clock,c.max_age);
      const bool aligned=fresh && rgb_fresh && overlayAllowed(s.capture,seconds(s.rgb->getTimestamp()),clock,c.max_age,c.sync_sec);
      cv::Mat background;
      if (rgb_fresh) {
        const auto key=geometryKey(*s.rgb,s.vehicle_from_rgb);
        if (key!=rgb_key_) {
          bev_.configure(s.rgb->getWidth(),s.rgb->getHeight(),intrinsics(*s.rgb),s.vehicle_from_rgb,c.view,c.preview_size);
          rgb_key_=key;
        }
        background=bev_.render(frameView(*s.rgb,CV_8UC3));
      }
      std::ostringstream text; text<<s.state<<" | depth "<<std::fixed<<std::setprecision(1)<<s.fps
        <<" FPS / host "<<s.host_ms<<" ms";
      if (!rgb_fresh) {text<<" | WAIT RGB";}
      else if (!aligned) {text<<" | OVERLAY WAIT SYNC";}
      else {text<<" | delta "<<std::abs(s.capture-seconds(s.rgb->getTimestamp()))*1000<<" ms";}
      auto image=mapPreview(aligned ? s.scan : emptyScan(c.scan),c.scan,c.view,c.preview_size,text.str(),background);
      if (c.publish_preview) {preview_pub_->publish(imageMessage(image,now()));}
      cv::Mat roi;
      if (fresh) {
        const bool right=c.camera_image && s.right &&
          std::abs(seconds(s.right->getTimestamp())-s.capture)<=c.sync_sec;
        auto frame=right ? s.right : s.depth;
        roi=roiPreview(frameView(*frame,right ? CV_8UC1 : CV_16UC1),c.scan,!right);
      } else {
        roi=cv::Mat(400,640,CV_8UC3,cv::Scalar(30,30,30));
        cv::putText(roi,"NO FRESH DEPTH",{20,40},cv::FONT_HERSHEY_SIMPLEX,.7,{220,220,220},1,cv::LINE_AA);
      }
      if (c.publish_preview) {roi_pub_->publish(imageMessage(roi,now()));}
      if (c.gui && !gui_failed_) {
        if (!windows_) {
          cv::namedWindow("ROI LiDAR - RGB BEV",cv::WINDOW_AUTOSIZE);
          cv::namedWindow("ROI LiDAR - select depth ROI",cv::WINDOW_AUTOSIZE);
          cv::setMouseCallback("ROI LiDAR - select depth ROI",mouse,this); windows_=true;
        }
        cv::imshow("ROI LiDAR - RGB BEV",image);
        roi_width_=fresh ? roi.cols : 0; roi_height_=fresh ? roi.rows : 0;
        cv::imshow("ROI LiDAR - select depth ROI",roi);
        const int key=cv::waitKey(1)&255;
        if (key=='q' || key==27) {stop_=true; rclcpp::shutdown();}
        if (key=='r') {set_parameters_atomically({rclcpp::Parameter("roi.x",0.0),rclcpp::Parameter("roi.y",0.0),
          rclcpp::Parameter("roi.width",1.0),rclcpp::Parameter("roi.height",1.0)});}
      }
    } catch (const cv::Exception & e) {
      gui_failed_=true; RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Preview: %s",e.what());
    } catch (const std::exception & e) {RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Preview geometry: %s",e.what());}
  }
public:
  Node():rclcpp::Node("roi_lidar")
  {
#define DECLARE(name,field,getter) declare_parameter(name,config_.field); names_.emplace_back(name);
    ROI_PARAMETERS(DECLARE)
#undef DECLARE
    rcl_interfaces::msg::ParameterDescriptor ro; ro.read_only=true;
    device_id_=declare_parameter<std::string>("device_id","",ro);
    frame_id_=declare_parameter<std::string>("frame_id","front_axle_bev",ro);
    camera_x_=declare_parameter<double>("bev.camera_x_m",-.16,ro);
    camera_y_=declare_parameter<double>("bev.camera_y_m",0,ro);
    camera_yaw_=declare_parameter<double>("bev.camera_yaw_deg",0,ro);
    if (frame_id_.empty() || !std::isfinite(camera_x_) || !std::isfinite(camera_y_) || !std::isfinite(camera_yaw_)) {
      throw std::invalid_argument("Invalid frame or camera mount");
    }
    startup_=point_cloud::startupParameters(*this); startup_.device_id=device_id_;
    // All runtime GUI belongs to the ROS/main thread. Measurement remains headless.
    startup_.roi_preview_enabled=false;
    config_=readParameters();
#ifdef __linux__
    if (config_.gui && !std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
      gui_failed_=true;
      RCLCPP_WARN(get_logger(),"No desktop display; GUI disabled, image topics remain available");
    }
#endif
    last_parameters_=get_parameters(names_);
    const auto qos=rclcpp::SensorDataQoS().keep_last(1);
    scan_pub_=create_publisher<sensor_msgs::msg::LaserScan>("~/scan",qos);
    preview_pub_=create_publisher<sensor_msgs::msg::Image>("~/bev_preview",qos);
    roi_pub_=create_publisher<sensor_msgs::msg::Image>("~/roi_preview",qos);
    status_pub_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());
    callback_=add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
      rcl_interfaces::msg::SetParametersResult out;
      try {auto c=readParameters(); for (const auto & p:ps) {apply(c,p);} validate(c); out.successful=true;}
      catch (const std::exception & e) {out.reason=e.what();} return out;
    });
    sync_timer_=create_wall_timer(50ms,[this](){syncParameters();});
    preview_timer_=create_wall_timer(5ms,[this](){preview();});
    worker_=std::thread([this](){run();});
  }
  ~Node() override
  {
    stop_=true; if (worker_.joinable()) {worker_.join();}
    if (windows_) {try {cv::destroyAllWindows();} catch (...) {}}
  }
};
}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<roi_lidar::Node>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("roi_lidar"),"%s",e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
