#include "point_cloud/depth_source.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace point_cloud
{
// A single table keeps declaration, snapshots and parameter validation in sync.
#define CLOUD_PARAMETERS(X) \
  X("camera.resolution", resolution, as_string) \
  X("camera.fps", fps, as_double) \
  X("depth.mode", mode, as_string) \
  X("depth.ir_dot_projector_intensity", dot_intensity, as_double) \
  X("depth.ir_flood_light_intensity", flood_intensity, as_double) \
  X("depth.confidence_threshold", confidence, as_int) \
  X("depth.left_right_check", lr_check, as_bool) \
  X("depth.left_right_threshold", lr_threshold, as_int) \
  X("depth.subpixel", subpixel, as_bool) \
  X("depth.subpixel_fractional_bits", subpixel_bits, as_int) \
  X("depth.extended_disparity", extended, as_bool) \
  X("depth.median_filter", median, as_string) \
  X("depth.spatial_filter", spatial, as_bool) \
  X("depth.speckle_filter", speckle, as_bool) \
  X("depth.hole_filling", hole_filling, as_bool) \
  X("depth.adaptive_median_filter", adaptive_median, as_bool) \
  X("points.pixel_stride", projection.pixel_stride, as_int) \
  X("points.min_depth_m", projection.min_depth_m, as_double) \
  X("points.max_depth_m", projection.max_depth_m, as_double) \
  X("ground.enabled", ground.enabled, as_bool) \
  X("ground.distance_m", ground.distance_m, as_double) \
  X("ground.max_depth_m", ground.max_depth_m, as_double) \
  X("ground.min_height_m", ground.min_height_m, as_double) \
  X("ground.max_height_m", ground.max_height_m, as_double) \
  X("ground.max_tilt_deg", ground.max_tilt_deg, as_double) \
  X("ground.min_inlier_ratio", ground.min_inlier_ratio, as_double) \
  X("publish.depth_image", publish_depth, as_bool) \
  X("input.max_age_sec", max_age_sec, as_double) \
  X("metrics.print_interval_sec", metrics_interval, as_double)

void applyParameter(Config & c, const rclcpp::Parameter & p)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
    (p.as_int() < std::numeric_limits<int>::min() || p.as_int() > std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument("Integer parameter is outside the supported range");
  }
#define APPLY(name, field, getter) if (p.get_name() == name) {c.field = p.getter(); return;}
  CLOUD_PARAMETERS(APPLY)
#undef APPLY
}

Config fromParameters(const std::vector<rclcpp::Parameter> & parameters)
{
  Config config;
  for (const auto & p : parameters) {applyParameter(config, p);}
  validate(config);
  return config;
}

bool hostIsBigEndian()
{
  const std::uint16_t value = 1;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0;
}

class PointCloudNode : public rclcpp::Node
{
public:
  PointCloudNode() : Node("point_cloud")
  {
#define DECLARE(name, field, getter) \
    declare_parameter(name, config_.field); parameter_names_.emplace_back(name);
    CLOUD_PARAMETERS(DECLARE)
#undef DECLARE
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    device_id_ = declare_parameter<std::string>("device_id", "", read_only);
    frame_id_ = declare_parameter<std::string>("frame_id", "point_cloud_optical_frame", read_only);
    view_frame_id_ = declare_parameter<std::string>("view_frame_id", "point_cloud_view", read_only);
    if (frame_id_.empty() || view_frame_id_.empty() || frame_id_ == view_frame_id_) {
      throw std::invalid_argument("frame_id and view_frame_id must be nonempty and different");
    }
    last_parameters_ = get_parameters(parameter_names_);
    config_ = fromParameters(last_parameters_);
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points", qos);
    filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points_filtered", qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/depth/image_raw", qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/depth/camera_info", qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = view_frame_id_;
    tf.child_frame_id = frame_id_;
    // Optical (+X right, +Y down, +Z forward) -> view (+X forward, +Y left, +Z up).
    // Both origins are the rectified right camera; this is NOT a ground/mount transform.
    tf.transform.rotation.x = -0.5;
    tf.transform.rotation.y = 0.5;
    tf.transform.rotation.z = -0.5;
    tf.transform.rotation.w = 0.5;
    tf_broadcaster_->sendTransform(tf);
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        try {
          auto candidate = fromParameters(get_parameters(parameter_names_));
          for (const auto & p : parameters) {applyParameter(candidate, p);}
          validate(candidate);
          result.successful = true;
        } catch (const std::exception & e) {
          result.successful = false;
          result.reason = e.what();
        }
        return result;  // Validation has no side effects, even if another callback rejects it.
      });
    parameter_timer_ = create_wall_timer(100ms, [this]() {syncParameters();});
    worker_ = std::thread([this]() {cameraLoop();});
  }

  ~PointCloudNode() override
  {
    stop_.store(true);
    if (worker_.joinable()) {worker_.join();}
  }

private:
  bool stopping() const {return stop_.load() || !rclcpp::ok();}

  void syncParameters()
  {
    const auto parameters = get_parameters(parameter_names_);
    bool changed = false, reopen = false;
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      if (parameters[i].to_parameter_msg() != last_parameters_[i].to_parameter_msg()) {
        changed = true;
        if (parameter_names_[i].compare(0, 7, "ground.") != 0) {reopen = true;}
      }
    }
    if (!changed) {return;}
    const auto next = fromParameters(parameters);
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config_ = next;
      if (reopen) {++revision_;}
    }
    last_parameters_ = parameters;
    RCLCPP_INFO(get_logger(), "%s", reopen ?
      "Parameters committed; reopening depth pipeline with the new settings" :
      "Ground filter settings applied without restarting the camera");
  }

  void status(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
  }

  sensor_msgs::msg::PointCloud2 cloudMessage(const Cloud & cloud, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.width = cloud.width;
    message.height = cloud.height;
    message.is_bigendian = hostIsBigEndian();
    message.is_dense = cloud.valid_points == static_cast<std::size_t>(cloud.width) * cloud.height;
    message.point_step = 3 * sizeof(float);
    message.row_step = message.width * message.point_step;
    for (std::uint32_t i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField field;
      field.name = std::string(1, "xyz"[i]);
      field.offset = i * sizeof(float);
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      message.fields.push_back(field);
    }
    message.data.resize(cloud.xyz.size() * sizeof(float));
    if (!message.data.empty()) {std::memcpy(message.data.data(), cloud.xyz.data(), message.data.size());}
    return message;
  }

  void clearCloud()
  {
    const auto empty = cloudMessage(Cloud{}, now());
    cloud_pub_->publish(empty);
    filtered_pub_->publish(empty);
  }

  void publishImages(
    const dai::ImgFrame & frame, const Intrinsics & k, const rclcpp::Time & stamp, bool publish_depth)
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.width = frame.getWidth();
    info.height = frame.getHeight();
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    info.k = {k.fx, 0, k.cx, 0, k.fy, k.cy, 0, 0, 1};
    info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    info.p = {k.fx, 0, k.cx, 0, 0, k.fy, k.cy, 0, 0, 0, 1, 0};
    info_pub_->publish(info);
    if (!publish_depth) {return;}
    sensor_msgs::msg::Image image;
    image.header = info.header;
    image.width = info.width;
    image.height = info.height;
    image.encoding = "16UC1";
    image.is_bigendian = hostIsBigEndian();
    image.step = image.width * sizeof(std::uint16_t);
    image.data.resize(static_cast<std::size_t>(image.step) * image.height);
    const auto & bytes = frame.getData();
    const auto stride = frame.getStride() ? frame.getStride() : image.step;
    for (std::uint32_t v = 0; v < image.height; ++v) {
      std::memcpy(image.data.data() + static_cast<std::size_t>(v) * image.step,
        bytes.data() + static_cast<std::size_t>(v) * stride, image.step);
    }
    depth_pub_->publish(std::move(image));
  }

  void cameraLoop()
  {
    std::string selected_id = device_id_;
    while (!stopping()) {
      Config c;
      std::uint64_t revision;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        c = config_;
        revision = revision_.load();
      }
      try {
        clearCloud();
        status("OPENING");
        DepthSource source(c, selected_id);
        selected_id = source.deviceId();
        RCLCPP_INFO(get_logger(), "Device=%s requested=%s FPS=%.1f dot=%.2f flood=%.2f stride=%d",
          selected_id.c_str(), c.resolution.c_str(), c.fps, c.dot_intensity, c.flood_intensity,
          c.projection.pixel_stride);
        status("WAITING FOR DEPTH");
        auto last_frame = std::chrono::steady_clock::now();
        auto report_start = last_frame;
        auto previous_capture = std::chrono::steady_clock::time_point::min();
        std::size_t frames = 0;
        double processing_ms = 0;
        bool stale = false, first = true;
        while (!stopping() && revision_.load() == revision) {
          auto frame = source.tryGet();
          const auto host_now = std::chrono::steady_clock::now();
          if (std::chrono::duration<double>(host_now - last_frame).count() > c.max_age_sec && !stale) {
            clearCloud();
            status("STALE: no fresh depth");
            stale = true;
          }
          if (!frame) {std::this_thread::sleep_for(2ms); continue;}
          const auto capture = frame->getTimestamp();
          const double age = std::chrono::duration<double>(host_now - capture).count();
          if (age < -0.01 || age > c.max_age_sec || capture <= previous_capture) {continue;}
          auto stamp = now();
          const auto delay = rclcpp::Duration::from_seconds(std::max(0.0, age));
          if (stamp.nanoseconds() >= delay.nanoseconds()) {stamp = stamp - delay;}
          if (frame->getType() != dai::ImgFrame::Type::RAW16) {
            throw std::runtime_error("Expected RAW16 millimeter depth");
          }
          const auto & transform = frame->getTransformation();
          if (!transform.isValid()) {throw std::runtime_error("Missing rectified depth intrinsics");}
          const auto matrix = transform.getIntrinsicMatrix();
          const Intrinsics k{matrix[0][0], matrix[1][1], matrix[0][2], matrix[1][2]};
          const auto & bytes = frame->getData();
          const auto stride = frame->getStride() ? frame->getStride() : frame->getWidth() * 2;
          auto cloud = projectDepth(bytes.data(), bytes.size(), frame->getWidth(),
            frame->getHeight(), stride, k, c.projection);
          if (revision_.load() != revision || stopping()) {break;}
          cloud_pub_->publish(cloudMessage(cloud, stamp));
          const auto raw_valid = cloud.valid_points;
          GroundOptions ground;
          {
            std::lock_guard<std::mutex> lock(config_mutex_);
            ground = config_.ground;
          }
          const auto ground_result = removeGround(cloud, ground);
          if (ground.enabled && !ground_result.detected) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Ground plane not found; points_filtered contains the original points");
          }
          filtered_pub_->publish(cloudMessage(cloud, stamp));
          publishImages(*frame, k, stamp, c.publish_depth);
          if (first || stale) {
            status("STREAMING");
            if (first) {
              RCLCPP_INFO(get_logger(), "Actual depth=%ux%u cloud=%ux%u fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                frame->getWidth(), frame->getHeight(), cloud.width, cloud.height, k.fx, k.fy, k.cx, k.cy);
            }
            first = false;
            stale = false;
          }
          previous_capture = capture;
          last_frame = host_now;
          ++frames;
          processing_ms += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - host_now).count();
          const double elapsed = std::chrono::duration<double>(host_now - report_start).count();
          if (elapsed >= c.metrics_interval) {
            const std::size_t total = static_cast<std::size_t>(cloud.width) * cloud.height;
            RCLCPP_INFO(get_logger(),
              "depth=%ux%u cloud=%ux%u FPS=%.1f valid=%zu/%zu (%.1f%%) age=%.1fms host=%.2fms XYZ=%.1fMB/s",
              frame->getWidth(), frame->getHeight(), cloud.width, cloud.height, frames / elapsed,
              raw_valid, total, 100.0 * raw_valid / total, age * 1000,
              processing_ms / frames, cloud.xyz.size() * sizeof(float) * frames / elapsed / 1e6);
            frames = 0;
            processing_ms = 0;
            report_start = host_now;
          }
        }
      } catch (const std::exception & e) {
        if (!stopping()) {
          clearCloud();
          status(std::string("ERROR: ") + e.what());
          RCLCPP_ERROR(get_logger(), "%s", e.what());
          for (int i = 0; i < 20 && !stopping() && revision_.load() == revision; ++i) {
            std::this_thread::sleep_for(100ms);
          }
        }
      }
    }
  }

  Config config_;
  std::string device_id_, frame_id_, view_frame_id_;
  std::vector<std::string> parameter_names_;
  std::vector<rclcpp::Parameter> last_parameters_;
  std::mutex config_mutex_;
  std::atomic_bool stop_{false};
  std::atomic<std::uint64_t> revision_{0};
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr parameter_timer_;
};
#undef CLOUD_PARAMETERS
}  // namespace point_cloud

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<point_cloud::PointCloudNode>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("point_cloud"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
