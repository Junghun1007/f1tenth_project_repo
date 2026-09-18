#pragma once

#include "point_cloud/depth_projection.hpp"
#include "point_cloud/ground_filter.hpp"
#include "point_cloud/bev_crop.hpp"
#include "point_cloud/obstacle_clusters.hpp"
#include "point_cloud/temporal_filter.hpp"
#include "point_cloud/blob_filter.hpp"
#include <map>

#include <depthai/depthai.hpp>

#include <memory>
#include <string>
#include <utility>

namespace point_cloud
{
struct Config
{
  std::string resolution{"400p"};
  double fps{50.0};
  std::string mode{"high_density"};
  double dot_intensity{0.5};
  double flood_intensity{0.0};
  int confidence{200};
  bool lr_check{true};
  int lr_threshold{10};
  bool subpixel{false};
  int subpixel_bits{3};
  bool extended{false};
  std::string median{"off"};
  bool spatial{false};
  bool speckle{false};
  bool hole_filling{false};
  bool adaptive_median{false};
  ProjectionOptions projection;
  GroundOptions ground;
  BevOptions bev;
  ClusterOptions cluster;
  TemporalOptions temporal;
  BlobOptions blob;
  bool publish_depth{true};
  double max_age_sec{0.5};
  double metrics_interval{1.0};
};

std::pair<std::uint32_t, std::uint32_t> resolutionSize(const std::string & resolution);
void validate(const Config & config);

// Owns one device session. Closing/reopening also reapplies both IR intensities.
class DepthSource
{
public:
  DepthSource(const Config & config, const std::string & device_id);
  ~DepthSource();
  std::shared_ptr<dai::ImgFrame> tryGet();
  std::string deviceId() const;
  RigidTransform rgbFromFrame(dai::ImgFrame & frame);

private:
  std::map<dai::CameraBoardSocket, RigidTransform> rgb_from_reference_;
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> queue_;
};
}  // namespace point_cloud
