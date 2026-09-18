#pragma once
#include "roi_lidar/scan.hpp"
#include <opencv2/core.hpp>
#include <string>
namespace roi_lidar
{
cv::Point mapPixel(double x, double y, const Options & o, int size);
cv::Mat mapPreview(const Scan & scan, const Options & o, int size, const std::string & status, const cv::Mat & background = {});
class BevProjector
{
  cv::Mat map_x_, map_y_;
public:
  void configure(int width, int height, const point_cloud::Intrinsics & k,
    const point_cloud::RigidTransform & vehicle_from_camera, const Options & o, int size);
  cv::Mat render(const cv::Mat & bgr) const;
};
cv::Mat roiPreview(const cv::Mat & gray_or_depth, const Options & o, bool depth);
}
