#include "depth_lidar/depth_lidar_preview.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace depth_lidar
{

cv::Mat makeRadarPreview(const ScanProjection & projection,
  const int width_px, const double max_range_m)
{
  if (width_px < 240 || !std::isfinite(max_range_m) || max_range_m <= 0.0) {
    throw std::invalid_argument("radar preview requires width >= 240 and positive finite range");
  }
  constexpr double pi = 3.14159265358979323846;
  const int radius_px = width_px / 2 - 38;
  const cv::Point origin(width_px / 2, radius_px + 64);
  cv::Mat image(origin.y + 68, width_px, CV_8UC3, cv::Scalar(255, 255, 255));
  const double pixels_per_meter = radius_px / max_range_m;
  const cv::Scalar grid(210, 210, 210);
  const cv::Scalar ink(65, 65, 65);
  const cv::Scalar point_color(190, 90, 0);
  const auto pointAt = [&](double range_m, double angle) {
    return cv::Point(origin.x - static_cast<int>(std::lround(
                       range_m * std::sin(angle) * pixels_per_meter)),
      origin.y - static_cast<int>(std::lround(range_m * std::cos(angle) * pixels_per_meter)));
  };
  const auto labelAt = [&](const std::string & label, cv::Point point, double scale) {
    int baseline = 0;
    const auto size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline);
    point.x = std::clamp(point.x, 2, std::max(2, width_px - size.width - 2));
    cv::putText(image, label, point, cv::FONT_HERSHEY_SIMPLEX, scale, ink, 1, cv::LINE_AA);
  };

  for (int ring = 1; ring <= 5; ++ring) {
    const double range_m = max_range_m * ring / 5.0;
    const int radius = static_cast<int>(std::lround(range_m * pixels_per_meter));
    cv::ellipse(image, origin, cv::Size(radius, radius), 0.0, 180.0, 360.0,
      grid, 1, cv::LINE_AA);
    std::ostringstream label;
    label << std::fixed << std::setprecision(max_range_m < 1.0 ? 2 : 1) << range_m << "m";
    if (width_px >= 600 || ring == 5 || (width_px >= 400 && ring % 2 == 0)) {
      labelAt(label.str(), cv::Point(origin.x + radius + 3, origin.y - 4), 0.32);
    }
  }
  for (int degrees = -90; degrees <= 90; degrees += 30) {
    const double angle = degrees * pi / 180.0;
    cv::line(image, origin, pointAt(max_range_m, angle), grid, 1, cv::LINE_AA);
    if (std::abs(degrees) != 90) {
      const cv::Point end = pointAt(max_range_m, angle);
      const std::string label = degrees == 0 ? "0" :
        (degrees > 0 ? "+" : "") + std::to_string(degrees);
      labelAt(label, end + cv::Point(-8, -7), 0.35);
    }
  }
  // The scan only covers the camera ROI's horizontal field of view.
  for (const double angle : {static_cast<double>(projection.angle_min),
      static_cast<double>(projection.angle_max)})
  {
    if (std::isfinite(angle)) {
      cv::line(image, origin, pointAt(max_range_m, angle),
        cv::Scalar(175, 155, 130), 1, cv::LINE_AA);
    }
  }
  std::size_t displayed_points = 0;
  for (std::size_t i = 0; i < projection.ranges.size(); ++i) {
    const double range = projection.ranges[i];
    const double angle = projection.angle_min + static_cast<double>(i) * projection.angle_increment;
    if (!std::isfinite(range) || range <= 0.0 || range > max_range_m
        || !std::isfinite(angle) || std::abs(angle) > pi / 2.0)
    {
      continue;
    }
    cv::circle(image, pointAt(range, angle), 2, point_color, cv::FILLED, cv::LINE_AA);
    ++displayed_points;
  }
  cv::drawMarker(image, origin, ink, cv::MARKER_CROSS, 10, 2, cv::LINE_AA);
  labelAt(width_px < 400 ? "0m" : "CAMERA / 0m",
    origin + cv::Point(width_px < 400 ? -8 : -43, 17), 0.35);
  labelAt(width_px < 400 ? "L (+)" : "LEFT (+)", cv::Point(8, origin.y + 17), 0.35);
  labelAt(width_px < 400 ? "R (-)" : "RIGHT (-)",
    cv::Point(width_px - (width_px < 400 ? 40 : 75), origin.y + 17), 0.35);
  labelAt("DEPTH SCAN / CAMERA FRAME", cv::Point(10, 20), width_px < 400 ? 0.36 : 0.5);
  labelAt("POINTS " + std::to_string(displayed_points) + " | FORWARD UP",
    cv::Point(10, 38), 0.35);
  if (displayed_points == 0U) {
    labelAt("NO VALID RETURNS", cv::Point(origin.x - 60, origin.y - radius_px / 2), 0.4);
  }
  return image;
}

} // namespace depth_lidar
