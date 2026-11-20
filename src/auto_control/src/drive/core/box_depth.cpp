// Copied from src/cone_drive_test/src/core/box_depth.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/box_depth.hpp"
#include "auto_control/drive/copied/bev_geometry.hpp"
#include <algorithm>
#include <cmath>

namespace auto_control::drive {

namespace {
void collect(const std::uint16_t * d, int stride, const DepthIntrinsics & k, const cv::Matx33d & R, const CameraMount & m,
             const cv::Rect & r, const BoxDepthOptions & o, std::vector<ObjectPoint> & out) {
  for (int v = r.y; v < r.y + r.height; v += o.pixel_stride) {
    for (int u = r.x; u < r.x + r.width; u += o.pixel_stride) {
      const std::uint16_t mm = d[std::size_t(v) * stride + u];
      const double z = mm * 0.001;
      if (mm == 0 || z < o.min_depth_m || z > o.max_depth_m) continue;
      const cv::Vec3d p = R * cv::Vec3d((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z);
      const double h = m.height + p[2];
      if (h < o.min_height_m || h > o.max_height_m) continue;
      out.push_back({m.x + p[0], m.y + p[1]});
    }
  }
}
}  // namespace

std::optional<double> ground_range_at_bottom(const DepthIntrinsics & k, const CameraMount & m, const cv::Rect2f & box) {
  const auto R = mountRotationVehicleFromCamera(m.roll_rad, m.pitch_down_rad, m.yaw_rad);
  const cv::Vec3d ray = R * cv::Vec3d((box.x + box.width / 2 - k.cx) / k.fx, (box.y + box.height - k.cy) / k.fy, 1.0);
  if (ray[2] > -1e-6) return std::nullopt;   // 지평선 위
  const double t = m.height / -ray[2];
  return t * std::hypot(ray[0], ray[1]);
}

std::optional<ConeObservation> localize_box(const std::uint16_t * depth_mm, int stride_px, const DepthIntrinsics & k,
                                            const CameraMount & m, const cv::Rect2f & box, int class_id, float score,
                                            const Footprint & footprint, const BoxDepthOptions & o,
                                            bool bottom_visible, GroundCheck * ground) {
  if (ground) *ground = GroundCheck::Skipped;
  const cv::Rect image(0, 0, k.width, k.height);
  const cv::Rect full = cv::Rect(cv::Point(int(std::floor(box.x)), int(std::floor(box.y))),
                                 cv::Point(int(std::ceil(box.x + box.width)), int(std::ceil(box.y + box.height)))) & image;
  if (full.empty() || !depth_mm) return std::nullopt;
  const double f = std::clamp(o.box_core_fraction, 0.05, 1.0);
  const cv::Rect core = cv::Rect(int(std::round(box.x + box.width * (1 - f) / 2)), int(std::round(box.y + box.height * (1 - f) / 2)),
                                 std::max(1, int(std::round(box.width * f))), std::max(1, int(std::round(box.height * f)))) & image;
  const auto R = mountRotationVehicleFromCamera(m.roll_rad, m.pitch_down_rad, m.yaw_rad);
  std::vector<ObjectPoint> pts;
  if (!core.empty()) collect(depth_mm, stride_px, k, R, m, core, o, pts);
  if (pts.size() < o.loc.min_points) { pts.clear(); collect(depth_mm, stride_px, k, R, m, full, o, pts); }
  // 방향: 박스 중심 픽셀의 시선을 지면에 투영
  const cv::Vec3d ray = R * cv::Vec3d((box.x + box.width / 2 - k.cx) / k.fx, (box.y + box.height / 2 - k.cy) / k.fy, 1.0);
  const double bearing = std::atan2(ray[1], ray[0]) + o.bearing_offset_rad;
  const auto est = localize(pts, m.x, m.y, bearing, footprint, o.loc);
  if (!est.valid) return std::nullopt;
  // 지면 거리 검사. 박스가 화면 아래 끝에 닿으면 콘 바닥이 안 보이므로 건너뛴다.
  // 지면 거리는 먼 곳에서 자세 오차에 민감하므로 깊이 최대 거리로 자른다 (지평선 위도 그 거리로 본다)
  if (o.ground_check_ratio > 1.0 && bottom_visible && box.y + box.height < k.height - 2.0) {
    const double g = std::min(ground_range_at_bottom(k, m, box).value_or(o.max_depth_m), o.max_depth_m);
    const double d = est.range;
    const bool bad = std::max(g, d) > o.ground_check_ratio * std::min(g, d) && std::abs(g - d) > o.ground_check_min_m;
    if (ground) *ground = bad ? GroundCheck::Rejected : GroundCheck::Passed;
    if (bad) return std::nullopt;
  }
  // 거리 보정 (카메라 바닥 위치 기준으로 같은 방향을 따라 늘이거나 줄임)
  const auto rescale = [&](double x, double y) {
    const double dx = x - m.x, dy = y - m.y, r = std::hypot(dx, dy);
    if (r < 1e-6) return Vec2{x, y};
    const double k = std::max(0.0, r * o.range_gain + o.range_offset_m) / r;
    return Vec2{m.x + dx * k, m.y + dy * k};
  };
  ConeObservation c;
  c.center = rescale(est.center_x, est.center_y);
  c.front = rescale(est.front_x, est.front_y);
  c.bearing = est.bearing;
  c.range = std::max(0.0, est.range * o.range_gain + o.range_offset_m);
  c.class_id = class_id;
  c.score = score;
  c.length = footprint.length;
  c.width = footprint.width;
  return c;
}

}  // namespace auto_control::drive
