#include "cone_drive_test/core/box_depth.hpp"
#include "cone_drive_test/copied/bev_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace cone_drive_test {

namespace {
struct ProjectedPoint { ObjectPoint position; double height; };

std::optional<ProjectedPoint> project(std::uint16_t mm, int u, int v, const DepthIntrinsics & k,
                                      const cv::Matx33d & r, const CameraMount & m, const BoxDepthOptions & o) {
  const double z = mm * 0.001;
  if (mm == 0 || z < o.min_depth_m || z > o.max_depth_m) return std::nullopt;
  const auto p = r * cv::Vec3d((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z);
  return ProjectedPoint{{m.x + p[0], m.y + p[1]}, m.height + p[2]};
}

void collect(const std::uint16_t * d, int stride, const DepthIntrinsics & k, const cv::Matx33d & R, const CameraMount & m,
             const cv::Rect & r, const BoxDepthOptions & o, std::vector<ObjectPoint> & out, DepthWork * work) {
  if (work) {
    const int count = ((r.width + o.pixel_stride - 1) / o.pixel_stride) * ((r.height + o.pixel_stride - 1) / o.pixel_stride);
    work->candidates += count;
    work->reads += count;
  }
  for (int v = r.y; v < r.y + r.height; v += o.pixel_stride) {
    for (int u = r.x; u < r.x + r.width; u += o.pixel_stride) {
      const auto p = project(d[std::size_t(v) * stride + u], u, v, k, R, m, o);
      if (p && p->height >= o.min_height_m && p->height <= o.max_height_m) out.push_back(p->position);
    }
  }
}

cv::Rect image_box(const cv::Rect2f & box, const DepthIntrinsics & k) {
  return cv::Rect(cv::Point(int(std::floor(box.x)), int(std::floor(box.y))),
                  cv::Point(int(std::ceil(box.x + box.width)), int(std::ceil(box.y + box.height)))) &
         cv::Rect(0, 0, k.width, k.height);
}

ConeObservation observation(const ObjectEstimate & est, const CameraMount & m, const BoxDepthOptions & o,
                           int class_id, float score, const Footprint & footprint, double bearing_adjust = 0) {
  const auto rescale = [&](double x, double y) {
    const double dx = x - m.x, dy = y - m.y, r = std::hypot(dx, dy);
    if (r < 1e-6) return Vec2{x, y};
    const double gain = std::max(0.0, r * o.range_gain + o.range_offset_m) / r;
    const double c = std::cos(bearing_adjust), s = std::sin(bearing_adjust);
    return Vec2{m.x + (dx * c - dy * s) * gain, m.y + (dx * s + dy * c) * gain};
  };
  return {rescale(est.center_x, est.center_y), rescale(est.front_x, est.front_y), est.bearing + bearing_adjust,
          std::max(0.0, est.range * o.range_gain + o.range_offset_m), class_id, score, footprint.length, footprint.width};
}

bool differs_from_ground(double ground, const ObjectEstimate & est, const BoxDepthOptions & o) {
  return std::max(ground, est.range) > o.ground_check_ratio * std::min(ground, est.range) &&
         std::abs(ground - est.range) > o.ground_check_min_m;
}

// A coarse grid has at most the first-pass quota of disjoint cells. Subsequent passes
// visit different pixels in each cell, using a step coprime with its area.
class Grid {
public:
  Grid(cv::Rect box, int quota) {
    if (box.empty()) return;
    quota = std::clamp(quota, 1, box.area());
    int cols = std::clamp(int(std::lround(std::sqrt(quota * double(box.width) / box.height))),
                           1, std::min(box.width, quota));
    const int rows = std::min(box.height, std::max(1, quota / cols));
    cols = std::min(box.width, std::max(cols, quota / rows));
    for (int row = 0; row < rows; ++row) for (int col = 0; col < cols; ++col) {
      const int x = col * box.width / cols, y = row * box.height / rows;
      const int w = (col + 1) * box.width / cols - x, h = (row + 1) * box.height / rows - y;
      const int area = w * h;
      int step = std::max(1, int(.618 * area));
      while (std::gcd(step, area) != 1) --step;
      cells_.push_back({box.x + x, box.y + y, w, area, (h / 2) * w + w / 2, step});
    }
    remaining_ = box.area();
  }

  std::optional<cv::Point> next() {
    if (remaining_ == 0) return std::nullopt;
    for (;;) {
      auto & cell = cells_[cursor_];
      cursor_ = (cursor_ + 1) % cells_.size();
      if (cell.remaining == 0) continue;
      const int n = cell.index;
      cell.index = (n + cell.step) % cell.area;
      --cell.remaining;
      --remaining_;
      return cv::Point(cell.x + n % cell.w, cell.y + n / cell.w);
    }
  }
  bool exhausted() const { return remaining_ == 0; }

private:
  struct Cell {
    int x, y, w, area, index, step;
    int remaining{area};
  };
  std::vector<Cell> cells_;
  std::size_t cursor_{};
  int remaining_{};
};
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
                                            bool bottom_visible, GroundCheck * ground, DepthWork * work) {
  if (ground) *ground = GroundCheck::Skipped;
  const cv::Rect image(0, 0, k.width, k.height);
  const cv::Rect full = image_box(box, k);
  if (full.empty() || !depth_mm) return std::nullopt;
  const double f = std::clamp(o.box_core_fraction, 0.05, 1.0);
  const cv::Rect core = cv::Rect(int(std::round(box.x + box.width * (1 - f) / 2)), int(std::round(box.y + box.height * (1 - f) / 2)),
                                 std::max(1, int(std::round(box.width * f))), std::max(1, int(std::round(box.height * f)))) & image;
  const auto R = mountRotationVehicleFromCamera(m.roll_rad, m.pitch_down_rad, m.yaw_rad);
  std::vector<ObjectPoint> pts;
  if (!core.empty()) collect(depth_mm, stride_px, k, R, m, core, o, pts, work);
  if (pts.size() < o.loc.min_points) { pts.clear(); collect(depth_mm, stride_px, k, R, m, full, o, pts, work); }
  // 방향: 박스 중심 픽셀의 시선을 지면에 투영
  const cv::Vec3d ray = R * cv::Vec3d((box.x + box.width / 2 - k.cx) / k.fx, (box.y + box.height / 2 - k.cy) / k.fy, 1.0);
  const double bearing = std::atan2(ray[1], ray[0]) + o.bearing_offset_rad;
  const auto est = localize(pts, m.x, m.y, bearing, footprint, o.loc);
  if (!est.valid) return std::nullopt;
  // 지면 거리 검사. 박스가 화면 아래 끝에 닿으면 콘 바닥이 안 보이므로 건너뛴다.
  // 지면 거리는 먼 곳에서 자세 오차에 민감하므로 깊이 최대 거리로 자른다 (지평선 위도 그 거리로 본다)
  if (o.ground_check_ratio > 1.0 && bottom_visible && box.y + box.height < k.height - 2.0) {
    const double g = std::min(ground_range_at_bottom(k, m, box).value_or(o.max_depth_m), o.max_depth_m);
    const bool bad = differs_from_ground(g, est, o);
    if (ground) *ground = bad ? GroundCheck::Rejected : GroundCheck::Passed;
    if (bad) return std::nullopt;
  }
  return observation(est, m, o, class_id, score, footprint);
}

MaskDepthFrame localize_masks(const std::uint16_t * depth, int stride, const DepthIntrinsics & k,
                              const CameraMount & m, const std::vector<MaskDepthTarget> & targets,
                              const BoxDepthOptions & o, bool trace) {
  constexpr int object_cap = 256, frame_cap = 2048;
  MaskDepthFrame out;
  out.objects.resize(targets.size());
  if (!depth) {
    for (auto & r : out.objects) r.status = MaskDepthStatus::NoDepth;
    return out;
  }
  const auto rotation = mountRotationVehicleFromCamera(m.roll_rad, m.pitch_down_rad, m.yaw_rad);
  const int first = std::min<int>(64, 1024 / std::max<std::size_t>(1, targets.size()));
  struct State {
    cv::Rect rect;
    std::optional<Grid> grid;
    std::vector<ObjectPoint> points;
    std::vector<std::pair<cv::Point, double>> contacts;
    ObjectEstimate estimate;
  };
  std::vector<State> states(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) states[i].rect = image_box(targets[i].box, k);

  const auto sample = [&](std::size_t i, int quota) {
    auto & s = states[i];
    auto & r = out.objects[i];
    const auto & target = targets[i];
    quota = std::min({quota, object_cap - r.work.candidates, frame_cap - out.work.candidates});
    if (quota <= 0 || s.rect.empty()) return false;
    if (!s.grid) s.grid.emplace(s.rect, first);
    std::array<cv::Point, 64> batch;
    std::array<bool, 64> hit;
    int count = 0;
    while (count < quota) {
      auto p = s.grid->next();
      if (!p) break;
      batch[count++] = *p;
    }
    if (!count) return false;
    r.work.candidates += count;
    out.work.candidates += count;
    const double mask_start = now_sec();
    for (int j = 0; j < count; ++j) hit[j] = target.contains(batch[j].x, batch[j].y);
    out.mask_ms += (now_sec() - mask_start) * 1000;
    for (int j = 0; j < count; ++j) {
      const auto p = batch[j];
      std::uint16_t mm = 0;
      if (hit[j]) {
        ++r.work.reads;
        ++out.work.reads;
        mm = depth[std::size_t(p.y) * stride + p.x];
        if (auto projected = project(mm, p.x, p.y, k, rotation, m, o)) {
          if (projected->height >= o.min_height_m && projected->height <= o.max_height_m) s.points.push_back(projected->position);
          if (o.ground_check_ratio > 1.0 && target.bottom_visible && s.rect.br().y < k.height - 2 &&
              p.y >= s.rect.br().y - std::max(2, s.rect.height / 8) &&
              std::abs(projected->height) <= .04) {
            const auto & pos = projected->position;
            s.contacts.emplace_back(p, std::hypot(pos.x - m.x, pos.y - m.y));
          }
        }
      }
      if (trace) out.samples.push_back({p, hit[j], mm != 0});
    }
    s.estimate = localize(s.points, m.x, m.y, std::nullopt, target.footprint, o.loc);
    return true;
  };

  for (std::size_t i = 0; i < targets.size(); ++i) sample(i, first);
  while (out.work.candidates < frame_cap) {
    std::vector<std::size_t> pending;
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const auto & s = states[i];
      if ((!s.estimate.valid || s.estimate.cluster_points < 2 * o.loc.min_points) &&
          (!s.grid || !s.grid->exhausted()) && !s.rect.empty() &&
          out.objects[i].work.candidates < object_cap) pending.push_back(i);
    }
    if (pending.empty()) break;
    const auto priority = [&](std::size_t i) {
      const auto & s = states[i];
      return s.estimate.valid ? s.estimate.range :
        std::hypot((2.0 * s.rect.x + s.rect.width - k.width) / k.width, double(k.height - s.rect.br().y) / k.height);
    };
    std::stable_sort(pending.begin(), pending.end(), [&](auto a, auto b) {
      if (states[a].estimate.valid != states[b].estimate.valid) return states[a].estimate.valid;
      return priority(a) < priority(b);
    });
    for (auto i : pending) sample(i, 32);
  }

  for (std::size_t i = 0; i < targets.size(); ++i) {
    const auto & s = states[i];
    const auto & target = targets[i];
    auto & r = out.objects[i];
    if (!s.estimate.valid) {
      if (!s.rect.empty() && (!s.grid || !s.grid->exhausted()) &&
          (out.work.candidates == frame_cap || r.work.candidates == object_cap)) r.status = MaskDepthStatus::BudgetExhausted;
      continue;
    }
    std::vector<double> contact_ranges;
    for (const auto & [p, range] : s.contacts) {
      const double window = o.loc.window_base_m + o.loc.window_per_m * s.estimate.range;
      if (std::abs(range - s.estimate.range) > window) continue;
      if (auto g = ground_range_at_bottom(k, m, cv::Rect2f(p.x, p.y, 0, 0))) contact_ranges.push_back(*g);
    }
    if (contact_ranges.size() >= 2) {
      auto middle = contact_ranges.begin() + contact_ranges.size() / 2;
      std::nth_element(contact_ranges.begin(), middle, contact_ranges.end());
      if (differs_from_ground(*middle, s.estimate, o)) { r.status = MaskDepthStatus::GroundRejected; continue; }
    }
    r.observation = observation(s.estimate, m, o, target.class_id, target.score, target.footprint, o.bearing_offset_rad);
    r.status = MaskDepthStatus::Ok;
  }
  return out;
}

}  // namespace cone_drive_test
