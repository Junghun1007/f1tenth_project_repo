#include "cone_drive_test/core/visibility.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace cone_drive_test {

std::vector<int> expected_visible(const std::vector<Vec2> & cones, const CameraGeometry & g, double size) {
  const double tan_h = std::tan(g.hfov_rad / 2), tan_v = tan_h * g.aspect;
  const double roi_top = 1.0 - g.roi_height_ratio;   // 화면 위에서부터 비율
  auto row = [&](double dist, double z) {            // 화면 세로 위치 0(위)~1(아래)
    const double down = std::atan2(g.height - z, dist);
    return 0.5 + std::tan(down - g.pitch_rad) / (2 * tan_v);
  };
  struct Item { int i; double near, a0, a1, frac; };
  std::vector<Item> items;
  const double h = size / 2;
  for (int i = 0; i < int(cones.size()); ++i) {
    const double x = cones[i].x - g.camera_x, y = cones[i].y;
    const double near = x - h;
    if (near <= 0.02 || x > g.max_range) continue;
    double a0 = 1e9, a1 = -1e9;
    for (double dx : {-h, h}) for (double dy : {-h, h}) { const double a = std::atan2(y + dy, x + dx); a0 = std::min(a0, a); a1 = std::max(a1, a); }
    const double lim = std::atan(tan_h);
    if (a0 < -lim || a1 > lim) continue;            // 화각 밖으로 잘림
    const double base = row(near, 0.0), top = row(x, g.cone_height);
    if (base > 1.0) continue;                         // 차량 바로 앞 사각
    // 세로로 ROI 안에 든 면적 비율 (밑변 13 cm → 윗변 3 cm 사다리꼴, 행에 대해 폭이 선형이라고 근사)
    const double span = std::max(base - top, 1e-9);
    const double cut = std::clamp((roi_top - top) / span, 0.0, 1.0);   // 위쪽 잘린 높이 비율
    const double wb = 1.0, wt = 0.03 / 0.13;
    auto area = [&](double u) { return wt * u + (wb - wt) * u * u / 2; };   // 위에서 u까지 면적(폭 wt→wb)
    const double frac = 1.0 - area(cut) / area(1.0);
    items.push_back({i, near, a0, a1, frac});
  }
  std::sort(items.begin(), items.end(), [](const Item & a, const Item & b) { return a.near < b.near; });
  std::vector<int> out;
  std::vector<std::pair<double, double>> covers;
  for (const auto & it : items) {
    double hidden = 0;
    for (const auto & c : covers) hidden += std::max(0.0, std::min(it.a1, c.second) - std::max(it.a0, c.first));
    const double vis = std::max(0.0, 1.0 - hidden / std::max(it.a1 - it.a0, 1e-9));
    if (vis * it.frac >= g.threshold) out.push_back(it.i);
    covers.emplace_back(it.a0, it.a1);
  }
  std::sort(out.begin(), out.end());
  return out;
}

}  // namespace cone_drive_test
