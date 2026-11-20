#include "camera_driver/tunnel_frame_stats.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace camera_driver {

namespace {
using Histogram = std::array<std::uint64_t, 256>;

// 정렬했을 때 k번째(0부터) 값
int kth(const Histogram & h, std::uint64_t k) {
  std::uint64_t seen = 0;
  for (int v = 0; v < 256; ++v) {
    seen += h[v];
    if (seen > k) return v;
  }
  return 255;
}

// numpy.percentile(..., method='linear')
double percentile(const Histogram & h, std::uint64_t n, double q) {
  if (n == 0) return 0.0;
  const double pos = (n - 1) * q / 100.0;
  const auto i = static_cast<std::uint64_t>(std::floor(pos));
  const double frac = pos - double(i);
  const int a = kth(h, i);
  if (frac <= 0.0 || i + 1 >= n) return a;
  return a + frac * (kth(h, i + 1) - a);
}

void add(Histogram & h, const cv::Mat & m, double & sum, std::uint64_t & n) {
  for (int r = 0; r < m.rows; ++r) {
    const auto * p = m.ptr<std::uint8_t>(r);
    for (int c = 0; c < m.cols; ++c) { ++h[p[c]]; sum += p[c]; }
  }
  n += std::uint64_t(m.rows) * m.cols;
}
}  // namespace

cv::Mat tunnel_gray_from_luma(const std::uint8_t * y, int width, int height, std::size_t stride) {
  static const cv::Mat lut = [] {
    cv::Mat t(1, 256, CV_8UC1);
    for (int v = 0; v < 256; ++v) {   // np.rint(np.clip((v - 16) * 255/219, 0, 255)) — 짝수 쪽 반올림
      const float x = std::clamp((float(v) - 16.f) * (255.f / 219.f), 0.f, 255.f);
      t.at<std::uint8_t>(0, v) = static_cast<std::uint8_t>(std::nearbyint(x));
    }
    return t;
  }();
  const cv::Mat luma(height, width, CV_8UC1, const_cast<std::uint8_t *>(y), stride);
  cv::Mat small, gray;
  cv::resize(luma, small, {640, 400}, 0, 0, cv::INTER_AREA);
  cv::LUT(small, lut, gray);
  return gray;
}

TunnelFrameStats tunnel_frame_stats(const cv::Mat & g) {
  TunnelFrameStats s;
  const int h = g.rows, w = g.cols, ya = h / 3, yb = 2 * h / 3, xa = w / 3, xb = 2 * w / 3;
  Histogram all{}, center{}, surround{};
  double sum_all = 0, sum_center = 0, sum_surround = 0;
  std::uint64_t n_all = 0, n_center = 0, n_surround = 0;
  add(all, g, sum_all, n_all);
  add(center, g(cv::Range(ya, yb), cv::Range(xa, xb)), sum_center, n_center);
  add(surround, g(cv::Range(0, ya), cv::Range(xa, xb)), sum_surround, n_surround);
  add(surround, g(cv::Range(ya, yb), cv::Range(0, xa)), sum_surround, n_surround);
  add(surround, g(cv::Range(ya, yb), cv::Range(xb, w)), sum_surround, n_surround);
  s.mean = n_all ? sum_all / n_all : 0.0;
  double var = 0;
  for (int v = 0; v < 256; ++v) var += all[v] * (v - s.mean) * (v - s.mean);
  s.std = n_all ? std::sqrt(var / n_all) : 0.0;
  s.center_mean = n_center ? sum_center / n_center : 0.0;
  s.surround_mean = n_surround ? sum_surround / n_surround : 0.0;
  s.p10 = percentile(all, n_all, 10);
  s.p90 = percentile(all, n_all, 90);
  const double q25 = percentile(surround, n_surround, 25), med = percentile(surround, n_surround, 50);
  const double q75 = percentile(surround, n_surround, 75);
  const double threshold = med - 0.5 * std::max(q75 - q25, 2.0);
  std::uint64_t dark = 0;
  for (int v = 0; v < 256; ++v) if (v < threshold) dark += center[v];
  s.center_dark_fraction = n_center ? double(dark) / n_center : 0.0;
  return s;
}

}  // namespace camera_driver
