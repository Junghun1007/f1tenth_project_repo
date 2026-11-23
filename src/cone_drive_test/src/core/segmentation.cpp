#include "cone_drive_test/core/segmentation.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/dnn.hpp>

namespace cone_drive_test {
namespace {
float half(std::uint16_t bits) {
  return static_cast<float>(cv::float16_t::fromBits(bits));
}
}  // namespace

SegTransform seg_transform(const cv::Rect & roi) {
  const double scale = std::min(double(seg_width) / roi.width, double(seg_height) / roi.height);
  const int w = std::clamp(int(std::nearbyint(roi.width * scale)), 1, seg_width);
  const int h = std::clamp(int(std::nearbyint(roi.height * scale)), 1, seg_height);
  return {roi, {(seg_width - w) / 2, (seg_height - h) / 2, w, h}};
}

cv::Rect2f SegTransform::source_box(const cv::Rect2f & box) const {
  const double sx = double(roi.width) / content.width, sy = double(roi.height) / content.height;
  const cv::Rect2f source(roi.x + (box.x - content.x) * sx, roi.y + (box.y - content.y) * sy,
                          box.width * sx, box.height * sy);
  return source & cv::Rect2f(roi);
}

cv::Point SegTransform::model_pixel(double rgb_x, double rgb_y) const {
  return {int(std::floor(content.x + (rgb_x - roi.x) * content.width / roi.width)),
          int(std::floor(content.y + (rgb_y - roi.y) * content.height / roi.height))};
}

std::vector<SegDetection> decode_seg(const std::uint16_t * packed, const SegTransform & transform,
                                     const std::array<float, 4> & thresholds, float nms_iou) {
  std::vector<SegDetection> result;
  for (int cls = 0; cls < 4; ++cls) {
    std::vector<cv::Rect2d> model_boxes;
    std::vector<float> scores;
    std::vector<int> anchors;
    for (int i = 0; i < seg_anchors; ++i) {
      const auto value = [&](int channel) { return half(packed[channel * seg_anchors + i]); };
      const float score = value(4 + cls);
      if (!std::isfinite(score) || score <= 0 || score < thresholds[cls]) continue;
      int best = 0;
      for (int j = 1; j < 4; ++j) if (value(4 + j) > value(4 + best)) best = j;
      if (best != cls) continue;
      const float cx = value(0), cy = value(1), w = value(2), h = value(3);
      if (!std::isfinite(cx) || !std::isfinite(cy) || !(w > 0 && h > 0) || !std::isfinite(w + h)) continue;
      const auto box = cv::Rect2f(cx - w / 2, cy - h / 2, w, h) & cv::Rect2f(0, 0, seg_width, seg_height);
      if (box.empty() || transform.source_box(box).empty()) continue;
      model_boxes.emplace_back(box);
      scores.push_back(score);
      anchors.push_back(i);
    }
    std::vector<int> keep;
    cv::dnn::NMSBoxes(model_boxes, scores, 0.0f, nms_iou, keep);
    for (int i : keep) {
      const cv::Rect2f box(model_boxes[i]);
      SegDetection det{transform.source_box(box), box, scores[i], cls};
      for (int k = 0; k < seg_channels; ++k) det.coefficients[k] = half(packed[(8 + k) * seg_anchors + anchors[i]]);
      result.push_back(det);
    }
  }
  std::stable_sort(result.begin(), result.end(), [](const auto & a, const auto & b) { return a.score > b.score; });
  if (result.size() > 50) result.resize(50);
  return result;
}

bool mask_contains(const std::uint16_t * prototypes, const SegDetection & det, cv::Point p) {
  const auto & box = det.model_box;
  if (p.x < box.x || p.y < box.y || p.x >= box.x + box.width || p.y >= box.y + box.height) return false;
  const float x = std::clamp((p.x + .5f) / 4 - .5f, 0.f, float(proto_width - 1));
  const float y = std::clamp((p.y + .5f) / 4 - .5f, 0.f, float(proto_height - 1));
  const int x0 = int(x), y0 = int(y), x1 = std::min(x0 + 1, proto_width - 1), y1 = std::min(y0 + 1, proto_height - 1);
  const float wx = x - x0, wy = y - y0;
  float corners[4]{};
  for (int k = 0; k < seg_channels; ++k) {
    const auto * plane = prototypes + k * proto_width * proto_height;
    const float c = det.coefficients[k];
    corners[0] += c * half(plane[y0 * proto_width + x0]);
    corners[1] += c * half(plane[y0 * proto_width + x1]);
    corners[2] += c * half(plane[y1 * proto_width + x0]);
    corners[3] += c * half(plane[y1 * proto_width + x1]);
  }
  const float top = corners[0] + (corners[1] - corners[0]) * wx;
  const float bottom = corners[2] + (corners[3] - corners[2]) * wx;
  return top + (bottom - top) * wy > 0;
}

}  // namespace cone_drive_test
