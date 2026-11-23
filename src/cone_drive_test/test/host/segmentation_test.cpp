#include "check.hpp"
#include "cone_drive_test/core/segmentation.hpp"

using namespace cone_drive_test;

TEST(segmentation_geometry_recovers_roi_and_centered_padding) {
  const auto full = seg_transform({0, 0, 1280, 800});
  CHECK(full.content == cv::Rect(64, 0, 512, 320));
  CHECK(full.source_box({64, 0, 512, 320}) == cv::Rect2f(0, 0, 1280, 800));
  CHECK(full.model_pixel((0 + .5) * 2, (0 + .5) * 2) == cv::Point(64, 0));
  const auto cropped = seg_transform({0, 224, 1280, 576});
  CHECK(cropped.content == cv::Rect(0, 16, 640, 288));
  CHECK(cropped.model_pixel((639 + .5) * 2, (399 + .5) * 2) == cv::Point(639, 303));
  CHECK(cropped.source_box({0, 16, 640, 288}) == cv::Rect2f(0, 224, 1280, 576));
}

TEST(segmentation_uses_class_nms_and_only_the_candidate_pixel) {
  std::vector<std::uint16_t> packed(seg_output_values);
  const auto set = [&](int channel, int anchor, float value) {
    packed[channel * seg_anchors + anchor] = cv::float16_t(value).bits();
  };
  for (int i = 0; i <= 1; ++i) {
    set(0, i, 320); set(1, i, 160); set(2, i, 80); set(3, i, 60);
    set(4, i, i == 0 ? .9f : .7f);
    set(8, i, 1.f);
  }
  const auto detections = decode_seg(packed.data(), seg_transform({0, 0, 1280, 800}), {.5f, .5f, .5f, .5f}, .65f);
  CHECK(detections.size() == 1 && detections[0].class_id == 0);
  auto * proto = packed.data() + seg_prediction_values;
  for (int y = 0; y < proto_height; ++y) for (int x = 0; x < proto_width; ++x)
    proto[y * proto_width + x] = cv::float16_t(x >= 80 ? 1.f : -1.f).bits();
  CHECK(mask_contains(proto, detections[0], {330, 160}));
  CHECK(!mask_contains(proto, detections[0], {310, 160}));
  CHECK(!mask_contains(proto, detections[0], {400, 160}));
}
