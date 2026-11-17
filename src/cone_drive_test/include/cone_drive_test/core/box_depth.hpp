// YOLO 박스 안 깊이 픽셀만 차량 좌표로 역투영해 콘 위치·크기 추정 (multiclass_yolo 방식, 전체 점구름 없음)
#pragma once
#include <cstdint>
#include <optional>
#include <opencv2/core.hpp>
#include "cone_drive_test/copied/object_localization.hpp"
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

struct DepthIntrinsics { double fx, fy, cx, cy; int width, height; };   // RGB에 정렬된 깊이 영상
struct CameraMount { double x, y, height, roll_rad, pitch_down_rad, yaw_rad; };   // 앞차축 기준 차량 좌표

struct BoxDepthOptions {
  double box_core_fraction{0.6};   // 박스 가운데 비율 (가장자리에는 배경이 섞임)
  int pixel_stride{1};
  double min_height_m{0.03};       // 지면 제거
  double max_height_m{1.0};
  double min_depth_m{0.15}, max_depth_m{4.0};
  LocalizationOptions loc;
};

// box는 깊이 영상 픽셀 좌표. 실패하면 nullopt
std::optional<ConeObservation> localize_box(const std::uint16_t * depth_mm, int stride_px, const DepthIntrinsics & k,
                                            const CameraMount & mount, const cv::Rect2f & box, int class_id, float score,
                                            const Footprint & footprint, const BoxDepthOptions & options);

}  // namespace cone_drive_test
