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
  // 검출 방향 보정 (rad, + = 반시계/왼쪽): 박스 중심 시선 방향에 더한다 = 카메라 기준으로 관측을 돌림.
  // 수동 주행 녹화 6회에서 같은 콘을 멀리서 본 위치가 가까이서 본 위치보다 일정하게 왼쪽(약 2.4°)으로 치우쳐 있었다
  double bearing_offset_rad{0.0};
  // 거리 보정: 카메라에서 콘까지 거리 r → r·range_gain + range_offset_m (앞면·중심 모두). 정답 배치 녹화 4회: 0.971, −0.046 m
  double range_gain{1.0}, range_offset_m{0.0};
  LocalizationOptions loc;
};

// box는 깊이 영상 픽셀 좌표. 실패하면 nullopt
std::optional<ConeObservation> localize_box(const std::uint16_t * depth_mm, int stride_px, const DepthIntrinsics & k,
                                            const CameraMount & mount, const cv::Rect2f & box, int class_id, float score,
                                            const Footprint & footprint, const BoxDepthOptions & options);

}  // namespace cone_drive_test
