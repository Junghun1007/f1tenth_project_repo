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
  // 지면 거리 검사: 박스 아래 끝(콘이 바닥에 닿는 곳)의 시선을 지면에 투영한 거리와 깊이로 잰 거리가
  // ground_check_ratio배 넘게 다르고 차이도 ground_check_min_m보다 크면 버린다 (0 = 끔).
  // 화면 가장자리에서 깊이가 엉터리로 가깝게 나와 먼 콘을 차 바로 앞에 찍는 경우를 막는다
  double ground_check_ratio{1.6};
  double ground_check_min_m{0.3};
  LocalizationOptions loc;
};

// 지면 거리 검사 결과 (진단용)
enum class GroundCheck { Passed, Skipped, Rejected };

// box는 깊이 영상 픽셀 좌표. 실패하면 nullopt
// bottom_visible = false (박스 아래 끝이 다른 박스에 가려짐)이면 지면 거리 검사를 건너뛴다: 가려진 콘은 아래 끝이 위로 올라가
// 지면 거리가 실제보다 멀게 나온다. ground(있으면)에 검사 결과를 남긴다
std::optional<ConeObservation> localize_box(const std::uint16_t * depth_mm, int stride_px, const DepthIntrinsics & k,
                                            const CameraMount & mount, const cv::Rect2f & box, int class_id, float score,
                                            const Footprint & footprint, const BoxDepthOptions & options,
                                            bool bottom_visible = true, GroundCheck * ground = nullptr);

// 박스 아래 끝 가운데 픽셀의 시선이 지면(z = 0)에 닿는 곳까지 카메라 바닥 위치에서의 수평 거리. 지면에 안 닿으면 nullopt
std::optional<double> ground_range_at_bottom(const DepthIntrinsics & k, const CameraMount & mount, const cv::Rect2f & box);

}  // namespace cone_drive_test
