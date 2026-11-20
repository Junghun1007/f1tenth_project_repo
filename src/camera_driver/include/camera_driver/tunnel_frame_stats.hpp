// 터널 TCN 입력용 프레임 통계 (tunnel_tcn_test/features.py frame_stats와 같은 계산).
// 원본 NV12(1.5 MB)를 30 Hz로 다른 프로세스에 보내는 대신 숫자 7개만 보낸다.
//   gray = Y 평면 → 640×400 INTER_AREA → BT.601 제한 범위 → 전체 범위 밝기(LUT)  (display.mono_image와 같음)
//   가운데 = 가로·세로 1/3~2/3, 둘레 = 가운데 위·왼쪽·오른쪽 칸 (모서리·아래 칸 제외)
//   백분위는 numpy 기본(선형 보간)과 같게 히스토그램으로 계산
#pragma once
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>

namespace camera_driver {

struct TunnelFrameStats {
  double mean{0}, std{0}, center_mean{0}, surround_mean{0}, p10{0}, p90{0}, center_dark_fraction{0};
};

cv::Mat tunnel_gray_from_luma(const std::uint8_t * y, int width, int height, std::size_t stride);
TunnelFrameStats tunnel_frame_stats(const cv::Mat & gray);   // CV_8UC1

}  // namespace camera_driver
