# 터널 TCN 입력 프레임 통계 (tunnel_tcn_test features.frame_stats와 같은 값, 0~255 밝기 단위).
# header.stamp = 촬영 시각 (원본 /tunnel/rgb_raw 영상과 같음)
std_msgs/Header header
float32 mean
float32 std
float32 center_mean
float32 surround_mean
float32 p10
float32 p90
float32 center_dark_fraction
