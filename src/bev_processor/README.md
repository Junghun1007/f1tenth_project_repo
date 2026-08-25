# bev_processor

`bev_processor`는 `camera_driver`가 발행하는 하단 rectified NV12와 프레임별
IMU 보정 행렬을 받아 CUDA에서 120x300 컬러 BEV를 생성하고, 같은 CUDA
stream에서 Gray와 거리별 white Top-hat까지 처리한다. CPU는 작은 BEV에서
상대 대비 기반 왼쪽·오른쪽 차선 시드만 선택한다.

## 처리 순서

1. 시작 시 OAK stereo depth와 IMU로 카메라 높이·roll·pitch를 측정한다.
2. 측정 자세와 설정된 카메라 X/Y/yaw로 고정 BEV LUT를 만든다.
3. OAK 측정 파이프라인을 닫고 `camera_driver`를 시작한다.
4. CUDA가 NV12 안정화, BEV sampling, BGR/Gray 생성을 한 번에 수행한다.
5. CUDA가 BEV의 far/middle/near 영역에 서로 다른 커널의 Top-hat을 적용한다.
6. CPU가 ROI 안의 ridge를 곡선 트랙으로 연결하고 국소 대비·기울기·곡선
   길이·좌우 시드 간 거리로 시드를 선택한다.

차량은 시작 측정이 완료될 때까지 정지해야 한다. 시작 측정 후 차선 시드
검출은 매 프레임 현재 BEV 좌표에서 수행하므로 차량 이동 자체를 고정된
영상 위치로 가정하지 않는다.

## 실행

저장소 루트에서 빌드하고 실행한다.

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select camera_driver bev_processor \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch bev_processor bev_processor.launch.py
```

CAN과 주행 중 가속도계 보정을 모두 끄고 gyro 고주파 진동만 억제하려면:

```bash
ros2 launch bev_processor bev_processor.launch.py \
  preview_enabled:=true \
  imu_stabilization_enabled:=true \
  imu_stabilization_high_frequency_only:=true \
  imu_stabilization_high_frequency_vibration_cutoff_hz:=3.0 \
  imu_stabilization_gyroscope_correction_gain:=1.0 \
  imu_stabilization_invalid_correction_hold_frames:=2
```

`cutoff_hz`를 높이면 더 빠른 진동만 보정하고, 낮추면 느린 흔들림까지
포함한다. `gyroscope_correction_gain`은 `0.0`이면 영상 회전 보정을 적용하지
않고 `1.0`이면 추정된 고주파 회전을 전량 상쇄한다. 권장 시작값은
`cutoff=3.0 Hz`, `gain=1.0`이다. 이 모드에서는 CAN dynamics가 필요 없다.

CAN 차량 가속도 보정을 사용하려면 먼저 다른 터미널에서
`vehicle_dynamics_monitor`를 SocketCAN 모드로 실행한다.

```bash
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py \
  input_mode:=socketcan can_interface:=can0 can_controller_id:=112

# 다른 터미널
ros2 launch bev_processor bev_processor.launch.py \
  imu_stabilization_can_longitudinal_compensation_gain:=0.7 \
  imu_stabilization_can_lateral_compensation_gain:=0.7 \
  imu_stabilization_moving_accelerometer_nudge_strength:=0.15 \
  imu_stabilization_moving_gravity_anchor_maximum_correction_rate_degps:=0.50 \
  imu_stabilization_invalid_correction_hold_frames:=2
```

BEV의 빠른 진동 보정은 기존과 동일한 gyro 전체 대역 방식을 유지한다. CAN
종·횡가속도를 IMU 가속도에서 제거한 잔여 중력은 주행 자세 drift를 줄이는
저주파 persistent anchor와 비누적 bounded nudge에 사용한다. 두 CAN gain과
nudge 강도는 `0.0~1.0`, anchor 최대 변화율은 `deg/s` 단위로 실행할 때
조절할 수 있다. RGB/IMU 매칭이 한두 프레임 실패하면 직전 정상 보정 행렬을
유지하고, 지정 횟수를 넘긴 연속 실패에만 zoom-only로 돌아간다.

CUDA 컴파일러를 자동으로 찾지 못하면 빌드 인자에
`-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`를 추가한다.

GUI 없이 성능을 확인하려면 다음과 같이 실행한다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  performance_measurement_enabled:=true
```

`performance_measurement_enabled`가 true이면 카메라와 BEV GUI가 모두
꺼진다. 상태 로그의 `CPU_seed_ms(avg/max)`는 CPU 시드 검출 시간이며,
Gray·Top-hat은 CUDA BEV 처리 시간에 포함된다.

## CUDA 전처리

`lane_seed_detection_enabled:=true`이면 BGR과 Gray를 같은 BEV 커널에서
만든 뒤 far/middle/near 영역에 설정된 morphology 커널을 적용한다.

```text
enhanced = max(top_hat - noise_floor, 0) * gain
```

기본값은 다음과 같다.

- near: 비율 0.45, gain 1.5, noise floor 17, kernel 7x7
- middle: 비율 0.35, gain 1.6, noise floor 13, kernel 17x17
- far: 비율 0.20, gain 1.65, noise floor 11, kernel 27x27
- Top-hat shape 1(ellipse), iteration 1, border 0(constant)

세 거리 비율의 합은 반드시 1이어야 하며 커널 폭·높이는 양의 홀수여야 한다.
`lane_seed_detection_enabled:=false`이면 CUDA Top-hat, CPU 시드 검출,
시드 토픽 발행을 모두 끈다.

## 상대 대비 시드 검출

보수적인 ROI 시드 검출을 먼저 수행하고, 안정 트랙만 슬라이딩 윈도우로
연장한 뒤 주행 중심선을 생성한다. 전체 순서는 다음과 같다.

1. ROI 각 행에서 Top-hat 응답과 폭 조건을 통과한 ridge를 찾는다.
2. 하단에서 상단으로 최대 횡이동량과 허용 gap을 적용해 곡선 트랙을 만든다.
3. 중앙값 기울기가 급변하는 번개 모양 구간은 분리한다.
4. 원본 Gray에서 ridge 법선 양쪽의 배경을 샘플링해 양측 국소 대비와
   배경 비대칭을 관찰한다. 이 단계는 대비를 추가로 강화하지 않는다.
5. 엄격한 대비와 최소 곡선 길이를 먼저 통과한 안정 트랙에만 선택적
   대비 완화를 적용한다.
6. 좌우 트랙의 공통 행에서 간격을 구하고 MAD 이상치를 제거한 평균이
   설정 거리 범위 안인 시드 쌍을 선택한다.
7. 최초 유효한 좌·우 쌍으로 역할을 잠그고, 이후 단일 차선은 화면
   중심이 아닌 직전 시드 곡선과의 거리로 좌·우를 유지한다.
   양쪽이 모두 설정 프레임 동안 사라져야 잠금을 초기화한다.
8. 매 프레임 행·열 방향을 모두 탐색한 뒤 후보를 하나로 통합한다.
   응답값·폭·gap·이동량·곡선 길이·국소 대비·기울기 기준은 두
   방향에서 동일하며, 좌·우 거리는 원본 좌표계의 곡선 간 거리로 평가한다.
9. 행·열 후보의 끝점이 설정 거리 이내에서 닿고 연결부 응답과 회전각
   기준을 통과하면 하나의 `RC` 트랙으로 이어서 점수와 시드를 다시 계산한다.
10. 선택된 트랙의 먼 쪽 끝에서 크기가 증가하는 회전 슬라이딩 윈도우를
    쌓고 각 창의 Top-hat 밝기 가중 무게중심 한 점을 추가한다.
11. ROI 시드와 슬라이딩 윈도우 점을 호 길이 간격으로 균일 재샘플링한다.
12. 양쪽 차선은 국소 접선에 수직인 대응점을 찾아 중점을 만들고 실측
    도로 폭과 좌·우 법선 방향의 단독 중심 오차를 갱신한다.
13. 한쪽만 보이면 기억한 도로 폭의 절반만큼 국소 법선 방향으로 이동하고,
    양쪽에서 한쪽으로 전환될 때 남는 위치·방향 차이를 짧게 제한한다.
14. 차량 가까운 점부터 짧은 구간을 직선 또는 접선 연속 Hermite
    곡선으로 피팅해 누적한다. 진행 반전, 회전량 급변, 자기교차를
    발견하면 이후 원거리 구간은 연결하지 않는다.

출력 토픽은 다음과 같다.

- `/camera/image_bev` (`bgr8`): 원본 120x300 컬러 BEV
- `/camera/image_bev_lane` (`mono8`): 최종 주행 중심선만 흰색인
  120x300 마스크. 양쪽 경계 자체는 이 토픽에 포함하지 않는다.

## GUI 표시

`lane_preview_enabled:=true`이면 enhanced Top-hat 위에 ROI, 시드,
판정 근거를 함께 표시한다.

- 노랑: ROI 상단·하단
- 흐린 초록: 엄격 대비 통과 근거
- 밝은 초록 2px 선: 최종 주행 중심선
- 주황: 안정 트랙에서 완화된 대비로 통과한 근거
- 청록: 선택된 왼쪽 시드와 지지 곡선
- 자홍: 선택된 오른쪽 시드와 지지 곡선
- 빨간 X: 기울기 급변으로 분리된 지점
- `WAIT L+R`: 최초 좌우 시드 쌍을 기다리는 상태
- `SIDE LOCK`: 좌우 역할 잠금, `SIDE LOCK:T`는 현재 프레임에
  시간 연속성으로 단일 차선의 역할을 붙였다는 뜻
- 상태 문자 끝의 `:RC`: 행·열 통합 추적 사용
- 흰색 원: 행·열 트랙이 실제로 병합된 연결 위치
- `CENTER:PAIR/LEFT/RIGHT/NONE`: 중심선 생성에 사용한 차선 상태,
  끝의 `:T`는 양쪽에서 한쪽으로 전환 보정 중이라는 뜻

`lane_preview_enabled:=false`이면 원본 컬러 BEV만 프리뷰한다.
프리뷰 창에 포커스를 둔 채 Space를 누르면 `preview_stop_topic`
(기본 `/auto/enabled`)으로 false를 발행한다.

## 실행 시 파라미터 변경

모든 시드·Top-hat 파라미터는 `config/bev_config.yaml`에 있고,
launch 인자로 실행마다 덮어쓸 수 있다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  lane_seed_roi_height_ratio:=0.40 \
  lane_seed_minimum_bilateral_contrast:=25.0 \
  lane_seed_maximum_slope_change_px_per_row:=2.0 \
  lane_seed_pair_minimum_distance_px:=50.0 \
  lane_seed_pair_maximum_distance_px:=95.0
```

전체 조절 인자는 다음 명령으로 확인한다.

```bash
ros2 launch bev_processor bev_processor.launch.py --show-args
```

슬라이딩 윈도우 값은 실행할 때 바로 덮어쓸 수 있다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  lane_seed_sliding_window_growth_ratio:=1.12 \
  lane_seed_sliding_window_maximum_turn_deg_per_window:=15.0 \
  lane_seed_sliding_window_maximum_turn_change_deg_per_window:=5.0 \
  lane_seed_sliding_window_heading_update_gain:=0.50
```

중심선 폭·구간 피팅·전환값도 같은 방법으로 조절한다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  lane_centerline_nominal_lane_width_px:=62.0 \
  lane_centerline_fit_segment_length_px:=12.0 \
  lane_centerline_fit_straight_maximum_residual_px:=1.5 \
  lane_centerline_fit_maximum_turn_deg_per_segment:=28.0 \
  lane_centerline_fit_maximum_turn_change_deg_per_segment:=10.0 \
  lane_centerline_transition_frames:=4 \
  lane_centerline_maximum_lateral_jump_px:=3.0
```

주요 파라미터 그룹:

- `lane_near_*`, `lane_middle_*`, `lane_far_*`:
  거리별 Top-hat gain, noise floor, kernel 크기와 영역 비율
- `lane_seed_roi_*`: 하단 제외 비율과 ROI 높이
- `lane_seed_minimum_response`, `*_run_width_px`:
  행별 ridge 후보의 Top-hat 응답과 폭
- `lane_seed_maximum_lateral_step_px`, `*_gap_rows`:
  곡선 트랙의 좌우 이동량과 누락 run/고립된 대비 실패 허용량
- `lane_seed_minimum_track_arc_length_px`:
  행 개수가 아닌 후보 곡선의 최소 실제 길이
- `lane_seed_minimum_bilateral_contrast`,
  `lane_seed_maximum_background_asymmetry`:
  원본 Gray에서 측정하는 양측 대비와 배경 균형
- `lane_seed_contrast_relaxation_*`:
  안정 트랙에만 적용하는 단계적 대비 완화
- `lane_seed_slope_*`:
  급격한 기울기 변화 억제
- `lane_seed_pair_*_distance_px`:
  MAD 이상치 제거 후 좌우 시드 평균 간격
- `lane_seed_sliding_window_enabled`,
  `lane_seed_sliding_window_minimum_seed_arc_length_px`:
  기존 ROI 검출을 통과한 안정 트랙만 먼 쪽 끝에서 슬라이딩 윈도우로
  연장할지 여부와 연장을 시작할 최소 곡선 길이
- `lane_seed_sliding_window_initial_*`,
  `lane_seed_sliding_window_growth_ratio`,
  `lane_seed_sliding_window_maximum_*`:
  첫 창의 폭/높이, 창마다 적용할 증가 비율, 최대 폭/높이. 원거리에서
  굵어지는 차선에 맞춰 폭과 높이가 함께 증가한다.
- `lane_seed_sliding_window_step_ratio`,
  `lane_seed_sliding_window_maximum_count`:
  현재 창 높이 대비 다음 창 이동 거리와 한 차선당 최대 창 개수
- `lane_seed_sliding_window_minimum_bright_pixels`,
  `lane_seed_sliding_window_maximum_consecutive_misses`:
  창 안에서 무게중심을 계산할 최소 밝은 픽셀 수와 연결 중 허용할 빈 창 수
- `lane_seed_sliding_window_maximum_turn_deg_per_window`:
  직전 진행 방향에서 다음 창으로 허용할 최대 회전각
- `lane_seed_sliding_window_maximum_turn_change_deg_per_window`,
  `lane_seed_sliding_window_heading_update_gain`:
  연속 창 사이 회전량 급변 제한과 방향 갱신 비율. 코너 방향과 반대로
  순간 진동하는 중심선 생성을 억제한다.
- `lane_centerline_enabled`, `lane_centerline_nominal_lane_width_px`,
  `lane_centerline_width_update_gain`:
  중심선 생성 여부, 양쪽 차선을 보기 전 사용할 기본 폭, 양쪽 차선에서
  실측한 도로 폭과 단일 차선별 중심 bias의 갱신 비율
- `lane_centerline_resample_spacing_px`:
  ROI 시드와 성긴 슬라이딩 윈도우 점을 동일 비중으로 만들 재샘플 간격
- `lane_centerline_outlier_distance_px`:
  이웃을 잇는 선분에서 벗어난 고립 중심점을 교정할 거리
- `lane_centerline_fit_segment_length_px`,
  `lane_centerline_fit_tail_window_px`:
  가까운 쪽부터 순차적으로 확정할 피팅 구간 길이와 구간 끝
  접선을 계산할 관측 길이
- `lane_centerline_fit_straight_maximum_residual_px`,
  `lane_centerline_fit_straight_maximum_heading_deg`:
  현재 직선을 그대로 연장할 최대 중앙 잔차와 방향 차이
- `lane_centerline_fit_maximum_residual_px`,
  `lane_centerline_fit_hermite_tangent_scale`:
  구간 곡선과 관측점의 최대 중앙 잔차, Hermite 접선 길이 비율
- `lane_centerline_fit_maximum_turn_deg_per_segment`,
  `lane_centerline_fit_maximum_turn_change_deg_per_segment`,
  `lane_centerline_fit_minimum_forward_progress_ratio`:
  구간당 최대 회전각, 인접 구간 회전량 변화 제한, 직전 진행
  방향으로 반드시 남아야 하는 최소 전진 비율. 자기교차와 되돌아가는
  선분은 파라미터와 관계없이 발행하지 않는다.
- `lane_centerline_transition_frames`,
  `lane_centerline_maximum_lateral_jump_px`,
  `lane_centerline_maximum_heading_jump_deg`:
  양쪽에서 한쪽 차선으로 전환될 때 허용할 짧은 보정 기간과 최초
  중심선 횡이동·방향 변화 한계
- `lane_seed_column_tracking_enabled`:
  행 후보와 함께 동일 기준의 열 방향 후보를 매 프레임 통합할지 여부
- `lane_seed_cross_direction_merge_enabled`:
  인접한 같은 방향 조각과, 끝점이 닿거나 run 영역이 겹치는
  행·열 후보를 하나의 트랙으로 이을지 여부
- `lane_seed_cross_direction_merge_maximum_endpoint_distance_px`:
  병합을 허용할 두 끝점의 최대 거리
- `lane_seed_cross_direction_merge_minimum_connector_support_ratio`:
  접합할 두 지점 사이에서 Top-hat 임계값을 통과해야 하는 최소 비율
- `lane_seed_cross_direction_merge_maximum_turn_angle_deg`:
  연결부에서 허용할 최대 방향 변화각
- `lane_seed_temporal_side_lock_*`:
  유효한 좌우 쌍의 역할 재초기화, 단일 차선의 짧은 누락 후 좌/우 역할
  유지, 잠금 초기화 프레임, 마지막 유효 곡선에서 허용할 기본 거리,
  누락 프레임당 추가 거리와 최대 거리. 단일 차선과 좌우 쌍 모두 이 동적
  거리 범위 안에서만 재연결된다.

차선 프리뷰에는 선택된 좌·우 색으로 회전 슬라이딩 윈도우가 표시된다.
통과한 창의 노란 점은 Top-hat 응답으로 계산한 밝기 가중 무게중심이며,
빨간 창은 밝은 픽셀 부족 또는 진행 방향 조건으로 중단된 창이다. 최종
중심선은 프리뷰의 밝은 초록색 선과 mono8 마스크로 출력된다.

## BEV 범위

BEV 출력 크기는 다음 식과 일치해야 한다.

```text
output_width  = round((y_max_m - y_min_m) / meter_per_pixel)
output_height = round((x_max_m - x_min_m) / meter_per_pixel)
```

기본 범위는 전방 0~3m, 좌우 -0.6~+0.6m, 1cm/pixel이며 출력은
120x300이다. `bev_interpolation`은 `bilinear`(기본),
`bicubic`, `adaptive` 중 하나를 사용한다.

## 시작 측정과 자세 공유

자동 모드는 OAK stereo depth 중앙 ROI의 노면에 RANSAC/PCA 평면을 맞춰
높이·roll·pitch를 구한다. IMU는 평면 후보 검증과 정지 상태 판정에
사용한다. 측정에 실패하면 임의 외부 파라미터로 계속하지 않고 노드 시작을
중단한다.

측정 후 사용한 지면 법선은 `/camera/startup_ground_normal`에
reliable + transient-local QoS로 한 번 발행한다. `camera_driver`도
같은 기준을 받아 프레임별 안정화 행렬을 만든다. LUT 생성 후에는 주행 중
자세 변화에 따라 LUT를 다시 만들지 않으며, 동적 보정 행렬이 CUDA sampling에
매 프레임 반영된다.

높이를 수동으로 쓰려면 `config/bev_config.yaml`에서 다음 값을
설정한다.

```yaml
manual_camera_height_enabled: true
manual_camera_height_m: 0.20
```

수동 높이 모드에서도 roll/pitch를 위한 IMU 안정화와 bias 보정은 수행한다.
카메라 X/Y/yaw는 자동 측정 대상이 아니므로 실제 장착값을 설정해야 한다.

사용 파일:

- launch: `launch/bev_processor.launch.py`
- BEV/시드 설정: `config/bev_config.yaml`
- 카메라 설정: `camera_driver/config/camera_config.yaml`
