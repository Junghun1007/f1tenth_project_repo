# bev_processor

## camera_driver 기준 자세 공유

시작 측정이 끝나면 BEV 고정 LUT에 실제로 사용한 roll/pitch를
`camera_optical_frame`의 단위 지면 법선으로 변환해
`/camera/startup_ground_normal`에 한 번 발행한다. QoS는 reliable +
transient-local이므로 같은 launch에서 뒤에 생성되는 `camera_driver`도 값을
수신한다. 기본 자동 설정(`measurement_attitude_source: depth`)에서는 이
벡터가 Depth 평면 법선이며 BEV와 가상 짐벌이 동일한 수치 기준을 사용한다.

통합 launch는 camera_driver의 외부 기준을 필수로 지정한다. 메시지가 없으면
자체 IMU 방향으로 다른 기준을 확정하지 않고 대기하며, 안정화 준비 전 프레임은
기존 zoom-only 정책으로 처리된다.

`camera_driver`가 발행하는 하단 70%의 rectified NV12와 프레임별 IMU 보정
행렬을 받아 CUDA에서 컬러 BEV로 변환하는 ROS 2 C++ 패키지다. 기본 입력은
1280x720의 하단 504행이며, 시작할 때 카메라 높이·roll·하향 pitch를 반드시
측정한다.

## 작동 순서

1. `bev_processor`가 OAK를 먼저 단독으로 연다.
2. 차량이 정지한 상태에서 stereo depth 중앙 ROI의 노면 평면과 IMU 중력
   방향을 측정한다.
3. depth RANSAC/PCA 노면 평면 법선에서 roll과 하향 pitch를 구한다.
4. 같은 depth 노면 평면의 offset 높이를 구하고, 그 값의 시간
   중앙값으로 카메라 높이를 구한다.
5. 측정한 높이·roll·pitch와 설정 파일의 X/Y/yaw로 BEV LUT를 한 번 만든다.
6. OAK 측정 파이프라인을 닫고 `camera_driver`를 시작한다.
7. 카메라 드라이버가 하단 crop과 roll/pitch 보정 행렬을 전달한다.
   `bev_processor`는 시작 LUT, 고정 줌, 동적 보정을 CUDA sampling 한 번으로
   합성해 컬러 BEV를 만든다.

높이와 roll/pitch는 origin과 동일하게 depth 노면 평면의 offset과 법선으로
구한다. IMU는 평면 후보 검증과 정지 상태 판정에 사용한다. 시작 측정에 실패하면 임의의
수동 외부 파라미터로 계속하지 않고 노드 시작을 중단한다. LUT 생성 후에는
BEV 노드가 IMU를 구독하거나 자세 변화에 따라 LUT를 다시 만들지 않는다.

카메라 X/Y 위치와 yaw는 시작 측정으로 구하지 않으므로 실제 장착값을
`config/bev_config.yaml`에 입력해야 한다. 높이·roll·pitch 입력 항목은 없고
시작 측정 결과만 사용한다. 자동 모드의 roll/pitch 출처는 `depth`다.

## 실행

측정이 끝날 때까지 차량을 완전히 정지시키고, 카메라 중앙에 장애물 없는
평평한 노면이 보이게 한다. 이후 기존 `camera_driver`의 1초 IMU 폐기와
4초 기준 자세 측정이 끝날 때까지 계속 정지 상태를 유지한다.

```bash
ros2 launch bev_processor bev_processor.launch.py
```

SSH 터미널에서 GUI 부하 없이 흔들림 보정과 BEV 변환 속도를
측정할 때는 연산 측정 모드를 켠다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  performance_measurement_enabled:=true
```

연산 측정 모드는 카메라와 BEV의 OpenCV GUI 프리뷰를 모두 강제로
끈다. `status_log_interval_sec` 주기마다 다음 형식의 로그가 같은
터미널에 출력된다.

```text
[PERF][CAMERA] capture_fps=80.0 stabilized_fps=79.8 \
stabilized_compute_ms(avg/max)=.../... \
latency_ms(depthai_to_host_avg/max=.../...,\
host_to_stabilized_avg/max=.../...,\
depthai_to_stabilized_avg/max=.../...) ...
[PERF][PIPELINE] stabilized_fps=79.8 bev_ready_fps=79.7 \
processed_fps=79.7 \
latency_ms(depthai_to_bev_input_avg/max=.../...,\
depthai_to_bev_ready_avg/max=.../...,\
bev_input_to_ready_avg/max=.../...) \
bev_compute_ms(avg/max)=... skipped=0 errors(...)=0/0/0
```

- `CAMERA.stabilized_fps`: 통합 launch에서는 하단 NV12+보정 행렬의 발행 속도
- `stabilized_compute_ms`: 통합 launch에서는 보정 행렬 계산과 하단 NV12
  메시지 준비까지의 평균/최대 시간(CPU 전체 프레임 워프는 포함하지 않음)
- `depthai_to_host`: DepthAI 프레임의 `getTimestamp()`부터 Jetson의
  카메라 캡처 스레드가 패킷을 꺼낸 시점까지의 평균/최대 시간
- `host_to_stabilized`: Jetson 패킷 수신부터 흔들림 보정된 NV12
  메시지 준비까지의 평균/최대 시간. 발행 스레드 대기 시간도 포함한다.
- `depthai_to_stabilized`: 같은 DepthAI timestamp부터 흔들림 보정
  메시지 준비까지의 전체 평균/최대 시간
- `PIPELINE.stabilized_fps`: 흔들림 보정 프레임이 BEV 입력
  콜백에 도착한 속도
- `bev_ready_fps`: BEV BGR8 결과가 다음 알고리즘용 ROS 출력으로
  발행 완료된 속도
- `depthai_to_bev_input`: DepthAI timestamp부터 BEV 입력 콜백까지의
  평균/최대 프레임 나이
- `depthai_to_bev_ready`: DepthAI timestamp부터 BEV 발행 완료까지의
  평균/최대 프레임 나이
- `bev_input_to_ready`: BEV 입력 콜백부터 BEV 발행 완료까지
  `steady_clock`으로 직접 잰 평균/최대 시간. timestamp 변환 오차의
  영향을 받지 않는다.
- `bev_compute_ms`: NV12 업로드, CUDA 커널, BGR8 다운로드, CUDA stream
  동기화와 활성화된 차선 재구성까지의 평균/최대 시간

지연 위치는 다음처럼 판별한다.

- `depthai_to_host`가 크면 OAK 내부 출력, USB/XLink 또는 DepthAI
  출력 큐 구간을 우선 확인한다.
- `host_to_stabilized`가 크면 Jetson 발행 스레드 대기와 흔들림 보정
  경로를 확인한다.
- `depthai_to_stabilized`는 작은데 `depthai_to_bev_input`만 크면
  카메라 ROS 발행부터 BEV 콜백 디스패치 구간을 확인한다.
- `bev_input_to_ready`가 크면 BEV 큐, 변환 또는 출력 메시지 준비
  구간을 확인한다.

`depthai_to_bev_ready`는 BEV 노드의 `publish()` 반환 시점까지다.
후속 주행 노드가 실제로 메시지를 받은 시점까지 확인하려면 그 노드의
구독 콜백에서 `now - message.header.stamp`를 추가로 측정한다.

`stabilizer=warmup`인 구간은 측정에서 제외하고 `ready`가 된 다음
값을 확인한다.

사용 파일은 하나씩이다.

- launch: `launch/bev_processor.launch.py`
- BEV 설정: `config/bev_config.yaml`
- 카메라 설정: `camera_driver/config/camera_config.yaml`

launch는 두 노드를 같은 multi-threaded component container에 올리고
intra-process 통신을 사용한다. BEV 시작 측정이 OAK 장치를 반환한 다음
카메라 드라이버가 장치를 연다. 카메라 원본 프리뷰는 끄고 작은 BEV 결과만
프리뷰한다.

## 변환 로직

`CudaBevProcessor`는 다음 처리를 한 커널에서 수행한다.

1. 고정 BEV LUT 좌표를 프레임별 보정 역행렬로 하단 crop 원본 좌표에 매핑한다.
2. 해당 NV12 Y/UV 값을 bilinear 보간한다.
3. YUV를 BGR로 변환하고 `bgr8` BEV를 발행한다.

전체 1280x720 Y/UV를 CPU `warpPerspective()`한 뒤 다시 BEV로 보간하는
중간 영상은 만들지 않는다. crop 비율은 빌드 없이 바꿀 수 있으며 카메라와
BEV에 같은 값이 전달된다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  bev_input_bottom_fraction:=0.70
```

Sobel, 미분 필터, 대비 강화, 밝기 임계값, morphology, 차선 추출과 상단
크롭은 CUDA 변환에는 적용하지 않는다. CUDA 변환 뒤의 선택적 CPU 단계인
`BevLaneReconstructor`가 컬러 BEV를 밝기 기반으로 검사한다.

## 차선 곡선 재구성

기본 설정에서는 두 결과를 동시에 발행한다.

- `/camera/image_bev` (`bgr8`): 변경하지 않은 원본 컬러 BEV
- `/camera/image_bev_lane` (`mono8`): 검은 배경에 흰색 주행 중심선만 그린 결과.
  한쪽 경계만 보이면 설정 폭의 절반만큼 법선 이동해 직접 계산한다.

GUI 프리뷰는 기본적으로 원본 BEV 위에 왼쪽 차선을 파란색, 오른쪽 차선을
빨간색, 주행 중심선을 노란색으로 합성한다. 파랑·빨강은 실제로 측정된
경계에만 표시하며 반대편 가상 차선은 그리지 않는다. 색상 오버레이는
발행 토픽에는 들어가지 않는다.
`lane_preview_enabled:=false`로 실행하면 프리뷰도 원본 BEV만 표시한다.

`lane_preview_sliding_windows_enabled:=true`면 실제 추적에 사용한
회전형 검색 창을 1px 박스로 프리뷰에만 표시한다. 왼쪽 차선 창은
청록색, 오른쪽 차선 창은 자홍색이며 픽셀을 찾지 못한 창은 같은 색의
어두운 테두리로 표시한다. 성능 측정이나 일반 주행 시 박스가 필요 없으면
`lane_preview_sliding_windows_enabled:=false`로 끄면 된다.

기본 `preview_x_origin_m:=0.02`는 차량 좌표 `X=0.02m`에 있는 카메라
위치를 프리뷰 `X=0.0m`로 표시한다. 따라서 현재 3m BEV는 카메라
기준 `X=0.0~2.98m`로 보인다.

`lane_output_lateral_margin_m:=0.70`은 원본 BEV 좌우에 각각 70cm의 검은
픽셀 공간을 추가한다. 원본 컬러 BEV 범위는 중앙에 그대로 표시되고, 한쪽
경계에서 계산한 중심선이 원본 BEV 밖에 있어도 이 공간에 그려진다.
기본 1cm/pixel에서는 차선 결과 폭이 `120 + 70 * 2 = 260px`이고 좌표 범위는
`Y=-1.3~+1.3m`이다. 원본 `/camera/image_bev` 크기는 120x300으로 유지된다.

재구성 단계는 다음 순서로 동작한다.

1. 컬러 BEV를 grayscale 밝기로 변환한다. 1.5m 이후에는 임계값을
   `lane_far_minimum_brightness`까지 점진적으로 낮춰 흐린 픽셀도 남긴다.
2. HSV saturation 80 이하인 흰색·회색 후보만 남겨 컬러 풍경을 제거한다.
3. 후보 양옆 4cm가 어둡고 중심과 배경의 밝기 차가 40 이상인지 검사한다.
4. `0.20~1.00m`의 가까운 행에서 폭 3cm 이하인 좌·우 시작점을 고른다.
   대각선·급커브는 같은 X의 Y 간격 대신 두 실측 곡선의 법선거리로
   차선 폭을 다시 검증한다.
5. 이전 실제 측정점의 방향으로 회전형 슬라이딩 윈도우의 다음 위치만 예측한다.
   시작점이 중간에 선택되면 차량 쪽으로도 같은 윈도우를 진행해
   가까운 실제 픽셀을 다시 찾는다.
6. 윈도우의 밝은 픽셀을 횡방향 덩어리로 나누고, 예측점에 가장 가까우며
   거리별 허용 폭 안에 있는 실제 덩어리 하나만 선택해 위치를 95% 보정한다.
7. 1.5m 이후에는 윈도우 반폭을 12cm에서 최대 22cm까지 넓혀 번진 곡선의
   중앙을 계속 측정한다.
8. 전역 다항식 없이 측정점 85%와 양옆 측정점 각각 7.5%만 섞어 국소 평활화한다.
9. 기본 화면 표시는 temporal hold를 사용하지 않고 매 프레임 현재
   픽셀로 좌·우 실측 경계와 주행 중심선을 다시 계산한다.
10. 법선거리 폭을 통과한 반대편은 `lane_minimum_counterpart_points`
    개만 보여도 중심선 계산에 함께 사용한다. 양쪽이 보이면 한 경계점을
    반대편 실측 곡선에 수직 투영하고 두 실측점의 중점을 중앙선으로 사용한다.
    한쪽 실측 경계가 더 길거나 중간 대응점이 잠시 비면, 그 구간만 더 긴
    경계에서 설정 폭의 절반만큼 법선 이동한 중앙선으로 이어 붙인다.
    좌·우 라벨이 순간 반전되어도 두 경계의 중점은 바뀌지 않는다.
    한쪽만 보이면 그 경계에서 설정 폭의 절반만큼 법선 이동해 중앙선을 만든다.
    이때 양쪽 법선 후보 중 직전 중앙선과 가까운 쪽을 골라 일시적인
    좌·우 오인식이 중앙선을 도로 밖으로 보내지 않게 한다.
11. 실제 마지막 측정점 이후에는 마지막 진행 방향으로 최대 12cm만 예측한다.

기본 `lane_centerline_preserve_reference_shape:=false`는 한쪽만 보일 때 매 프레임
현재 실측 경계의 국소 접선을 구하고, 그 법선 방향으로 설정 폭의 절반인
32.5cm를 이동한다. 차량이 회전해 BEV의 경계 방향이 바뀌면 노란 중심선도
현재 프레임에서 즉시 다시 계산된다. 양쪽이 보일 때는 설정 폭으로 이동한
가상선이 아니라 실측 두 경계의 중점을 쓴다.

`lane_centerline_temporal_current_weight:=0.72`는 양쪽 중점의 현재 프레임을
72% 반영해 작은 프레임 지터를 줄인다. 양쪽에서 한쪽으로 바뀌는 첫 프레임은
직전 중앙선과의 횡방향 차이를 최대 15cm까지 일시 보정한다. 이 보정은 매
프레임 70%만 남기므로 강한 저역통과처럼 실제 횡방향 변화를 계속 숨기지 않는다.
급커브에서도 동일-X 횡방향 이동으로 되돌아가지 않는다. 법선 오프셋이
국소 회전반경보다 커져 접히기 시작하면 기준선과 같은 진행 방향을 유지하는
가장 긴 구간만 표시한다. 그 구간도 최소 점 개수를 만족하지 못하면 잘못된
위치의 가상 경로 대신 해당 프레임의 중심선 출력을 생략한다.

실행하면서 값을 바꾸는 예시는 다음과 같다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  lane_observation_maximum_x_m:=1.30 \
  lane_reconstruction_maximum_x_m:=3.0 \
  lane_maximum_extrapolation_m:=0.0 \
  lane_minimum_brightness:=160 \
  lane_far_minimum_brightness:=110 \
  lane_minimum_local_contrast:=35 \
  lane_sliding_window_measurement_weight:=0.90 \
  lane_expected_width_m:=0.65 \
  lane_width_tolerance_m:=0.08 \
  lane_single_initial_tolerance_m:=0.20 \
  lane_minimum_counterpart_points:=3 \
  lane_centerline_from_single_boundary_enabled:=true \
  lane_output_lateral_margin_m:=0.70 \
  lane_output_line_thickness_m:=0.02 \
  lane_preview_overlay_alpha:=0.8
```

시간 연속성과 한쪽 경계 기반 중심선을 조절하는 예시는 다음과 같다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  lane_temporal_maximum_lateral_jump_near_m:=0.06 \
  lane_temporal_maximum_lateral_jump_far_m:=0.12 \
  lane_temporal_confirmation_frames:=4 \
  lane_temporal_tracking_enabled:=false \
  lane_temporal_hold_frames:=0 \
  lane_centerline_from_single_boundary_enabled:=true \
  lane_centerline_midpoint_smoothing_weight:=0.65 \
  lane_centerline_temporal_current_weight:=0.72 \
  lane_centerline_transition_maximum_correction_m:=0.15 \
  lane_centerline_transition_correction_decay:=0.70 \
  lane_centerline_preserve_reference_shape:=false \
  lane_centerline_tangent_window_m:=0.20 \
  lane_centerline_maximum_curvature_per_m:=1.25 \
  lane_centerline_maximum_heading_step_deg:=8.0
```

원거리 곡선을 놓치면 far 밝기 임계값과 원거리 윈도우 폭을 다음처럼 조절한다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  lane_far_minimum_brightness:=95 \
  lane_sliding_window_half_width_far_m:=0.25
```

다른 밝은 물체를 따라가면 `lane_far_minimum_brightness` 또는
`lane_minimum_local_contrast`를 높이고 `lane_sliding_window_half_width_far_m`나
`lane_tracked_mark_width_far_m`를 줄인다. 컬러 물체가 남으면
`lane_maximum_saturation:=60`처럼 더 엄격하게 설정한다. 실제 차선까지
끊기면 이 값들을 반대 방향으로 한 단계씩 완화한다.

모든 조절 가능한 차선 인자는 다음 명령으로 확인한다.

```bash
ros2 launch bev_processor bev_processor.launch.py --show-args
```

주기 상태 로그의 `lane_compute_ms(avg/max)`는 `reconstruct()`만 측정하므로
CUDA BEV 변환, ROS 발행, GUI 프리뷰 시간을 포함하지 않는다. Jetson에서 이
값을 확인하면 차선 검출 단계의 실제 평균·최대 계산 시간을 바로 알 수 있다.

필터 없는 원본 컬러 프리뷰를 보려면 `lane_preview_enabled:=false`, 차선
재구성을 완전히 끄려면 `lane_reconstruction_enabled:=false`를 사용한다.

## 시작 측정

OAK stereo depth의 중앙 ROI에 RANSAC/PCA 평면을 맞추어 노면 inlier를
찾는다. 정지 상태의 calibrated accel/gyro 1200개를 400Hz로 측정해
노면 후보와 정지 상태를 검증한다. roll/pitch는 depth 노면 법선에서, 높이는 각 depth
프레임에서 RANSAC/PCA로 정밀화한 노면 평면 offset을 구하고, 그 값의
시간 중앙값을 사용한다. 45개 평면의 시간 안정성까지 통과해야 성공한다.
Pro-series OAK의 IR dot projector는 시작 측정 동안 `1.0`으로 사용한다.

높이를 수동으로 쓰려면 `config/bev_config.yaml`에서 다음과 같이
설정한다. 높이는 지면에서 카메라 광학 중심까지의 수직 거리다.

```yaml
manual_camera_height_enabled: true
manual_camera_height_m: 0.20
```

수동 모드에서는 stereo depth 파이프라인·IR projector·노면 평면 측정을
생략한다. 단, roll/pitch를 위한 calibrated IMU 안정화와 bias 보정은
기존과 동일하게 수행한다. `false`면 위 Origin depth 평면 방식으로
자동 측정한다. 수동 높이는 `measurement_minimum_height_m`과
`measurement_maximum_height_m` 범위 내에 있어야 한다.

Stereo는 1280x800, FAST_ACCURACY, LR check(5), confidence threshold 55,
5-bit subpixel을 사용하며 post-processing filter는 적용하지 않는다.
RVC2에서 CAM_A RGB 정렬 depth와 동일한 좌표계를 유지하기 위해 disparity
shift는 0으로 고정한다. Extended disparity도 사용하지 않는다. DepthAI 3.6의
암묵적 AutoCalibration은 사용자 EEPROM을 변경하지 않도록 명시적으로 끈다.

각 측정 파라미터의 선정 방법과 조정 방향은 `config/bev_config.yaml`의
한글 주석에 적혀 있다. IMU 장착 bias는
`measurement_imu_roll_bias_deg`와 `measurement_imu_pitch_bias_deg`로
보정하며, 평평한 기준면에서 반복 측정한 일정한 편차가 확인되기 전에는
0을 유지한다.

정상 시작 로그에는 다음 항목이 출력된다.

```text
[bev_processor] Measuring startup camera height ... roll/pitch ... source ...
[bev_processor] BEV_STARTUP_MEASUREMENT: height_source=..., attitude_source=..., height=..., roll=..., ...
[bev_processor] Startup IMU: ...
[bev_processor] Startup attitude selection: selected=..., ...
[bev_processor] Startup ground-plane diagnostics: ...
[bev_processor] BEV LUT installed from depth-plane attitude + depth-plane offset height: ...
```

상태 로그의 `extrinsics=startup_measured, fixed_lut=true`는 시작 측정 자세의
고정 LUT를 사용 중이라는 뜻이다.

## BEV 범위

BEV 범위와 현재 값은 `config/bev_config.yaml`에서 관리한다. 출력 크기는
다음 식과 일치해야 한다.

```text
output_width  = round((y_max_m - y_min_m) / meter_per_pixel)
output_height = round((x_max_m - x_min_m) / meter_per_pixel)
```

## 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select camera_driver bev_processor \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

차선 재구성 합성 테스트만 실행하려면 다음을 사용한다.

```bash
colcon test --packages-select bev_processor \
  --ctest-args -R bev_lane_reconstructor_test --output-on-failure
```

CUDA 컴파일러를 자동으로 찾지 못하면
`-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`를 추가한다.
