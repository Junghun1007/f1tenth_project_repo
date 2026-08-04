# bev_processor

`camera_driver`가 IMU로 안정화해 발행하는 전체 1280x720 NV12 영상을
CUDA에서 컬러 BEV로 변환하는 ROS 2 C++ 패키지다. 실행 모드는 하나이며,
시작할 때 카메라 높이·roll·하향 pitch를 반드시 측정한다.

## 작동 순서

1. `bev_processor`가 OAK를 먼저 단독으로 연다.
2. 차량이 정지한 상태에서 stereo depth 중앙 ROI의 노면 평면과 IMU 중력
   방향을 측정한다.
3. depth RANSAC/PCA 노면 평면 법선에서 roll과 하향 pitch를 구한다.
4. 같은 depth 노면 평면의 offset 높이를 구하고, 그 값의 시간
   중앙값으로 카메라 높이를 구한다.
5. 측정한 높이·roll·pitch와 설정 파일의 X/Y/yaw로 BEV LUT를 한 번 만든다.
6. OAK 측정 파이프라인을 닫고 `camera_driver`를 시작한다.
7. 카메라 드라이버가 roll/pitch 흔들림을 영상에서 보정하고,
   `bev_processor`는 시작 LUT를 바꾸지 않은 채 컬러 BEV 변환만 수행한다.

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
[PERF][CAMERA] capture_fps=110.0 stabilized_fps=109.8 \
stabilized_compute_ms(avg/max)=.../... \
latency_ms(depthai_to_host_avg/max=.../...,\
host_to_stabilized_avg/max=.../...,\
depthai_to_stabilized_avg/max=.../...) ...
[PERF][PIPELINE] stabilized_fps=109.8 bev_ready_fps=109.6 \
processed_fps=109.6 \
latency_ms(depthai_to_bev_input_avg/max=.../...,\
depthai_to_bev_ready_avg/max=.../...,\
bev_input_to_ready_avg/max=.../...) \
bev_compute_ms(avg/max)=... skipped=0 errors(...)=0/0/0
```

- `CAMERA.stabilized_fps`: 흔들림 보정된 NV12 프레임의 발행 속도
- `stabilized_compute_ms`: 흔들림 보정 homography 적용과 NV12 출력
  메시지 준비까지의 평균/최대 시간
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

`CudaBevProcessor`는 다음 처리만 수행한다.

1. BEV LUT 좌표에서 NV12 Y/UV 값을 bilinear 보간한다.
2. YUV를 BGR로 변환한다.
3. LUT 기반 BEV 워핑 결과를 `bgr8`로 발행한다.

Sobel, 미분 필터, 대비 강화, 밝기 임계값, morphology, 차선 추출과 상단
크롭은 CUDA 변환에는 적용하지 않는다. CUDA 변환 뒤의 선택적 CPU 단계인
`BevLaneReconstructor`가 컬러 BEV를 밝기 기반으로 검사한다.

## 차선 곡선 재구성

기본 설정에서는 두 결과를 동시에 발행한다.

- `/camera/image_bev` (`bgr8`): 변경하지 않은 원본 컬러 BEV
- `/camera/image_bev_lane` (`mono8`): 검은 배경에 흰색 좌·우 차선만 다시 그린 결과

GUI 프리뷰는 기본적으로 원본 BEV 위에 검출 차선을 옅은 파란색, 투명도
`0.35`로만 합성한다. 이 오버레이는 위 두 발행 토픽에는 들어가지 않는다.
`lane_preview_enabled:=false`로 실행하면 프리뷰도 원본 BEV만 표시한다.

재구성 단계는 다음 순서로 동작한다.

1. 컬러 BEV를 grayscale 밝기로 변환한다. 1.8m 이후에는 임계값을
   `lane_far_minimum_brightness`까지 점진적으로 낮춰 흐린 픽셀도 남긴다.
2. HSV saturation 100 이하인 흰색·회색 후보만 남겨 컬러 풍경을 제거한다.
3. 후보 양옆 5cm가 어둡고 중심과 배경의 밝기 차가 25 이상인지 검사한다.
4. `0.20~1.00m`의 가까운 행에서 폭 10cm 이하인 좌·우 시작점을 고른다.
5. 이전 실제 측정점의 방향으로 회전형 슬라이딩 윈도우의 다음 위치만 예측한다.
6. 윈도우의 밝은 픽셀을 횡방향 덩어리로 나누고, 예측점에 가장 가까우며
   거리별 허용 폭 안에 있는 실제 덩어리 하나만 선택해 위치를 90% 보정한다.
7. 1.8m 이후에는 윈도우 반폭을 12cm에서 최대 22cm까지 넓혀 번진 곡선의
   중앙을 계속 측정한다.
8. 전역 다항식 없이 측정점 85%와 양옆 측정점 각각 7.5%만 섞어 국소 평활화한다.
9. 실제 마지막 측정점 이후에는 마지막 진행 방향으로 최대 20cm만 연장한다.

실제로 검출된 좌·우 차선은 고정 차선 폭 모델로 덮어쓰지 않는다. 한쪽 전체가
검출되지 않았을 때만 실제로 검출된 반대쪽 차선의 법선 방향으로 누락된 쪽을
생성한다. 두세 개 창에서 픽셀이 연속으로 없으면 추적을 종료하므로 원거리까지
차선을 임의로 만들어 내지 않는다.

실행하면서 값을 바꾸는 예시는 다음과 같다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  lane_observation_maximum_x_m:=1.8 \
  lane_reconstruction_maximum_x_m:=2.7 \
  lane_maximum_extrapolation_m:=0.2 \
  lane_minimum_brightness:=160 \
  lane_far_minimum_brightness:=110 \
  lane_minimum_local_contrast:=25 \
  lane_sliding_window_measurement_weight:=0.90 \
  lane_expected_width_m:=0.625 \
  lane_output_line_thickness_m:=0.02 \
  lane_preview_overlay_alpha:=0.35
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
`lane_maximum_saturation:=80`처럼 더 엄격하게 설정한다. 실제 차선까지
끊기면 이 값들을 반대 방향으로 한 단계씩 완화한다.

모든 조절 가능한 차선 인자는 다음 명령으로 확인한다.

```bash
ros2 launch bev_processor bev_processor.launch.py --show-args
```

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
