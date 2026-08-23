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

기존 밝기·채도·슬라이딩 윈도우·중심선 재구성 알고리즘은 제거되었다.
현재 시드 검출 순서는 다음과 같다.

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
8. 행 방향으로 유효한 시드 쌍을 완성하지 못하면 영상을 전치해
   열 방향으로 한 번만 다시 탐색한다. 응답값·폭·gap·이동량·곡선
   길이·국소 대비·기울기 기준은 행 추적과 동일하다.

출력 토픽은 다음과 같다.

- `/camera/image_bev` (`bgr8`): 원본 120x300 컬러 BEV
- `/camera/image_bev_lane` (`mono8`): 선택된 왼쪽·오른쪽
  시드 지지 곡선만 흰색인 120x300 마스크

현재 단계는 시드 탐색까지만 수행한다. 주행 중심선이나 완성된 차선 곡선은
아직 생성하지 않는다.

## GUI 표시

`lane_preview_enabled:=true`이면 enhanced Top-hat 위에 ROI, 시드,
판정 근거를 함께 표시한다.

- 노랑: ROI 상단·하단
- 초록: 엄격 대비 통과 근거
- 주황: 안정 트랙에서 완화된 대비로 통과한 근거
- 청록: 선택된 왼쪽 시드와 지지 곡선
- 자홍: 선택된 오른쪽 시드와 지지 곡선
- 빨간 X: 기울기 급변으로 분리된 지점
- `WAIT L+R`: 최초 좌우 시드 쌍을 기다리는 상태
- `SIDE LOCK`: 좌우 역할 잠금, `SIDE LOCK:T`는 현재 프레임에
  시간 연속성으로 단일 차선의 역할을 붙였다는 뜻
- 상태 문자 끝의 `:C`: 현재 프레임에서 열 방향 fallback 실행

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

주요 파라미터 그룹:

- `lane_near_*`, `lane_middle_*`, `lane_far_*`:
  거리별 Top-hat gain, noise floor, kernel 크기와 영역 비율
- `lane_seed_roi_*`: 하단 제외 비율과 ROI 높이
- `lane_seed_minimum_response`, `*_run_width_px`:
  행별 ridge 후보의 Top-hat 응답과 폭
- `lane_seed_maximum_lateral_step_px`, `*_gap_rows`:
  곡선 트랙 연결 허용량
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
- `lane_seed_column_fallback_enabled`:
  행 방향 시드 쌍 실패 시 동일 기준의 열 방향 탐색 사용 여부
- `lane_seed_temporal_side_lock_*`:
  최초 좌우 쌍 후 단일 차선의 역할 유지, 잠금 초기화 프레임,
  재탐색 최대 곡선 거리

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
