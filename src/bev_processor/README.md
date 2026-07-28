# bev_processor

`camera_driver`의 왜곡 보정된 1280x720 NV12 영상을 CUDA로 BEV 변환하는
ROS 2 C++ 패키지다. 실행 진입점은 자동과 수동 두 개로 분리되어 있다.

## 실행 모드

### `bev_processor_auto`

시작할 때 OAK stereo depth의 노면 점들에 RANSAC과 PCA로 평면을 맞춰
카메라 높이와 노면 법선을 측정하고, 정지 상태에서 평균낸 IMU 중력
법선과 신뢰도 기반으로 융합해 roll과 하향 pitch를 결정한다. 측정값으로
BEV LUT를 생성한 뒤 그 실행이 끝날 때까지 자세를 고정한다. 주행 중
실시간 자세 추정은 하지 않는다.

센서 시작 직후의 과도값을 버리기 위해 1초간 워밍업하고, 100 Hz IMU
200개 샘플과 중앙 320x160 stereo ROI를 사용한다. 2픽셀 간격으로 만든
3D 점들 중 노면 평면 inlier만 사용하고, 안정된 평면 30프레임의 높이와
평균 법선을 구한다. IMU와 Depth가 통계적 허용 범위 안에서 일치하면
법선을 분산 역가중으로 융합한다. 충돌하면 신뢰도가 설정값 이상 우세한
센서만 선택하고, 우세한 센서가 없으면 재측정한다.

자동 측정 중에는 Pro-series OAK의 IR dot projector를 기본 세기 0.5로
켜서 무늬가 적은 노면의 stereo 대응점을 보강한다. 세기는
`measurement_ir_dot_projector_intensity`로 설정하며, 활성화에 실패하면
passive stereo로 조용히 진행하지 않고 측정을 중단한다.

IMU 고정 장착 오차는 `measurement_imu_roll_bias_deg`와
`measurement_imu_pitch_bias_deg`로 보정할 수 있다. bias는
`IMU 측정값 - 신뢰하는 실제값`의 부호로 입력한다. 반복 측정으로
확인하기 전에는 0을 유지한다. 높이는 항상 Depth 평면값을 사용한다.
시작 로그의 `source`는 두 법선을 융합했으면 `imu_depth_fused`, 충돌 후
한 센서를 선택했으면 `depth_selected` 또는 `imu_selected`로 표시된다.

```bash
ros2 launch bev_processor bev_processor_auto.launch.py
```

설정 파일은 `config/bev_config_auto.yaml`이다. 카메라 X/Y 위치와 yaw,
카메라 내부 파라미터, BEV 범위와 출력 크기는 이 파일에서 읽는다.

### `bev_processor_manual`

OAK stereo/IMU 측정을 전혀 실행하지 않는다. 높이, roll, 하향 pitch를
포함한 모든 외부 파라미터를 `config/bev_config_manual.yaml`에서 그대로
읽어 BEV LUT를 생성한다.

```bash
ros2 launch bev_processor bev_processor_manual.launch.py
```

직접 측정값은 다음 항목에 입력한다.

```yaml
camera_x_m: 0.0
camera_y_m: 0.0
camera_z_m: 0.17
camera_roll_deg: 0.0
camera_downward_pitch_deg: 13.0
camera_yaw_deg: 0.0
```

두 launch 모두 `camera_driver`와 BEV를 같은 multi-threaded component
container에서 실행해 NV12 intra-process 경로를 사용한다. 카메라 원본
프리뷰는 끄고 작은 BEV 결과만 프리뷰한다.

## 공통 변환 로직

자동/수동은 C++ 노드나 CUDA 코드를 복제하지 않는다. 두 launch 모두
동일한 `bev_processor::BevProcessorNode`를 로드하며 다음 공통 경로를
사용한다.

1. 선택된 파라미터로 카메라 모델을 완성한다.
2. 동일한 `mountRotationVehicleFromCamera()`와 `generateRemap()`으로
   지면 역투영 LUT를 만든다.
3. 동일한 `CudaBevProcessor`가 NV12 샘플링, 색 변환, BEV 워핑을
   수행한다.

차이는 LUT 생성에 넣는 높이/roll/pitch의 출처뿐이다.

- auto: 시작 시 OAK IMU와 stereo 노면 평면의 적응형 융합값
- manual: `bev_config_manual.yaml`의 직접 측정값

이전의 `startup_measurement_enabled` launch 옵션은 제거했다. 따라서
launch 기본값이 YAML의 모드를 덮어쓸 수 없다. 예전
`camera_bev.launch.py`와 `bev_processor.launch.py`는 오래된 설치 파일의
재실행을 막기 위해 오류 안내만 출력한다. 예전 독립 실행 파일인
`bev_processor_node`도 같은 안내 후 종료한다. 각 YAML에는 필수
`processor_mode`가 있고 노드명도 각각 `bev_processor_auto`,
`bev_processor_manual`로 고정된다. 잘못된 YAML을 넘기면 C++ 기본값으로
조용히 실행하지 않고 즉시 오류로 종료한다.

BEV 파라미터는 launch에서 별도로 덮어쓰지 않는다. 변경 후에는 노드를
재시작해야 하며, 실행 중 parameter 변경으로 LUT가 다시 만들어지지는
않는다.

## 현재 공통 BEV 범위

두 설정 파일의 영상 범위와 해상도는 동일하다.

```yaml
x_min_m: 0.10
x_max_m: 2.5
y_min_m: -0.5
y_max_m: 0.5
meter_per_pixel: 0.01
output_width: 100
output_height: 240
```

계산식은 다음과 같다.

```text
output_width  = (0.5 - (-0.5)) / 0.01 = 100
output_height = (2.5 - 0.10) / 0.01   = 240
```

## 로그 확인

자동 모드에서는 다음 로그가 모두 나와야 한다.

```text
[bev_processor_auto] Measuring startup camera height/roll/pitch ...
[bev_processor_auto] BEV_STARTUP_MEASUREMENT: source=imu_depth_fused, ...
[bev_processor_auto] Startup IMU: raw=..., corrected=...
[bev_processor_auto] Startup attitude fusion: ...
[bev_processor_auto] Startup ground-plane diagnostics: ...
[bev_processor_auto] Startup extrinsics mode=auto, source=OAK adaptive IMU+depth: ...
```

수동 모드에서는 측정 로그가 나오면 안 되고 다음처럼 표시되어야 한다.

```text
[bev_processor_manual] BEV processor mode=manual started: ...
[bev_processor_manual] Startup extrinsics mode=manual, source=manual config: ...
```

상태 로그에도 `mode=auto` 또는 `mode=manual`과 실제 적용 중인
높이/roll/하향 pitch가 출력된다.

`camera_height_estimator`는 별도 도구이며 BEV에 값을 전달하거나 토픽을
발행하지 않는다.

## 빌드

Jetson의 워크스페이스 루트에서 실행한다.

```bash
cd ~/Desktop/f1tenth_test0724/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select camera_driver bev_processor \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

CUDA 컴파일러를 자동으로 찾지 못하면 경로를 지정한다.

```bash
colcon build \
  --packages-select camera_driver bev_processor \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
```

프리뷰나 토픽 발행 여부를 바꾸려면 사용할 auto/manual YAML의
`preview_enabled`, `publish_enabled`, `preview_max_fps`를 수정한다.
