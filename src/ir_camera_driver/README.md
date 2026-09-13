# ir_camera_driver

OAK-D Pro W의 CAM_B/C 모노 카메라와 IR emitter를 사용하는 독립 터널
가시성 튜닝 패키지다. 원본 IR 영상과 고정 지면 BEV를 동시에 최대 30 FPS로
표시하고, 별도 제어 창에서 flood·노출 시간·ISO를 실행 중 직접 입력할 수 있다.
다음 두 실행 모드를 제공한다.

- `reprojection_enabled=true`: 좌우 rectified 영상과 center-aligned disparity를
  이용해 스테레오 광학 중심 사이의 가상 시점 영상 하나를 만든다.
- `reprojection_enabled=false`: `selected_camera`로 선택한 LEFT/CAM_B 또는
  RIGHT/CAM_C 렌즈 영상을 그대로 표시한다.

OAK 장치는 한 프로세스가 배타적으로 연다. 이 패키지를 실행하는 동안
`camera_driver`, `bev_processor`, `camera_height_estimator` 또는
`depthai_ros_driver`를 동시에 실행하면 안 된다.

## 중앙 가상 시점 재투영

DepthAI `StereoDepth`가 공장 캘리브레이션으로 왜곡 보정과 stereo
rectification을 수행하고, disparity를 두 카메라 사이의 지정 위치에 맞춘다.
호스트의 CUDA kernel은 각 가상 픽셀에서 다음 위치의 좌우 밝기를 bilinear
sampling한 뒤 평균한다.

```text
r = virtual_camera_position_ratio
x_left  = x_virtual + r * disparity
x_right = x_virtual - (1-r) * disparity
I_virtual = 0.5 * (I_left(x_left) + I_right(x_right))
```

`r=0.5`는 두 스테레오 렌즈의 정확한 중간이다. disparity가 없거나 영상
경계를 벗어난 픽셀은 `selected_camera` 영상으로 채워 검은 구멍을 피한다.

이 결과가 차량 중심 시점이 되려면 스테레오 렌즈 중간이 차량 중심선에
장착되어 있어야 한다. 좌우 렌즈의 차량 기준 횡방향 위치를 각각
`y_left`, `y_right`로 실측했다면 차량 중심 `y=0`에 해당하는 비율은 다음과
같이 설정할 수 있다.

```text
r = (0 - y_left) / (y_right - y_left)
```

장치 전체가 차량 중심에서 앞뒤로 벗어났거나 yaw/roll/pitch가 틀어진 경우는
이 비율만으로 보정되지 않는다. 그 경우에는 차량 `base_link`에 대한 별도
외부 캘리브레이션이 필요하다. 이번 패키지는 IR 차선 가시성 시험을 위한
스테레오 사이 가상 시점까지만 만든다.

## 원본/BEV 프리뷰

기본 카메라 요청 속도와 원본/BEV 화면 갱신 상한은 모두 30 FPS다. 캡처
스레드는 DepthAI의 크기 1/non-blocking 큐를 계속 비우고 최신 프레임만
공유하므로 GUI가 느려져도 과거 프레임이 쌓이지 않는다. 두 프리뷰는 같은
프레임으로 함께 갱신된다.

- 중앙 재투영: `1280x800 @ 30 FPS`
  - RVC2 800P integer disparity 구성
  - CENTER 정렬에 필수인 left-right check 활성화
  - subpixel과 추가 후처리 필터 비활성화
  - 중앙 합성은 Jetson CUDA에서 수행
- 단일 렌즈: `1280x800 @ 30 FPS`
  - stereo/disparity/CUDA 재투영을 시작하지 않음

BEV는 프레임의 rectified intrinsics와 YAML에 입력한 카메라 높이·장착 자세를
이용해 평면 노면을 투영한다. 기본 범위와 해상도는 기존 `bev_processor`와
동일한 전방 0~3m, 좌우 ±0.6m, 1cm/pixel (`120x300`)이다. 장착 높이와
pitch가 실제와 다르면 BEV 형상도 틀어지므로 실측값으로 수정해야 한다.

실제 화면 갱신률과 처리 시간은 매초 출력되는 `[IR_CAMERA]` 로그에서
`original`, `BEV`, `reproject`, `BEV` 항목으로 확인한다.

## IR와 노출

기본값은 IR laser dot projector `1.0`, flood light `0.0`이다. Dot projector는
stereo disparity용 texture를 만들고, flood light는 어두운 장면을 균일하게
비추는 용도다. 터널 차선 자체의 IR 반사를 확인하려면
`ir_flood_light_intensity`도 단계적으로 올려 비교할 수 있다.

수동 노출이 기본값이다. 제어 창에서 값을 적용하면 좌우 카메라에 같은 노출과
ISO가 즉시 전달된다. `AUTO EXPOSURE` 버튼 또는 `A` 키로 자동 노출을 다시
켤 수 있다. 설정 노출 시간은 프레임 주기보다 짧아야 하며 30 FPS 기본값에서
허용 범위는 `10~33333 us`다.

## 빌드

```bash
cd ~/Desktop/f1tenth_test0724/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select ir_camera_driver \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

DepthAI C++ 3.6 이상, OpenCV 4, CUDA Toolkit이 필요하다. 기존
`camera_driver`와 `bev_processor`를 빌드할 수 있는 Jetson 환경이면 동일한
의존성을 사용할 수 있다.

## 실행

중앙 가상 시점과 IR dot projector:

```bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  reprojection_enabled:=true \
  virtual_camera_position_ratio:=0.5 \
  ir_dot_projector_intensity:=1.0
```

재투영 없이 왼쪽 렌즈를 센서 최대 속도로 표시:

```bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  reprojection_enabled:=false selected_camera:=LEFT
```

재투영 없이 오른쪽 렌즈를 표시:

```bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  reprojection_enabled:=false selected_camera:=RIGHT
```

IR flood light도 함께 사용:

```bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  ir_dot_projector_intensity:=1.0 \
  ir_flood_light_intensity:=0.5
```

실행하면 원본, BEV, `OAK IR live controls`의 세 창이 열린다. 제어 창의
입력 칸을 마우스로 클릭하고 숫자를 입력한 다음 Enter 또는 `APPLY` 버튼을
누르면 다음 값이 즉시 적용된다.

- Flood intensity: `0.0~1.0`
- Exposure: `10~33333 us` (카메라 FPS를 바꾸면 최대값도 바뀜)
- ISO: `100~1600`

키 조작:

- `I`: 설정된 dot/flood intensity로 IR ON/OFF 전환
- `A`: 자동 노출로 전환
- `Tab`: 다음 입력 칸 선택
- `B`: 현재 원본과 BEV를 각각 PNG로 저장
- `Q` 또는 `Esc`: 종료

저장 파일명에는 `center/left/right`, `ir_on/ir_off`, `original/bev` 상태가
포함된다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `reprojection_enabled` | `true` | stereo 중앙 가상 시점 생성 여부 |
| `selected_camera` | `LEFT` | 재투영 OFF 영상 및 invalid disparity fallback |
| `virtual_camera_position_ratio` | `0.5` | LEFT 0.0에서 RIGHT 1.0 사이 가상 위치 |
| `reprojection_fps` | `30.0` | 800P stereo 카메라 요청 FPS |
| `single_camera_fps` | `30.0` | 800P 단일 카메라 요청 FPS |
| `ir_enabled` | `true` | 시작 시 IR emitter 활성화 |
| `ir_dot_projector_intensity` | `1.0` | dot projector 세기 `0.0~1.0` |
| `ir_flood_light_intensity` | `0.0` | flood light 세기 `0.0~1.0` |
| `manual_exposure_enabled` | `true` | 좌우 동일 수동 노출 적용 |
| `manual_exposure_us` | `5000` | 수동 노출 시간 |
| `manual_sensitivity_iso` | `800` | 수동 ISO `100~1600` |
| `preview_max_fps` | `30.0` | 원본/BEV 프리뷰 공통 FPS 상한 |
| `bev.camera_height_m` | `0.20` | 지면 기준 카메라 광학 중심 높이 |
| `bev.camera_pitch_down_deg` | `14.0` | 카메라 하향 장착 pitch |
| `bev.*_m`, `bev.meter_per_pixel` | `0~3m`, `±0.6m`, `0.01` | BEV 범위와 해상도 |
| `capture_directory` | `.` | `B` 키 PNG 저장 위치 |

## 제한사항

- 깊이 불연속, 가림 영역, 반사체에서는 disparity가 없거나 잘못될 수 있으며
  해당 픽셀은 선택한 렌즈 영상으로 대체된다.
- 좌우 밝기 차이가 크면 평균 합성 경계가 보일 수 있다. 정량 시험에는 동일
  수동 노출을 권장한다.
- BEV는 평면 노면과 고정 장착 자세를 가정하며 IMU 흔들림 보정은 하지 않는다.
- USB 연결이 HIGH(USB 2)로 표시되거나 Jetson 전력·열 상태가 좋지 않으면
  실제 FPS가 30보다 낮아질 수 있다.
- OAK-D Pro W는 dot projector와 flood LED가 기본적으로 꺼진 장치이므로 이
  패키지가 시작 시 명시적으로 켜고 종료 시 모두 끈다.
