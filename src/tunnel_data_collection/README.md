# 터널 RGB/BEV 데이터 수집

이 패키지는 한 번의 launch로 다음 작업을 수행한다.

- 8BitDo 조이스틱 수동 주행과 VESC 통신 시작
- OAK RGB 카메라와 BEV 처리 시작
- `OAK fixed-reference pitch-roll stabilization` RGB 창과 `BEV image` 창 표시
- RGB, BEV, 조이스틱 및 차량 상태를 하나의 rosbag2 세션에 저장
- 저장한 bag을 RGB/BEV PNG 프레임과 timestamp CSV로 추출

bag에는 ROS `sensor_msgs/Image` 메시지가 영상 코덱 없이 그대로 기록된다. 따라서
MP4/AVI를 원본으로 사용하는 것보다 적합하다. RGB는 `/camera/image_rect`의 정류 및
IMU 안정화된 NV12 영상이고 BEV는 `/camera/image_bev`의 BGR8 영상이다. CAM_A 센서의
왜곡 보정 전 Bayer raw 데이터는 아니다.

## 빌드

Ubuntu ROS2 환경에서 저장소 루트로 이동한 뒤 빌드한다.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select \
  camera_driver bev_handoff oak_startup bev_processor \
  joy_initializer manual_control vesc_bridge vehicle_bringup \
  tunnel_data_collection \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

기존 `bev_processor.launch.py`도 수정되었으므로 `tunnel_data_collection`만 빌드하면
설치 공간에 이전 BEV launch가 남을 수 있다. 위 목록처럼 함께 빌드한다.

## 주행과 녹화

차량 구동 바퀴를 띄운 상태에서 조이스틱 매핑과 VESC emergency stop을 먼저 확인한다.
OAK를 사용하는 다른 launch(`camera_driver`, `auto_drive`, `depth_lidar` 등)는 종료한다.
데스크톱 세션에서 다음 명령을 실행한다.

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py
```

기본 출력 위치는 명령을 실행한 경로 아래의 다음 폴더다.

```text
tunnel_recordings/tunnel_YYYYMMDD_HHMMSS_microseconds+timezone/
```

시작 직후 BEV가 카메라 높이와 자세를 측정하므로 차량을 평평한 바닥에 정지시킨다.
터미널에 BEV 준비 완료가 표시되고 두 영상 창이 갱신되는 것을 확인한 다음 주행한다.
터널 통과가 끝나면 차량을 정지시키고 launch 터미널에서 `Ctrl+C`를 한 번 누른다.
rosbag 종료 메시지가 출력되고 셸 프롬프트가 돌아올 때까지 전원을 끄지 않는다. 영상
창의 X 버튼만 누르면 해당 미리보기만 닫히며 녹화와 주행 노드는 계속 실행된다.
디스크 오류 등으로 rosbag 프로세스가 먼저 종료되면 launch 전체와 수동 주행 노드도
종료된다. 터미널의 recorder 오류를 확인한 뒤 원인을 해결하고 새 세션으로 다시 실행한다.

저장 위치와 FPS를 지정할 수도 있다.

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py \
  output_root:=/data/tunnel_recordings \
  recording_fps:=20.0 \
  max_bag_size:=4294967296 \
  vehicle_namespace:=autopilot03 \
  vesc_port:=/dev/ttyTHS1 \
  controller_name_contains:=8BitDo
```

1280x800 NV12 RGB만 해도 30 FPS에서 약 46 MB/s, 약 2.8 GB/min이다. BEV와 bag
인덱스가 추가되므로 충분히 빠른 SSD와 여유 공간을 사용한다. 공간이 부족하면
`recording_fps:=10.0`으로 낮춘다.

기본적으로 sqlite3 파일 하나가 4 GiB에 도달하면 같은 세션 안에서 다음 파일로
분할된다. `max_bag_size`는 byte 단위이며 전체 세션 크기 제한이 아니다.

수집 전 확인 명령:

```bash
df -h /data
ros2 topic hz /camera/image_rect
ros2 topic hz /camera/image_bev
ros2 topic echo /autopilot03/joy --once
```

녹화 완료 후 bag 상태를 확인한다.

```bash
ros2 bag info /data/tunnel_recordings/tunnel_20260922_153000_000000+0900
```

`/camera/image_rect`와 `/camera/image_bev`의 메시지 수가 0보다 커야 한다. 두 토픽은
독립적인 최대 FPS 제한을 사용하므로 메시지 수와 timestamp가 완전히 같을 필요는 없다.

## PNG 프레임 추출

ROS 환경과 워크스페이스를 source한 터미널에서 실행한다.

```bash
ros2 run tunnel_data_collection extract_frames \
  /data/tunnel_recordings/tunnel_20260922_153000_000000+0900
```

기본 출력은 bag 옆의 `<bag_name>_frames` 폴더다.

```text
<bag_name>_frames/
├── rgb/                  # PNG RGB 원본 프레임(BGR 배열로 디코딩 후 PNG 저장)
├── bev/                  # PNG BEV 프레임
├── rgb_frames.csv        # frame/header/bag timestamp와 영상 형식
├── bev_frames.csv
├── paired_frames.csv     # 기본 20 ms 안의 일대일 최근접 RGB-BEV 쌍
└── extraction_errors.txt # 변환 실패가 있을 때만 생성
```

모든 프레임 대신 3장마다 한 장을 추출하거나 출력 위치와 pairing 범위를 지정할 수 있다.

```bash
ros2 run tunnel_data_collection extract_frames BAG_DIRECTORY \
  --output /data/tunnel_dataset/run_01 \
  --every-nth 3 \
  --max-pair-delta-ms 25
```

출력 폴더가 이미 존재하고 비어 있지 않으면 덮어쓰지 않고 중단한다. 같은 bag에서 다른
조건으로 다시 추출할 때는 새 출력 폴더를 지정한다. `paired_frames.csv`만 학습/분석에
사용하면 서로 다른 시점의 RGB와 BEV가 섞이는 것을 피할 수 있다.

## 기록 토픽

- `/camera/image_rect`: 1280x800, NV12, 설정한 최대 recording FPS
- `/camera/image_bev`: BGR8 BEV, 설정한 최대 recording FPS
- `/camera/imu`, `/camera/startup_ground_normal`
- `/autopilot03/joy`
- `/autopilot03/manual/current_duty`
- `/autopilot03/manual/current_brake_current`
- `/autopilot03/manual/gear`
- `/autopilot03/vesc/measured_erpm`, `/autopilot03/vesc/connected`
- `/autopilot03/vesc/duty`, `/autopilot03/vesc/brake_current`,
  `/autopilot03/vesc/servo_position`

`vehicle_namespace`를 변경하면 차량 관련 기록 토픽도 같은 namespace로 변경된다.

## 현재 제약

- GUI에는 Ubuntu 데스크톱의 `DISPLAY`/Wayland 접근이 필요하다. 순수 SSH 세션에서는
  X forwarding 또는 로컬 모니터가 없으면 두 창을 띄울 수 없다.
- recording FPS는 최대값이다. SSD 쓰기 속도, CPU 변환, DDS 또는 카메라 상태에 따라
  실제 프레임 수가 낮아질 수 있으므로 `ros2 bag info`와 CSV timestamp를 확인한다.
- bag의 NV12가 픽셀 보존 원본이다. PNG RGB는 NV12 색공간 변환을 거치므로 Bayer raw
  또는 무손실 RGB 센서 출력과 같다는 의미는 아니다.
- 수동 운전 안전 로직은 기존 `manual_drive.launch.py`를 그대로 사용한다. 이 패키지는
  조향/가속 명령을 새로 만들지 않는다.
