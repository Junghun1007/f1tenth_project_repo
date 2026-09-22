# 터널 RGB / 스테레오 IR BEV 데이터 수집

이 패키지는 주행 제어와 분리되어 있으며 다음 두 영상만 rosbag2에 저장한다.
조이스틱과 수동 주행은 사용자가 별도 터미널에서 기존 방식으로 실행한다.

- `/camera/image_rect`: OAK CAM_A의 깨끗한 원본 RGB 영상, 1280x800 NV12, 30 Hz
- `/camera/image_bev_ir`: CAM_B/C 스테레오 흑백 영상을 각 렌즈 보정값으로 지면에
  투영하고 겹치는 영역을 합성한 BEV, 기본 120x300 mono8, 최대 30 Hz

RGB로 만든 기존 `/camera/image_bev`는 실행하지도, 녹화하지도 않는다. 화면에는
RGB, 스테레오 중앙 흑백 영상, 스테레오 IR BEV와 IR 제어 창이 표시되지만 rosbag에는
위 두 토픽만 들어간다. 영상에는 박스, 상태 글자 등 오버레이를 그리지 않는다.

## Ubuntu에서 빌드

```bash
cd ~/Desktop/hsj/f1tenth_0906ML
git pull origin 0906ML
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --packages-up-to tunnel_data_collection \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

OAK를 여는 다른 launch(`camera_driver`, `bev_processor`, `depth_lidar`, 자동 주행 등)는
먼저 종료한다. 이 수집 launch에서는 `ir_camera_driver` 하나가 CAM_A/B/C를 함께 연다.

## 실행과 녹화 시작/종료

별도 `OAK IR live controls` 창에서 영상에 직접 적용할 값을 입력한다.
`TARGET` 버튼으로 IR/BEV 또는 RGB를 선택하고 ISO와 밝기(Exposure, 마이크로초)를
입력한 뒤 Enter 또는 APPLY를 누르면 실제 카메라 설정이 바뀐다.
FLOOD는 선택 대상과 관계없이 IR 조명에 적용된다. AUTO EXPOSURE는 선택한
카메라의 자동 노출을 켠다. 노출 시간을 늘리면 밝아지지만 움직임 번짐도 증가할 수 있다.

`START RECORDING`으로 녹화를 시작하고 `STOP RECORDING`으로 종료한다.
녹화 상태는 같은 창의 IDLE/RECORDING으로 확인한다. 제어 창은 저장 영상에 포함되지
않으며, 입력한 값은 프로그램 재시작 시 YAML 기본값으로 돌아간다.
이 창은 실제 카메라 제어 화면이며 가상 영상을 만드는 시뮬레이터가 아니다.

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py \
  output_root:=/data/tunnel_recordings
```

launch 직후에는 카메라 화면만 시작되고 **녹화는 대기 상태**다. 이 패키지는 조이스틱을
구독하거나 차량 제어 노드를 실행하지 않는다. 녹화는 제어 창 버튼 또는 아래 ROS2
서비스로 시작하고 종료한다.

```bash
# 녹화 시작
ros2 service call /tunnel_recorder/start std_srvs/srv/Trigger "{}"

# 녹화 종료
ros2 service call /tunnel_recorder/stop std_srvs/srv/Trigger "{}"

# 현재 상태 확인
ros2 topic echo /tunnel_recorder/recording --once
ros2 topic echo /tunnel_recorder/session_path --once
```

매번 시작할 때 `/data/tunnel_recordings/tunnel_날짜_시간/` 형식의 새 bag 세션을 만든다.
터널을 여러 번 통과할 때 서비스로 시작하고 통과 후 종료하면 주행별 bag이 나뉜다.
launch 전체를 끝낼 때는 녹화를 먼저 종료한 뒤 `Ctrl+C`를 누른다.

수동 주행은 별도 터미널에서 기존 명령으로 실행한다. 수동 주행 프로세스를 시작하거나
종료해도 카메라와 녹화 프로세스에는 영향을 주지 않는다.

기본 4 GiB마다 같은 세션 안에서 sqlite3 파일만 분할한다. 값은 byte 단위다.

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py \
  output_root:=/data/tunnel_recordings max_bag_size:=8589934592
```

## 녹화 확인

```bash
ros2 topic hz /camera/image_rect
ros2 topic hz /camera/image_bev_ir
ros2 bag info /data/tunnel_recordings/tunnel_20260922_153000_000000+0900
```

`ros2 bag info`에는 두 영상 토픽만 나타나야 한다. 카메라 처리량과 SSD 쓰기 속도에
따라 실제 주기는 30 Hz보다 낮을 수 있으므로 실제 장비에서 `ros2 topic hz`로 확인한다.

## 프레임 이미지 추출

저장된 bag은 나중에 RGB와 BEV PNG로 분리할 수 있다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run tunnel_data_collection extract_frames \
  /data/tunnel_recordings/tunnel_20260922_153000_000000+0900
```

모든 프레임은 기본값이며, 3프레임마다 한 장만 저장하려면 다음과 같이 실행한다.

```bash
ros2 run tunnel_data_collection extract_frames BAG_DIRECTORY \
  --output /data/tunnel_dataset/run_01 \
  --every-nth 3 \
  --max-pair-delta-ms 25
```

결과 폴더에는 `rgb/`, `bev/`, 각 timestamp CSV, 시간상 가장 가까운 두 영상을 연결한
`paired_frames.csv`가 생성된다. NV12 RGB는 PNG 저장 시 BGR 배열로 변환되고 BEV는
흑백 값을 유지한다.

## 주요 파라미터

- `output_root`: 주행별 bag을 저장할 상위 경로
- `max_bag_size`: sqlite3 파일 하나의 최대 byte 수
- `ir_dot_projector_intensity`: 스테레오 특징점용 IR dot 세기, 0.0~1.0
- `ir_flood_light_intensity`: 터널 전체 조명용 IR flood 세기, 0.0~1.0

카메라 FPS와 해상도, BEV 범위는
`ir_camera_driver/config/ir_camera_config.yaml`에서 조정한다. 차량 장착 높이와 pitch가
바뀌면 시작 시 평평한 바닥에서 자동 측정을 끝낸 뒤 주행해야 한다. 측정과 IMU 초기화
중에는 차량을 움직이지 않는다.

## 현재 한계

DepthAI는 빌드 시 선택한 라이브러리 경로를 `ir_camera_driver` 설치 폴더에 기록하고,
launch가 카메라 프로세스의 `LD_LIBRARY_PATH` 앞에 추가한다. `/usr/local`의 DepthAI로
빌드했는데 `/opt/ros/humble`의 다른 버전이 로딩되는 충돌을 방지한다.
이 설정은 launch로 실행한 카메라 프로세스에 적용된다. 설치 경로를 바꾸면 재빌드한다.

- GUI 실행에는 Ubuntu 데스크톱의 `DISPLAY` 또는 Wayland 접근이 필요하다.
- RGB와 BEV는 같은 장치에서 각각 30 Hz로 생성하지만 하드웨어 트리거로 같은 노출 시각에
  묶인 프레임은 아니다. 추출기의 timestamp pairing으로 가까운 프레임을 연결한다.
- 스테레오 BEV는 바닥 평면 투영이므로 카메라 가림, 반사, 매우 어두운 표면 및 장착 자세
  오차에 영향을 받는다.
- 1280x800 NV12 RGB는 약 46 MB/s이고 rosbag 오버헤드가 추가되므로 빠른 SSD와 충분한
  저장 공간이 필요하다.
