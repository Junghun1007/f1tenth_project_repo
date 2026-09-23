# 터널 RGB / 스테레오 IR / BEV MP4 수집

주행 제어와 분리된 카메라·녹화 패키지다. 한 번의 녹화 세션마다 깨끗한 영상 세 개를
서로 다른 H.264 MP4 파일로 저장한다. FFmpeg 입력을 `bgr24`로 고정해 RGB 및 스테레오
영상이 초록색으로 저장되는 OpenCV VideoWriter 문제를 피한다.

- `rgb.mp4`: CAM_A 원본 RGB (`/camera/image_rect`)
- `stereo_ir.mp4`: CAM_B/C 스테레오 중앙 흑백 영상 (`/camera/image_stereo_ir`)
- `bev_ir.mp4`: CAM_B/C를 지면에 투영한 흑백 BEV (`/camera/image_bev_ir`)

프리뷰와 카메라 제어 창은 MP4에 포함되지 않는다. RGB 기반 BEV도 사용하지 않는다.

## 빌드

```bash
cd ~/Desktop/hsj/f1tenth_0906ML
git pull --ff-only origin 0906ML
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --packages-up-to tunnel_data_collection \
  --symlink-install --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

OAK를 여는 다른 카메라 launch는 먼저 종료한다.

FFmpeg와 H.264 인코더가 준비됐는지 확인한다.

```bash
ffmpeg -hide_banner -encoders | grep libx264
```

명령이 없거나 `libx264`가 표시되지 않으면 다음 명령으로 Ubuntu 패키지를 설치한다.

```bash
sudo apt update
sudo apt install -y ffmpeg
```

## 기본 실행

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py \
  output_root:=$HOME/Desktop/hsj/tunnel_recordings
```

기본 설정은 카메라 30 FPS, MP4 30 FPS, RGB와 스테레오 1280x800이며 RGB·스테레오·BEV·
제어 창을 모두 표시한다. 실행 직후 녹화 상태는 IDLE이다. 제어 창의 `START RECORDING`과
`STOP RECORDING` 버튼 또는 다음 서비스를 사용한다.

```bash
ros2 service call /tunnel_recorder/start std_srvs/srv/Trigger "{}"
ros2 service call /tunnel_recorder/stop std_srvs/srv/Trigger "{}"
```

각 세션은 다음 형태로 저장된다.

```text
tunnel_recordings/tunnel_YYYYMMDD_HHMMSS_microseconds+timezone/
├── rgb.mp4
├── stereo_ir.mp4
├── bev_ir.mp4
└── recording_metadata.json
```

`recording_metadata.json`에는 코덱, 설정 FPS, 해상도, 실제 저장 프레임 수, 파일 크기와
스트림별 인코딩 오류가 기록된다. 세 토픽 중 하나라도 프레임이 없으면 정지 서비스가 실패
상태를 반환하므로 빈 녹화를 바로 확인할 수 있다.

## 녹화 FPS 설정

카메라는 30 FPS로 두고 MP4만 10 FPS로 저장하는 예시다. 입력이 설정보다 빠르면 recorder가
timestamp 기준으로 프레임을 건너뛴다. `recording_fps`를 카메라 FPS보다 크게 설정해도 없는
프레임을 복제하지는 않는다.

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py \
  output_root:=$HOME/Desktop/hsj/tunnel_recordings \
  camera_fps:=30.0 recording_fps:=10.0
```

기본 인코더는 FFmpeg의 `libx264`다. `h264_crf`가 작을수록 화질과 파일 크기가 커지며,
기본값 18은 프레임 데이터 추출에 적합한 고화질 설정이다. `h264_preset`은 인코딩 속도와
압축 효율을 정하며 기본값 `ultrafast`는 실시간 프레임 누락을 줄이는 설정이다.

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py \
  recording_fps:=30.0 h264_crf:=18 h264_preset:=ultrafast
```

## 프리뷰 창 선택

RGB만 표시한다. 녹화는 보이지 않는 스테레오와 BEV도 계속 저장한다. 제어 창을 끄면 녹화는
서비스 명령으로 시작하고 종료한다.

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py \
  rgb_preview_enabled:=true \
  stereo_preview_enabled:=false \
  bev_preview_enabled:=false \
  controls_preview_enabled:=false
```

RGB와 스테레오 영상, 제어 창만 표시하는 예시다.

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py \
  rgb_preview_enabled:=true \
  stereo_preview_enabled:=true \
  bev_preview_enabled:=false \
  controls_preview_enabled:=true
```

모든 프리뷰를 끄는 것도 가능하지만, 이 경우 실행 상태와 녹화 제어는 ROS2 토픽·서비스로
확인한다.

## 해상도 설정

RGB와 스테레오 입력 해상도를 각각 지정한다. 너비와 높이는 짝수이며 RGB/스테레오 모두
최대 1280x800 범위여야 한다. BEV 출력 해상도는 거리 범위와 meter-per-pixel 설정에서
결정되며 기본값은 120x300이다.

```bash
ros2 launch tunnel_data_collection tunnel_record.launch.py \
  rgb_width:=960 rgb_height:=540 \
  stereo_width:=640 stereo_height:=400 \
  camera_fps:=30.0 recording_fps:=30.0
```

## 카메라 밝기와 IR 조절

`OAK IR live controls` 창에서 TARGET을 눌러 RGB 또는 IR/BEV를 선택한다. ISO와
Exposure(마이크로초)를 입력하고 Enter 또는 APPLY를 누르면 실제 영상에 바로 적용된다.
FLOOD는 IR 조명에만 적용된다. AUTO EXPOSURE는 선택한 카메라에 적용된다.

## MP4에서 사진 추출

예를 들어 RGB MP4의 모든 프레임을 PNG로 추출한다.

```bash
mkdir -p rgb_frames
ffmpeg -i rgb.mp4 rgb_frames/%08d.png
```

30 FPS MP4에서 세 프레임마다 한 장을 추출해 약 10장/초로 만들려면:

```bash
mkdir -p rgb_10fps
ffmpeg -i rgb.mp4 -vf "select='not(mod(n,3))'" -fps_mode vfr rgb_10fps/%08d.png
```

## 확인 및 제한

```bash
ros2 topic hz /camera/image_rect
ros2 topic hz /camera/image_stereo_ir
ros2 topic hz /camera/image_bev_ir
ros2 topic echo /tunnel_recorder/recording --once
```

- MP4는 설정 FPS로 재생되며 프레임별 원본 timestamp는 저장하지 않는다. 실제 프레임 수는
  metadata JSON에서 확인한다.
- H.264는 손실 압축이므로 추출한 PNG가 센서 원본 픽셀과 완전히 동일하지는 않다. 더 높은
  화질이 필요하면 `h264_crf:=12`, 파일 크기를 줄이려면 `h264_crf:=23` 정도로 조정한다.
- 세 영상을 동시에 1280x800 30 FPS로 저장하려면 빠른 SSD가 필요하다.
- DepthAI는 빌드 시 선택한 라이브러리를 launch에서도 우선 사용한다. 설치 위치가 바뀌면
  다시 빌드한다.
