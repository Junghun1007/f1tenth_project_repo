# line_detactor

`/camera/image_bev`의 BGR8 BEV에서 Fast-SCNN HighRes로 좌우 차선을 추론한다.
끝점 접선에 맞춘 3차 베지어 보간으로 검출 조각을 연결하고, 좌우 각각 최대
한 개의 중심선과 확장된 결과 영상을 발행한다. 기존 자동주행/조향 토픽은
변경하지 않으며 경로계획 소비자는 아직 연결하지 않았다.

## 기본 동작

- 모델 입력: 가로 120 × 세로 300, RGB `[0,1]`, FP32 BCHW `[1,3,300,120]`
- 모델 출력: left/right logits `[1,2,300,120]`
- 좌우 독립 sigmoid threshold 기본 `0.5`; 중복 픽셀은 로짓이 높은 쪽에 배정
- 좌우 **각각 30px** 검은 여백 추가: 결과는 **180×300**, 모델 입력/BEV 배율은 유지
- 파란색: 왼쪽, 빨간색: 오른쪽. 최종 선 두께 기본 2px
- 결과 상태: `NONE / LEFT_ONLY / RIGHT_ONLY / BOTH`
- 연결과 약한 스플라인 보정 기본 ON. 각각 별도 ON/OFF 가능
- 모든 거리/곡률 파라미터는 확대 전 BEV 픽셀 기준

프리뷰는 원본 BEV에 최종 선을 겹쳐 보여주며 바깥 여백은 검은색이다.
발행하는 BGR 결과는 카메라 배경과 성능 배너가 없는 **검은 배경의 차선 영상**이다.
모델 파일은 패키지 내 `fast_scnn_highres_120x300_batch_1.onnx`를 사용한다.

## 조각 연결 방식

1. 좌우 마스크별 연결 성분에서 면적/길이가 작은 조각을 제외한다. 후보 수는
   `connection_max_fragments`로 제한하며 초과하면 면적이 큰 성분부터 처리한다.
2. 각 성분을 세선화하고 가까운 끝점부터 가장 먼 끝점까지의 중심선 경로를
   추출한다. 한 성분에 곁가지가 있어도 주 경로 하나만 남긴다. 끝점이 없는
   폐곡선은 제외한다. 길이는 중심선의 호 길이로 측정한다.
3. 중심선을 약 1px 간격으로 샘플링한다. 선택적인 자연 3차 스무딩 스플라인은
   **관측된 조각 내부에만** 적용하며 양 끝점은 유지한다. 보정량 제한은 2D
   이동량에 적용한다. 조각 밖으로 나가거나 자기 교차가 생기면 원래 중심선을 쓴다.
4. 끝점 주변 여러 점으로 접선을 추정한다. 두 끝점의 위치, 방향 차이와 예측
   허용 폭을 검사한 뒤 끝점 접선이 맞는 3차 베지어 연결 후보를 만든다.
5. 곡선의 최대 곡률, 길이/직선거리 비율, 자기 교차, 확장 프레임 이탈을 검사한다.
   통과한 후보 중 짧고 완만한 연결을 선택한다. 화면 밖으로 나갔다가 다시
   들어오는 연결도 좌우 여백 안에서는 허용한다.
6. 가까운 조각에서 먼 조각으로 연결 후보를 비교해 각 채널에서 한 개의 연속
   경로만 선택한다. 관측된 길이를 가점으로, 긴 보간과 먼 시작 위치를 감점으로
   사용한다. 보간 길이는 최소 검출 길이를 만족시키는 데 사용하지 않는다.
7. 같은 행에서 좌우 순서가 뒤집히거나 그려진 두 선이 겹치면 관측 길이가
   짧은 쪽을 제외한다. 두 길이가 10% 이내로 비슷하면 양쪽 모두 제외한다.

### 기존 보정을 유지한 이유

스플라인은 검출 조각 내부의 잔떨림을 줄이고, 베지어는 **조각 사이의 공백**을
메운다. 두 역할을 분리했으며 보간한 전체 차선을 다시 스플라인으로 피팅하지
않는다. `smoothing_enabled:=false`로 내부 보정만 끌 수 있다.

`connection_enabled:=false`에서는 이전 GPU 행별 마스크 보정 프리뷰로 돌아간다.
이 모드에서는 여러 조각이 남을 수 있어 최종 결과 토픽을 발행하지 않는다.
두 기능을 모두 끄면 원본 모델 마스크를 보여준다.

### "두껍게 예측"의 의미

`connection_corridor_half_width_px`는 연결 후보를 찾는 허용 폭이며 실제 선을
굵게 그리는 값이 아니다. `connection_direction_tolerance_deg`에 따라 거리와
함께 허용 폭이 증가한다. 약간 어긋난 직선 조각도 연결할 수 있지만 접선 일치,
최대 곡률 등의 조건은 계속 적용된다. 실제 표시 두께는 `result_line_width_px`다.

### 화면 밖 연결의 범위

화면 밖의 차선은 새로 관측한 것이 아니라 **두 검출 조각 사이의 보간 추정**이다.
반대편 끝점이 없는 곳까지 임의로 연장하지 않는다. 확장 영역을 넘어서는 곡선은
클리핑해서 연결하지 않고 후보 자체를 제외한다. 여백을 늘려도 검출 범위가
늘어나거나 장애물이 없는 영역이라는 의미는 아니다.

현재 연결 순서는 BEV 아래쪽에서 위쪽으로 진행하는 차선을 대상으로 한다.
앞뒤로 크게 되돌아오는 헤어핀, 갈림길의 모든 가지, 복잡한 루프는 지원하지 않는다.
좌우 순서 검사는 보수적이므로 크게 접히는 커브에서 정상 후보를 제외할 수도 있다.
노이즈 제외와 연결 판정은 기하 기반 휴리스틱이며 기본값은 실차 튜닝 전 시작값이다.
시간축 추적, 장애물 마스크 검사, 자동주행 제어는 포함하지 않는다.

## 빌드

JetPack CUDA·TensorRT 개발 패키지, OpenCV, ROS 2 Humble 및
`rosidl_default_generators`가 필요하다. PyTorch/Python CUDA wheel은 사용하지 않는다.
새 ROS 메시지가 추가되어 패키지를 다시 빌드하고 환경을 다시 읽어야 한다.

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
colcon build --packages-select line_detactor
source install/setup.bash
```

TensorRT 개발 패키지가 없다면 Jetson에서 설치한다.

```bash
sudo apt install libnvinfer-dev libnvonnxparsers-dev libnvinfer-plugin-dev
```

첫 실행은 ONNX에서 장치 전용 TensorRT FP32 엔진을 만들기 때문에 시간이 걸린다.
기본 캐시는 ONNX 옆의 `*.trt<TensorRT-major>.fp32.engine`이며 다음부터 재사용한다.
다른 Jetson/TensorRT 버전의 엔진을 복사하지 말고 해당 장치에서 생성한다.

## 실행 및 YAML 지정

터미널 1: 원본 BEV 발행. 해당 BEV YAML의 `publish_enabled`는 `true`여야 한다.
초기 depth/IMU 측정이 끝날 때까지 차량을 정지시킨다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  preview_enabled:=false \
  lane_seed_detection_enabled:=false
```

터미널 2: 저장소 루트에서 YAML을 지정해 연결 프리뷰/결과 발행을 시작한다.

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:="$(pwd)/src/line_detactor/config/line_detactor.yaml" \
  connection_enabled:=true \
  result_padding_px:=30
```

허용 폭을 늘려 조금 어긋난 파편도 연결 후보로 고려:

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:="$(pwd)/src/line_detactor/config/line_detactor.yaml" \
  connection_corridor_half_width_px:=6.0
```

연결은 사용하고 기존 스플라인만 끄기:

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:="$(pwd)/src/line_detactor/config/line_detactor.yaml" \
  connection_enabled:=true \
  smoothing_enabled:=false
```

원본 마스크와 비교:

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:="$(pwd)/src/line_detactor/config/line_detactor.yaml" \
  connection_enabled:=false \
  smoothing_enabled:=false
```

YAML 구조는 `line_detactor: ros__parameters:`를 유지한다.
우선순위는 **명시한 launch 인자 > 지정 YAML > 노드 기본값**이다.
`params_file`을 생략하면 설치된 YAML을 사용한다. 파라미터는 시작 시 읽으므로
변경 후 재실행한다. `preview_enabled:=false`여도 연결 및 결과 발행은 수행한다.
`result_publish_enabled:=false`는 발행만 끄고 연결 프리뷰는 유지한다.

## 경로계획용 출력 계약

기본 토픽과 타입:

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/line_detactor/result` | `line_detactor/msg/LaneResult` | 상태, 좌우 곡선, 관측/보간 구분, 영상, 라벨, 좌표 메타데이터 |
| `/line_detactor/result_image` | `sensor_msgs/msg/Image` (`bgr8`) | 같은 결과의 차선 전용 영상. 기본 180×300, 배너 없음 |

`LaneResult` 하나에 프레임과 좌표를 묶어 발행한다. `header`와 두 이미지의 헤더는
입력 BEV의 timestamp/frame_id를 유지한다. QoS는 best effort, volatile, KeepLast(1).
새 입력 프레임마다 결과를 만들며 검출이 없으면 빈 곡선, 검은 영상과 `NONE`을
발행한다. 처리 실패/입력 중단 시 새 메시지가 없으므로 소비자는 timestamp로
최신성을 판단해야 한다.

- `state`: `0=NONE`, `1=LEFT_ONLY`, `2=RIGHT_ONLY`, `3=BOTH`
- `left/right.points`: 가까운 쪽에서 먼 쪽으로 정렬한 단일 점 목록. 없으면 빈 배열
- `left/right.provenance`: 점마다 `0=MODEL`, `1=BRIDGE`; 점 배열과 같은 길이
- `observed_length_px`: 보간을 제외한 원본 중심선 지지 길이. 확률/신뢰도 점수가 아님
- `image`: 검은 배경에 파란 왼쪽/빨간 오른쪽 선만 표시
- `labels`: `mono8`, 배경 `0`, 왼쪽 관측 `1`, 오른쪽 관측 `2`, 왼쪽 보간 `3`, 오른쪽 보간 `4`
- `source_width/height`, `padding_left/right`: 원본과 확장 영상의 픽셀 관계

**점 좌표는 미터가 아닌 확장 영상 픽셀**이며 x는 오른쪽, y는 아래쪽, z는 0이다.
헤더의 frame_id를 이유로 점을 미터 단위 TF 위치로 해석하면 안 된다.
원본 BEV 좌표로 되돌릴 때는 다음 관계를 사용한다.

```text
u_source = u_result - padding_left
v_source = v_result
```

미터 변환에는 실행 중인 `bev_processor`의 배율과 원점을 사용한다. 여기서는
배율을 추정하거나 별도 기본값으로 고정하지 않는다. 보간된 선도 차선 경계의
추정치일 뿐 장애물 없는 경로를 뜻하지 않는다. 라벨 0 역시 자유 공간이 아니다.
라벨 영상의 두께는 렌더링용이므로 정확한 중심 좌표/보간 여부는 점 목록을 쓴다.

## 속도 표시

- `infer`: CUDA 이벤트로 측정한 순수 TensorRT 추론 시간
- `model FPS`: 추론 시간의 역수이며 전체 처리 FPS가 아님
- `connect`: CPU 성분 추출, 중심선/내부 보정, 연결 후보 평가, 선택 및 결과 래스터화 시간
- `correct`: 연결 ON에서는 GPU 라벨 생성·D2H 시간 + `connect`; 연결 OFF에서는 기존 행 보정 시간
- `limit`: 내부 스플라인 이동량 제한. **보간 구간의 최대 이동량 제한이 아님**
- `lanes`: 최종 상태. 연결 OFF에서는 `RAW`
- `view FPS`: 전체 처리 빈도, 기본 상한 30 FPS

로그는 단계별 평균/최대 ms를 출력한다. `connect`는 `correct`의 일부이므로
두 값을 더하지 않는다. 프리뷰 합성/창 표시와 ROS 직렬화·발행은 `correct`에
포함하지 않는다. 기본 연결 경로는 logits 전체 대신 120×300 mono8 라벨
36,000바이트를 D2H하며 기존 BGR 프리뷰도 생성한다. 성능은 Jetson에서 측정해야 한다.

## 주요 파라미터

| 파라미터 | 기본값 | 영향 |
|---|---:|---|
| `connection_enabled` | `true` | 조각 연결 및 최종 차선 구성 |
| `result_padding_px` | `30` | 좌우 각각 추가할 여백. 총 추가 폭은 값의 2배 |
| `connection_min_component_area_px` | `8` | 이보다 작은 마스크 성분 제외 |
| `connection_min_fragment_length_px` | `8.0` | 이보다 짧은 중심선 조각 제외 |
| `connection_max_fragments` | `24` | 좌우 각각 처리할 후보 수 상한 |
| `connection_tangent_window_px` | `8.0` | 접선 추정 구간. 크면 안정적이나 급한 커브의 방향을 평균냄 |
| `connection_max_gap_px` | `80.0` | 연결 가능한 끝점 간 최대 직선거리 |
| `connection_corridor_half_width_px` | `4.0` | 방향 예측의 기본 허용 반폭 |
| `connection_direction_tolerance_deg` | `20.0` | 거리 증가에 따른 방향 예측 폭 확대 |
| `connection_max_turn_deg` | `180.0` | 양 끝 접선의 허용 방향 차이 |
| `connection_max_curvature_per_px` | `0.12` | 연결 곡선의 최대 곡률. 낮추면 급한 연결을 더 엄격히 제외 |
| `connection_max_arc_ratio` | `1.8` | 곡선 길이/직선거리 상한. 낮추면 우회하는 연결 제외 |
| `connection_min_lane_length_px` | `20.0` | 최종 차선에 필요한 관측 길이. 보간 길이는 제외 |
| `result_line_width_px` | `2` | 출력 선의 두께. 연결 허용 폭과 별개 |
| `result_publish_enabled` | `true` | 연결 결과 메시지/영상 발행 |
| `smoothing_enabled` | `true` | 관측 조각 내부의 약한 스플라인 |
| `smoothing_strength` | `8.0` | 내부 평활화 강도. 0이면 중심 이동 없음 |
| `smoothing_correction_limit_enabled` | `true` | 내부 보정량 제한 사용 |
| `smoothing_max_correction_px` | `2.0` | 관측 중심의 최대 2D 이동량. 기존 행 모드에서는 수평 이동량 |
| `smoothing_min_segment_rows` | `12` | 연결 모드: 약 1px 호 길이 샘플 수. 기존 모드: 연속 행 수 |
| `smoothing_max_row_jump_px` | `4.0` | 기존 행 모드에서만 사용하는 구간 분리 기준 |
| `mask_threshold` | `0.5` | 모델 좌우 sigmoid 임계값 |
| `overlay_alpha` | `0.75` | 프리뷰 차선 불투명도. 발행 라벨/중심선에 영향 없음 |
| `preview_fps` | `30.0` | 최대 처리 빈도 |
| `preview_scale` | `2.0` | 창 확대 배율. 좌표와 거리 파라미터에 영향 없음 |
| `warmup_iterations` | `10` | TensorRT 워밍업 횟수 |
| `status_log_interval_sec` | `1.0` | 성능 로그 간격 |

## 수동주행과 함께 실행

자동주행을 켜는 `auto_drive.launch.py` 대신 별도 터미널에서 수동주행을 실행한다.

```bash
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py \
  vesc_port:=/dev/ttyTHS1 \
  controller_name_contains:=8BitDo \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112
```

`Q`, `ESC` 또는 프리뷰 창 닫기로 종료한다.
이번 변경의 빌드·실행·테스트는 요청에 따라 수행하지 않았다.
