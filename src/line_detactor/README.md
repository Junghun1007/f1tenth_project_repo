# line_detactor

Fast-SCNN HighRes로 원본 BEV의 좌우 차선을 추론한다. 후처리는 **작은 노이즈
성분 제거와 화면 좌우 경계에서 잘린 차선의 바깥쪽 보간**만 수행한다.

원본 `/camera/image_bev`, BEV 변환, 모델 입력은 **120×300 그대로**다.
프리뷰/결과 도화지에만 좌우 30px씩 검은 여백을 추가해 **180×300**으로 만든다.
좌우 차선은 파란색/빨간색이다. 기존 경로계획·조향 제어는 변경하지 않는다.

## 후처리 범위

1. 모델 출력의 좌우 채널에 독립 sigmoid threshold를 적용한다. 중복 픽셀은
   로짓이 큰 쪽에 배정한다. 입력 RGB `[1,3,300,120]`, 출력 `[1,2,300,120]`이다.
2. 좌우 마스크 각각에서 `connection_min_component_area_px`보다 작은 연결
   성분만 삭제한다. 그 외 성분은 위치·두께·모양·분리 상태를 그대로 유지한다.
3. **실제로 원본 영상의 x=0 또는 x=width-1에 닿은 성분만** 보간 후보로 조사한다.
   중심선은 끝점/방향 추정에만 사용하며 검출 마스크를 중심선으로 다시 그리지 않는다.
4. 같은 모델 채널, 같은 화면 경계에 있는 서로 다른 성분의 끝점끼리만 검사한다.
   양쪽 끝점의 접선이 모두 화면 밖을 향하고, 끝점 사이 거리·방향 허용 폭·곡률
   조건을 통과하면 3차 베지어 곡선으로 보간한다.
5. 곡선은 해당 경계의 바깥에만 있어야 하고 확장 도화지를 벗어나면 제외한다.
   추가 선의 두께가 원본 영역을 침범하지 않도록 래스터화 후에도 원본 영역을
   마스킹한다. 새 선끼리 겹치면 후순위 연결은 제외한다. 짧은 후보부터 선택하며
   한 끝점은 한 번만 사용한다.

**내부 끊김 연결, 스플라인 평활화, 차선을 한 개로 강제 선택하기, 좌우 순서에
따른 차선 전체 삭제는 제거했다.** 결과적으로 같은 쪽 차선 조각이 여러 개 남을
수 있다. 이번 모드는 검출을 보존하는 영상 후처리이며 단일 경로 생성기가 아니다.

노이즈 제거는 영상 전체의 작은 연결 성분에만 적용한다. 화면 밖은 원래 검출
입력이 없으므로 확장 여백에는 승인된 보간 선만 생긴다. 한쪽 끝점만 있으면
무작정 연장하지 않는다. 상하 경계 잘림, BEV 내부의 검은 유효영역 경계, 일반적인
가림/내부 검출 공백은 이 보간의 대상이 아니다. 화면 밖 보간은 관측이 아닌 추정이다.

## 빌드 및 실행

Jetson의 CUDA·TensorRT 개발 패키지, OpenCV, ROS 2 Humble,
`rosidl_default_generators`가 필요하다. PyTorch는 사용하지 않는다.
이번 변경에는 ROS 메시지 필드 변경도 있으므로 재빌드 후 환경을 다시 읽는다.

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
colcon build --packages-select line_detactor
source install/setup.bash
```

터미널 1에서 원본 BEV를 발행한다. BEV YAML의 `publish_enabled`는 `true`여야
하며 초기 depth/IMU 측정이 끝날 때까지 차량을 정지시킨다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  preview_enabled:=false \
  lane_seed_detection_enabled:=false
```

터미널 2: 저장소 루트에서 파라미터 파일을 지정한다.

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:="$(pwd)/src/line_detactor/config/line_detactor.yaml"
```

테스트 YAML을 사용하는 경우:

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:=/absolute/path/test.yaml \
  connection_enabled:=true
```

기존 테스트 YAML에서 **`smoothing_*` 전부와 `connection_min_lane_length_px`를
삭제**한다. 이 파라미터들은 더 이상 읽거나 적용하지 않는다. 예전 YAML에
`smoothing_enabled: true`가 남아 있어도 보정은 다시 켜지지 않는다. 예전
`smoothing_*` launch 인자도 제거한다. 새 항목은 아래처럼 추가한다.

```yaml
line_detactor:
  ros__parameters:
    connection_enabled: true
    connection_min_component_area_px: 8
    connection_border_endpoint_distance_px: 6.0
    result_padding_px: 30
```

같은 키가 이미 있으면 중복 추가하지 말고 수정한다. 다른 기존 `connection_*`
설정은 아래 설명에 따라 재사용할 수 있다. 생략한 항목은 노드 기본값을 쓴다.
우선순위는 명시한 launch 인자 > 지정 YAML > 노드 기본값이다. 변경 후 재실행한다.

`connection_enabled:=false`는 노이즈 제거/보간/결과 발행을 끄고 원본 추론
프리뷰를 표시한다. `result_publish_enabled:=false`는 발행만 끈다.
`preview_enabled:=false`여도 후처리와 결과 발행은 동작한다.

## 파라미터

거리 단위는 확대 전 BEV 픽셀이다. 모두 테스트/실차 검증 전 시작값이다.

| 파라미터 | 기본값 | 영향 |
|---|---:|---|
| `connection_enabled` | `true` | 작은 성분 제거 + 바깥 경계 보간 활성화 |
| `result_padding_px` | `30` | 좌우 각각의 도화지 여백. 0이면 바깥 보간 없음 |
| `connection_min_component_area_px` | `8` | 면적이 이보다 작은 성분만 삭제. 1이면 삭제 없음 |
| `connection_border_endpoint_distance_px` | `6.0` | 중심선 끝점과 실제 화면 경계 검출 픽셀 사이 허용 거리 |
| `connection_min_fragment_length_px` | `8.0` | 보간 방향 추정에 필요한 최소 중심선 길이. 미달해도 원본 마스크 유지 |
| `connection_max_fragments` | `24` | 한 채널에서 조사할 경계 성분 수. 초과 성분도 마스크는 유지 |
| `connection_tangent_window_px` | `8.0` | 끝점 접선을 추정하는 구간 길이 |
| `connection_max_gap_px` | `80.0` | 두 경계 끝점 사이 최대 거리 |
| `connection_corridor_half_width_px` | `4.0` | 방향 예측 허용 반폭. 표시 두께와 별개 |
| `connection_direction_tolerance_deg` | `20.0` | 거리에 따라 예측 폭을 넓히는 각도 |
| `connection_max_turn_deg` | `180.0` | 연결 시작/도착 접선의 최대 방향 차이 |
| `connection_max_curvature_per_px` | `0.12` | 보간 곡선의 최대 곡률. 작게 하면 급한 연결 제외 |
| `connection_max_arc_ratio` | `1.8` | 보간 길이/끝점 직선거리 상한 |
| `result_line_width_px` | `2` | 바깥 보간 선의 두께만 변경. 모델 차선 두께는 유지 |
| `result_publish_enabled` | `true` | 결과 메시지/영상 발행 |
| `mask_threshold` | `0.5` | 모델 좌우 sigmoid 임계값 |
| `overlay_alpha` | `0.75` | 프리뷰 오버레이 불투명도 |
| `preview_fps` | `30.0` | 최대 처리 빈도 |
| `preview_scale` | `2.0` | 프리뷰 확대 배율. 좌표/파라미터에 영향 없음 |
| `warmup_iterations` | `10` | TensorRT 워밍업 횟수 |
| `status_log_interval_sec` | `1.0` | 로그 간격 |

## 결과 토픽과 좌표

- `/line_detactor/result`: `line_detactor/msg/LaneResult`
- `/line_detactor/result_image`: `sensor_msgs/msg/Image`, `bgr8`, 차선만 그린 검은 영상

`LaneResult.image`는 같은 BGR 결과이고 `labels`는 `mono8`이다.
라벨은 `0=배경`, `1=왼쪽 모델`, `2=오른쪽 모델`, `3=왼쪽 바깥 보간`,
`4=오른쪽 바깥 보간`이다. 검출이 없으면 검은 영상과 NONE을 발행한다.
원본 timestamp/frame_id를 유지하며 QoS는 best effort, volatile, KeepLast(1)이다.

`processing_mode=BORDER_ONLY`이다. `state`는 **남아 있는 마스크의 좌우 존재 여부**
(`0=NONE`, `1=LEFT_ONLY`, `2=RIGHT_ONLY`, `3=BOTH`)이며 한 개의 연속 차선이
확보되었다는 의미가 아니다. 계획 입력으로 해석할 때 이 차이를 반영해야 한다.

`left/right` 점 목록은 **경계 방향 추정에 사용한 성분 중심선 및 보간 곡선의
진단용 구간 목록**이다. 내부 성분/폐곡선까지 전부 표현하는 목록이 아니다.
`segment_starts`의 각 시작 오프셋에서 다음 오프셋 직전(마지막은 배열 끝)까지가
한 구간이며 **서로 다른 구간을 선으로 잇지 않는다**. 점별 provenance는
`0=모델 지지`, `1=보간`이다. 관측 길이도 진단에 사용한 경계 성분만 집계한다.
유지된 전체 검출 결과는 `labels/image`가 기준이다.

점은 확장 영상의 픽셀 좌표(x 오른쪽, y 아래쪽, z=0)이며 미터/TF 위치가 아니다.

```text
u_source = u_result - padding_left
v_source = v_result
```

미터 변환에는 실제 `bev_processor` 배율/원점을 사용한다. 검은 여백/라벨 0은
자유 공간을 뜻하지 않는다. 입력 중단/프레임 처리 실패 시 새 메시지가 없으므로
소비자는 timestamp로 최신성을 확인해야 한다. 경로계획 소비자 연결은 포함하지 않는다.

## 속도와 기타

프리뷰 `infer`는 순수 TensorRT 시간, `model FPS`는 그 역수다.
`connect`는 CPU 노이즈 제거·경계 끝점 추출·보간·결과 생성 시간이고 `correct`는
GPU 라벨 생성/D2H까지 포함한 후처리 시간이다. `connect`는 `correct`에 포함된다.
프리뷰 합성/창 표시, ROS 직렬화·발행은 해당 측정에 포함하지 않는다.
`view FPS`는 전체 처리 빈도다. 로그에는 단계별 평균/최대 ms가 표시된다.

logits 전체는 CPU에 복사하지 않고 120×300 mono8 라벨을 전달한다. ONNX 기본
경로는 패키지의 `fast_scnn_highres_120x300_batch_1.onnx`다. 최초 실행 시 장치별
FP32 TensorRT 엔진을 생성하고 ONNX 옆 `*.trt<TensorRT-major>.fp32.engine`를 재사용한다.
`Q`/`ESC`/창 닫기로 종료한다. 자동주행·조향 명령은 발행하지 않는다.

이번 변경의 빌드·실행·테스트는 요청에 따라 수행하지 않았다.
