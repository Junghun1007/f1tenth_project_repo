# line_detactor

Fast-SCNN HighRes로 원본 BEV의 좌우 차선과 정지선을 추론한다. 좌우 차선 후처리는 **작은 노이즈
성분 제거와 화면 좌우 경계에서 잘린 차선의 바깥쪽 보간**만 수행한다.

원본 `/camera/image_bev`, BEV 변환, 모델 입력은 **120×300 그대로**다.
프리뷰/결과 도화지에만 좌우 30px씩 검은 여백을 추가해 **180×300**으로 만든다.
좌우 차선은 파란색/빨간색, 정지선은 초록색, 중앙 경로는 노란색이다. 기존 경로계획·조향 제어는 변경하지 않는다.

## 정지선 모델 적용 순서

현재 기본 모델은 **`models/fast_scnn_stop_line_120x300_batch_1.onnx`**다.
기존 2채널 ONNX는 보존하지만 새 노드에서 사용하지 않는다. 진행 중인 0910 학습이
끝나면 **PC에서** 아래 명령으로 최종 best.pt를 변환한 뒤 이 패키지를 젯슨으로 옮긴다.
현재 코드 수정 시점에는 학습이 진행 중이므로 최종 배포 ONNX를 자동으로 확정하지 않았다.

현재 Windows 작업공간 `C:\Users\godld\Desktop\traffic`에서:

```powershell
line_detector/.venv-export/Scripts/python.exe line_detactor_ros/tools/export_stop_line_onnx.py
```

기본적으로 옆 `line_detector/training/highres/stop_line_0910/runs`의 최신 best.pt를
선택한다. 특정 모델은 `--checkpoint "실제/best.pt"`로 지정한다. 기존 학습 환경을
변경하지 않도록 별도 `.venv-export`를 준비했다. 다른 PC에서는 별도 가상환경에
`tools/requirements-export.txt`를 설치하면 된다. 모델 정의는 tools/model_definitions에
포함되어 있어 --checkpoint를 지정하면 원래 학습 코드 없이도 변환할 수 있다.

변환은 정적 FP32 입력/출력 `[1,3,300,120]`, opset17, 로짓 출력이다. 고정 입력의
비균등 adaptive pooling을 같은 구간의 AvgPool/평균 연산으로 표현하고 ONNX checker와
ONNX Runtime CPU로 PyTorch 출력과 비교한다. 최종 테스트26장은 변환 검증에 사용하지 않는다.
`*.onnx` 옆 `*.json`에 원본 체크포인트 epoch/SHA256와 수치 검증 결과를 기록한다.
변환에만 PyTorch가 필요하며 **젯슨 노드 실행에는 PyTorch/ONNX Runtime이 필요 없다.**

명시적으로 예전 `model_path`를 지정한 YAML은 새 ONNX 경로로 수정한다. 기본 경로를
사용한다면 YAML 변경은 필요 없다. 새 ONNX는 다른 이름이므로 기존 엔진 캐시와 분리된다.
명시된 엔진 캐시가 2채널이면 새 엔진으로 재생성한다. 학습 후 ONNX를 다시 갱신했다면
이전 캐시보다 새 수정 시간이 유지되어야 하며, 파일 시간을 보존하며 복사한 경우 해당
캐시를 제거하고 재생성한다. 엔진은 배포할 젯슨에서 생성한다.

공식 참고: [PyTorch ONNX export](https://docs.pytorch.org/docs/stable/onnx),
[TensorRT 엔진 호환성](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/engine-compatibility.html).

## 후처리 범위

1. 모델 출력의 좌우/정지선 채널에 독립 sigmoid threshold를 적용한다. 좌우 중복은
   기존처럼 로짓이 큰 쪽에 배정한다. 정지선은 독립 마스크에 남겨 좌우와의 교차점을
   보존한다. 입력 RGB `[1,3,300,120]`, 출력 로짓 `[1,3,300,120]`이다.
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

정지선에는 위 성분 제거·경계 보간을 적용하지 않는다. 정지선 마스크는 별도로 패딩만
추가하고, 화면 밖 패딩은 항상 0이다. 결과 영상에서는 정지선의 초록색이 겹친 차선보다
우선 표시된다. 노란 중앙 경로는 마지막에 덧그리지만 기존 좌우 라벨/정지선 마스크/상태는 변하지 않는다. 원본에서 흰색이 보이지 않는다는
이유로 모델이 복원한 정지선을 지우지 않는다.

**내부 끊김 연결, 스플라인 평활화, 차선을 한 개로 강제 선택하기, 좌우 순서에
따른 차선 전체 삭제는 제거했다.** 결과적으로 같은 쪽 차선 조각이 여러 개 남을
수 있다. 이 정책은 차선 마스크에 적용된다. 별도 중앙 경로 생성기는 관측 라벨 `1/2`로
경로 후보를 만들고 연결·평활화하며, 그 결과로 차선 마스크를 수정하지 않는다.

노이즈 제거는 영상 전체의 작은 연결 성분에만 적용한다. 화면 밖은 원래 검출
입력이 없으므로 확장 여백의 차선 라벨에는 승인된 보간 선만 생긴다.
노란 중앙 경로는 관측 차선에서 안쪽으로 이동한 결과가 여백에 들어올 수 있다. 한쪽 끝점만 있으면
무작정 연장하지 않는다. 상하 경계 잘림, BEV 내부의 검은 유효영역 경계, 일반적인
가림/내부 검출 공백은 이 보간의 대상이 아니다. 화면 밖 보간은 관측이 아닌 추정이다.

## 빌드 및 실행

Jetson의 CUDA·TensorRT 개발 패키지, OpenCV, ROS 2 Humble,
`rosidl_default_generators`가 필요하다. PyTorch는 사용하지 않는다.
이번 변경에는 `LaneResult` 중앙 경로 필드 추가가 있으므로 발행/구독 환경 모두
같은 메시지 버전으로 재빌드하고 환경을 다시 읽는다.

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
  publish_enabled:=true
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
| `mask_threshold` | `0.5` | 좌우/정지선 각각의 sigmoid 임계값 |
| `overlay_alpha` | `0.75` | 프리뷰 오버레이 불투명도 |
| `preview_fps` | `30.0` | 전용 GUI 스레드의 최대 표시 빈도. 추론/결과 발행 제한 아님 |
| `preview_scale` | `2.0` | 프리뷰 확대 배율. 좌표/파라미터에 영향 없음 |
| `warmup_iterations` | `10` | TensorRT 워밍업 횟수 |
| `status_log_interval_sec` | `1.0` | 로그 간격 |

## 노란 중앙 경로와 YAML 조정

`centerline_enabled: true`가 기본이다. `connection_enabled: true`에서 동작한다.
`connection_enabled: false`는 기존 raw 프리뷰 모드이며 중앙 경로/결과 메시지를 생성하지 않는다.
`centerline_enabled: false`로 중앙 경로만 끌 수 있다.

중앙 경로 생성은 다음 순서다.

1. 작은 성분 제거 후 **관측 라벨 1/2만** 사용한다. 정지선과 바깥 보간 라벨 3/4는 제외한다.
2. 각 연결 조각에서 순서가 있는 골격 경로를 추출하고 실제 거리 좌표로 변환한다.
   조각의 방향은 아래쪽 영상 중앙(차량 근처)에서 먼 쪽으로 정한다. 폐곡선은 후보에서 제외한다.
3. 대응되는 반대편 차선이 폭·방향 조건을 만족하면 중점, 아니면 안쪽 법선 방향으로
   도로 폭의 절반(기본 32.5cm)을 이동한 후보를 만든다. 코너에서는 아래 설명처럼
   관측이 충분한 바깥 차선의 오프셋 경로와 진행 방향을 더 높은 비중으로 반영한다.
4. 차량 근처부터 거리·진행 방향·관측 경계 여유 거리가 맞는 후보들을 연결한다.
   짧은 공백은 허용하며 큰 공백은 경로를 끝낸다. 원본 좌우 차선을 하나로 강제 병합하지 않는다.
5. 국소 Gaussian 평활화 후, 경로상 거리 기준 2차 다항식 평활화를 추가한다.
   직선에 가까운 곳은 강하게, 코너는 약하게 적용한다. 각 단계에서 이동량을 제한하고
   시작/끝점을 고정한다. 여유 거리/진행 방향 조건을 위반하면 강도를 낮춰 재시도한다.
6. 프리뷰와 `result_image`에 **노란색(BGR 0,255,255)**으로 표시한다.
   노란 선은 `overlay_alpha`와 무관하게 표시되며, 정지선과 겹쳐도 정지선 마스크는 보존한다.

검토한 Python 미리보기의 계산 방식을 C++/OpenCV로 이식했다. ROS에서는 스크린샷 색 추출을
하지 않고 실제 라벨을 사용하며, 경계까지의 거리도 골격 대신 관측 마스크 픽셀로 확인한다.
골격 추출/거리 샘플링과 경계 처리 차이 때문에 미리보기와 픽셀 단위로 같은 출력은 보장하지 않는다.
추가 Python/SciPy 런타임 의존성은 없다.

튜닝용 예제 YAML을 복사하고 실행한다. BEV 노드가 먼저 이미지를 발행해야 한다.

```bash
cp src/line_detactor/config/centerline_preview.yaml /tmp/centerline_preview.yaml
ros2 launch line_detactor line_detactor.launch.py \
  params_file:=/tmp/centerline_preview.yaml
```

노드 이름은 YAML의 `line_detactor.ros__parameters`와 일치해야 한다. 파일에 없는 값은
노드 기본값을 사용한다. **YAML은 시작 시 읽으므로 수정 후 노드를 재시작한다.**
`ros2 param set`으로 이 계산 설정을 실시간 갱신하는 기능은 구현하지 않았다.
명시적 launch 인자는 YAML보다 우선한다.

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:=/tmp/centerline_preview.yaml \
  centerline_smoothing_strength:=0.7 \
  centerline_smoothing_window_m:=0.65
```

아래 표의 이름에는 모두 `centerline_` 접두사를 붙인다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `enabled` | true | 중앙 경로 계산·표시 |
| `lane_width_m` | 0.65 | 차선 사이 도로 폭 |
| `bev_width_m`, `bev_height_m` | 1.2, 3.0 | **패딩 제외 원본** BEV 실제 가로/전방 범위 |
| `sample_spacing_m` | 0.015 | 차선 골격 후보의 거리 간격, 0.005~0.10m |
| `min_fragment_length_m` | 0.08 | 후보로 사용할 조각 최소 길이 |
| `tangent_window_m` | 0.06 | 차선 진행 방향을 계산하는 거리 범위 |
| `width_tolerance_m` | 0.12 | 양쪽 대응 시 도로 폭 허용 오차 |
| `pair_along_tolerance_m` | 0.055 | 양쪽 대응 시 진행 방향 위치 오차 |
| `pair_heading_tolerance_deg` | 40.0 | 양쪽 대응 시 접선 방향 차이 |
| `max_gap_m` | 0.12 | 후보 사이 연결 거리 상한 |
| `max_start_distance_m` | 0.65 | 영상 아래 중앙에서 첫 경로 후보까지 허용 거리 |
| `min_clearance_m` | 0.16 | 관측 차선 마스크와 경로 사이 최소 여유 거리 |
| `outside_margin_m` | 0.12 | 좌우 원본 영상 바깥의 경로 허용 범위; 결과 패딩 크기 이내로 제한 |
| `max_samples` | 2000 | 후보 샘플 예산; 초과 프레임은 빈 경로와 경고 출력 |
| `line_width_px` | 2 | 노란 선 두께, 1~10px |
| `corner_outer_enabled` | true | 코너에서 바깥 차선 기준 경로 우선 반영 |
| `corner_outer_weight` | 0.85 | 바깥 차선 기준 경로의 최대 혼합 비중, 0~1 |
| `corner_outward_offset_m` | 0.05 | 코너에서 추가로 바깥쪽으로 이동할 목표 거리(m), 0이면 이동 끔 |
| `corner_outer_window_m` | 0.60 | 회전 방향과 관측 길이를 평가하는 거리 범위 |
| `corner_outer_tangent_window_m` | 0.15 | 바깥 차선 기준 접선 추정 범위 |
| `corner_outer_min_length_m` | 0.30 | 기준으로 삼을 최소 연속 관측 길이; 평가 범위보다 작아야 함 |
| `corner_outer_min_turn_deg` | 8.0 | 바깥 차선 가중치를 올리기 시작하는 누적 회전각 |
| `corner_outer_full_turn_deg` | 25.0 | 충분한 관측에서 최대 가중치에 도달하는 누적 회전각 |
| `smoothing_enabled` | true | 최종 경로의 두 단계 평활화 |
| `smoothing_sigma_m` | 0.04 | 첫 단계 국소 평활화 표준편차 |
| `smoothing_window_m` | 0.65 | 두 번째 단계 국소 2차 다항식 계산 범위 |
| `smoothing_max_shift_m` | 0.03 | **각 평활화 단계**의 최대 이동량; 두 단계 누적은 최대 6cm |
| `smoothing_strength` | 1.0 | 최종 평활화 강도, 0~1; 0이면 최종 평활화 생략 |
| `straight_turn_deg` | 12.0 | 이 각도 이하에서는 직선 평활화 강도 유지 |
| `corner_turn_deg` | 35.0 | 이 각도 이상에서는 두 번째 평활화 강도 억제 |
| `turn_window_m` | 0.30 | 직선/코너 판정 방향 변화 측정 범위 |

출렁임이 남으면 `smoothing_window_m`을 0.65→0.8 정도로 늘려 보고,
코너가 과하게 완화되면 `smoothing_strength`를 1.0→0.7로 줄이거나 창 크기를 줄인다.
최종 평활화를 꺼도 골격 방향 추정과 후보 전환에 사용하는 작은 필터는 유지된다.
거리 계산에는 `bev_processor`의 실제 범위를 사용해야 한다. 기본값은 전방 0~3m,
좌우 대칭 1.2m에 대응한다. 아래 중앙을 차량 근처의 시작 기준으로 사용하므로
비대칭/전방 크롭 영상은 원점 변환을 추가하기 전 이 가정을 그대로 적용하면 안 된다.
패딩은 관측 영역이나 도로 폭에 포함되지 않는다.

`LaneResult`에 추가한 필드:

- `centerline_points`: 가까운 곳부터 정렬된 확장 영상 픽셀 좌표, z=0. 미터/TF 좌표가 아님.
- `centerline_support`: 점 수와 같은 길이. 1=한쪽 오프셋, 2=양쪽 중점,
  3=6.5cm보다 긴 짧은 연결 구간, 4=바깥 차선 기준 혼합(`CENTER_OUTER`).
  국소 생성 근거이며 평활화 후 정확도 확률은 아님. 4에도 양쪽/한쪽 관측 모두 가능하다.
- `centerline_mask`: image와 같은 크기의 `mono8` 0/255 마스크.
- `centerline_valid`: 두 점 이상의 기하 경로가 존재함. 주행 가능 판정은 아님.
- `centerline_sample_limit_reached`: 계산 예산 초과 여부.
- `centerline_bev_width_m`, `centerline_bev_height_m`: 계산에 사용한 원본의 실제 범위.

생성 불가/비활성화 시 점과 support 배열은 비고, 마스크는 0, valid는 false다.
관측 경계가 화면 밖에 있으면 충돌 여부를 확인할 수 없다. 경계 여유 거리에는
픽셀·샘플 간격 여유분을 추가하지만 차량 외곽/장애물/조향 곡률 제한이나 프레임 간
추적을 포함하지 않는다. `labels=0`은 주행 가능 공간이 아니다.
이번 ROS 이식은 사용자 요청에 따라 빌드·테스트·젯슨 실행을 수행하지 않았다.

### 코너에서 바깥 차선 형상 우선 반영

코너 정점 부근에서 양쪽 경계의 중점 후보가 달라져, 경로 선택의 기준이 바뀌는 문제를
줄이기 위해 바깥 경계의 오프셋 경로를 우선 반영한다. **좌회전은 오른쪽(빨강),
우회전은 왼쪽(파랑)** 차선이 바깥이다. 단순히 가장 긴 조각을 바깥 차선으로 보지 않고,
차선을 따라 측정한 부호 있는 회전각으로 판별한다.

바깥 차선의 곡률 숫자를 중앙선에 그대로 복사하면 반경이 맞지 않으므로,
그 경계를 안쪽으로 도로 폭의 절반만큼 이동한 **경로 위치와 진행 방향**을 혼합한다.
기존 양쪽 중점 후보와 한쪽 오프셋 후보 모두 같은 바깥 기준으로 가까워지도록 해
후보를 선택하는 쪽이 바뀌어도 경로 형태의 차이가 줄도록 구성했다.

```text
최종 후보 = (1 - 가중치) × 기존 후보 + 가중치 × 바깥 경계의 안쪽 오프셋
가중치 상한 = centerline_corner_outer_weight (기본 0.85)
```

실제 가중치는 회전각, 국소 관측 길이, 한 방향으로 도는 일관성, 끝점까지의 여유에
따라 0~상한 사이에서 서서히 변한다. 양쪽 회전 증거가 충돌하면 두 증거의 차이만큼만
가중치를 적용한다. 30cm 미만의 짧은 조각, 직선, 바깥 차선 부재, 너무 작은 곡률 반경으로
뒤집힌 오프셋, 기존 경계 여유 거리 조건 위반에서는 해당 기준의 영향력을 낮추거나 제외한다.
짧은 조각은 경로의 다른 후보로는 계속 사용할 수 있다.

이 변경은 **현재 프레임의 관측을 이용한 공간적 안정화**다. 이전 프레임 경로를 고정하거나
시간 평균하지 않으며, 실제 검출 위치가 움직이는 경우의 흔들림까지 제거한다고 보장하지 않는다.
주행 중 과거 경로를 재사용하려면 차량 이동 보정이 별도로 필요하다.

```yaml
line_detactor:
  ros__parameters:
    centerline_corner_outer_enabled: true
    centerline_corner_outer_weight: 0.85
    centerline_corner_outer_window_m: 0.60
    centerline_corner_outer_tangent_window_m: 0.15
    centerline_corner_outer_min_length_m: 0.30
    centerline_corner_outer_min_turn_deg: 8.0
    centerline_corner_outer_full_turn_deg: 25.0
```

바깥 차선이 안정적으로 보이면 `corner_outer_weight`를 0.85에서 0.95로 올려 비중을
높일 수 있다. 1.0도 관측 신뢰도에 따라 실제 가중치가 낮아진다.
기존 방식과 비교하려면 `corner_outer_enabled: false`로 실행한다.
두 설정 파일(`line_detactor.yaml`, `centerline_preview.yaml`)과 launch 인자에 같은 값을
연결했다. YAML 변경 후 재시작한다. 이번 코너 수정도 요청에 따라 빌드·테스트는 수행하지 않았다.

## 코너 바깥쪽 경로 오프셋과 결과 전용 프리뷰

`centerline_corner_outward_offset_m`은 기존 중앙 경로 후보를 관측된 바깥 차선
쪽으로 이동하는 추가 거리다. 기본값은 0.05m이며, `0.0`으로 끌 수 있다.
`centerline_corner_outer_weight`는 기존 형상 혼합 비중이고, 새 오프셋은 거리다.
두 값은 별개이며 `centerline_corner_outer_enabled: true`가 필요하다.

좌회전에서는 오른쪽 바깥 경계, 우회전에서는 왼쪽 바깥 경계를 향해 이동한다.
양쪽 차선에서 만드는 후보 모두 동일한 바깥 경계의 법선과 신뢰도를 사용한다.
기존 코너 판별의 회전각·관측 길이·끝점 신뢰도에 따라 이동량이 0~설정값으로
변하며, 직선이나 바깥 경계 관측이 없는 구간에서는 적용하지 않는다.

이동 구간이 관측 차선과 `centerline_min_clearance_m` 간격이나 결과 영상 범위를
침범하면 이동량을 줄인다. 후보 평활화·연결 후에도 기존 전체 경로 검사를 수행한다.
설정값은 목표 이동 거리이지 최소 이동량이나 바깥 차선과의 고정 간격 보장이 아니다.
간격 검사는 경로점 기준이며 차량 폭·후륜 궤적까지 검사하는 기능은 아니다.
최종 이동한 경로를 노란색으로 표시하고 그대로 `auto_control`에 전달한다.

```yaml
line_detactor:
  ros__parameters:
    centerline_corner_outer_enabled: true
    centerline_corner_outward_offset_m: 0.05
    preview_enabled: true
    preview_result_only_enabled: true
```

`preview_result_only_enabled: true`는 검은 배경에 좌우 차선·정지선·노란 경로와
성능 배너만 표시한다. 원본 BEV 패딩·알파 합성을 생략한다. `false`는 원본 BEV 위에
결과를 겹쳐 표시한다. `connection_enabled: false`인 raw 모드에서는 이 옵션을
적용하지 않고 기존 raw 추론 프리뷰를 표시한다.
연결 모드에서는 사용되지 않던 GPU 원본 오버레이 생성과 해당 D2H 복사도 생략한다.
모델 추론·중앙선 생성·ROS 결과 메시지의 계산량은 그대로다. 실제 속도 향상은 미측정이다.

자동주행에서도 YAML 또는 명시적 launch 인자로 조절할 수 있다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  line_detactor_params_file:=/absolute/path/line_detactor_test.yaml \
  centerline_corner_outward_offset_m:=0.05 \
  preview_enabled:=true \
  preview_result_only_enabled:=true
```

기존 BEV/auto_control/CAN 인자를 함께 전달할 수 있다. 명시한 launch 값이
YAML보다 우선하며 변경 후 재실행한다.

## 결과 토픽과 좌표

- `/line_detactor/result`: `line_detactor/msg/LaneResult`
- `/line_detactor/result_image`: `sensor_msgs/msg/Image`, `bgr8`, 차선·정지선·노란 중앙 경로를 그린 검은 배경 영상

`LaneResult.image`는 같은 BGR 결과이고 `labels`는 기존 좌우 차선용 `mono8`이다.
라벨은 `0=배경`, `1=왼쪽 모델`, `2=오른쪽 모델`, `3=왼쪽 바깥 보간`,
`4=오른쪽 바깥 보간`이다. `stop_line_mask`는 별도의 `mono8` 0/255 정지선 마스크이며
image/labels와 같은 180×300(기본 패딩)의 좌표를 쓴다. 교차점은 labels와 stop_line_mask
양쪽에 동시에 남을 수 있다. `stop_line_present`는 정지선 픽셀이 1개 이상인지를 나타내며
확정된 정지 명령은 아니다. 정지선만 있으면 state=NONE이어도 stop_line_present=true다.
세 종류 모두 없으면 검은 영상, NONE, 빈 정지선 마스크를 발행한다.
원본 timestamp/frame_id를 유지하며 QoS는 best effort, volatile, KeepLast(1)이다.

`processing_mode=BORDER_ONLY`이다. `state`는 **남아 있는 마스크의 좌우 존재 여부**
(`0=NONE`, `1=LEFT_ONLY`, `2=RIGHT_ONLY`, `3=BOTH`)이며 한 개의 연속 차선이
확보되었다는 의미가 아니다. 계획 입력으로 해석할 때 이 차이를 반영해야 한다.

`left/right` 점 목록은 **경계 방향 추정에 사용한 성분 중심선 및 보간 곡선의
진단용 구간 목록**이다. 내부 성분/폐곡선까지 전부 표현하는 목록이 아니다.
`segment_starts`의 각 시작 오프셋에서 다음 오프셋 직전(마지막은 배열 끝)까지가
한 구간이며 **서로 다른 구간을 선으로 잇지 않는다**. 점별 provenance는
`0=모델 지지`, `1=보간`이다. 관측 길이도 진단에 사용한 경계 성분만 집계한다.
전체 좌우 검출은 `labels`, 정지선 검출은 `stop_line_mask`가 기준이며 `image`는 합성 표시다.

점은 확장 영상의 픽셀 좌표(x 오른쪽, y 아래쪽, z=0)이며 미터/TF 위치가 아니다.

```text
u_source = u_result - padding_left
v_source = v_result
```

미터 변환에는 실제 `bev_processor` 배율/원점을 사용한다. 검은 여백/라벨 0은
자유 공간을 뜻하지 않는다. 입력 중단/프레임 처리 실패 시 새 메시지가 없으므로
소비자는 timestamp로 최신성을 확인해야 한다.
`vehicle_bringup auto_drive.launch.py`는 이 결과의 중앙선 점열을 `auto_control`에 연결한다.
`line_detactor_params_file`로 차선·중앙선 YAML을 지정하며, BEV 배율은 `bev_params_file`에서
검출기와 제어기에 함께 적용한다. 정지선 검출은 이 주행 제어의 정지 명령으로 사용하지 않는다.

## 속도와 기타

프리뷰 `infer`는 순수 TensorRT 시간, `model FPS`는 그 역수다.
`connect`는 CPU 노이즈 제거·경계 끝점 추출·보간·중앙 경로 생성/평활화·결과 생성 시간이고 `correct`는
GPU 라벨 생성/D2H까지 포함한 후처리 시간이다. `connect`는 `correct`에 포함된다.
프리뷰 합성/창 표시, ROS 직렬화·발행은 해당 측정에 포함하지 않는다.
`view FPS`는 전용 GUI 스레드의 실제 표시 빈도다. 로그의 `input`, `processed`,
`preview`는 각각 수신·추론/후처리 완료·화면 표시 FPS이며 단계별 평균/최대 ms도 표시한다.

추론 작업자는 새 BEV를 받으면 즉시 처리하며 프리뷰 FPS를 기다리지 않는다.
차선·중앙선 결과를 먼저 발행한 뒤 표시용 최신 결과를 GUI 스레드에 전달한다.
영상 합성, `imshow`, `waitKey`는 GUI 스레드에서 수행한다. 프리뷰를 끄더라도
추론 주기는 동일하다. 프리뷰를 닫으면 기존처럼 검출 노드가 종료된다.

입력과 프리뷰 모두 최신 한 프레임을 보관한다. 처리가 입력보다 느리면 중간 영상은
건너뛰고 `skipped`에 집계한다. 모든 카메라 프레임의 처리나 고정 FPS를 보장하지 않으며,
ROS 전송과 GPU/CPU 처리 지연은 남는다. 오래된 프레임을 무제한 적재하지 않는다.

logits 전체는 CPU에 복사하지 않고 120×300 mono8 두 장(좌우 라벨 + 정지선 마스크)을
한 번에 전달한다. 36KB에서 72KB로 늘며 CUDA stream, pinned memory, GPU 전처리·threshold·
오버레이 경로는 유지한다. FP16/INT8로 정밀도를 바꾸지 않고 기존 TensorRT FP32를 유지한다.
ONNX 기본 경로는 패키지의 `fast_scnn_stop_line_120x300_batch_1.onnx`다. 최초 실행 시 장치별
FP32 TensorRT 엔진을 생성하고 ONNX 옆 `*.trt<TensorRT-major>.fp32.engine`를 재사용한다.
`Q`/`ESC`/창 닫기로 종료한다. 자동주행·조향 명령은 발행하지 않는다.

Windows에서 ONNX checker/ONNX Runtime CPU 수치 비교를 수행했다. 변경 시점의 best(epoch10)
스냅샷으로 검증 입력10개(검증 이미지8장+합성2개)에서 세 채널 마스크가 모두 일치했고,
로짓 최대 절대오차는 약0.000290이었다. 해당 ONNX는 원본 작업공간의
`line_detector/training/highres/stop_line_0910/onnx_checks/current_best_snapshot.onnx`에 있는
검증용이며 최종 배포 모델이 아니다. 학습이 끝나면 위 변환 명령을 다시 실행한다.

사용자 지침에 따라 **ROS 빌드·노드 실행·젯슨 TensorRT 엔진 검증은 수행하지 않았다.**
새 메시지 필드가 있으므로 실제 젯슨에서 패키지와 관련 소비자를 재빌드해야 한다.
