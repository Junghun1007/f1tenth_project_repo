# auto_control — 자동 주행

차선 추종 + 콘 기억·회피 + 신호등 정지 + 앞차 추종 + duty-only 속도 제어 + 브라우저 GUI.
주행 판단부는 `cone_drive_test`에서 실차 검증한 코드를 옮겨 왔다(파일 첫 줄에 출처). `cone_drive_test` 패키지를 지워도
빌드·실행에 영향이 없다. 설계: `docs/superpowers/specs/2026-11-20-auto-control-cone-integration-design.md`

## 구성

```
[bev_processor_container (한 프로세스, 영상·깊이·차선 결과를 복사 없이 공유)]
 bev_processor  시작 측정(카메라 높이·roll·pitch) → 카메라 장착값 공유
 camera_driver  OAK CAM_A NV12 1280×800 70fps + IMU 영상 안정화 + 스테레오 깊이(CAM_A 정렬 640×400, 30fps)
   ├─ BEV(CUDA) → line_detactor(차선 Fast-SCNN) → LaneResult(중앙선·차선 경계·정지선 거리)
   └─ 직접 전달 NV12
 auto_control::AutoDriveNode (src/drive/auto_drive_node.cpp)
   ├ 검출 스레드: 최신 프레임만, 평균 40fps(yolo.max_fps) — YOLOX-tiny(con/truck/sedan/traffic_light)
   │    신호등: 화면 전체 추론, 점수 가장 높은 신호등 박스의 HSV 색 / 콘·차량: 하단 ROI 안, 박스 안 깊이 → 위치
   ├ 제어: LaneResult마다 콘 기억 → Frenet 회피 계획·롤아웃 → 신호등 정지·앞차 추종 속도 상한 → Stanley·duty-only → 상태기계
   ├ 웹: http://<jetson-ip>:8765 (SSE 텔레메트리, BEV 원본·카메라 MJPEG은 켰을 때만)
   └ 출력: /auto_drive/auto/{duty,brake_current,servo_position}
 (tunnel_enabled:=true) shared_ir_bev · ir_line_detactor · TunnelLaneSelector

[safety_node (Python)] /vesc/*의 유일한 발행자. MANUAL로 시작(조이스틱 수동 주행) → [주행 시작] 승인 시 AUTO
[vesc_bridge] [vehicle_dynamics_monitor] [joy_initializer]
```

## 연산 구조 (오린 나노 8GB 기준으로 정리)

- **영상은 프로세스 밖으로 나가지 않는다**: 카메라·BEV·차선·YOLO·주행 컴포넌트가 한 컨테이너. 프로세스 사이로는 작은 메시지만 오간다
- **원본 GPU 업로드 1회** (`perception.yaml` `gpu_shared_upload_enabled`): camera_driver가 원본 NV12를 GPU 버퍼 풀에 한 번 올리고
  BEV(bev_processor)와 YOLO가 같은 버퍼를 읽는다 (전에는 BEV용 호스트 크롭 복사 + BEV 업로드 + YOLO 업로드).
  신호등 색 판정과 웹 카메라 보기만 같은 시각의 호스트 프레임을 쓴다. 풀이 바쁘거나 실패하면 그 프레임만 예전 경로
- **터널 OAK 부하**: RGB 솎기 장치 Script 게이트를 없애고 호스트에서 버림 (스테레오 게이트는 유지: 터널 안에서 StereoDepth를 쉬게 함), IR 왜곡 보정은 IR BEV LUT에 합침.
  `camera_config.yaml`의 `tunnel.ir_resolution`(800p|400p): 왼쪽 흑백 센서를 깊이와 같이 써서 800p면 터널 밖에서도 센서가 800p로 돈다
- **터널 TCN**: camera_driver가 프레임 통계 7개를 C++로 계산해 `/tunnel/rgb_stats`로 보낸다 (전에는 원본 1.5 MB × 30 Hz를 Python 프로세스로)
- **콜백 그룹**: IMU(400 Hz) · 차량 입력 · 차선 제어 · 상태 · 웹을 나눠 회피 계획 중에도 IMU 요레이트가 밀리지 않는다
- **웹 카메라 보기**: 변환·그리기·JPEG 인코딩을 검출 스레드 밖에서 (켜도 검출 FPS 그대로)
- **메시지 패키지**: `LaneResult`는 `line_detactor_msgs` (auto_control은 CUDA 패키지 없이 메시지만 의존)

## 빌드·실행 (Jetson, 워크스페이스 루트)

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch vehicle_bringup auto_drive.launch.py                  # 자동 주행 모드
ros2 launch vehicle_bringup auto_drive.launch.py mode:=detect     # 검출 테스트 (자동 명령 없음, 조이스틱 수동 주행)
ros2 launch vehicle_bringup auto_drive.launch.py tunnel_enabled:=true
# 옵션: speed_mps:=1.0  drive_params_file:=<yaml>  web_port:=8765  vehicle_namespace:=autopilot03
#       vesc_port:=/dev/ttyTHS1  joy_enabled:=false  preview_enabled:=true(Jetson 차선 창)
```

브라우저로 `http://<jetson-ip>:8765`를 연다. 인증이 없으므로 차량 전용 네트워크에서만 쓴다.
수동 주행 launch, cone_drive_test 등 OAK·VESC를 쓰는 다른 실행은 끈다.

## 출발

1. 차를 평평한 바닥에 세운 채 실행 → bev_processor가 카메라 자세를 측정(점검 영역 "카메라 자세 PASS")
2. 차선·YOLO 모델 READY (첫 실행은 TensorRT 엔진 생성으로 수 분) → 상태 READY
3. [주행 시작]: 점검 통과(자세·모델·VESC·ERPM·차선·브라우저 연결, 터널이면 IR 모델) → safety_node가 정지를 확인하고 AUTO
   - **조이스틱을 먼저 만질 필요 없다.** 조이스틱 입력이 없으면(미연결·꺼짐) 조작 해제로 본다
   - 조이스틱이 연결돼 있으면 페달·조향이 풀려 있어야 AUTO를 허용한다 (RT를 밟은 채 넘어가지 않게)
   - 조이스틱이 없을 때는 X 비상 정지를 쓸 수 없다 (상단 "조이스틱: 없음 (X 정지 불가)" 표시). 브라우저 [정지]·창 닫기로 멈춘다

## 안전 동작

| 상황 | 동작 |
|---|---|
| 실행 직후 | safety_node MANUAL. 자동 명령은 무시 |
| [정지] / 고장 | 제동 → 정지 확인 → MANUAL. 다시 [주행 시작] 가능 |
| 고장 조건 | 차선 끊김(`drive.lane_timeout_sec`), ERPM 0.2 s, VESC 끊김, 모든 브라우저 heartbeat가 `web.link_loss_stop_sec`(1 s) 끊김 |
| 조이스틱 X | 언제든 자동 명령 차단 → 제동 → 정지 확인 → MANUAL (주행 컴포넌트가 멈춰도 동작) |
| 자동 명령 끊김 | 0.3 s 안 오면 safety_node가 제동 |

## 화면
- **파라미터 → YAML 저장**에서 현재 적용값을 이름 있는 프리셋으로 저장한다.
  예를 들어 `ir개선`은 브라우저의 날짜·분을 붙여 `202611251852_ir개선.yaml`로 저장한다.
  파일은 Jetson의 기본 `~/.cache/auto_control/presets/`에 있으며, 저장된 프리셋을 선택해
  **불러오기**를 누르면 설정을 검증한 뒤 카메라·BEV 초기화와 함께 새 컨테이너에서 적용한다.
  같은 분에 같은 이름으로 저장하면 숫자 접미사가 붙는다. 기본 설정 파일은 `202611251743_ver3.yaml`의 튜닝값을 사용한다.
- Refit 옆 **초기화**는 수동 모드의 정차 상태에서 인식 컨테이너를 재시작한다.
  카메라를 닫고 다시 열면서 시작 자세 보정을 수행한다. 초기화 버튼은 현재 설정을,
  프리셋 불러오기는 지정한 YAML 값을 새 인식 컨테이너에 넘긴다. VESC·조이스틱·안전 노드는 계속 실행된다.
- **파라미터 → 차선 → RGB · 카메라**에서 전체 자동, 자동 노출·고정 ISO 또는 전체 수동을 선택한다.
  고정 ISO 모드는 `rgb_cam.iso`를 그대로 쓰고 RGB 밝기에 따라 호스트가 노출 시간만 조절한다.
  설정은 실행 중에 적용되고 YAML로 저장할 수 있다. **카메라 노출** 패널에는 현재값과 RGB 시작 측정값이 표시된다.
- 파라미터 "IR 카메라 · 터널 안 (차선)" / "IR 카메라 · 터널 밖 (장애물 깊이)": 스테레오 흑백 카메라 노출(µs, 밖은 0=자동노출)·ISO·점 투사기·플러드 조명.
  바꾸면 camera_driver가 바로 다시 적용한다 (터널 안 IR 프레임 노출 검사 기준도 같이 바뀜)
- **주행 → IR 카메라 · 터널 안 (차선)**에서 IR 영상 해상도를 시작값(기본 800p)과 400p 사이에 바꿀 수 있다. **적용**을 누르면 OAK가 전송하는 CAM_B 영상이 전환되며, 카메라 화면에 실제 해상도·노출·ISO가 표시된다.
  터널 안 기본값은 노출 20,000 µs, ISO 500, 점 투사기 0, 플러드 조명 1이다.
  센서 구동 해상도까지 낮추려면 camera YAML의 `tunnel.ir_resolution`을 `400p`로 바꾸고 다시 시작한다.
- 조이스틱 버튼 (8BitDo): **A** 주행 시작, **B** Refit, **X** 정지 (X는 safety_node에서도 제동 → MANUAL). `joy.start_button`/`joy.refit_button`/`joy.stop_button`
- 차량 추종: 앞차가 `acc.lost_timeout_sec`(0.3 s) 넘게 안 보이면 크루즈를 끝내고 정상 주행

- 상단: 모드, 상태, 안전 전환(AUTO/제동 중/MANUAL), 차선 입력(RGB/IR 터널), 신호, [주행 시작] [정지] [Refit], 녹화, [프리뷰] [BEV 원본] [카메라], [파라미터]
- 터널 안(ENTERING~EXITING): [카메라]는 IR 카메라(CAM_B 왼쪽 흑백) 원본, [BEV 원본]은 IR BEV(120×300, 2배 확대)를 보여 준다
- **파라미터 → 차선**에서 RGB와 터널 IR의 검출, 차선 연결, 중앙선, 코너, 평활화 및 정지선 거리를 센서별로 조절한다. 터널 IR 카메라 설정도 이 탭에 있다. 가장자리 보간선은 기본적으로 중앙선 계산에 사용하며, 관측 차선 길이는 모델이 검출한 조각으로 계산한다. 추론 방식과 보간 여백은 YAML 저장 후 해당 파일로 재시작할 때 적용한다. 나머지 검출기 설정은 다음 프레임부터 적용된다.
- **파라미터 → 차선 → 공통 → 차선 검사 모드**에서 0(추가 검사 끔), 1(기존 검사, 기본값), 2(회피 방향별 차선 복귀)를 선택한다. 모드 2는 이탈 방향과 복귀 시점을 차량 자세로 판단한다. 왼쪽으로 이탈한 경우 복귀에서 처음 보이는 적합한 경계를 왼쪽으로 지정하고, 같은 쪽 바깥 후보는 제외한다. 오른쪽은 대칭으로 처리한다. 차체가 차선 안에 돌아온 뒤 반대쪽 경계의 폭이 최소 50 cm일 때 양쪽을 사용한다.
- **[Refit]**: 주행 중이면 먼저 멈춘 뒤, 카메라를 닫지 않고 실시간 깊이로 지면을 다시 맞춰(bev_processor) 높이·roll·pitch·BEV·영상 안정화 기준을 갱신하고
  콘·차량 기억을 모두 지운다. 측정 실패(흔들림 등)면 이전 보정을 유지하고 사유를 표시. 평평한 바닥에 세우고 몇 초간 움직이지 말 것
- BEV: 차선·중앙선·회피 경로·관측 콘·기억 콘·앞차·정지선
- 주행 상태: 상태, 속도를 줄이는 이유(신호등 정지·앞차 추종·콘 회피), 안전 전환·차선 입력·터널·조이스틱
- 신호등·정지선: 램프, 검출 점수, 단계(대기/정지선 추적/정지 접근/정지 유지), 정지선 거리, 속도 상한, 검출 FPS,
  앞범퍼 위치와 정차 허용 구간 막대
- 단계별 연산(ms/FPS): 차선 지연(촬영→제어)·전처리·추론·형상, YOLO 전처리·추론·후처리, 깊이 정합, 기억+회피 계획, 제어, 웹
- 파라미터 창 [장애물] [주행] [차선] [신호등]에서 변경을 한 번에 적용하고 YAML로 저장할 수 있다. 다음 실행에서 `drive_params_file:=~/.cache/auto_control/auto_drive_saved.yaml`을 지정하면 저장한 값을 불러온다.
- 실차 없이 화면 보기: `web/index.html?demo&still` (`&scene=red|green|lead|tunnel`, `&cam=1`, `&params=1`)

## 신호등 정지 (`traffic.*`)

| 상황 | 동작 |
|---|---|
| 정지 조건 | **빨간불 + 정지선**. 빨간불만으로는 서지 않음 |
| 빨간불, 정지선 아직 없음 | 정상 속도로 달리며 찾음 (빨간불을 본 뒤 `traffic.red_memory_m` 4 m 동안 기억). 찾으면 정지선 앞에 정차 |
| 찾았는데 너무 가까움 | 통과 (예상 정지 거리 = 응답 지연 + v²/2·`traffic.max_decel_mps2`가 정지선까지 거리 + 0.05 m보다 길 때). 그 정지선을 지날 때까지 다시 서지 않음 |
| 정지 중 신호등이 안 보임 | 초록불을 볼 때까지 정지 유지 |
| 정지선이 안 보임 (차 아래 등) | 바퀴 이동 거리로 위치를 이어 가 그 앞에 정차 |
| 색 판정 | 같은 색 연속 3프레임 (`traffic.signal_confirm_frames`) |
| 신호등 여러 개 | 가장 크고 화면 가운데에 가까운 것 |
| 초록불 | 바로 출발 |

- 색: 고른 신호등 박스 픽셀을 HSV로 보고 빨강/초록 에너지 비교
- 정지선: line_detactor 정지선 거리, 연속 확인(`traffic.line_confirm_frames`) 후 추적
- 정차 위치: 앞범퍼가 정지선 앞 0~`traffic.stop_range_max_m`. 종방향은 duty-only(전류 제동 없이 duty를 줄이고 관성 정지)
- 웹 신호등 패널 단계: 대기 / 빨간불·정지선 찾는 중 / 정지선 추적 / 정지 접근 / 정지 유지 / 통과(못 설 거리)

## 기존 auto_control에서 바뀐 것

- 제어: `auto_control_node`(Stanley + 전류 단계 제동 + 신호등 통과 판단·임시 정지 + 곡률 감속) → cone_drive_test 제어 루프
- 장애물: 깊이 점구름 군집(`ObstacleDetectorNode`·`AvoidancePlan`) → YOLO 박스 깊이 + 콘 기억 + Frenet 회피
- 신호등: `traffic_detection_test`(전용 YOLOX-s 640×160) → 멀티클래스 YOLOX-tiny의 traffic_light
- 화면: Python 원격 프리뷰·점검 창 → 내장 웹 서버(C++)
- 안전 전환: `manual_control auto_handover`(AUTO → MANUAL 한 방향) → `safety_node`(MANUAL ↔ AUTO 반복)
- 다른 패키지: `line_detactor`(LaneResult에 차선 경계·기준 중앙선, 좌우 재판정), `camera_driver`(깊이 CAM_A 정렬,
  IMU 보정에 카메라 앞쪽 거리 항), `bev_handoff`(차선 힌트, BEV 보기 탭)
- 그대로: 카메라·BEV·차선 파이프라인, 터널 IR 전환(`AUTO_CONTROL_TUNNEL.md`)

## 기록·분석 (`tools/`)

- 주행 시작마다 `~/auto_control_data/cone_obs_<시각>.csv` (콘 관측): `python3 tools/analyze_cone_obs.py <csv>`
- 웹 [● 녹화 시작]으로 수동 주행 녹화: `~/auto_control_data/manual/<시각>_<상황>/`
  - `python3 tools/analyze_manual.py <폴더>...` (사람이 지나간 빈틈·추천 여유)
  - `python3 tools/analyze_layout.py <폴더>` (정답 배치로 검출 위치 보정)
  - 계획 파라미터 맞추기: `cmake -S src/auto_control/test/host -B build/acd && cmake --build build/acd --target fit_planner`
    → `build/acd/fit_planner --fit ~/auto_control_data/manual/*`

## 검증

개발 PC(ROS·CUDA·TensorRT 없음)에서:

```bash
cmake -S src/auto_control/test/host -B build/acd && cmake --build build/acd -j4 && ctest --test-dir build/acd   # 주행부 25개
python3 -m pytest -q src/auto_control/test/python                                                             # 안전 전환
PYTHONPATH=src/auto_control python3 -m pytest -q src/auto_control/test/tunnel                                 # 터널 선택기
node src/auto_control/test/web/bev_geometry.test.js                                                            # 웹 BEV 좌표 (playwright)
```

ROS 2 Humble 헤더로 `auto_control`(TensorRT 끔, `-DAUTO_CONTROL_TENSORRT=OFF`)을 실제로 빌드하고, 컨테이너에 컴포넌트를 올려
가짜 차선·VESC·시작 측정 입력으로 점검 → [주행 시작] → AUTO(조이스틱 없음) → duty 출력 → [정지] → MANUAL → 재출발까지 확인했다
(YOLO는 `model.yolo_backend:=CPU`, `model.yolo_precision:=fp32`).

**Jetson에서 확인 필요**: colcon 전체 빌드(CUDA·TensorRT·DepthAI), 깊이 CAM_A 정렬과 콘 위치(1.25배 확대 RGB ↔ 깊이 내부 파라미터 변환),
YOLO 엔진·FPS, 신호등이 화면 전체 추론에서 잡히는지, IMU 요레이트 방향, 실차 주행·터널 전환.
