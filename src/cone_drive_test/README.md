# cone_drive_test — 콘 회피 실차 테스트 패키지

`tools/avoidance_sim`에서 검증한 회피 알고리즘(콘 기억 + Frenet DP + 롤아웃 검사)을 실차에서 시험하는 **독립 패키지**다.
필요한 코드는 multiclass_yolo, line_detactor, auto_control, camera_driver, bev_processor, oak_startup, manual_control에서
복사했다(각 파일 첫 줄에 출처). 그 패키지들을 지워도 빌드·실행에 영향이 없다. 하드웨어 드라이버 `vesc_bridge`,
`joy_initializer`만 launch에서 함께 실행한다.

설계: `docs/superpowers/specs/2026-11-16-cone-drive-test-design.md`

## 구성

```
[주행 프로세스 cone_drive_test_node (C++)]
 OAK CAM_A NV12 1280×800 70fps ─┬─ IMU → 영상 안정화 (camera_driver 복사)
                          ├─ [차선 스레드] 매 프레임: CUDA BEV(120×300, GPU) → TensorRT Fast-SCNN INT8(GPU 버퍼 직접, line_detactor 기본과 같음)
                          │      → lane_connector·centerline(코너 바깥 이동) → 제어
                          └─ [검출 스레드] 최신 프레임 중 평균 40fps(yolo.max_fps): YOLOX(ROI 65%, NV12 CUDA 전처리) → 박스 안 깊이(CAM_A 정렬) → 콘 위치·크기
 제어: 오도메트리(ERPM+자이로) → 콘 기억 → 회피 계획 → Stanley·속도 계획 → duty/제동 단계 제어 → 상태기계
 출력: cone_drive/auto/{duty,brake_current,servo_position}  ← /vesc/*에는 직접 내지 않음
 웹: http://<jetson-ip>:8765 (SSE 텔레메트리, BEV·카메라 MJPEG, 명령)

[안전 전환 프로세스 safety_node (Python, manual_control 복사)]  ← vesc/*의 유일한 발행자
 MANUAL로 시작(조이스틱 주행) · START 승인 시 AUTO · X 버튼 = 언제든 자동 명령 차단 → 제동 → 정지 확인 → MANUAL
 자동 명령이 0.3 s 끊기면 제동
```

토픽은 모두 `vehicle_namespace`(기본 `autopilot03`) 아래 상대 이름이다. 예) `/autopilot03/vesc/duty`.

## 빌드 (Jetson, 워크스페이스 루트)

```bash
source /opt/ros/humble/setup.bash
# DepthAI 헤더가 무거워 병렬 컴파일 시 메모리 부족(cc1plus Killed)이 날 수 있다 → 2개로 제한 (안 되면 1)
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --packages-up-to cone_drive_test --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

필요: ROS 2 Humble, `/usr/local` DepthAI 3.6 SDK(기존 워크스페이스와 같음, `cmake/DepthaiWorkspace.cmake`),
CUDA, TensorRT 개발 라이브러리, OpenCV 4, `vesc_bridge`, `joy_initializer`(실행 시).

## 실행

다른 카메라 노드(camera_driver, bev_processor, multiclass_yolo …)와 기존 수동/자동 주행 launch는 **끄고** 실행한다.

```bash
# 추적 주행 모드 (기본): 점검이 끝나면 브라우저 [주행 시작]으로 출발
ros2 launch cone_drive_test cone_drive_test.launch.py

# 검출 테스트 모드: 자동 명령 없음, 조이스틱 수동 주행 가능
ros2 launch cone_drive_test cone_drive_test.launch.py mode:=detect

# 옵션
#   params_file:=<yaml>  vehicle_namespace:=autopilot03  vesc_port:=/dev/ttyTHS1
#   vesc_enabled:=false  joy_enabled:=false  web_port:=8765
```

노트북 브라우저로 `http://<jetson-ip>:8765`를 연다. 인증이 없으므로 차량 전용 네트워크에서만 쓴다.

## 화면

- 상단: 모드(검출 테스트 / 추적 주행, 정지 상태에서만 전환), 상태, [주행 시작] [정지] [Refit], [프리뷰] [BEV 원본] [카메라], [파라미터]
- **프리뷰**를 끄면 Jetson은 오버레이 좌표·단계 통계·사용률·영상을 만들지 않고 상태만 초당 2회 보낸다(버튼과 연결 감시는 유지)
- BEV: 차선(파랑/빨강), 중앙선(노랑 점선), 관측 콘(주황, 점수), 기억 콘(분홍 점선), 회피 경로(청록, 롤아웃 실패 시 점선), 차량.
  **BEV 원본**을 켜면 실제 BEV 영상을 밑에 깐다(켰을 때만 GPU→CPU 다운로드·JPEG)
- 카메라: 원본 + YOLO 박스·ROI 선 + 검출 목록 (켰을 때만 인코딩)
- 오른쪽: 출발 전 점검(카메라 자세, 모델별 로딩 단계·시간), 차량, 회피, 시스템(CPU·이 프로세스·GPU·메모리), 단계별 평균 ms/FPS
- 파라미터 창: [장애물] [주행] 탭. 값을 바꾸고 [적용](일괄 검증, 하나라도 틀리면 전부 거부), [YAML 저장](`~/.cache/cone_drive_test/cone_drive_test_saved.yaml`, 다음 실행에 `params_file:=`로 사용)

화면 예시: `sim_output/preview/*.png` (시뮬레이터 데이터, `web/index.html?demo&still`로 실차 없이 볼 수 있음)

## 콘 기억 방식 (`memory.mode`, 파라미터 창 [장애물] → 콘 기억에서 실시간 전환)

| 값 | 방식 | 특징 |
|---|---|---|
| 0 (기본) | **월드 기억** | 콘을 오도메트리(ERPM 이동 + 자이로 회전) 월드 좌표에 기억. 다가가며 위치를 계속 보정. 자이로·속도 배율이 틀리면 코너에서 기억 콘이 어긋나 유령 콘이 생길 수 있음 |
| 1 | **차선 기준 단기 기억** | 보이는 구간은 관측되는 콘만 사용(안 보이면 바로 삭제), 위치는 거리 가중 평균(화면 끝 가림 오류 완화). 카메라 사각지대로 들어간 콘만 차선 기준 (앞뒤 거리, 옆 거리)로 뒷범퍼를 지날 때까지 유지. 앞뒤 거리만 바퀴 이동 거리로 줄이므로 자이로 오차의 영향을 받지 않음 |

전환하면 두 기억을 모두 비우고 새로 쌓는다. 차선 기준 방식 관련 값: `memory.lane_confirm_hits`, `memory.lane_miss_delete`,
`memory.lane_match_radius`, `memory.lane_visible_s_min`(사각지대 경계, 앞차축 기준).

## IMU 영상 안정화

시작 측정 후 차량을 약 5초 정지해 두면(1 s 버림 + 4 s 기준 측정) 적용된다. 점검 영역의 "IMU 영상 안정화" 줄에 1초마다
적용률·평균 보정각·IMU 매칭 실패·한계(3°) 초과 횟수를 표시한다. camera_driver와 달리 1.25배 확대를 하지 않으므로
보정 화면이 원본 안에 다 들어오는지는 검사하지 않는다(BEV는 화면 아래쪽만 사용). 타이밍 값은 camera_driver와 같다
(`stabilization.maximum_frame_imu_wait_sec` 0.005, `..._age_sec` 0.012, `maximum_prediction_sec` 0.015, `maximum_correction_deg` 3.0).

## 안전 동작

| 상황 | 동작 |
|---|---|
| 실행 직후 | 안전 프로세스 MANUAL. 자동 명령은 무시된다 |
| [주행 시작] | 추적 주행 모드 + 점검 통과(자세·두 모델 READY·VESC·ERPM·차선·브라우저 연결)일 때만 승인 → 안전 프로세스가 정지·페달 해제를 확인해야 AUTO |
| [정지] / 고장 | 전류 제동 → 0.05 m/s 미만 0.5 s → MANUAL(조이스틱). 복구돼도 자동 재출발 없음 |
| 고장 조건 | 차선 0.25 s 끊김, ERPM 0.2 s 끊김, VESC 끊김, 모든 브라우저 heartbeat가 `web.link_loss_stop_sec`(1 s) 끊김 |
| 조이스틱 X | 언제든 자동 명령 차단 → 제동 → 정지 확인 → MANUAL (주행 프로세스가 멈춰도 동작) |
| 주행 프로세스 멈춤 | 자동 명령이 0.3 s 안 오면 안전 프로세스가 제동 |
| Refit | 주행 중이면 먼저 정지 → MANUAL 확인 → 카메라 다시 측정(평평한 바닥, 정지) → BEV LUT·안정화 기준 갱신. 실패하면 이전 보정 유지 |

## 파라미터

`config/cone_drive_test.yaml`. 실시간 항목은 브라우저 파라미터 창 또는 `ros2 param set`으로 바꾼다.
기본값 출처: 회피·기억 = 시뮬레이터 `config.py`(지그재그 탐색 결과 포함: 차선 안 여유 0.06/0.015, 차선 밖 0.10/0.005),
주행 = `auto_control.yaml`, 검출 = multiclass_yolo, 차선 = `line_detactor.yaml`.
조향 서보 값은 자동 주행 = `auto_control.yaml`(중앙 0.5), 수동 주행 = `config/safety.yaml`(중앙 0.46)로 원래 시스템과 같다.

## Jetson 첫 실행 점검 순서

1. 빌드 후 검출 모드로 실행. 로그와 브라우저 점검 영역에서 **두 모델 엔진 생성 → READY** 확인(첫 실행은 수 분)
2. 차량을 평평한 바닥에 세운 채 시작 → **카메라 자세 PASS**(높이 약 0.20 m, pitch 약 13°)
3. 검출 모드에서 콘을 앞에 놓고 [카메라]·BEV로 **콘 위치가 실제와 맞는지** 확인 (깊이 정렬 확인)
4. 바퀴를 들고 `mode:=drive`, [주행 시작] → 조향 방향·duty 확인, X 버튼·[정지]·브라우저 닫기로 정지되는지 확인
5. 파라미터 창에서 `drive.maximum_speed_mps`를 0.6 정도로 낮춰 저속 주행 → 차선 추종 → 콘 한 개 → 지그재그 순서로

## 검증 범위

이 저장소의 개발 환경(ROS·OAK·CUDA·TensorRT 없음)에서 확인한 것:

```bash
cmake -S src/cone_drive_test/test/host -B build/host && cmake --build build/host -j4 && ctest --test-dir build/host
python -m pytest -q src/cone_drive_test/test/python
```

- 콘 기억, Frenet DP, 롤아웃: 시뮬레이터 골든 데이터와 일치(DP 54/55 경우 1e-9 이내, 대칭 1건은 거울상, 롤아웃 30/30), DP 평균 0.7 ms
- 차선 후처리(복사한 lane_connector·centerline), 박스 깊이 위치 추정, 종방향·조향 제어, 상태기계, 파라미터, 텔레메트리, 웹 서버
- 제어 루프 통합(검출 모드 무출력, START→AUTO→명령, 정지→MANUAL 요청, 오래된 검출 폐기, 콘 회피 계획)
- 원본 테스트 그대로: multiclass_yolo 후처리·ONNX(CPU) 스모크, camera_driver 안정화·운동 추정
- 안전 전환 상태(시작 MANUAL, 요청, 자동 명령 끊김 제동, X)
- IMU(400 Hz) 갱신이 회피 계획을 기다리지 않음: 오도메트리 락 분리 후 최악 0.024 ms (분리 전 계획 중 수 초까지 대기), ThreadSanitizer 경고 없음
- INT8 Q/DQ 차선 모델 입출력 형식 (onnxruntime CPU)
- DepthAI·rclcpp 코드는 API 모양만 흉내 낸 스텁으로 문법만 확인

**확인하지 못한 것 (Jetson에서 확인 필요)**: colcon 빌드, OAK 실영상·깊이 CAM_A 정렬 정확도, TensorRT 엔진·FPS,
VESC·조이스틱 동작, 실차 주행. 특히 깊이를 `setDepthAlign(CAM_A)`로 RGB에 정렬했는데(multiclass_yolo는 점을 투영),
RGB는 undistort 출력이라 가장자리에서 약간 어긋날 수 있다 → 점검 3번에서 확인.

## 콘 기억 위치 틀어짐 분석 (처음 기억한 위치 ↔ 가까이서 본 위치)

- 오도메트리: 정지 중 자이로 영점(요레이트 평균, 시정수 약 2 s)을 배워 빼고 적분한다 (안 빼면 약 0.15°/s씩 방향이 돎).
- 콘 위치는 깊이 영상으로 계산하므로, 그 순간의 차 자세도 **깊이 영상 촬영 시각**으로 찾는다 (스테레오가 RGB보다 느려 최대 한 프레임 어긋남).
- 주행 시작마다 `~/cone_drive_test_data/cone_obs_<시각>.csv`에 콘 관측을 남긴다 (`log.cone_observations`, `log.dir`).
  한 줄 = 관측 하나: 시각(RGB·깊이·자세), 기억 id, 차량 기준 위치·앞면·거리, 그때 자세·속도·요레이트·자이로 영점, 월드 위치, 기억 위치.
- 분석:
  ```bash
  python3 src/cone_drive_test/tools/analyze_cone_obs.py ~/cone_drive_test_data/cone_obs_<시각>.csv
  ```
  콘마다 처음 본 위치와 가까이서(1 m 안) 본 위치의 차이, 거리 비례 치우침(스테레오 거리 배율), 시각 어긋남(자세를 몇 ms 옮기면 같은 콘 위치가 가장 모이는지)을 낸다.
  실험: 콘 4~5개를 차선 안 좌우에 세우고 (1) 직진에 가깝게 (2) 회피하며, 콘 옆을 지나가게 한 번씩.

## 수동 주행 녹화로 회피 파라미터 찾기

1. 웹 상단에서 상황(지그재그 콘 / 유도 경로 / 기타)을 고르고 **● 녹화 시작** → 수동으로 주행 → **■ 녹화 종료**.
   수동 주행 중에도 인식·기억·회피 계획은 계속 계산되므로, 사람이 달린 경로와 같은 순간 계획기가 낸 경로가 함께 남는다.
2. 저장: `<log.dir>/manual/<시각>_<상황>/` (`log.dir` 기본 `~/cone_drive_test_data`)
   - `frames.csv`: 차선 결과마다 시각, odom 자세, 속도, 요레이트, 상태, 계획 모드, 차선(왼쪽·오른쪽·중앙선), 계획 경로, 계획에 쓴 콘
   - `cone_obs.csv`: 콘 관측 (위 콘 기억 분석과 같은 형식)
   - `params.yaml`, `meta.json`: 그때 파라미터 전부, 차 치수
3. 부딪힌 주행은 버리고 분석:
   ```bash
   python3 src/cone_drive_test/tools/analyze_manual.py ~/cone_drive_test_data/manual/*_zigzag ~/cone_drive_test_data/manual/*_guided
   ```
   콘마다 사람이 지나간 최소 빈틈, 차체 차선 이탈, 계획기 빈틈, 복귀 거리를 내고, 최소 여유(안전 하한)·권장 여유·차선 허용 이탈·복귀 거리를 추천한다.

### 녹화로 계획 파라미터 맞추기 (계획 경로 ≈ 사람이 달린 경로)

녹화 프레임마다 그때의 차선·기억 콘·자세·속도로 실제 계획기(AvoidancePlanner)를 다시 돌리고, 그 순간 이후 사람이 달린 경로와
같은 주행 거리(0.3~2.0 m)끼리 비교한다. `--fit`이면 권장·최소 여유, 차선 허용 이탈, 계획 거리, 경로 가중치를 하나씩 늘리고 줄여 보며
차이가 가장 작은 값을 찾는다 (최소 여유는 2 cm 아래로 내리지 않음).

```bash
cmake -S src/cone_drive_test/test/host -B build/cdt_host && cmake --build build/cdt_host --target fit_planner
build/cdt_host/fit_planner --fit ~/cone_drive_test_data/manual/*      # 결과는 params.yaml 형식
build/cdt_host/fit_planner --set=plan.s_max=2.0 ~/cone_drive_test_data/manual/*   # 특정 값으로 점수만
```

2026-11-20 녹화 6회(지그재그 3, 유도 경로 3)로 맞춘 값이 지금 기본값이다: 경로 차이 RMS 30.2 → 23.3 cm
(지그재그 31.8 → 24.2, 유도 경로 26.3 → 21.5). 맞추는 데 안 쓴 2회에서는 23.4 → 22.0 cm.
남는 차이의 상당 부분은 파라미터로 줄지 않는다: 계획 실패 프레임 약 10%는 최소 여유를 0으로 내려도 6% 남는데,
기억 콘의 위치 오차·같은 콘의 중복 기억으로 차선 안 길이 막혀 보이는 경우다.
