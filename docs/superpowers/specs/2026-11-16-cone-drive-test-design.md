# cone_drive_test — 콘 회피 실차 테스트 패키지 설계

작성일 2026-11-16 · 기준 브랜치 `origin/1113ML` · 개발 브랜치 `claude/bold-tesla-jtg2o6`

## 1. 목적과 배경

`tools/avoidance_sim`에서 검증한 제안 회피 알고리즘(콘 기억 + Frenet DP + 롤아웃 검사)을 실차에서 시험한다.
기존 실차 구성은 차선(line_detactor)·제어(auto_control)·장애물(Depth 군집)이 여러 패키지와 여러 노드에 나뉘어 있고,
multiclass_yolo는 OAK를 직접 여는 단독 프리뷰라 주행 파이프라인과 함께 쓸 수 없다.

사용자 결정:
- 기존 패키지(multiclass_yolo, line_detactor, auto_control, camera_driver, bev_processor, oak_startup, point_cloud, roi_lidar,
  manual_control)에서 **필요한 부분만 복사**해 **새 독립 패키지**로 만든다. 기존 패키지를 지워도 빌드·실행에 영향이 없어야 한다.
- 프리뷰는 **노트북 브라우저**로 본다 (Jetson은 수치·좌표만 보내고 그림은 브라우저가 그림).
- **연산량 최적화**에 집중한다.
- 테스트는 이 환경에서 확인 가능한 중요한 부분만 간단히 실행한다.

## 2. 목표 / 비목표 / 반목표

### 목표
1. multiclass_yolo로 콘 위치·크기 추정, line_detactor 방식으로 차선·중앙선 인식, 이를 바탕으로 회피 주행
2. 횡·종방향 제어는 auto_control(Stanley, 곡률/장애물 속도 계획, duty·전류 제동), BEV 흔들림 안정화(camera_driver IMU 짐벌) 적용
3. 실행 시 출발 전 점검: 모델 로드(엔진 생성·시험 추론), 카메라 자세 추정(Depth 지면 + IMU)
4. 브라우저 프리뷰: BEV 위 차선·중앙선·장애물(위치·크기)·회피 경로 오버레이(항상 표시), BEV 원본 켜기/끄기,
   단계별 평균 FPS·연산 시간, CPU·GPU·메모리 사용률
5. 장애물·주행 파라미터를 실시간 조절하는 별도 창(버튼으로 여닫기)
6. 주행 시작 / 주행 정지(정지 후 조이스틱 수동 주행)
7. 모드 2개: **검출 테스트**(기존 multiclass_yolo와 같은 기능, 액추에이터 출력 없음) / **추적 주행**(콘 추적 기반 주행)
8. Refit: 주행 중 BEV가 틀어지면 정차 → 카메라 자세 재추정 → 재출발 대기

### 비목표
신호등, 터널, IR 차선, TCN, 정지선 정지 제어, RC카 추적, 데이터셋 수집, INT8/448×224 모델 전환 UI,
기존 HighGUI 프리뷰와 Tk 도구 창, ROS 토픽으로 BEV 영상 발행.

### 반목표 (기술적으로 돌아가도 실패로 보는 것)
- 사용자의 START 없이 duty가 나가는 것. 검출 테스트 모드에서 액추에이터 토픽을 발행하는 것
- 다른 워크스페이스 패키지에 빌드/실행 의존하는 것
- 프레임 큐가 쌓여 지연이 늘어나는 것, 프리뷰가 제어 주기를 늦추는 것
- 브라우저 연결이 끊긴 채 차가 계속 달리는 것 (heartbeat 끊김 → 정지)

## 3. 제약

- Jetson + ROS 2 Humble + `/usr/local` DepthAI 3.6 + CUDA + TensorRT (기존 multiclass_yolo 빌드 환경과 동일)
- OAK 장치 하나: 이 패키지가 직접 열고, 실행 중에는 다른 카메라 노드와 함께 쓸 수 없다
- 외부에 남는 것은 하드웨어 드라이버 패키지 2개뿐: `vesc_bridge`(`/vesc/duty`, `/vesc/brake_current`, `/vesc/servo_position`,
  `/vesc/measured_erpm`, `/vesc/connected`)와 `joy_initializer`(`/joy`, SDL 표준). **이 패키지의 launch가 둘을 함께 실행**한다
  (사용자 결정, 실행 인자로 끌 수 있음). 코드 복사 대상은 아니다. 토픽 이름은 파라미터
- 이 개발 환경에는 ROS·OAK·CUDA·TensorRT가 없다 → ROS 노드 전체 빌드와 실차 실행은 검증하지 못함.
  순수 C++ 로직(계획·추적·제어·기하)과 웹 화면, ONNX CPU 실행만 여기서 확인한다

## 4. 접근 방식

| 안 | 내용 | 장단점 |
|---|---|---|
| A. 기존 파이프라인에 YOLO 컴포넌트 추가 | camera_driver→bev_processor→line_detactor→auto_control 그대로, YOLO만 끼움 | 코드 재사용 최대. 하지만 다른 패키지 의존 → **사용자 요구(독립)와 맞지 않아 제외** |
| B. 독립 패키지, 단일 프로세스 (**선택**) | OAK 수신부터 VESC 명령까지 한 C++ 프로세스, 스레드 분리, GPU 버퍼 공유 | 노드 간 직렬화·복사 없음 → 연산량 최소. 코드는 복사로 늘어나지만 검증된 코드 그대로 사용 |
| C. 독립 패키지, 다중 노드 | 같은 패키지 안에서 카메라/차선/검출/제어 노드 분리 | 디버그는 쉬우나 영상 전달 비용(intra-process여도 NV12/BEV 복사) → 연산량 목표에 불리 |

프리뷰: 사용자가 브라우저 방식 선택. 웹 서버는 **같은 C++ 프로세스 안**(헤더 하나짜리 cpp-httplib, MIT, vendored)에 둔다.
별도 Python 노드(auto_control 원격 프리뷰 방식)는 ROS 메시지 요약·전달이 한 단계 더 생기므로 쓰지 않는다. *(builder 판단)*

## 5. 전체 구조

```
OAK (CAM_A NV12 1280×800 40fps, Stereo depth 400p, IMU 400 Hz)
 └─ [T1 카메라] 최신 1장만 보관 (장치·호스트 큐 크기 1, non-blocking)
      ├─ IMU → ImuImageStabilizer (camera_driver 복사) ─────────────┐
      ├─ [T2 차선] 매 프레임: CUDA BEV(NV12+안정화 homography → 120×300 BGR, GPU) 
      │        → TensorRT Fast-SCNN(GPU 버퍼 직접 입력) → 라벨 → 중앙선(CPU) → LaneObs
      ├─ [T3 검출] 최신 프레임만: CUDA NV12 전처리(ROI 65%) → TensorRT YOLOX → NMS
      │        → 박스 안 Depth 점만 지면·높이 필터 → 앞면 거리 → 콘 위치·크기
      └─ [T4 제어] LaneObs 도착마다: 오도메트리 → 콘 기억 갱신 → 회피 계획 → 속도 계획
               → Stanley → PID/제동 → 상태기계 → /vesc/* 발행 (추적 주행 모드 + START 승인 시만)
 [T5 웹] HTTP 정적 페이지 + SSE 텔레메트리(≤30 Hz) + BEV JPEG(켰을 때만) + 명령 POST + heartbeat
 [T6 시스템] 1 Hz: CPU(/proc/stat), 메모리(/proc/meminfo), GPU(Jetson sysfs load)
```

- 좌표계: 앞차축 기준 차량 좌표(x 앞, y 왼쪽, m) — 시뮬레이터·auto_control과 같음
- 스레드 간 전달은 "최신 값 하나" 슬롯(mutex + shared_ptr 교체). 대기열 없음
- 각 단계는 시작·끝 시각을 lock-free 링버퍼에 기록 → 1초 평균 ms/FPS를 웹으로

## 6. 구성 요소

### 6.1 카메라·시작 측정 (복사: oak_startup, camera_driver 일부, multiclass_yolo 일부)
- `oak_startup_measurement`·`ground_plane_estimator`·`startup_attitude`를 그대로 복사: 정지 상태에서 Depth 지면 RANSAC + IMU로
  높이·roll·pitch 측정 (기존 multiclass_yolo/bev_processor와 같은 기준값, `fixed_camera_*`는 측정 실패 시 수동 대체값)
- 측정 후 측정용 파이프라인을 닫고 본 파이프라인을 연다(bev_processor 방식)
- 본 파이프라인은 multiclass_yolo의 DepthAI 구성(CAM_A NV12 + 400p 스테레오 + RGB/Depth 시각 정합)에 IMU 출력을 더한다.
  camera_driver의 거대한 노드(3.5k줄) 전체가 아니라 RGB+IMU 수신, undistort, 안정화 관련 부분만 가져온다
- `ImuImageStabilizer`(+`vehicle_motion_estimator`)를 그대로 복사. 시작 측정의 지면 법선을 외부 기준으로 넣는다.
  CAN 가감속 보상은 입력이 없으므로 기본 끔(`high_frequency_only` 또는 gain 0) *(builder 판단: 실차에 dynamics 토픽이 있으면 파라미터로 켬)*

### 6.2 BEV와 차선 (복사: bev_processor, line_detactor 일부)
- `bev_geometry`(LUT 생성), `cuda_bev_processor.cu`(NV12+homography+BEV 한 번에) 그대로 복사. BEV 0~3 m × ±0.6 m, 1 cm/px → 120×300
- `tensorrt_lane_backend` + `cuda_lane_pipeline` 그대로 복사, 입력은 CUDA BEV 버퍼 직접(`infer_device_bgr`)
- 후처리 `lane_connector`(작은 성분 제거·경계 보간), `centerline`(중앙선, 코너 바깥쪽 이동 정책 포함) 복사.
  정지선 채널은 무시(계산은 모델에 포함되어 있어 그대로 두되 후처리 생략)
- 출력 `LaneObs`: 중앙선(코너 바깥 이동 포함, 제어용), 좌우 차선 점, 촬영 시각.
  회피 계획의 기준선(코너 바깥 이동 없는 중앙선)은 **근처에 콘이 있을 때만** 바깥 이동을 끈 설정으로 중앙선을 한 번 더 계산한다
  (기존 C++은 한 가지만 출력, 평소 연산 증가 없음) *(builder 판단)*

### 6.3 YOLO 검출·위치 추정 (복사: multiclass_yolo)
- `yolox_detector`, `tensorrt_yolox_backend`, `cuda_yolox_preprocessor`, `postprocess`, `object_localization` 그대로 복사.
  모델 `multiclass_yolox_tiny_512x320_holdout10.onnx`(+labels, metadata)를 패키지에 복사
- Depth 점 처리: multiclass_yolo가 쓰던 roi_lidar/point_cloud의 깊이 투영·지면 필터 중 **박스 안 점에 필요한 함수만** 복사.
  깊이 영상 전체를 점구름으로 만들지 않고 박스 픽셀만 역투영한다 (연산 절감)
- 출력: 검출마다 클래스, 점수, 박스, 앞면 거리, 중심 위치, 크기(클래스별 사각형), 촬영 시각
- 회피에는 `con`만 사용(파라미터로 다른 클래스 포함 가능). 다른 클래스는 프리뷰 표시만
- 검출 주기는 카메라와 분리: 최신 프레임만 처리, 상한 FPS 파라미터(기본 = 카메라 FPS)

### 6.4 콘 기억·회피 계획 (시뮬레이터 Python → C++ 이식, 새 코드)
- `tools/avoidance_sim/avoidance_sim/planners.py`의 `ConeMemory`, `ProposedPlanner`(center 모드, DP 단계
  lane-hard → lane-hard 완화 → free, 뒷차축 tractrix, 롤아웃 검사, 재계획 margin, 실패 시 감속)를 C++로 옮긴다
- 파라미터 기본값은 `config.py`의 현재 값(지그재그 탐색 결과 포함: 차선 안 soft 0.06 / hard 0.015, 차선 밖 0.10 / 0.005)
- 관측 시점 보정: 검출의 촬영 시각에 해당하는 오도메트리 자세(최근 2초 링버퍼 보간)로 월드(odom) 좌표에 넣는다
- "보여야 하는데 안 보임" 판정(miss)은 시뮬레이터의 카메라 래스터 대신 **간단한 기하 판정**(카메라 화각·ROI 상단 행·거리 범위
  안이고 더 가까운 기억 콘에 가리지 않음)으로 한다 *(builder 판단: 래스터 z-buffer는 실차에서 불필요한 연산)*
- 근처 콘이 없으면 계획 없이 중앙선(코너 바깥 이동 포함) 그대로 → 계산 0

### 6.5 오도메트리
- 속도: ERPM → m/s (auto_control `erpm_to_speed_mps`, 기어비 파라미터 복사)
- 요레이트: OAK IMU 자이로를 카메라 장착 회전으로 차량 z축에 투영, 시작 측정의 bias 제거
- 뒷차축 적분(시뮬레이터 `Odometry`와 같은 식)

### 6.6 제어 (복사: auto_control control_core + 노드 일부)
- `control_core`(경로 정렬, Stanley, 곡률 속도 계획, SpeedPid, 제동 프로파일, servo/ERPM 변환) 그대로 복사
- auto_control_node(2.5k줄) 중 종방향 단계 제어(duty 감속 → 부족 시 전류 제동 → 회복, 시작 duty), 장애물 속도 램프,
  조향 필터·변화율 제한, watchdog(경로·ERPM·연결 신선도), 정차 시 조향 유지를 가져온다. 신호등·터널 분기는 버린다
- 제어 경로 = 회피 계획 경로(콘 근처) 또는 중앙선. 롤아웃 실패 시 `rollout_fail_speed`로 감속(시뮬레이터와 같음)

### 6.7 상태기계·안전
```
BOOT → INSPECTING(자세 측정, 모델 로드) → READY ⇄ (START) → DRIVING → (STOP/고장) → STOPPING → MANUAL
                                               ↖──────────── START 재요청 ────────────┘
REFIT: READY/MANUAL에서 즉시, DRIVING에서는 STOPPING → 정차 확인 후 → REFITTING → READY
```
- START 조건(auto_control과 같은 취지): 추적 주행 모드, 자세 측정 PASS, 두 모델 READY, VESC 연결, 신선한 ERPM,
  유효한 최신 차선, 브라우저 heartbeat 정상. 버튼을 눌러도 제어 스레드가 이 조건을 다시 검사한 뒤에만 승인
- **프로세스 2개 (사용자 결정)**
  - 주행 프로세스(C++, 위 T1~T6): `/vesc/*`에 직접 발행하지 않는다. 자동 명령은 비공개 토픽
    `/cone_drive/auto/{duty,brake_current,servo_position}`으로만 낸다
  - 안전 전환 프로세스(Python, manual_control의 `auto_handover_node` + `handover_state` + `actuator_commander` 조이스틱 로직
    + duty/제동 프로파일을 **그대로 복사**): `/vesc/*`의 **유일한 발행자**. AUTO 상태에서는 비공개 자동 명령을 전달,
    X 버튼 또는 주행 프로세스의 수동 전환 요청이 오면 자동 명령을 차단하고 제동 → 정지 확인 → 조이스틱 수동 주행.
    주행 프로세스가 멈추거나 오래된 명령을 내도 X 버튼으로 차가 선다. 자동 명령이 timeout을 넘기면 제동
  - 기존 auto_handover와 다른 점: 이 실행에서는 X가 "영구 차단"이 아니라 **START 재승인 전까지 차단**(브라우저 START를
    주행 프로세스가 승인하면 안전 프로세스에 AUTO 재허용 요청 → 정지·페달 해제 상태에서만 수락)
- STOPPING: 주행 프로세스가 duty 0 + 전류 제동, 정지(|v|<0.05 m/s 0.5 s) 확인 후 안전 프로세스에 수동 전환 요청 → MANUAL
- 고장(차선/ERPM/연결 신선도, 브라우저 heartbeat `link_loss_stop_sec` 초과, 모델 오류) → STOPPING. 복구돼도 자동 재출발 없음
- **두 모드 모두 이 패키지가 VESC를 담당**(사용자 결정). 검출 테스트 모드는 자동 명령을 절대 내지 않고 START가 비활성이며,
  안전 프로세스는 처음부터 MANUAL(조이스틱 주행 가능). 제어 계산·계획은 프리뷰용으로 돌린다(파라미터로 끌 수 있음)
- 모드는 실행 인자 `mode:=detect|drive`로 시작하고, READY/MANUAL 상태에서 브라우저로 바꿀 수 있다.
  발행자는 항상 안전 프로세스 하나라 전환 중 명령 충돌이 없다

### 6.8 Refit
- 정차 확인 → 본 파이프라인 정지 → 시작 측정 재실행(같은 함수) → BEV LUT·안정화 기준 갱신 → 본 파이프라인 재시작 → READY
- 측정 실패 시 이전 값 유지, 화면에 원인 표시. TensorRT 엔진은 유지(재생성 없음)

### 6.9 웹 프리뷰
- 같은 프로세스 HTTP 서버(기본 `0.0.0.0:8765`), 정적 파일(HTML/JS/CSS)은 패키지 share에 설치
- **텔레메트리(SSE, ≤30 Hz, 구독자 있을 때만 생성)**: 상태·모드·속도·목표 속도·조향·CTE·duty, 좌우 차선 점(간축 ≤40점씩),
  중앙선·회피 경로(≤40점), 현재 검출(클래스·점수·위치·크기·기억/관측 구분), 기억 콘, 롤아웃 결과, 단계별 ms/FPS, CPU/GPU/메모리 %
- **BEV 원본**: 토글 ON + 연결된 클라이언트가 있을 때만 CPU 다운로드 + JPEG 인코딩(≤15 fps, 품질 70, 120×300) → MJPEG 스트림.
  OFF면 GPU→CPU 다운로드 자체를 하지 않는다
- **검출 테스트 모드의 카메라 보기**: 기존 multiclass_yolo처럼 원본 영상 + 박스 + ROI 선. 켰을 때만 640×400 JPEG(≤10 fps)
- 화면 배치(예시)
```
┌────────────── 상단: 모드[검출|주행]  상태 READY  [시작] [정지] [Refit] [BEV 원본 ON/OFF] [파라미터] ┐
│  BEV 캔버스 (원본 배경 선택, 오버레이 항상)        │  차량: 속도 / 목표 / 조향 / CTE / duty        │
│   - 좌우 차선(파랑/빨강), 중앙선(노랑)             │  점검: 자세 PASS · 모델 2/2 READY · VESC · ERPM│
│   - 콘 사각형(관측 실선 / 기억 점선), 점수         │  단계  평균 ms   FPS                          │
│   - 회피 경로(청록), 롤아웃 실패 시 빨강           │   수신 / 안정화+BEV / 차선 추론 / 중앙선       │
│   - 차량 외곽, 0.5 m 격자                          │   YOLO 전처리 / 추론 / 후처리 / Depth 정합    │
│                                                    │   기억+계획 / 제어 / 웹                       │
│                                                    │  CPU  GPU  메모리 %                           │
└──────────────────────────────────────────────────────────────────────────────────────────┘
```
- **파라미터 창**: 버튼으로 여닫는 오버레이 패널. 탭 [장애물] [주행]. 숫자 입력 + Apply(일괄 검증 후 원자적 적용).
  실시간 적용 항목과 재시작 필요 항목을 구분 표시. [저장]은 현재 값을 YAML로 기록(다음 실행에 사용)
  - 장애물: 클래스별 임계값, NMS, ROI %, 검출 FPS 상한, 위치 추정(box core, 최소 점 수·비율, 거리 창), 지면 제거·최대 높이,
    기억(gate, merge, 확인 hits, miss 삭제, 뒤 삭제 거리), 계획(clearance 4종, 가중치, 롤아웃 시간, 재계획 margin)
  - 주행: 최대/최소 속도, 장애물 속도, 롤아웃 실패 속도, 횡가속 한계, 감속도, Stanley gain·softening·heading lookahead,
    조향 필터, 최대 duty, 코너 바깥 이동
- 명령 API: `POST /api/start|stop|refit|mode|params`, `GET /api/params`, `POST /api/heartbeat`(200 ms). 인증 없음(차량 전용 망 전제, auto_control 원격 프리뷰와 같음)

### 6.10 시스템 사용률
- CPU: `/proc/stat` 차분, 프로세스 CPU: `/proc/self/stat`
- 메모리: `/proc/meminfo` (Jetson은 CPU·GPU 공유 메모리)
- GPU: `/sys/devices/platform/gpu.0/load` 또는 `/sys/devices/gpu.0/load`(0~1000) 순서로 탐색, 없으면 `--` *(builder 판단)*

## 7. 연산량 최적화 원칙
1. 영상은 GPU에 머문다: NV12 업로드 1회 → BEV/YOLO 전처리 모두 GPU, 차선 모델은 BEV GPU 버퍼 직접 입력
2. 최신 프레임만 처리, 큐 없음. 검출은 차선과 별도 스레드·별도 CUDA stream
3. Depth는 박스 안 픽셀만 역투영 (전체 점구름 없음)
4. 회피 계획은 콘이 근처에 있을 때만, C++ 벡터 연산. 목표 평균 ≤3 ms
5. 프리뷰는 좌표·수치만, 영상은 토글 ON + 클라이언트 연결 시에만 인코딩
6. 신호등·터널·IR·TCN 모델과 코드 없음
7. 프레임당 동적 할당 최소화(버퍼 재사용), 로그는 1초 요약

## 8. 복사 목록 (출처 → 패키지 안 위치, 네임스페이스는 `cone_drive_test`로 변경)
| 출처 | 가져오는 것 |
|---|---|
| oak_startup | ground_plane_estimator, oak_startup_measurement, startup_attitude |
| camera_driver | imu_image_stabilizer, vehicle_motion_estimator, (노드에서) OAK RGB+IMU 구성·undistort 부분 |
| bev_processor | bev_geometry, cuda_bev_processor.cu |
| line_detactor | tensorrt_lane_backend, cuda_lane_pipeline, lane_connector, centerline, processing_profile(필요 시), 모델 `fast_scnn_stop_line_120x300_batch_1.onnx`+json |
| multiclass_yolo | yolox_detector, tensorrt_yolox_backend, cuda_yolox_preprocessor, postprocess, object_localization, 모델+labels+metadata, DepthAI RGB/Depth 정합 부분 |
| point_cloud / roi_lidar | 박스 점 역투영·지면/높이·고립점 필터에 필요한 함수만 |
| auto_control | control_core 전체, 노드의 종방향 단계 제어·장애물 램프·watchdog·조향 처리 |
| manual_control | auto_handover_node, handover_state, actuator_commander, duty_command_profile, brake_current_profile (Python 그대로, 안전 전환 프로세스) |
| avoidance_sim | ConeMemory, ProposedPlanner (Python → C++) |
| 외부 라이브러리 | cpp-httplib (단일 헤더, MIT, `third_party/`) |

## 9. 검증
여기서 실행(간단히, 핵심만):
1. 회피 계획·콘 기억 C++ ↔ Python 시뮬레이터 비교: 시뮬레이터로 고정 입력(지그재그·유도선·duplicate 관측) 결과를 JSON으로 만들고
   C++ 결과가 허용 오차 안인지
2. control_core: 기존 auto_control 테스트 중 핵심(Stanley 부호, 속도 계획 단조성) 이식
3. YOLO 후처리: ONNX를 onnxruntime CPU로 돌려 C++ 후처리 결과와 비교(OpenCV C++ 설치 가능 시)
4. 웹 프리뷰: 시뮬레이터 주행 데이터를 같은 텔레메트리 JSON으로 재생하는 작은 mock 서버 → Playwright로 화면 캡처
   (사용자에게 보여줄 **프리뷰 샘플 이미지**: 주행 모드, BEV 원본 ON/OFF, 파라미터 창, 검출 테스트 모드)
5. 상태기계: START 거부 조건, heartbeat 끊김 → STOPPING, 검출 모드에서 자동 명령 없음 (ROS 없이 상태 클래스 단위 테스트)
6. 안전 전환 프로세스: X 버튼 → 자동 명령 차단, 자동 명령 timeout → 제동, START 재허용은 정지·페달 해제에서만 (rclpy 없이 상태 로직 테스트)

여기서 못 하는 것(Jetson에서 사용자 확인): colcon 전체 빌드, OAK 실영상, TensorRT/CUDA 실행과 FPS, VESC·조이스틱 동작, 실차 주행.
문서에 Jetson 첫 실행 점검 순서를 적는다.

## 10. builder에게 남기는 것
- C++ 이식 계획기의 시뮬레이터 일치 정도: 같은 입력에서 경로 횡오차 ≤1 cm, 모드 선택 동일을 목표로 하되 부동소수점 차이는 허용
- 파일 분할, 클래스 이름, 스레드 동기화 세부, JSON 필드 이름
- 기존 코드 복사 시 불필요한 분기(신호등·터널·IR) 제거 범위
- 파라미터 YAML 구조(기본값은 기존 YAML/시뮬레이터 값 그대로)
