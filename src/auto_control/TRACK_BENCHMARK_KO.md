# 트랙 주행 성능 기록

`auto_drive.launch.py`의 `benchmark_run`으로 네 회차를 선택한다. 각 회차는 기존 START가 승인된 시점부터 연속 60초를 기록한다. 신호 대기·차선 유실·터널 전환·수동 전환 시간도 그 60초에 포함된다. 60초가 끝나면 기록을 종료하고 기존 `/auto_control/stop` 서비스를 자동 호출한다. 정지 요청은 파일 저장보다 먼저 보내므로 저장 지연이나 실패와 관계없이 실행된다. 같은 실행에서 START를 다시 눌러도 기록을 재시작하지 않는다. 다음 회차는 launch를 재시작한다.

| 회차 | 차선·코너 주행 | 장애물 검출·회피·감속 | 신호등 인식·정지 | 터널 |
|---|---|---|---|---|
| 1 | ON | OFF | OFF | OFF |
| 2 | ON | ON | OFF | OFF |
| 3 | ON | ON | ON | OFF |
| 4 | ON | ON | ON | ON |

기존 속도·조향·회피 파라미터와 모델 설정을 사용한다. 회피 방식은 기존 `avoidance_deformation_only` 설정을 따른다. 회차 옵션은 위 기능의 활성화 여부를 우선 적용한다. `benchmark_run:=0`은 일반 실행이다.

실험 실행에서는 카메라·BEV·차선·IR 프리뷰, 원격 프리뷰, 시작 검사 GUI와 runtime tools GUI를 끈다. 시작 검사와 START 서비스는 동작한다. 영상 데이터셋 수집, 별도 차선 프로파일 CSV, 기존 driving CSV는 끄고 아래의 동일한 경량 기록기를 사용한다. 제어 상태·속도·조향·제동·코너 정보는 기존 30 Hz 요약 메시지에서 수집한다. 영상 픽셀은 기록하지 않는다.

## Jetson에서 빌드

저장소와 ROS 환경을 준비한 워크스페이스 루트에서 실행한다. `bev_handoff`의 계측 헤더를 사용하는 카메라·BEV·차선·장애물·신호등 컴포넌트도 다시 빌드해야 한다.

```bash
git pull --ff-only
colcon build --packages-up-to vehicle_bringup auto_control --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 회차별 실행

아래 명령은 한 번에 한 회차만 실행한다. 기존에 사용하는 차량 포트·속도·외부 YAML 옵션이 있다면 같은 값을 네 회차에 모두 추가한다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py benchmark_run:=1
```

```bash
ros2 launch vehicle_bringup auto_drive.launch.py benchmark_run:=2
```

```bash
ros2 launch vehicle_bringup auto_drive.launch.py benchmark_run:=3
```

```bash
ros2 launch vehicle_bringup auto_drive.launch.py benchmark_run:=4
```

카메라 초기 자세 측정과 모델 준비가 끝나면 터미널에 `READY_TO_START`가 표시된다. 별도 터미널에서 ROS 환경을 로드하고 기존 START 서비스를 호출한다.

```bash
ros2 service call /auto_control/start std_srvs/srv/Trigger '{}'
```

응답이 성공하고 `Recording 60 seconds`가 표시되면 기록이 시작된다. `Capture saved` 메시지와 해당 폴더의 `manifest.json`이 저장 완료를 나타낸다. `Automatic STOP accepted by auto_control` 메시지는 제어기가 정지 요청을 수락했음을 뜻한다. 실제 차량 정지를 확인한 후 Ctrl+C로 launch를 종료한다. 서비스 연결 실패·응답 시간 초과·거절 시 오류를 출력하고 재시도한다. 필요하면 기존 조종기 또는 아래 STOP 명령을 사용할 수 있다.

```bash
ros2 service call /auto_control/stop std_srvs/srv/Trigger '{}'
```

기록 중 STOP을 눌러도 최초 START부터 60초 측정은 계속된다. launch를 Ctrl+C로 종료하면 그때까지의 기록을 `interrupted`로 저장한다. 프로세스를 강제 종료하거나 전원이 끊기면 메모리에 모으던 기록을 잃을 수 있다. 최대 100,000건을 메모리에 수집하고, 완료 시 별도 저장 스레드에서 파일을 작성한다. 초과 건수는 manifest의 `dropped_events`에 남긴다. 기록기는 60초 완료 시 기존 STOP 서비스를 호출하며, START는 사용자가 실행한다.

## 결과 파일

기본 폴더는 `~/.ros/auto_control/benchmarks/날짜_시각_runN/`이다. `benchmark_directory:=/원하는/경로`로 바꿀 수 있다. 같은 회차를 다시 실행해도 새 폴더를 만든다.

| 파일 | 내용 |
|---|---|
| `manifest.json` | 회차·기능 조합·시작 UTC·측정 길이·완료/중단·종류별 수집 수·기록 누락·초기 상태·시작 로그 |
| `parameters.json` | 실제 노드 파라미터와 조회 실패 사유 |
| `events.jsonl` | 수신 시각·경과 시간과 모든 계측/주행/상태 이벤트 |
| `lane_rgb.csv` | RGB 프레임별 촬영 시각·시퀀스·큐 대기·전처리·추론·후처리·결과 유효성·연산 ms·역수 FPS |
| `lane_ir.csv` | IR 차선 결과가 존재할 때 같은 정보 |
| `runtime_metrics.csv` | 노드·단계별 실제 이벤트 FPS, 평균·p95·최대 ms, 표본 수, 프레임별 역수 FPS 평균 |
| `system.csv` | CPU 전체/코어별·GPU 부하·RAM·swap·클럭·온도·전력. 지원되는 센서만 기록 |

일부 단계에서 이벤트가 없으면 해당 CSV가 생성되지 않는다. 원시 이벤트에는 카메라 수신·시퀀스 누락·무효 프레임 누적 수, BEV 입력·처리·전달 누적 수, 센서→호스트 지연, RGB 호스트 대기, BEV 입력 대기, 신호등/TCN 결과, 터널 상태·선택 소스, 제어 지연, ROS 로그와 실행 중 파라미터 변경도 포함된다.

## 분석 시 해석

- `mean_reciprocal_fps`는 양수인 각 시간에 `1000/ms`를 적용한 산술평균이다. 0 ms인 횟수 전용 계측은 역수 FPS를 null로 기록한다. 실제 처리율은 `fps`로 기록한다.
- `rgb_sensor_to_host`, `rgb_host_wait`, `bev_input_wait`는 지연·대기 시간이다. `camera_to_bev`, `bev_transform`, `lane_inference`, `lane_postprocess`, `control` 등은 각 코드 구간의 시간이다. GPU 관련 시간은 기존 검출기의 계측값을 사용한다.
- `/perf/*`와 runtime 계측은 약 1초 창이다. 측정 경계와 창이 정확히 일치하지 않으며, runtime 계측의 마지막 미완료 창은 포함되지 않을 수 있다. 프레임별 차선 CSV는 기록기에 도착한 결과를 `[시작, 시작+60초)` 구간에서 수집한다. 센서 시퀀스 누락과 기록기/ROS 수신 누락을 같은 원인으로 단정하지 않는다.
- p95는 각 보고 창의 p95다. 전체 60초 p95를 구할 때 창별 p95를 평균 내지 않는다. 차선은 프레임 CSV에서 다시 계산할 수 있다.
- RGB·IR 결과는 나누어 분석한다. 터널 내부의 RGB BEV 5 Hz는 복귀 대기를 위한 의도적인 제한이다. 4회차는 현재 코드에 따라 스테레오/IR 요청 FPS가 30이며, 2·3회차의 기본 깊이 요청은 40 FPS다. 실제 적용값은 `parameters.json` 및 시작 로그에서 확인한다.
- 파라미터 조회는 준비 중 수행하며, 기록 중 변경 이벤트도 남긴다. 조회 실패·누락·센서 미지원 값은 파일에 명시하거나 필드를 생략한다. 누락을 0% 부하로 해석하지 않는다.
- 먼저 카메라 실제 수신률과 시퀀스 누락을 비교하고, BEV 전달률, RGB/IR 차선 결과율, 큐 대기, 연산 시간, 제어 지연 순서로 비교한다. 서로 다른 트랙 장면에 따른 연산량 변화도 주행/터널 상태와 함께 확인한다.

네 회차가 끝나면 기본 결과 폴더 전체를 묶어 전달하면 된다.

```bash
tar -czf track_benchmarks.tar.gz -C "$HOME/.ros/auto_control" benchmarks
```
