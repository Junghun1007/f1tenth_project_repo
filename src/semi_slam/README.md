# semi_slam

정답 트랙(4 m × 4 m, 반시계)을 일정 속도로 여러 바퀴 돌며 **바퀴 거리 + IMU 회전으로 그린 궤적**을 정답 트랙과 비교해
오도메트리 보정 값(속도별 거리 배율, 자이로 배율·잔여 영점, 카메라–오도메트리 시간차)을 찾는 패키지.
cone_drive_test에서 주행에 필요한 부분(BEV, IMU 안정화, 차선 인식, Stanley, duty-only 종방향, 안전 전환)만 복사한 독립 패키지다.

## 빌드·실행 (Jetson)

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --executor sequential --packages-up-to semi_slam --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch semi_slam semi_slam.launch.py
```

브라우저: `http://<jetson-ip>:8766` (cone_drive_test와 동시에 켜지 않는다: 카메라·VESC를 같이 쓴다)

## 주행 순서

1. 상단 **방향**을 고르고, 차를 **아래쪽 직선이 시작되는 곳 도로 가운데**에 놓는다 (지도에 초록 화살표로 표시).
   - 반시계: 왼쪽 아래 코너가 끝나는 x ≈ 1.0 m에서 오른쪽(+x)을 보고
   - 시계: 오른쪽 아래 코너가 끝나는 x ≈ 3.0 m에서 왼쪽(−x)을 보고
   - 두 경우 모두 정지선 S가 약 1 m 앞. 앞범퍼가 다른 곳이면 `slam.start_shift_m` (진행 방향 앞 +)
   정지선 S까지 약 1 m는 가속만 하고, **S를 처음 지나는 순간부터** 기록·매핑·바퀴 세기를 시작한다. 그때 위치는 S, 방향·옆 위치는 BEV 차선으로 잡는다.
2. 출발 전 점검 칩이 모두 초록이 될 때까지 기다린다 (카메라 자세 측정, 차선 모델).
3. 상단에서 **일정 속도**(m/s)와 **바퀴 수**를 넣고, IMU 안정화(전체/끄기), 정지선 위치 보정(켬/끔)을 고른다.
4. **주행 시작** → 정지선 S를 지날 때마다 바퀴를 세고, 목표 바퀴를 채우면 정지 → 수동 주행으로 넘어간다.
5. 멈추면 기록을 저장하고 **보정 값을 계산해 오차가 줄었을 때만 적용**한다 (화면 상단 상태 문구, 보정 이력).
   이상하면 **보정 되돌리기**. 중간에 정지하면 기록만 저장하고 보정하지 않는다.

같은 속도를 여러 번, 그리고 속도를 바꿔(예: 1.0 / 1.2 / 1.4 / 1.6) 돌리면 속도별 거리 배율 표가 채워진다.

## 매핑·보정 방법

- 위치: `x += v·k(v)·cos ψ·dt`, `ψ += g·(ω − b0 − b)·dt`. `b0`는 출발 직전 정지 중에 잰 자이로 영점.
- 차선 인식의 **왼쪽 선 → 정답 안쪽 흰 선, 오른쪽 선 → 바깥 흰 선**으로 비교한다 (1.5 m 안의 점만).
  가운데 점선(중앙선)은 그림에 표시만 한 것이고, 코너에서는 한쪽 선 + 반폭 가정이라 **매핑·보정에 쓰지 않는다** (주행 경로로만 씀).
- 정지선 S: 관측 거리로 통과 지점을 예측해 바퀴를 세고, 켜져 있으면 통과할 때 진행 방향 위치를 S에 맞춘다.
- 보정: 주행 기록 전체를 다시 적분하며 (k, g, b, 시간차)를 Nelder–Mead로 찾는다. 비용 = 차선 점과 정답 흰 선 거리 제곱 평균(점당 15 cm 상한) + 정지선 앞뒤 오차.
- 정확도 = 1 − 정합 RMS ÷ 차선 반폭(32.5 cm).
- 한계: 일정 속도로 같은 트랙만 돌면 자이로 배율과 잔여 영점이 서로 일부 대신할 수 있다 (궤적은 거의 같음).
  속도를 바꿔 여러 번 돌리면 구분이 좋아진다.

## 저장 위치 (`slam.data_dir`, 기본 `~/semi_slam_data`)

| 파일 | 내용 |
|---|---|
| `calibration.json` | 지금 쓰는 보정 값 (다음 실행에도 이어서) |
| `calibration_history.jsonl` | 보정 이력 (적용/건너뜀/되돌림, 보정 전후 RMS) |
| `odometry_calibration.yaml` | 다른 패키지(cone_drive_test 등)가 읽기 쉬운 형식 |
| `runs/<날짜_시각_v속도>/` | `meta.json`, `speed.csv`, `gyro.csv`, `lanes.csv`(차선 점·정지선 거리), `crossings.csv`, `laps.csv`, `corrections.csv`, `trajectory.csv`, `calibration.json` |

기록 폴더는 `load_run()`으로 다시 읽어 PC에서 보정을 다시 돌릴 수 있다 (`src/core/run_store.hpp`).

## 정답 트랙

`maps/track_4x4.csv` (`line,x_m,y_m`: outer, inner, center, stop_S, start_T1). 치수는 사용자 트랙 그림에서 읽은 값으로
`tools/make_track_csv.py`에 적혀 있다. 선 위치는 테이프 가운데이고, 차선 폭은 실측(테이프 안쪽 64 cm + 테이프 3 cm = 가운데 간격 67 cm)을 쓴다
(바깥 선 0.165 m 안쪽·코너 R 0.62, 안쪽 선 0.835 m·r 0.174, 정지선 x 1.97, 출발선 x 2.03).

## 호스트 테스트 (ROS 없이)

```bash
cmake -S src/semi_slam/test/host -B build/semi_slam_host && cmake --build build/semi_slam_host -j4 && ctest --test-dir build/semi_slam_host
```

`slam_test`는 거리·자이로·영점 오차와 차선 잡음·영상 지연을 넣은 모의 주행으로 바퀴 세기와 보정 값 복원을 확인한다.
웹 화면만 볼 때: `web/index.html?demo`.
