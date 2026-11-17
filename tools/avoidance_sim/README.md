# avoidance_sim — 라바콘 회피 2D 시뮬레이터

설계: `docs/superpowers/specs/2026-11-15-obstacle-avoidance-sim-design.md`

## 설치

```bash
pip install numpy matplotlib pyyaml pytest
# 한글 표시용 (Ubuntu)
sudo apt install fonts-nanum && rm -rf ~/.cache/matplotlib
```

## 실행 (이 폴더에서)

```bash
cd tools/avoidance_sim

# 콘 배치 에디터: 클릭으로 추가, 드래그로 이동, 우클릭 삭제 → [실행]으로 기준안/제안안 비교 재생
python -m avoidance_sim.editor
python -m avoidance_sim.editor --layout my_layout.json
# 저장한 배치를 일반 실행에서도 사용 (구간은 --scenario)
python -m avoidance_sim.run --scenario stage1_cones --layout my_layout.json --planner baseline proposed

# GUI: 기준안(위) vs 제안안(아래) 비교. 스페이스=일시정지, ←/→=프레임 이동, ↑/↓=재생 속도
python -m avoidance_sim.run --scenario stage1_cones --planner baseline proposed

# 기억 방식 비교: baseline(기억 없음) / proposed(월드 기억) / proposed_lane(차선 기준 단기 기억, 실차 memory.mode=1)
python -m avoidance_sim.run --scenario stage1_cones --planner baseline proposed proposed_lane

# 1단계 지그재그 묶음 간격 (기본 0.66 m) / 남는 폭 (기본 0.47 m)
python -m avoidance_sim.run --scenario stage1_cones --stage1-spacing 0.60 --stage1-gap 0.47

# 시나리오: stage0_lap | stage1_cones | stage2_guide | full_lap
python -m avoidance_sim.run --scenario stage2_guide --planner proposed

# 지그재그 폭 넓히기 (규격 0.47 m)
python -m avoidance_sim.run --scenario stage1_cones --stage1-gap 0.53

# 노이즈 끄기 / GIF 저장 / 지표만
python -m avoidance_sim.run --scenario stage1_cones --no-noise
python -m avoidance_sim.run --scenario stage1_cones --planner baseline proposed --save out.gif
python -m avoidance_sim.run --scenario stage1_cones --headless

# 파라미터 덮어쓰기 (config.py의 이름 그대로)
python -m avoidance_sim.run --config my.yaml     # 예) control: {obstacle_speed: 0.7}

# 배치: 시나리오 × 알고리즘 × 시드
python -m avoidance_sim.batch --scenario stage1_cones stage2_guide --seeds 10
python -m avoidance_sim.batch --scenario stage1_cones --stage1-gap 0.47 0.53 0.59 --seeds 5

# 과적합 점검: 처음 보는 무작위 배치(지그재그·유도선 좌우·가장자리 콘) × 시드 10번대
python -m avoidance_sim.robustness 123

python -m pytest -q tests
```

## 구성

| 파일 | 내용 |
|---|---|
| `track.py` | 트랙 좌표, 1단계·2단계 콘 배치 |
| `vehicle.py` | 기구학 자전거 모델 (휠베이스 0.33, 최대 조향 30°) |
| `camera.py` | 핀홀 카메라(앞차축 −0.09 m, 높이 0.20 m, pitch 13°, 줌 1.0, ROI 65 %) + 픽셀 단위 가림, multiclass_yolo식 위치 추정 |
| `sweep.py` | 지그재그(간격 60~80 cm, 침범 18~20 cm) 파라미터 탐색 |
| `perception.py` | BEV 1.2×3 m 시야, 차선 관측(코너 바깥쪽 이동 포함), 콘 관측, 오도메트리 |
| `editor.py` | 콘 배치 에디터 |
| `control.py` | Stanley, 곡률 기반 속도 계획 (auto_control 이식) |
| `planners.py` | `baseline`(현재 실차: 메모리 없음) / `proposed`(콘 메모리 + Frenet DP + 롤아웃 검사) |
| `sim.py` | 시뮬 루프와 지표 |
| `viewer.py` | matplotlib GUI |

## 화면 보는 법

- 왼쪽: 트랙 위에서 본 화면. 파란 사각형 = 차, 하늘색 면 = BEV 시야, 청록선 = 따라가는 경로, 빨간색 = 접촉
- 오른쪽: 차량 시점(앞=위). 주황 테두리 = 실제 콘, 노란 × = 이번 관측, 분홍 원 = 기억 중인 콘
- 상태줄 `mode=lane`은 차선 안 회피, `mode=free`는 차선이 막혀 밖으로 나간 경우, `rollout=FAIL`이면 감속

## 실차에서 가져온 정책·가정

- **코너 바깥쪽 이동** (1113ML `line_detactor.yaml`): `corner_outward_offset 0.08 m`, 진입 2.2 m 전부터 시작,
  0.6 m 전부터 최대. `config.py`의 `PerceptionParams.corner_*`로 조절
- **카메라**: 차량 바로 앞(앞차축 기준 약 0.23 m 이내)은 안 보임. 콘은 높이 0.18 m 사각뿔대로 그려
  픽셀 단위로 가림을 계산 → 카메라(0.20 m)가 콘보다 약간 높아 뒤 콘의 윗부분이 보일 수 있음
- **YOLO 위치 추정**: 보이는 픽셀 박스의 가운데 60% 깊이 → 가장 가까운 충분한 거리 구간.
  가려진 콘의 박스에 앞 콘이 섞이면 앞 콘 거리로 놓이는 오류가 재현됨 (카메라 패널의 빨간 박스)
- pitch 13°, 콘 높이 0.18 m, YOLO ROI 80 %는 실측이 아닌 가정 → `CameraParams`에서 수정
- 차선은 BEV 0~3 m 전체에서 안다고 가정(`lane_camera_frustum=False`). 켜면 경로가 앞차축 약 0.23 m 앞부터
  시작해 Stanley의 최근접점이 경로 시작점이 되므로, heading lookahead 0.05 m여도 실제로는 일찍 꺾는다

## 속도 (참고: 클라우드 4코어 기준)

| | 계산 시간 / 주행 시간 |
|---|---|
| 기준안 1·2단계 | 0.8~1.2 s / 5 s |
| 제안안 1단계 | 3.6 s / 7 s (DP 평균 13 ms, 최대 30 ms) |
| 제안안 2단계 | 6 s / 6 s (DP 평균 25 ms, 최대 50 ms) |
| GUI 재생 | 1화면 약 13 ms, 비교 2화면 약 29 ms / 프레임 |


## 지그재그 파라미터 탐색 결과 (간격 60~80 cm, 침범 18~20 cm)

`python -m avoidance_sim.sweep` · 결과: `sim_output/sweep/`

1. 1차: 72개 조합 × 15개 배치 × 시드 4 = 4320회 (`zigzag_sweep.*`)
2. 2차 검증: 선정에 안 쓴 시드 200번대 × 6 (`zigzag_verify.*`, `zigzag_final_default.*`)

| 파라미터 | 결과 |
|---|---|
| `soft_clearance` (차선 안) | **유일하게 확실한 효과.** 0.14 → 72 %, 0.10 → 81 %, 0.06 → 85 %, 0.03 → 69 % |
| `hard_clearance` (차선 안) | 0.005 → 0.015에서 소폭 개선 (soft 0.06일 때 88 → 90 %) |
| `obstacle_speed` 0.6/1.0, `stanley_gain` 1.0~1.6, `steering_current_weight` 0.4/0.7, `lane_tolerance` | 차이 없음 (±2 %p, 노이즈 수준). 그대로 둠 |

→ 기본값 변경: 차선 안 모드 `soft_clearance 0.10→0.06`, `hard_clearance 0.005→0.015`.
차선 밖 모드는 따로 `free_*_clearance`(0.10/0.005, 이전 값)를 쓴다. 유도선형 배치에서 0.06을 쓰면 접촉이 늘었기 때문.

검증 시드 결과 (무접촉, 이전 기본값 79 % → **91 %**):

| 간격\침범 | 18 cm | 19 cm | 20 cm |
|---|---|---|---|
| 60 cm | 6/6 | 6/6 | **1/6** |
| 65 cm | 6/6 | 6/6 | 6/6 |
| 70 cm | 6/6 | 6/6 | 5/6 |
| 75 cm | 6/6 | 5/6 | 5/6 |
| 80 cm | 6/6 | 6/6 | 6/6 |

- 접촉은 거의 전부 0~1.5 cm 스침(평균 0.4 cm). 차선 이탈은 4320회 중 1회(60 cm/20 cm).
- **60 cm·20 cm는 파라미터로 해결 안 됨**: 남는 폭 45 cm(규격 47 cm보다 좁음)에 묶음 간격까지 짧아 이 차량
  (휠베이스 0.33, 조향 30°)의 한계. 실차에서 이 배치가 나오면 속도를 더 낮추는 것 외엔 방법이 없음
  (`obstacle_speed 0.6`도 효과 없었음 → 추종 오차보다 기하 한계).
- 대가: 유도선형(corridor) 무작위 배치는 평균 접촉 4.4 → 5.0회로 약간 나빠짐. 유도선형은 어느 값에서도
  무접촉이 드물어 별도 개선이 필요하다.

## 결과를 볼 때 주의할 점 (과적합 점검)

- 규칙 자체는 단계 이름이 아니라 일반 규칙이다: "차선 안 경로가 있으면 차선 안, 차가 지나갈 틈이 아예 없을 때만
  차선 밖". 코드에 1·2단계 좌표나 형태는 들어 있지 않다 (지표 계산과 시나리오 정의만 예외).
- 무작위 배치 검증(`robustness`)에서 유도선 좌/우·크기·길이를 바꿔도 6개 중 5개 무접촉, 넓은 지그재그와
  가장자리 콘은 모두 무접촉, 차선 중앙 콘 1개·코너 안 콘도 무접촉이었다.
- 규격 지그재그(간격 50 cm, 남는 폭 47 cm)는 위치를 바꾸면 1~2회, 최대 3.7 cm 접촉한다. 1단계 시나리오에서
  "1 cm 미만"으로 보인 건 그 배치에 유리했던 결과다. 이 차량(휠베이스 0.33, 조향 30°)에 한계에 가까운 배치다.
- **YOLO**: multiclass_yolo 기본(holdout10 512x320, ROI 65 %, 카메라 줌 없음)을 따른다. ROI 위로 잘린 부분은
  보이지 않는 것으로 계산하므로 콘 검출 거리는 앞차축 기준 약 1.5 m다 (pitch 13° 가정). ROI 80 %(roi80 모델)이면 더 멀리 본다.
- 사람이 손으로 정한 값: 비용 가중치(`w_*`), 차선 밖 모드의 약한 차선 비용(`w_lane_linear_free=4`, 2단계를
  보고 정함), 노이즈 크기, 카메라 pitch 13°·콘 높이 0.18 m. 실측으로 바꿔 다시 확인할 것.
