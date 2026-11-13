# 정지 상태 RGB·스테레오 파라미터 탐색

이 폴더 하나로 실행하는 독립 실험 도구다. `COLCON_IGNORE`로 기존 colcon 빌드에서도 제외한다. ROS 노드·주행 코드와 연결하지 않는다. OAK를 직접 열어 RGB·깊이·IMU만 수신하며, 실험이 끝나면 폴더를 제거할 수 있다. 자동으로 고른 설정을 주행 YAML에 적용하지 않는다.

## 비교 조건

| 항목 | 설정 |
|---|---|
| RGB | CAM_A, 1280×800 NV12, 80 FPS 요청, 렌즈 보정 ON |
| 스테레오 | CAM_B/C, 센서 640×400, 모든 조합에서 40 FPS 요청 |
| 매칭 입력 크기 | 640×400, 512×320, 320×200 |
| 고정 ROI | 전체 폭, 하단 65%. 위쪽 35%를 좌우 ImageManip에서 제거한 뒤 StereoDepth에 입력 |
| 실제 매칭 크기 | 640×260, 512×208, 320×130 |
| 좌우 일관성 검사 | ON / OFF |
| 중앙값 필터 | 3×3 / OFF |
| 기타 깊이 설정 | FAST_DENSITY, confidence 200, LR threshold 10, 서브픽셀·확장 시차·추가 필터 OFF |
| 깊이 출력 범위 | 0.1~3 m. 이 값은 계산 후 필터이며 전체 크기 깊이 영상은 계속 전송됨 |
| IR dot / flood | 0.7 / 0.0. `--dot-intensity`로 모든 조합에 같은 값 적용 가능 |
| IMU | calibrated accel+gyro 400 Hz 요청 |
| 전송 큐 | 호스트 최신 1장, 장치 XLink 영상 큐 1/nonblocking |

3×2×2로 총 12개 스테레오 조합이다. 기본 2회 반복하고, 매 반복 앞뒤 RGB 단독 조건을 추가해 총 28번 측정한다. 조합 순서는 고정 seed로 섞는다. 매번 새 프로세스로 장치를 열고 닫으며, 첫 RGB·깊이 도착 후 5초 준비, 20초 측정한다. 준비·측정만 약 11분 40초이며 장치 재연결 시간이 추가된다. SDK·장치가 작은 해상도나 크롭 입력을 지원하지 않으면 해당 조합은 실패로 기록하고 다음 조합을 실행한다.

ROI는 **센서 영상 하단에서 위로 65%**라는 해석이다. 크롭 위치는 각각 y=140,112,70이다. ImageManip의 변환 메타데이터를 유지해 StereoDepth에 전달하고, 수신 깊이 크기와 변환 메타데이터 유효성을 검사한다. 하드웨어에서 깊이 정확도와 크롭 후 보정 결과는 별도로 확인해야 한다.

## Jetson 실행

차량은 정지해 두고 일반 auto_drive·카메라 프로그램을 종료한다. 장치가 움직이지 않고 같은 물체가 보이는 장면을 유지한다. 이 도구는 VESC·조향·START·STOP 인터페이스를 사용하지 않는다.

저장소 루트에서 실행한다. 기존 프로젝트와 같은 `/usr/local` DepthAI C++ SDK 3.6 이상, CMake, C++17 컴파일러, Python 3가 필요하다. Python depthai·ROS·추가 pip 패키지는 필요하지 않다. 첫 실행에서 이 폴더의 작은 측정 프로그램만 빌드한다.

```bash
git pull --ff-only origin 1030ML
python3 tools/stereo_rgb_sweep/sweep.py
```

다른 위치에 SDK가 설치돼 있다면 사용 중인 SDK 경로를 지정한다.

```bash
python3 tools/stereo_rgb_sweep/sweep.py --sdk-prefix /usr/local
```

실행할 조합만 먼저 확인할 수 있다.

```bash
python3 tools/stereo_rgb_sweep/sweep.py --dry-run
```

각 조합을 60초씩 측정하려면 다음과 같이 실행한다. 깊이 40 FPS와 ROI는 그대로 고정된다.

```bash
python3 tools/stereo_rgb_sweep/sweep.py --duration 60
```

기본은 OAK Pro의 IR dot 0.7이다. projector가 없는 장치는 모든 조건에서 0으로 실행한다.

```bash
python3 tools/stereo_rgb_sweep/sweep.py --dot-intensity 0
```

최초 RGB 단독 성공 후 장치 ID를 고정해 후속 실험이 같은 OAK를 사용하도록 한다. USB SuperSpeed가 아니면 비교를 중단한다. RGB 단독 대조군이 실패하면 실행을 중단하고 로그를 남긴다. 스테레오 조합 실패는 나머지 실험을 계속한다. Ctrl+C는 현재 프로세스를 종료하고 완료된 결과를 보존한다.

## 결과 읽기

기본 저장 위치는 `tools/stereo_rgb_sweep/results/날짜_시각/`이다. 진행 상황을 터미널에 출력하고, 각 조합이 끝날 때 요약을 갱신한다.

| 파일 | 내용 |
|---|---|
| `plan.json` | 전체 조합·반복·요청 FPS·고정 ROI·측정 시간 |
| `ranking.csv` | 조합별 평균/최저 RGB 수신 Hz, 최저 깊이 Hz, 평균 RGB 지연, 시퀀스 누락, 깊이 유효 비율 |
| `results.json` | 모든 완료·실패 결과, 장치 ID, IMU 보고율, 지연 평균/p95/최대 |
| 각 조합의 `config.json` | 실제 실험 설정 |
| 각 조합의 `console.log` | SDK 로그와 오류 |
| 각 조합의 `result.json` | 개별 결과. 실패 시 사유 |
| 각 조합의 `rgb_frames.csv`, `depth_frames.csv` | 수신 경과 시각·센서 시각·시퀀스·지연·실제 프레임 크기 |

`ranking.csv`는 **모든 반복 성공 + 모든 반복에서 깊이 수신 38 Hz 이상**인 조합을 먼저 배치하고, RGB 평균 수신률이 높은 순서로 정렬한다. 38 Hz는 40 FPS 요청 대비 5% 허용폭을 둔 실측 선별 기준이다. FPS 설정 자체는 항상 40이다. 미완료·실패·깊이 처리율 미달 조합도 표에 남긴다.

깊이 유효 비율은 매초 한 깊이 프레임에서 0.1~3 m 값을 가진 픽셀 비율이다. 좌우 검사를 끄면 잘못된 깊이도 유효 값으로 남을 수 있어, 이 비율만으로 정확도를 판단할 수 없다. 수신률 1위는 수신 성능 후보이며 작은 장애물·물체 경계·실제 거리 오차도 확인해야 한다. RGB 단독 before/after 수신률은 `results.json`에 남긴다.

이 도구는 카메라·전송 수신 부하를 비교한다. 기존 차선·장애물 검출·회피 추론의 동시 실행 부하는 주행 실험 결과와 함께 판단한다. 3 m만 필요하다는 이유로 시차 검색 폭까지 줄이지 않으며, 가까운 물체 검출 범위를 보존한다.

결과 폴더 전체를 전달하면 순위·지연·프레임 누락을 분석할 수 있다.

```bash
tar -czf stereo_rgb_sweep_results.tar.gz -C tools/stereo_rgb_sweep results
```

## 오프라인 검증

```bash
python3 -m unittest discover -s tools/stereo_rgb_sweep -p 'test_*.py' -v
```

로컬 검증은 C++ 빌드와 조합 생성·순위·오류 저장 테스트다. Jetson/OAK에서 측정한 실제 최적 조합은 실행 후 결과로 결정한다.
