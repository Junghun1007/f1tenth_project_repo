# 1113ML 적용 패치

`1113ML_camera_rates.patch` — oak_fps_bench 측정 결과 반영 (camera_driver만 변경)

- `sensor_fps` / `publish_fps` 70 → 60 (tunnel_enabled에서 70으로 강제하던 코드 제거, 30 미만이면 시작 시 오류)
- `imu_rate_hz` 400 → 200
- `tunnel.ir_resolution` 800p → 400p
- TCN 입력(약 30 Hz) 선택 격자에 4 ms 허용 오차: RGB 60 fps에서 17/33/50 ms 간격이 섞이던 것을 33 ms로 고정

`1113ML_params_no_presets.patch` — 파라미터 기본값 정리와 프리셋 기능 제거 (auto_control만 변경)

- `config/auto_drive.yaml`: `tunnel.validation_mode` true → false (나머지 값은 `202611251743_ver31.yaml`과 이미 같음)
- 이름 있는 프리셋 저장·목록·불러오기 제거 (웹 UI, `/api/presets*`, `ParameterPresets`)
- 웹 "YAML 저장"은 `~/.cache/auto_control/auto_drive_saved.yaml` 한 파일 저장으로 되돌림
- 카메라 초기화(현재 설정으로 인식 컨테이너 재시작)는 그대로

`1113ML_ir_gate_idle.patch` — 터널 IR 게이트 스크립트 (camera_driver만 변경)

- 터널 밖(route 0)에서 IR 프레임을 꺼내 버리던 것을 전환 명령만 기다리도록 변경 (장치 CPU 절약)
- route 2(400p 보조 출력)의 2 ms 반복 확인 제거, 보조 출력이 없으면 시작 해상도 출력 사용

`1113ML_compute_opt.patch` — auto_control 연산 최적화 (결과는 그대로)

- 회피 계획 `frenet_dp`: 도달 불가능한 이전 칸·기울기 한계 밖 쌍을 순회하지 않음, 뒷차축 적분을 판정 뒤로
  (무작위 장면 1500개에서 원본과 경로·모드 비트 단위 동일, 콘이 있을 때 약 1.6~3배 빠름)
- 롤아웃 `rollout_ok`: 10 ms 스텝마다 경로를 새로 할당·복사하지 않고 재사용, `from_frenet`의 쓰지 않는 벡터 4개 제거
- 수동 주행 녹화 데이터(`RecordFrame`)는 녹화 중에만 만듦 (예전엔 매 차선 프레임마다 만들고 버림)
- 웹 카메라 보기: Y·UV 평면을 먼저 640x400으로 줄인 뒤 변환 (변환+축소 단일 스레드 1.4 ms → 0.3 ms)

`1113ML_tunnel_rates.patch` — tunnel_enabled에서의 차선 처리율 (line_detactor, auto_control 런치, ir_camera_driver)

- RGB 차선 검출기가 터널 상태로 추론 FPS를 바꾸던 호스트 도착 시각 게이트 제거
  (터널 밖 70 → 실제 약 40 FPS로 떨어지던 문제, 터널 안 5 FPS 제한). 터널 안 RGB는 camera_driver가 약 30 FPS로 솎음
- shared_ir_bev: 터널 밖에서 1 ms(초당 1000회) 확인 타이머를 50 ms로, 터널 안은 1 ms 그대로

`1113ML_stage_panel.patch` — 웹 단계별 연산 표 (auto_control, vehicle_bringup 런치)

- 측정하는 모든 단계를 테마별(카메라 수신 / 차선 RGB / 차선 IR / 장애물 / 판단·제어 / TCN / 웹)로 늘 표시, 없으면 "미사용 (이유)"
- 목표 FPS 열(카메라 설정·YOLO 상한·터널 안 30 등)과 달성 색, 부하 열(ms × FPS)과 그룹별 부하 합
- 새 측정: RGB·IMU 수신, IR 차선 분리, TCN 입력 FPS·입력 지연·처리·추론, 깊이 짝 대기, 검출 1회 전체, 박스 수, 회피 경로 탐색

`1113ML_tunnel_follow_tcn.patch` — 터널 차선 전환 정책 제거 (auto_control)

- TCN 판정(state inside/outside)이 오면 그 자리에서 RGB↔IR 전환. 차선 유무·경로 유효성·결과 신선도·보고 나이·LIVE 여부를 보지 않음
- 강제 IR도 바로 전환 (유효 IR 경로 대기 ENTERING 제거), 주행 중에도 켜고 끌 수 있음, 출발 시 IR 차선 도착 확인 제거
- 선택기는 선택된 출처의 결과만 넘김 (TCN이 끊겨도 차선 결과를 막지 않음). 같은 출처에서 시각이 되돌아간 결과만 버림
- camera_driver의 IR 프레임 입력 검사(노출 확인·150 ms 대기)는 그대로 (`tunnel.validation_mode`로 켜고 끔)

적용 (git 저장소가 아니어도 됨, 1113ML 워크스페이스 루트에서, 순서대로):

```bash
patch -p1 --dry-run < tools/patches/1113ML_camera_rates.patch        # 먼저 확인
patch -p1 < tools/patches/1113ML_camera_rates.patch
patch -p1 --dry-run < tools/patches/1113ML_params_no_presets.patch
patch -p1 < tools/patches/1113ML_params_no_presets.patch
patch -p1 --dry-run < tools/patches/1113ML_ir_gate_idle.patch
patch -p1 < tools/patches/1113ML_ir_gate_idle.patch
patch -p1 --dry-run < tools/patches/1113ML_compute_opt.patch
patch -p1 < tools/patches/1113ML_compute_opt.patch
patch -p1 --dry-run < tools/patches/1113ML_tunnel_rates.patch
patch -p1 < tools/patches/1113ML_tunnel_rates.patch
patch -p1 --dry-run < tools/patches/1113ML_stage_panel.patch
patch -p1 < tools/patches/1113ML_stage_panel.patch
patch -p1 --dry-run < tools/patches/1113ML_tunnel_follow_tcn.patch
patch -p1 < tools/patches/1113ML_tunnel_follow_tcn.patch
colcon build --packages-select camera_driver auto_control line_detactor ir_camera_driver vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
```
