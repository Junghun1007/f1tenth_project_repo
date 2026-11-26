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

적용 (git 저장소가 아니어도 됨, 1113ML 워크스페이스 루트에서, 순서대로):

```bash
patch -p1 --dry-run < tools/patches/1113ML_camera_rates.patch        # 먼저 확인
patch -p1 < tools/patches/1113ML_camera_rates.patch
patch -p1 --dry-run < tools/patches/1113ML_params_no_presets.patch
patch -p1 < tools/patches/1113ML_params_no_presets.patch
patch -p1 --dry-run < tools/patches/1113ML_ir_gate_idle.patch
patch -p1 < tools/patches/1113ML_ir_gate_idle.patch
colcon build --packages-select camera_driver auto_control --cmake-args -DCMAKE_BUILD_TYPE=Release
```
