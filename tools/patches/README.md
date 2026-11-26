# 1113ML 적용 패치

`1113ML_camera_rates.patch` — oak_fps_bench 측정 결과 반영 (camera_driver만 변경)

- `sensor_fps` / `publish_fps` 70 → 60 (tunnel_enabled에서 70으로 강제하던 코드 제거, 30 미만이면 시작 시 오류)
- `imu_rate_hz` 400 → 200
- `tunnel.ir_resolution` 800p → 400p
- TCN 입력(약 30 Hz) 선택 격자에 4 ms 허용 오차: RGB 60 fps에서 17/33/50 ms 간격이 섞이던 것을 33 ms로 고정

적용 (git 저장소가 아니어도 됨, 1113ML 워크스페이스 루트에서):

```bash
patch -p1 --dry-run < tools/patches/1113ML_camera_rates.patch   # 먼저 확인
patch -p1 < tools/patches/1113ML_camera_rates.patch
colcon build --packages-select camera_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
```
