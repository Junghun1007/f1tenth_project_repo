# oak_fps_bench — OAK RGB·스테레오 수신 FPS/지연 진단

RGB 70 fps + 깊이 30 fps로 요청했는데 실제로는 더 느리고 수신 ms가 늘어나는 원인을 가르기 위한 도구다.
ROS 패키지가 아니며 colcon 빌드에서 빠진다(`COLCON_IGNORE`).

## 실행 (Jetson)

```bash
# 이 폴더만 가져오기 (1113ML 등 작업 브랜치를 그대로 둔 채)
git fetch origin claude/kind-darwin-31flh9
git checkout origin/claude/kind-darwin-31flh9 -- tools/oak_fps_bench

# 카메라 단독 측정만 (약 6~7분)
bash tools/oak_fps_bench/run_all.sh

# 실제 auto_drive launch(detect 모드)까지 함께 측정 (+2~5분, 첫 실행은 TensorRT 엔진 생성으로 더 걸림)
bash tools/oak_fps_bench/run_all.sh --ros --ws <워크스페이스 루트>
# 터널 켠 실제 구성도
bash tools/oak_fps_bench/run_all.sh --ros --ros-tunnel --ws <워크스페이스 루트>
```

- OAK를 쓰는 다른 실행(ros2 launch, cone_drive_test 등)은 모두 끈다. 떠 있으면 스크립트가 멈춘다
- 차는 세워 두고 조명은 평소 주행 환경과 비슷하게 둔다 (자동노출이 지연에 영향을 준다)
- `--ros`는 `mode:=detect`로 띄우므로 자동 명령은 나가지 않는다
- `sudo` 비밀번호가 필요한 항목(jetson_clocks --show, dmesg 등)은 건너뛴다. 먼저 `sudo -v`를 해 두면 같이 수집된다

결과는 `~/oak_fps_bench_<시각>/summary.txt` 한 파일에 모인다. 전체 로그 묶음은 같은 이름의 `.tar.gz`다.

## 카메라 단독 측정 설정

| 이름 | 바꾼 것 | 가르는 것 |
|---|---|---|
| rgb_only / rgb_only_noundist | RGB만, 왜곡 보정 켬/끔 | RGB 혼자서 70이 나오는지, 왜곡 보정 비용 |
| depth_only | 깊이만 | 깊이 혼자서 30이 나오는지 |
| prod | 실제 구성 (터널 꺼짐) | 기준선 |
| prod_rgb_meta | RGB 픽셀 전송만 끔 | 회복되면 전송(USB·호스트 수신), 그대로면 장치 연산 |
| prod_noundist / prod_noalign / prod_light_stereo | 왜곡 보정 / 깊이 정렬 / LR·median 끔 | 장치 연산 중 무엇이 무거운지 |
| prod_noimu / prod_chunkdefault | IMU 끔 / XLink chunk 기본값 | IMU 패킷·전송 방식 영향 |
| prod_fixed_exp | RGB·IR 수동 노출 3 ms | 노출이 FPS·지연에 주는 영향 |
| ratio_60_30 / ratio_70_35 | 정수배 FPS | 센서 동기 문제 여부 |
| tunnel800_outside / tunnel400_outside | tunnel_enabled 구성 (스크립트 2개, IR 출력) | 터널 기능의 장치 부하 |

## 지표

- `sensor`: 장치 순번·촬영 시각으로 계산한 실제 촬영 FPS. `rx`: 호스트가 받은 FPS. `gaps`: 순번 누락 수
  - sensor가 낮으면 센서가 덜 찍음 (노출·동기·센서 모드). sensor는 정상인데 rx가 낮고 gaps가 크면 중간에서 버려짐
- `lat`: 노출 중간 → 호스트 수신 (camera_driver `depthai_to_host`와 같은 기준). `lat_end`: 노출 길이의 절반을 뺀 값
- `SYNC`: 깊이 프레임 시작과 가장 가까운 RGB 프레임 시작의 차이. 0 근처에 붙어 있으면 센서가 하드웨어로 동기됨
- `css`/`mss`: OAK 장치 CPU(LeonCSS/LeonMSS) 사용률. LeonCSS는 XLink 전송과 Script 노드도 맡는다
