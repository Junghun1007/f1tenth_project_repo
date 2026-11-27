# 차선 경로 실험실 (lane_path_lab)

녹화한 BEV로 C3 차선 모델 검출 → 차선 연결 → 중앙선을 다시 돌리고, 그 차선 위에서
**기존 경로(차량 현재 코너 오프셋 방식)** 와 **새 경로 계획(`planner.js`)** 을 프레임마다 비교하는 오프라인 웹뷰.

## 새 경로 계획 (planner.js)

0. 출발은 차량 위치·방향·조향 곡률 그대로 (기준선을 첫 방향으로 뒤로 늘려 차량 옆에서 시작). 차선 중앙 복귀는
   max(1.2 m, 속도×1.5 s) 뒤부터 서서히 — 코너 뒤 중앙 복귀에서 휘청거리지 않게.
1. 코너 반영을 끈 차선 중심선을 기준선으로, 점마다 법선 방향으로 좌·우 관측 차선까지 거리를 잰다 (통로).
   한쪽 차선이 안 보이면 차선 폭(0.65 m)으로 추정하고, 추정한 쪽은 바퀴 여유 판정에서 뺀다.
2. 경로 곡률·현재 속도·감속도로 점마다 예상 속도를 구하고, 내륜차 `κ·L²/2·(1−(v/V₀)²)`를 회전 안쪽 여유에 더한다.
3. 통로 안에서 곡률 제곱합이 최소인 스플라인(제어점 0.3 m 간격)을 맞춘다. 시작점·시작 방향은 차량에 고정.
4. 뒷차축 궤적(슬립 포함)과 차체 네 모서리를 시뮬레이션해 관측 차선 여유가 2 cm 미만이면 그 구간 통로를 좁혀 다시 맞춘다 (최대 4회).

- 경로 생성은 `line_detactor/src/lane_connector.cpp`, `centerline.cpp`를 **그대로 링크**한다 (차량과 같은 코드).
- 파라미터 목록·적용 방식은 `bev_handoff/lane_tuning.hpp` X-매크로와 `line_detactor_node.cpp frame_lane_settings()`와 같다.
- 기본값 = `config/auto_drive.yaml`의 `lane.*` (차량 현재값). 웹의 "바뀐 값 YAML"로 바꾼 줄만 복사할 수 있다.

## 실행

```bash
cmake -S src/auto_control/tools/lane_path_lab -B build/lane_path_lab && cmake --build build/lane_path_lab -j4
python3 src/auto_control/tools/lane_path_lab/server.py --data <세션 폴더들의 상위 폴더>
# → http://127.0.0.1:8765
```

세션 폴더 = `rgb_bev/`(120×300 BEV jpg), 선택으로 `rgb/`(원본). 처음 실행 때 모델 출력(logit)을
`build/lane_path_lab/cache/`에 저장하므로 이후 임계값을 포함한 모든 파라미터 변경은 경로 생성만 다시 돈다 (프레임당 수 ms).

## 차량과 다른 점

- 추론: 차량은 TensorRT INT8(`lane.model: 2`), 여기는 OpenCV DNN으로 FP32 ONNX(`lane_c3_120x300.onnx`). 경계 픽셀이 조금 다를 수 있다.
- `side_tracking`(좌우 판정 고정)은 프레임 간 상태가 필요해 적용하지 않는다 (차량 설정도 0).
- "최소 반경"은 경로 점 ±0.24 m 세 점으로 계산한 값이다.
- BEV는 앞차축 0.23 m 앞부터 보이므로 차량 바로 옆·뒤 차선은 한 프레임만으로는 알 수 없다 (실차에선 이전 프레임 기억이 필요).
- 코너·평활화 파라미터는 웹에서 숨기고 기존 경로 비교용으로 차량 현재값(auto_drive.yaml)으로 고정한다.
