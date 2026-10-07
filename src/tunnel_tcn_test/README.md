# tunnel_tcn_test

Jetson Orin Nano Super에서 스테레오 IR 영상으로 터널 진입·탈출을 확인하는
ROS 2 Humble 테스트 패키지입니다. `ir_camera_driver`와 추론 화면을 함께
실행합니다. 조이스틱 수동 주행은 별도 터미널에서 실행합니다.

## 빌드와 실행

Jetson의 프로젝트 루트에서 실행합니다. 기존 카메라 패키지와 같은
DepthAI C++ 3.6+, OpenCV, CUDA 개발 환경이 필요합니다. TensorRT Python
바인딩은 **JetPack에 포함된 TensorRT 8.6 또는 10.x**를 사용합니다.
Jetson 런타임에 PyTorch, ONNX Runtime 또는 PyCUDA를 설치할 필요는 없습니다.

```bash
source /opt/ros/humble/setup.bash
python3 -c "import tensorrt, numpy, cv2; print('TensorRT', tensorrt.__version__)"
colcon build --symlink-install --packages-up-to tunnel_tcn_test \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch tunnel_tcn_test tunnel_tcn_test.launch.py
```

Python TensorRT가 없으면 현재 JetPack에 맞는 `python3-libnvinfer` 패키지를
설치합니다. 임의의 최신 pip TensorRT로 JetPack 라이브러리를 교체하지 않습니다.
기존 워크스페이스가 별도 `depthai_DIR`을 사용한다면 동일 CMake 인자를 사용합니다.

기본값은 **1280×800, 30FPS, 노출 30000µs(30ms), ISO 800, projector 1.0,
flood 0.0**입니다. 첫 실행은 해당 Jetson에서 엔진을 생성하므로 시간이 걸립니다.
엔진은 `~/.cache/tunnel_tcn_test`에 저장하고 모델·TensorRT·CUDA·GPU 정보로
구분해 재사용합니다. 초기화가 끝나면 `TensorRT ready`가 출력됩니다.

실행 시 기본값을 바꾸려면:

```bash
ros2 launch tunnel_tcn_test tunnel_tcn_test.launch.py \
  exposure_us:=30000 iso:=800 projector:=0.5 flood:=0.0
```

여러 OAK 장치가 연결되면 `device_id:=장치_MXID`를 지정합니다. 카메라를 이미
열고 있는 다른 카메라/자율주행 launch는 종료해야 합니다. 일반 수동 주행
`manual_drive.launch.py`와는 함께 실행할 수 있습니다.

## 화면 조작

두 창이 표시됩니다.

1. **Tunnel TCN - partial INT8**: 스테레오 중앙 영상 하단에 NONE/ENTRY/EXIT
   확률, 최근 확정 이벤트, 추정 터널 상태, 실제 수신 FPS, 추정 누락 프레임,
   추론 시간과 프레임 처리 시간을 표시합니다. 영상 위에 글자를 덮지 않고
   하단에 표시 영역을 추가합니다.
2. **OAK IR live controls**: 입력 칸을 클릭하고 숫자를 입력한 다음 Enter를
   누르면 해당 값을 적용합니다. APPLY는 모든 입력값을 적용합니다.

| 제어 | 범위/의미 |
|---|---|
| Brightness: exp [us] | 10–33333µs, 기본 30000µs |
| ISO | 100–1600, 기본 800 |
| IR projector | 0.0–1.0, 기본 1.0. 스테레오용 점 패턴 투사기 |
| IR flood | 0.0–1.0, 기본 0.0. 균일 적외선 조명 |
| Tab | 다음 입력 칸 선택 |
| I | 설정된 IR emitter 켜기/끄기 |
| A | 자동 노출로 전환. 노출/ISO 입력 후 Enter로 수동 복귀 |
| R, 추론 창 | 새로운 독립 주행 시작: 상태와 과거 입력 초기화 |
| Q 또는 Esc | 창이 속한 테스트 launch 종료 |

카메라 제어 창의 `IR actual`은 센서 메타데이터에서 읽은 실제 노출/ISO입니다.
추론 하단의 Camera 항목은 마지막 요청 설정입니다. 자동 노출일 때는 `AUTO`를
표시하므로 요청값과 실제 센서값을 구분할 수 있습니다.

초기 상태는 UNKNOWN입니다. 첫 ENTRY 후 INSIDE, 첫 EXIT 후 OUTSIDE로 바뀝니다.
ENTRY 뒤에는 EXIT만, EXIT 뒤에는 ENTRY만 허용합니다. 낮은 확률이나 잠깐 정지로
이 상태가 초기화되지는 않습니다. 최근 이벤트는 경과 시간을 함께 계속 표시하므로
지난 이벤트와 새 이벤트를 구분할 수 있습니다. 연결이 끊기면 STALE 상태가 표시됩니다.

## 모델과 시간 처리

포함 모델은 `BrightnessTCN31_2s_v1`의 `int8_minmax_last_two_conv`입니다.
첫 Conv와 최종 Linear는 FP32로 유지하고 뒤의 Conv 두 개에 INT8 Q/DQ를 적용했습니다.
입력은 float32 `[1,21,31]`, 출력은 float32 logits `[1,3]`입니다.
softmax 후 임계값 0.8, 쿨다운 0.1초, 교대 규칙을 적용합니다.

640×400 INTER_AREA 전처리, 31개 밝기 특징, 과거 2초의 21개 시점을 사용합니다.
30FPS의 마지막 21프레임(0.67초)을 넣는 방식이 아닙니다. 시간 미분은 원본
수신 프레임 사이에서 계산하며, 0.5초 평활값을 포함합니다.

카메라의 촬영 중간 시각을 ROS 시각으로 변환해 입력 timestamp로 사용합니다.
정상적인 10/30FPS 입력에서는 원본 학습 특징 계산과 일치합니다. 불규칙하게
수신되면 변화율은 실제 시간 간격으로 계산하고, 각 문맥 시점보다 늦지 않은
최근 특징을 선택합니다. 누락 영상은 만들어내지 않습니다. 실제 FPS가 27 미만이면
LOW SOURCE FPS를 표시합니다. 이는 요청한 30FPS를 보장한다는 의미가 아닙니다.

시작 또는 카메라 설정 변경 뒤 2.5초는 WARMUP을 표시하고 **확정 이벤트를
출력하지 않습니다**. 설정 변경에 따른 밝기 과도 변화를 구분하기 위한 동작입니다.
설정 변경, 0.5초 초과 입력 공백, 역행 timestamp는 특징 이력을 재구성하지만
이미 판정한 터널 상태는 유지합니다. 새로운 주행은 R로 명시적으로 초기화합니다.
초기 2.5초에 경계를 통과하면 이벤트가 출력되지 않으므로 LIVE 이후에 테스트합니다.

이미지가 0.25초 이상 오래되면 추론하지 않습니다. 화면의 `TRT+transfer`는
GPU 전송을 포함한 추론 호출 시간이고, `frame processing`은 특징 추출과
추론을 포함합니다. 카메라 노출/USB/ROS 전송/화면 표시 전체 지연은 아닙니다.

## 결과 확인 및 재현 검증

```bash
ros2 topic echo /tunnel_tcn/result
ros2 topic hz /camera/image_stereo_ir
```

`/tunnel_tcn/result`는 JSON 문자열을 담은 `std_msgs/String`입니다. source stamp,
세 클래스 확률, 상태, 해당 프레임의 확정 이벤트(없으면 null), FPS와 지연을 담습니다.
`/camera/ir_settings`는 제어 변경 시 마지막 설정을 발행합니다.

장치에서 모델 변환 결과를 확인하려면, 카메라 없이 저장된 5개 영상의 특징을
재생하는 검증 명령을 사용할 수 있습니다.

```bash
ros2 run tunnel_tcn_test verify_model --backend tensorrt \
  --output /tmp/tunnel_tcn_tensorRT_check.json
```

모든 영상의 확정 이벤트 종류·프레임이 로컬 부분 INT8 참조와 같고, 최대 확률
차이가 0.005 이하인지 검사합니다. 성공 시 `passed: true`입니다. 이 검증의
warmup은 0초로, 원본 모델 시작 패딩까지 비교합니다. 실제 주행 화면은 위의
2.5초 warmup을 사용합니다. 포함된 참조 특징은 기존 검증/테스트 데이터이며
추가 학습이나 보정 데이터로 사용하지 않습니다.

개발 PC에서는 ONNX Runtime을 별도로 설치한 환경으로 다음을 실행할 수 있습니다.

```bash
PYTHONPATH=src/tunnel_tcn_test python3 -m unittest discover -s src/tunnel_tcn_test/test -v
PYTHONPATH=src/tunnel_tcn_test python3 -m tunnel_tcn_test.verify --backend onnx
```

로컬 검증은 TensorRT 장치 검증을 대체하지 않습니다. 이번 구현 환경에는
ROS 2/CUDA/Jetson이 없어 카메라 노드의 Jetson 빌드와 실카메라 주행은 미실행입니다.
검증 결과는 `test/local_replay_report.json`과 `test/verification.json`에 기록합니다.

## 기존 카메라 패키지 변경

`ir_camera_driver`에 projector 입력 칸, 설정 변경 토픽, 선택적 sensor timestamp,
`rgb_enabled`와 `bev_processing_enabled`를 추가했습니다. 기존 RGB/BEV 기본값은
켜짐이고, 이 테스트 launch에서만 둘 다 끕니다. 카메라 FPS 스케줄은 매번 현재
시각에서 33.3ms를 더하는 대신 목표 주기를 누적해 처리 지연이 누적되지 않게 했습니다.
이 launch는 불필요한 RGB 캡처·BEV 변환·높이 측정·IMU 보정을 실행하지 않습니다.

TensorRT 구현 참고: [NVIDIA Python API](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/python-api-docs.html),
[TensorRT 8.6 ExecutionContext](https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-861/api/python_api/infer/Core/ExecutionContext.html).
센서 시각 참고: [DepthAI 호스트 동기화](https://docs.luxonis.com/software-v3/depthai/depthai-components/device).
