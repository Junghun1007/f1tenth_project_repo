# bev_processor

DepthAI에서 렌즈 왜곡 보정된 단일 `CAM_A` 영상을 차량 지면 기준 BEV로
변환하는 ROS 2 C++ 노드다. BEV 노드에서는 렌즈 왜곡 보정을 다시 하지
않으며, 초기화 또는 자세 변경 시 역투영 LUT를 만든 뒤 프레임마다
`cv::remap()`만 수행한다. 처리 결과는 ROS 이미지로 다시 왕복하지 않고
노드 내부의 `cv::Mat`을 OpenCV 창에 직접 표시할 수 있다.

기본 좌표 원점은 전륜축 중앙의 지면점이다. `+X`는 차량 전방, `+Y`는
차량 좌측, `+Z`는 위쪽이며 BEV 영상의 위쪽이 전방이다.

## 실행

```bash
ros2 launch bev_processor bev.launch.py
```

위 launch는 왜곡 보정이 활성화된 카메라 드라이버도 함께 시작한다.
기본값은 카메라 프리뷰를 숨기고 `BEV processed image` 창만 표시한다.
입력은 `/image/normal`, 선택적 ROS 출력은 비압축 `bgr8` 형식의
`/image/bev`다.

프리뷰 실행 방법:

```bash
# BEV 프리뷰만
ros2 launch bev_processor bev.launch.py

# 왜곡 보정 프리뷰와 BEV 프리뷰를 동시에
ros2 launch bev_processor bev.launch.py normal_preview_enabled:=true
```

이미 `/image/normal`을 발행하는 카메라 노드가 실행 중이라면 카메라를
중복 실행하지 않고 BEV processor만 시작할 수 있다.

```bash
ros2 launch bev_processor bev_processor_only.launch.py
```

`processing_rate_hz`, `preview_fps`, `publish_rate_hz`는 각각 BEV 변환,
직접 프리뷰, `/image/bev` 디버그 토픽 발행 주기다.
`publish_enabled: false`로 두면 BEV 창은 유지하면서 대용량 디버그 영상
토픽만 끌 수 있다.

설정은 `config/bev_config.yaml`에 있다. 현재 내부 파라미터, 장착 위치,
BEV 범위는 임시값이다. 특히 `fx`, `fy`, `cx`, `cy`는 DepthAI가 출력한
최종 rectified 해상도에 맞는 실제 값으로 교체해야 한다.

IMU를 사용하지 않을 때는 `use_imu: false`로 두며 평탄한 노면과 안정된
차량 자세를 가정한다. 나중에 IMU 방향이 확인되면 부호 설정을 검증한 후
`use_imu: true`로 바꾼다.
