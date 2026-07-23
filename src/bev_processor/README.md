# bev_processor

DepthAI에서 렌즈 왜곡 보정된 단일 `CAM_A` 영상을 차량 지면 기준 BEV로
변환하는 ROS 2 C++ 노드다. BEV 노드에서는 렌즈 왜곡 보정을 다시 하지
않으며, 초기화 또는 자세 변경 시 역투영 LUT를 만든 뒤 프레임마다
`cv::remap()`만 수행한다.

기본 좌표 원점은 전륜축 중앙의 지면점이다. `+X`는 차량 전방, `+Y`는
차량 좌측, `+Z`는 위쪽이며 BEV 영상의 위쪽이 전방이다.

## 실행

```bash
ros2 launch bev_processor bev.launch.py
```

위 launch는 카메라 드라이버와 normal image publisher까지 함께 시작한다.
입력은 `/image/normal`, 출력은 비압축 `bgr8` 형식의 `/image/bev`다.

설정은 `config/bev_config.yaml`에 있다. 현재 내부 파라미터, 장착 위치,
BEV 범위는 임시값이다. 특히 `fx`, `fy`, `cx`, `cy`는 DepthAI가 출력한
최종 rectified 해상도에 맞는 실제 값으로 교체해야 한다.

IMU를 사용하지 않을 때는 `use_imu: false`로 두며 평탄한 노면과 안정된
차량 자세를 가정한다. 나중에 IMU 방향이 확인되면 부호 설정을 검증한 후
`use_imu: true`로 바꾼다.
