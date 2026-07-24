# Pending camera files

정리 전 카메라 구현을 보관하는 폴더다. 이 폴더 아래의 ROS 패키지는
`COLCON_IGNORE`로 차단되어 기본 `colcon build` 대상이 아니다.

## 폴더 구성

- `ros_packages/`: 카메라 드라이버, 영상 처리·뷰어, BEV ROS 패키지
- `function_test/`: 왜곡 보정 단독 기능 테스트와 입출력 이미지
- `legacy_camera/`: 이전 카메라 노드, 보정 도구와 보정 데이터
- `JETSON_RUN_COMMANDS.txt`: 보관 시점의 실행 명령 기록

다시 개발할 때는 필요한 구현을 검토한 뒤 새 구조로 `src/`에 복원한다.
이 폴더의 패키지를 그대로 활성 코드로 간주하지 않는다.
