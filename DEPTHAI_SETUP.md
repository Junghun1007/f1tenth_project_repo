# DepthAI C++ 경로 고정

모든 OAK 관련 패키지는 워크스페이스 옆의 **`depthai-core/install`**에 설치한
동일한 DepthAI C++ 3.6+ 공유 라이브러리를 사용한다.

```text
1030ML/
├── depthai-core/
│   ├── CMakeLists.txt
│   ├── build/
│   └── install/                 # 이 SDK만 사용
│       ├── include/
│       └── lib/cmake/depthai/
└── f1tenth_project_repo/
    ├── cmake/DepthaiWorkspace.cmake
    ├── scripts/install_depthai.sh
    ├── src/
    ├── build/
    ├── install/
    └── log/
```

`1030ML` 상위 경로나 사용자 이름은 하드코딩하지 않는다. 각 PC에서 저장소 위치를
기준으로 계산하므로 두 폴더를 나란히 두면 된다. `lib64`와 multiarch lib 경로도 지원한다.

## 처음 한 번 SDK 설치

기존 `depthai-core` 체크아웃을 사용한다. ROS 2 Humble, C++ 빌드 도구, OpenCV 개발
패키지와 DepthAI 빌드 의존성이 설치되어 있어야 한다. Python의 `pip install depthai`는
C++ SDK 설치를 대신하지 않는다.

```bash
cd ~/Desktop/1030ML/f1tenth_project_repo
bash scripts/install_depthai.sh
```

스크립트는 `../depthai-core/build`를 Release/OpenCV/공유 라이브러리 설정으로 빌드하고
`../depthai-core/install`에 설치한다. 기본 병렬 수는 2이며, 필요하면
`DEPTHAI_BUILD_JOBS=4 bash scripts/install_depthai.sh`로 조절한다.
다운로드, `sudo`, 시스템 `/usr/local` 설치 또는 `ldconfig` 변경은 수행하지 않는다.
SDK 소스를 업데이트했을 때만 다시 실행하면 된다.

## ROS 패키지 재빌드

기존 SDK로 빌드한 패키지는 처음 한 번 다시 빌드한다. 새 터미널에서 진행하고,
빌드 전에 이 워크스페이스의 `install/setup.bash`를 source하지 않는다.
`.bashrc`에서 자동 source한다면 해당 설정을 잠시 해제한다.

```bash
cd ~/Desktop/1030ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
colcon build --packages-up-to point_cloud roi_lidar --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch point_cloud point_cloud.launch.py
```

카메라/BEV/IR/신호등 패키지도 사용한다면 해당 패키지와 의존 패키지를 재빌드한다.
전체 환경이 갖춰진 차량 PC에서는 같은 옵션으로 `--packages-up-to ...`를 생략해
워크스페이스 전체를 빌드할 수 있다. 브랜치 변경 후 삭제된 패키지의
`local_setup.bash`가 없다는 메시지는 별도의 오래된 install 문제이므로 설치 산출물을
백업하고 필요한 패키지를 새로 빌드해야 한다.

## 기존 산출물을 모두 폐기하고 전체 재빌드

실행 중인 ROS 노드를 종료한 뒤 차량 PC의 새 터미널에서 실행한다.
`build/`, `install/`, `log/`를 삭제하며 소스와 루트의 개인 YAML은 유지한다.
기존 터미널 환경에 남은 작업공간 경로도 해제한 뒤 ROS Humble만 다시 불러온다.

```bash
cd ~/Desktop/1030ML/f1tenth_project_repo &&
git switch 1030ML &&
git pull --ff-only origin 1030ML &&
rm -rf -- build install log &&
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH &&
source /opt/ros/humble/setup.bash &&
bash scripts/install_depthai.sh &&
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release &&
source install/setup.bash
```

패키지 선택 제한 없이 전체 패키지를 빌드한다. 메모리 사용을 줄이기 위해 패키지는
순차 실행하고 각 CMake 빌드의 병렬 수는 2로 제한한다. SDK 설치 스크립트는 기존
DepthAI 빌드를 재사용한다. 위 명령은 SDK 소스와 SDK 빌드 폴더를 삭제하지 않는다.
빌드가 성공하면 `ros2 launch point_cloud point_cloud.launch.py` 등으로 실행한다.

## 이후 실행

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/1030ML/f1tenth_project_repo/install/setup.bash
ros2 launch point_cloud point_cloud.launch.py
```

`source install/setup.bash`는 공통 환경 훅으로 SDK의 라이브러리 경로를
`LD_LIBRARY_PATH` 맨 앞에 배치한다. 중복 경로는 제거하므로 반복 source해도
누적되지 않는다. `ros2 run`과 컴포넌트 컨테이너에도 같은 환경이 적용된다.
기존 launch의 `depthai_library_dir` 인자는 기본값 그대로 사용한다. 명시적으로
다른 경로를 전달하면 해당 launch의 실행 SDK를 바꾸므로 일반 실행에서는 생략한다.

빌드 시 이전 `depthai_DIR` 캐시를 덮어쓰며 `/opt/ros`나 `/usr/local`의 다른 DepthAI로
자동 대체하지 않는다. SDK가 없거나 정적 라이브러리이면 설치 명령과 함께 빌드를 중단한다.
선택된 config/공유 라이브러리는 CMake 로그의 `Workspace DepthAI` 항목에 출력하고,
각 패키지의 `share/<package>/depthai_runtime_dir.txt`에도 기록한다.

```bash
cat install/point_cloud/share/point_cloud/depthai_runtime_dir.txt
ldd install/point_cloud/lib/point_cloud/point_cloud_node | grep depthai
```

경로 정책과 실제 공유 라이브러리 충돌에 대한 ROS 독립 검증:

```bash
python3 -m unittest discover -s tests -p 'test_depthai_workspace.py' -v
python3 src/bev_processor/test/test_depthai_runtime_launch.py
```
