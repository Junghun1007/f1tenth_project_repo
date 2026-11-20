# auto_control 주행부 (cone_drive_test에서 옮김): ROS·DepthAI·CUDA 없이 빌드되는 소스. 패키지 빌드와 호스트 테스트가 공유
set(ACD_PKG_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
file(GLOB ACD_CORE_SOURCES CONFIGURE_DEPENDS ${ACD_PKG_DIR}/src/drive/core/*.cpp)   # 새 파일도 빌드 때 다시 검색
list(APPEND ACD_CORE_SOURCES
  ${ACD_PKG_DIR}/src/drive/copied/control_core.cpp
  ${ACD_PKG_DIR}/src/drive/copied/centerline.cpp
  ${ACD_PKG_DIR}/src/drive/copied/lane_connector.cpp
  ${ACD_PKG_DIR}/src/drive/copied/processing_profile.cpp
  ${ACD_PKG_DIR}/src/drive/copied/bev_geometry.cpp
  ${ACD_PKG_DIR}/src/drive/copied/stop_line_distance.cpp)
# YOLOX 검출기: TensorRtYoloxBackend를 호출하므로 백엔드(실제 또는 disabled)와 같은 라이브러리에 둔다
set(ACD_DETECTOR_SOURCES ${ACD_PKG_DIR}/src/drive/copied/yolox_detector.cpp)
# 내장 웹 서버 (cpp-httplib, ROS 불필요)
set(ACD_WEB_SOURCES ${ACD_PKG_DIR}/src/drive/runtime/web_server.cpp)
# 브라우저 1대가 SSE 1 + MJPEG 최대 2 + keep-alive POST 1개를 잡는다 → 기본 8개면 2~3대에서 heartbeat·STOP이 밀림
set(ACD_WEB_DEFINITIONS CPPHTTPLIB_THREAD_POOL_COUNT=32)
# 검출 워커 (YOLO → 신호등 색·박스 깊이). ROS 없이 빌드되지만 YoloxDetector를 쓰므로 검출기 라이브러리와 함께 링크
set(ACD_RUNTIME_SOURCES ${ACD_PKG_DIR}/src/drive/runtime/detector.cpp)
