# ROS·DepthAI·CUDA 없이 빌드되는 소스 (패키지 빌드와 호스트 테스트가 공유)
set(CDT_PKG_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
file(GLOB CDT_CORE_SOURCES CONFIGURE_DEPENDS ${CDT_PKG_DIR}/src/core/*.cpp)   # 새 파일도 빌드 때 다시 검색
list(APPEND CDT_CORE_SOURCES
  ${CDT_PKG_DIR}/src/copied/control_core.cpp
  ${CDT_PKG_DIR}/src/copied/centerline.cpp
  ${CDT_PKG_DIR}/src/copied/lane_connector.cpp
  ${CDT_PKG_DIR}/src/copied/processing_profile.cpp
  ${CDT_PKG_DIR}/src/copied/bev_geometry.cpp
  ${CDT_PKG_DIR}/src/copied/startup_attitude.cpp
  ${CDT_PKG_DIR}/src/copied/ground_plane_estimator.cpp
  ${CDT_PKG_DIR}/src/copied/imu_image_stabilizer.cpp
  ${CDT_PKG_DIR}/src/copied/vehicle_motion_estimator.cpp
  ${CDT_PKG_DIR}/src/copied/stop_line_distance.cpp)
# 내장 웹 서버 (cpp-httplib, ROS 불필요)
set(CDT_WEB_SOURCES ${CDT_PKG_DIR}/src/runtime/web_server.cpp)
# 브라우저 1대가 SSE 1 + MJPEG 최대 2 + keep-alive POST 1개를 잡는다 → 기본 8개면 2~3대에서 heartbeat·STOP이 밀림
set(CDT_WEB_DEFINITIONS CPPHTTPLIB_THREAD_POOL_COUNT=32)
