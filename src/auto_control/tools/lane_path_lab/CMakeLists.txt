# 호스트 빌드 (ROS 불필요):
#   cmake -S src/auto_control/tools/lane_path_lab -B build/lane_path_lab && cmake --build build/lane_path_lab -j4
cmake_minimum_required(VERSION 3.16)
project(lane_path_lab CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(SRC_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs dnn)
add_executable(lane_path_lab
  ${CMAKE_CURRENT_LIST_DIR}/lane_path_lab.cpp
  ${SRC_ROOT}/line_detactor/src/lane_connector.cpp
  ${SRC_ROOT}/line_detactor/src/centerline.cpp
  ${SRC_ROOT}/line_detactor/src/processing_profile.cpp)
target_include_directories(lane_path_lab PRIVATE
  ${SRC_ROOT}/line_detactor/include ${SRC_ROOT}/bev_handoff/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lane_path_lab ${OpenCV_LIBS})
