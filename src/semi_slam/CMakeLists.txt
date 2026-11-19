cmake_minimum_required(VERSION 3.18)
project(semi_slam LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# ARM GCC가 DepthAI 헤더마다 내는 ABI 안내(note: parameter passing ... changed in GCC 10.1) 끄기. 동작과 무관
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-psabi>)
endif()

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/cmake/DepthaiWorkspace.cmake")
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d)

include(cmake/core_sources.cmake)
add_library(ss_core STATIC ${CDT_CORE_SOURCES} ${CDT_WEB_SOURCES})
target_include_directories(ss_core PUBLIC include third_party ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ss_core PUBLIC ${OpenCV_LIBS} Threads::Threads)
set_target_properties(ss_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(ss_core PUBLIC ${CDT_WEB_DEFINITIONS})

# Jetson 전용: CUDA BEV + TensorRT 차선 모델
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 17)
find_package(CUDAToolkit REQUIRED)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/local/TensorRT/include)
find_library(TENSORRT_LIBRARY nvinfer HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/TensorRT/lib)
find_library(TENSORRT_PARSER nvonnxparser HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/TensorRT/lib)
find_library(TENSORRT_PLUGIN nvinfer_plugin HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/TensorRT/lib)
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT TENSORRT_PARSER OR NOT TENSORRT_PLUGIN)
  message(FATAL_ERROR "TensorRT 개발 헤더/라이브러리가 필요합니다")
endif()
add_library(ss_gpu STATIC
  src/copied/oak_startup_measurement.cpp
  src/runtime/camera_pipeline.cpp
  src/runtime/lane_worker.cpp
  src/copied/cuda_bev_processor.cu
  src/copied/tensorrt_lane_backend.cpp
  src/copied/cuda_lane_pipeline.cu)
target_link_libraries(ss_gpu PUBLIC ss_core depthai::core CUDA::cudart ${TENSORRT_LIBRARY} ${TENSORRT_PARSER} ${TENSORRT_PLUGIN})
target_include_directories(ss_gpu PRIVATE ${TENSORRT_INCLUDE_DIR})
set_target_properties(ss_gpu PROPERTIES CUDA_SEPARABLE_COMPILATION OFF)

add_executable(semi_slam_node src/slam_node.cpp)
ament_target_dependencies(semi_slam_node rclcpp std_msgs ament_index_cpp)
target_link_libraries(semi_slam_node ss_gpu)

ament_python_install_package(semi_slam_py)
install(TARGETS semi_slam_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/safety_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch models maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY web/ DESTINATION share/${PROJECT_NAME}/web)

# 호스트 테스트 (ROS 없이): cmake -S test/host -B build/host
ament_package()
