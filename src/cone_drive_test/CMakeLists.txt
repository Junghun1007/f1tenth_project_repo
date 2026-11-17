cmake_minimum_required(VERSION 3.18)
project(cone_drive_test LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# ARM GCC가 DepthAI 헤더(std::pair 인자)마다 내는 ABI 안내(note: parameter passing ... changed in GCC 10.1) 끄기. 동작과 무관
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-psabi>)
endif()
option(CONE_DRIVE_TEST_TENSORRT "TensorRT + CUDA (Jetson). OFF는 개발 PC 문법 확인용" ON)

find_package(ament_cmake REQUIRED)
# DepthAI SDK 위치 고정 (multiclass_yolo와 같은 순서: ament_cmake 다음, 다른 의존성보다 먼저)
include("${CMAKE_CURRENT_LIST_DIR}/cmake/DepthaiWorkspace.cmake")
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs dnn calib3d)

include(cmake/core_sources.cmake)
add_library(cdt_core STATIC ${CDT_CORE_SOURCES} ${CDT_WEB_SOURCES})
target_include_directories(cdt_core PUBLIC include third_party ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cdt_core PUBLIC ${OpenCV_LIBS} Threads::Threads)
set_target_properties(cdt_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(cdt_core PUBLIC ${CDT_WEB_DEFINITIONS})

set(CDT_GPU_SOURCES
  ${CDT_DETECTOR_SOURCES}
  src/copied/oak_startup_measurement.cpp
  src/runtime/camera_pipeline.cpp
  src/runtime/perception.cpp)
if(CONE_DRIVE_TEST_TENSORRT)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  find_package(CUDAToolkit REQUIRED)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/local/TensorRT/include)
  find_library(TENSORRT_LIBRARY nvinfer HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/TensorRT/lib)
  find_library(TENSORRT_PARSER nvonnxparser HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/TensorRT/lib)
  find_library(TENSORRT_PLUGIN nvinfer_plugin HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/TensorRT/lib)
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT TENSORRT_PARSER OR NOT TENSORRT_PLUGIN)
    message(FATAL_ERROR "TensorRT 개발 헤더/라이브러리가 필요합니다 (multiclass_yolo README와 같음)")
  endif()
  list(APPEND CDT_GPU_SOURCES
    src/copied/cuda_bev_processor.cu
    src/copied/tensorrt_lane_backend.cpp
    src/copied/cuda_lane_pipeline.cu
    src/copied/tensorrt_yolox_backend.cpp
    src/copied/cuda_yolox_preprocessor.cu)
else()
  list(APPEND CDT_GPU_SOURCES src/copied/tensorrt_disabled.cpp)
endif()
add_library(cdt_gpu STATIC ${CDT_GPU_SOURCES})
target_link_libraries(cdt_gpu PUBLIC cdt_core depthai::core)
if(CONE_DRIVE_TEST_TENSORRT)
  target_include_directories(cdt_gpu PRIVATE ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(cdt_gpu PUBLIC CUDA::cudart ${TENSORRT_LIBRARY} ${TENSORRT_PARSER} ${TENSORRT_PLUGIN})
  set_target_properties(cdt_gpu PROPERTIES CUDA_SEPARABLE_COMPILATION OFF)
endif()

add_executable(cone_drive_test_node src/drive_node.cpp)
ament_target_dependencies(cone_drive_test_node rclcpp std_msgs ament_index_cpp)
target_link_libraries(cone_drive_test_node cdt_gpu)

ament_python_install_package(cone_drive_test_py)
install(TARGETS cone_drive_test_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/safety_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY web/ DESTINATION share/${PROJECT_NAME}/web PATTERN "demo" EXCLUDE)

if(BUILD_TESTING)
  # 호스트 테스트는 test/host (ROS 없이): cmake -S test/host -B build/host
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(handover_state test/python/test_handover_state.py)
  endif()
endif()
ament_package()
