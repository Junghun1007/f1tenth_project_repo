cmake_minimum_required(VERSION 3.18)
project(ir_camera_driver LANGUAGES CXX CUDA)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgcodecs)
find_package(rclcpp REQUIRED)
find_package(depthai 3.6 CONFIG QUIET)
if(NOT depthai_FOUND)
  message(FATAL_ERROR
    "DepthAI C++ 3.6+ was not found. ir_camera_driver links directly "
    "against depthai::core. Install the same DepthAI C++ library used by "
    "camera_driver and provide depthai_DIR."
  )
endif()
find_package(Threads REQUIRED)

add_library(ir_center_reprojector STATIC
  src/cuda_center_reprojector.cu
)
set_target_properties(
  ir_center_reprojector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ir_center_reprojector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(ir_center_reprojector PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>
)
target_link_libraries(
  ir_center_reprojector PUBLIC CUDA::cudart ${OpenCV_LIBS})

add_executable(ir_camera_driver_node
  src/ir_camera_driver_node.cpp
)
target_include_directories(ir_camera_driver_node
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(ir_camera_driver_node
  rclcpp
)
target_link_libraries(ir_camera_driver_node
  ir_center_reprojector
  depthai::core
  ${OpenCV_LIBS}
  Threads::Threads
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  TARGETS
    ir_center_reprojector
    ir_camera_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
