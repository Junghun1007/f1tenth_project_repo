cmake_minimum_required(VERSION 3.18)
project(manual_capture LANGUAGES CXX CUDA)

# Self-contained: the BEV geometry, ground-plane measurement, CUDA center
# reprojection and the IMU image stabilizer (camera_driver) are copied into
# this package so removing it (or the packages they came from) affects nothing else.

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
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgcodecs imgproc)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Workspace-wide DepthAI SDK selection (same as every other OAK package).
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")
find_package(Threads REQUIRED)

# Camera/Qt independent logic and geometry (unit tested).
add_library(manual_capture_core STATIC
  src/capture_logic.cpp
  src/capture_writer.cpp
  src/bev_geometry.cpp
  src/bev_projector.cpp
)
set_target_properties(manual_capture_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(manual_capture_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(manual_capture_core PUBLIC ${OpenCV_LIBS} Threads::Threads)

# Copy of camera_driver's IMU pitch/roll stabilizer + the vehicle-motion wrapper
# auto_control uses (ERPM standstill, CAN acceleration, lever arm).
add_library(manual_capture_imu STATIC
  src/imu_image_stabilizer.cpp
  src/imu_tilt_tracker.cpp
)
set_target_properties(manual_capture_imu PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(manual_capture_imu PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(manual_capture_imu PUBLIC ${OpenCV_LIBS} Threads::Threads)

# Copy of oak_startup: stationary IMU + stereo ground-plane measurement.
add_library(manual_capture_ground STATIC
  src/ground_plane_estimator.cpp
  src/startup_attitude.cpp
  src/oak_startup_measurement.cpp
)
set_target_properties(manual_capture_ground PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(manual_capture_ground PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(manual_capture_ground PUBLIC depthai::core ${OpenCV_LIBS} Threads::Threads)

# Copy of the IR center-view CUDA reprojector.
add_library(manual_capture_reprojector STATIC src/cuda_center_reprojector.cu)
set_target_properties(manual_capture_reprojector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(manual_capture_reprojector PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_compile_options(manual_capture_reprojector PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>)
target_link_libraries(manual_capture_reprojector PUBLIC CUDA::cudart ${OpenCV_LIBS})

add_executable(manual_capture_node
  src/main.cpp
  src/main_window.cpp
  src/camera_engine.cpp
  include/manual_capture/main_window.hpp
)
set_target_properties(manual_capture_node PROPERTIES AUTOMOC ON)
ament_target_dependencies(manual_capture_node geometry_msgs rclcpp sensor_msgs std_msgs)
target_link_libraries(manual_capture_node
  manual_capture_core
  manual_capture_imu
  manual_capture_ground
  manual_capture_reprojector
  depthai::core
  Qt5::Widgets
  ${OpenCV_LIBS}
  Threads::Threads
)

install(TARGETS manual_capture_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(manual_capture_test
    test/capture_logic_test.cpp
    test/bev_projector_test.cpp
    test/imu_tilt_tracker_test.cpp
  )
  target_link_libraries(manual_capture_test manual_capture_core manual_capture_imu)
  # Same standalone test as camera_driver (own main), run on the copy.
  add_executable(manual_capture_imu_stabilizer_test test/imu_image_stabilizer_test.cpp)
  target_link_libraries(manual_capture_imu_stabilizer_test manual_capture_imu)
  add_test(NAME manual_capture_imu_stabilizer_test COMMAND manual_capture_imu_stabilizer_test)
endif()

ament_package()
