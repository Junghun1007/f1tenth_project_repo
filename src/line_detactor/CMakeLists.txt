cmake_minimum_required(VERSION 3.18)
project(line_detactor LANGUAGES C CXX CUDA)

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneCurve.msg"
  "msg/LaneResult.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)
rosidl_get_typesupport_target(lane_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc)
find_package(CUDAToolkit REQUIRED)
find_package(Threads REQUIRED)

find_path(TENSORRT_INCLUDE_DIR
  NAMES NvInfer.h
  HINTS
    /usr/include/aarch64-linux-gnu
    /usr/include/x86_64-linux-gnu
    /usr/local/TensorRT/include
    /usr/local/include
)
find_library(TENSORRT_NVINFER_LIBRARY
  NAMES nvinfer
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
)
find_library(TENSORRT_ONNX_PARSER_LIBRARY
  NAMES nvonnxparser
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
)
find_library(TENSORRT_PLUGIN_LIBRARY
  NAMES nvinfer_plugin
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
)
if(
  NOT TENSORRT_INCLUDE_DIR OR
  NOT TENSORRT_NVINFER_LIBRARY OR
  NOT TENSORRT_ONNX_PARSER_LIBRARY OR
  NOT TENSORRT_PLUGIN_LIBRARY
)
  message(FATAL_ERROR
    "TensorRT development files were not found. Install the JetPack "
    "libnvinfer-dev, libnvonnxparsers-dev, and libnvinfer-plugin-dev packages."
  )
endif()

add_library(line_tensorrt_backend STATIC
  src/cuda_lane_pipeline.cu
  src/tensorrt_lane_backend.cpp
)
set_target_properties(line_tensorrt_backend PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(line_tensorrt_backend
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${TENSORRT_INCLUDE_DIR}
)
target_link_libraries(line_tensorrt_backend
  PRIVATE
    CUDA::cudart
    ${TENSORRT_NVINFER_LIBRARY}
    ${TENSORRT_ONNX_PARSER_LIBRARY}
    ${TENSORRT_PLUGIN_LIBRARY}
)

add_library(line_detactor_component SHARED
  src/line_detactor_node.cpp
  src/lane_connector.cpp
)
target_compile_definitions(line_detactor_component
  PRIVATE LINE_DETACTOR_BUILDING_DLL
)
target_include_directories(line_detactor_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(line_detactor_component
  ament_index_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
)
target_link_libraries(line_detactor_component
  line_tensorrt_backend
  "${lane_typesupport}"
  ${OpenCV_LIBS}
  Threads::Threads
)

rclcpp_components_register_nodes(
  line_detactor_component
  "line_detactor::LineDetactorNode"
)

add_executable(line_detactor_node src/main.cpp)
ament_target_dependencies(line_detactor_node rclcpp)
target_link_libraries(line_detactor_node line_detactor_component)

install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch models tools
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.engine" EXCLUDE
)
install(
  TARGETS
    line_tensorrt_backend
    line_detactor_component
  EXPORT export_line_detactor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  TARGETS line_detactor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_line_detactor HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  ament_index_cpp
  CUDAToolkit
  OpenCV
  rclcpp
  rclcpp_components
  sensor_msgs
)

ament_package()
