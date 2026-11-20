cmake_minimum_required(VERSION 3.8)
project(line_detactor_msgs)
# 차선 검출 결과 메시지만 담는다. 받는 쪽(auto_control 등)이 CUDA·TensorRT 패키지 없이 빌드되게 분리
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneResult.msg"
  "msg/SpeedLimitState.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
