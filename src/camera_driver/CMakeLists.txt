cmake_minimum_required(VERSION 3.16)
project(camera_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc)
find_package(depthai 3 CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_library(camera_driver_component SHARED
  src/camera_driver_node.cpp
)
target_compile_definitions(camera_driver_component
  PRIVATE CAMERA_DRIVER_BUILDING_DLL
)
target_include_directories(camera_driver_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_driver_component
  rclcpp
  rclcpp_components
  sensor_msgs
)
target_link_libraries(camera_driver_component
  depthai::core
  ${OpenCV_LIBS}
  Threads::Threads
)

rclcpp_components_register_nodes(
  camera_driver_component
  "camera_driver::CameraDriverNode"
)

add_executable(camera_driver_node
  src/main.cpp
)
ament_target_dependencies(camera_driver_node
  rclcpp
)
target_link_libraries(camera_driver_node
  camera_driver_component
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
  TARGETS camera_driver_component camera_driver_node
  EXPORT export_camera_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_camera_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs)

ament_package()
