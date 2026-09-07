cmake_minimum_required(VERSION 3.16)
project(traffic_detection_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core dnn highgui imgproc)
find_package(depthai 3 CONFIG QUIET)
if(NOT depthai_FOUND)
  message(FATAL_ERROR
    "DepthAI C++ 3.x was not found. Install depthai-core as a shared C++ "
    "library and provide depthai_DIR or CMAKE_PREFIX_PATH."
  )
endif()
find_package(Threads REQUIRED)

add_library(traffic_yolox_detector STATIC
  src/yolox_detector.cpp
)
set_target_properties(
  traffic_yolox_detector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(traffic_yolox_detector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(traffic_yolox_detector PUBLIC ${OpenCV_LIBS})

add_library(traffic_detection_test_component SHARED
  src/traffic_detection_test_node.cpp
)
target_compile_definitions(traffic_detection_test_component
  PRIVATE TRAFFIC_DETECTION_TEST_BUILDING_DLL
)
target_include_directories(traffic_detection_test_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(traffic_detection_test_component
  ament_index_cpp
  rclcpp
  rclcpp_components
)
target_link_libraries(traffic_detection_test_component
  traffic_yolox_detector
  depthai::core
  ${OpenCV_LIBS}
  Threads::Threads
)

rclcpp_components_register_nodes(
  traffic_detection_test_component
  "traffic_detection_test::TrafficDetectionTestNode"
)

add_executable(traffic_detection_test_node
  src/main.cpp
)
ament_target_dependencies(traffic_detection_test_node rclcpp)
target_link_libraries(
  traffic_detection_test_node traffic_detection_test_component)

install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  TARGETS
    traffic_yolox_detector
    traffic_detection_test_component
  EXPORT export_traffic_detection_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  TARGETS traffic_detection_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_traffic_detection_test HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp OpenCV rclcpp rclcpp_components)

ament_package()
