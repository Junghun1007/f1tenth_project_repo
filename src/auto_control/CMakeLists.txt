cmake_minimum_required(VERSION 3.8)
project(auto_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(line_detactor REQUIRED)
find_package(traffic_detection_test REQUIRED)
find_package(Threads REQUIRED)

add_executable(auto_control_node
  src/auto_control_node.cpp
  src/control_core.cpp
  src/performance_measurement.cpp
  src/driving_log.cpp
)
target_include_directories(auto_control_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(auto_control_node
  builtin_interfaces
  rclcpp
  std_msgs
)
# These packages also export camera/inference components. Control consumes
# only their ROS messages; do not link every exported implementation target.
target_link_libraries(auto_control_node
  Threads::Threads
  line_detactor::line_detactor__rosidl_typesupport_cpp
  traffic_detection_test::traffic_detection_test__rosidl_typesupport_cpp
)

install(TARGETS auto_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES
  README.md
  AUTO_CONTROL_PARAMETER_TUNING_KO.txt
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(staged_brake_test test/staged_brake_test.cpp src/control_core.cpp)
  target_include_directories(staged_brake_test PRIVATE include)
  add_test(NAME staged_brake_test COMMAND staged_brake_test)
endif()

ament_package()
