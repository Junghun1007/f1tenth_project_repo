cmake_minimum_required(VERSION 3.8)
project(bev_processor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(bev_processor_node src/bev_processor_node.cpp)
target_compile_features(bev_processor_node PUBLIC cxx_std_17)
target_include_directories(bev_processor_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(
  bev_processor_node
  rclcpp
  sensor_msgs
)
target_link_libraries(bev_processor_node ${OpenCV_LIBRARIES})

install(TARGETS bev_processor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
