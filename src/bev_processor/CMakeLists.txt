cmake_minimum_required(VERSION 3.8)
project(bev_processor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(bev_geometry STATIC
  src/bev_geometry.cpp
)
target_include_directories(bev_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(bev_geometry PUBLIC ${OpenCV_LIBS})

add_executable(bev_processor_node
  src/bev_processor_node.cpp
)
target_include_directories(bev_processor_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(bev_processor_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(bev_processor_node
  bev_geometry
  ${OpenCV_LIBS}
  Threads::Threads
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(bev_geometry_test test/bev_geometry_test.cpp)
  target_link_libraries(bev_geometry_test bev_geometry ${OpenCV_LIBS})
  add_test(NAME bev_geometry_test COMMAND bev_geometry_test)
endif()

install(TARGETS
  bev_geometry
  bev_processor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(OpenCV)
ament_package()
