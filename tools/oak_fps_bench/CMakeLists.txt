# OAK FPS 진단 벤치마크 (ROS 패키지 아님, colcon은 COLCON_IGNORE로 건너뜀)
#   cmake -S tools/oak_fps_bench -B /tmp/oak_fps_bench_build -DCMAKE_BUILD_TYPE=Release
#   cmake --build /tmp/oak_fps_bench_build -j4
cmake_minimum_required(VERSION 3.16)
project(oak_fps_bench CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 워크스페이스와 같은 SDK (/usr/local, DEPTHAI_SETUP.md). /opt/ros의 DepthAI와 섞지 않는다
set(F1TENTH_DEPTHAI_PREFIX "/usr/local" CACHE PATH "Prefix of the installed DepthAI C++ SDK")
find_package(depthai 3 CONFIG REQUIRED
  PATHS
    "${F1TENTH_DEPTHAI_PREFIX}/lib/cmake/depthai"
    "${F1TENTH_DEPTHAI_PREFIX}/lib64/cmake/depthai"
    "${F1TENTH_DEPTHAI_PREFIX}/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/depthai"
    "${F1TENTH_DEPTHAI_PREFIX}/share/depthai/cmake"
  NO_DEFAULT_PATH)
find_package(Threads REQUIRED)

add_executable(oak_fps_bench oak_fps_bench.cpp)
target_link_libraries(oak_fps_bench PRIVATE depthai::core Threads::Threads)
target_compile_definitions(oak_fps_bench PRIVATE OAKBENCH_DEPTHAI_VERSION="${depthai_VERSION}")
get_target_property(_dai_lib depthai::core IMPORTED_LOCATION_RELEASE)
if(NOT _dai_lib)
  get_target_property(_dai_lib depthai::core IMPORTED_LOCATION)
endif()
if(_dai_lib)
  get_filename_component(_dai_dir "${_dai_lib}" DIRECTORY)
  set_target_properties(oak_fps_bench PROPERTIES BUILD_RPATH "${_dai_dir}")
endif()
message(STATUS "oak_fps_bench: depthai ${depthai_VERSION} from ${depthai_DIR}")
