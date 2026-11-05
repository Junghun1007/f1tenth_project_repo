# One SDK for every ROS package: <workspace>/../depthai-core/install.
# Include this before loading ament dependencies which may import DepthAI.
include_guard(DIRECTORY)

get_filename_component(F1TENTH_DEPTHAI_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/../../depthai-core" ABSOLUTE)
set(F1TENTH_DEPTHAI_PREFIX "${F1TENTH_DEPTHAI_ROOT}/install")

# Ignore an old depthai_DIR cache, ROS underlays and /usr/local. Mixing the
# headers from one SDK with another SDK's shared library fails at runtime.
unset(_f1tenth_depthai_config_dir CACHE)
unset(_f1tenth_depthai_config_dir)
find_path(_f1tenth_depthai_config_dir
  NAMES depthaiConfig.cmake depthai-config.cmake
  PATHS
    "${F1TENTH_DEPTHAI_PREFIX}/lib/cmake/depthai"
    "${F1TENTH_DEPTHAI_PREFIX}/lib64/cmake/depthai"
    "${F1TENTH_DEPTHAI_PREFIX}/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/depthai"
    "${F1TENTH_DEPTHAI_PREFIX}/share/depthai/cmake"
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
if(NOT _f1tenth_depthai_config_dir)
  message(FATAL_ERROR
    "The workspace DepthAI C++ SDK is missing from ${F1TENTH_DEPTHAI_PREFIX}. "
    "Place depthai-core beside the workspace and run "
    "bash scripts/install_depthai.sh from the workspace root. "
    "System/ROS DepthAI installations are intentionally not used.")
endif()

set(depthai_DIR "${_f1tenth_depthai_config_dir}" CACHE PATH
  "DepthAI SDK belonging to this workspace" FORCE)
list(PREPEND CMAKE_PREFIX_PATH "${F1TENTH_DEPTHAI_PREFIX}")
find_package(depthai 3.6 CONFIG REQUIRED
  PATHS "${depthai_DIR}" NO_DEFAULT_PATH)

get_target_property(_f1tenth_depthai_type depthai::core TYPE)
if(NOT _f1tenth_depthai_type STREQUAL "SHARED_LIBRARY")
  message(FATAL_ERROR
    "${F1TENTH_DEPTHAI_PREFIX} must contain a shared DepthAI SDK. "
    "Run bash scripts/install_depthai.sh to build with BUILD_SHARED_LIBS=ON.")
endif()

# Resolve the imported library for the shell hook as well as the RPATH.
get_target_property(_f1tenth_depthai_library depthai::core IMPORTED_LOCATION)
if(NOT _f1tenth_depthai_library)
  get_target_property(_f1tenth_depthai_configs depthai::core IMPORTED_CONFIGURATIONS)
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _f1tenth_depthai_build_type)
  foreach(_config ${_f1tenth_depthai_build_type} RELEASE RELWITHDEBINFO
      NOCONFIG ${_f1tenth_depthai_configs})
    get_target_property(_f1tenth_depthai_library depthai::core
      "IMPORTED_LOCATION_${_config}")
    if(_f1tenth_depthai_library)
      break()
    endif()
  endforeach()
endif()
if(NOT _f1tenth_depthai_library OR NOT EXISTS "${_f1tenth_depthai_library}")
  message(FATAL_ERROR "The workspace DepthAI shared library is missing; "
    "run bash scripts/install_depthai.sh again.")
endif()
get_filename_component(_f1tenth_depthai_real_root "${F1TENTH_DEPTHAI_ROOT}" REALPATH)
get_filename_component(_f1tenth_depthai_real_library "${_f1tenth_depthai_library}" REALPATH)
file(RELATIVE_PATH _f1tenth_depthai_relative_library
  "${_f1tenth_depthai_real_root}" "${_f1tenth_depthai_real_library}")
if(IS_ABSOLUTE "${_f1tenth_depthai_relative_library}" OR
    _f1tenth_depthai_relative_library MATCHES "^\\.\\./")
  message(FATAL_ERROR "The DepthAI package points outside the sibling depthai-core: "
    "${_f1tenth_depthai_library}. Run bash scripts/install_depthai.sh again.")
endif()
get_filename_component(F1TENTH_DEPTHAI_LIBRARY_DIR
  "${_f1tenth_depthai_library}" DIRECTORY)

list(PREPEND CMAKE_BUILD_RPATH "${F1TENTH_DEPTHAI_LIBRARY_DIR}")
list(PREPEND CMAKE_INSTALL_RPATH "${F1TENTH_DEPTHAI_LIBRARY_DIR}")

# Existing camera/BEV launch files read this metadata for their child process.
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/depthai_runtime_dir.txt"
  CONTENT "$<TARGET_FILE_DIR:depthai::core>\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/depthai_runtime_dir.txt"
  DESTINATION share/${PROJECT_NAME})

# Also cover ros2 run, standalone viewers and component containers. Register
# the shell hook without a .dsv shortcut so re-sourcing moves the SDK to the
# front even when an older SDK has since been prepended to the environment.
if(APPLE)
  set(F1TENTH_DEPTHAI_LOADER_VARIABLE DYLD_LIBRARY_PATH)
else()
  set(F1TENTH_DEPTHAI_LOADER_VARIABLE LD_LIBRARY_PATH)
endif()
ament_environment_hooks("${CMAKE_CURRENT_LIST_DIR}/f1tenth_depthai.sh.in")
message(STATUS "[${PROJECT_NAME}] Workspace DepthAI config: ${depthai_DIR}")
message(STATUS "[${PROJECT_NAME}] Workspace DepthAI library: ${_f1tenth_depthai_library}")
