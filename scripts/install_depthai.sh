#!/usr/bin/env bash
# Install the sibling checkout as the single C++ SDK for this ROS workspace.
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_dir="$(dirname -- "$workspace_dir")/depthai-core"
if [[ ! -f "$sdk_dir/CMakeLists.txt" ]]; then
  printf 'DepthAI source not found: %s\nPlace depthai-core beside %s.\n' \
    "$sdk_dir" "$workspace_dir" >&2
  exit 1
fi

mode="${1:-build}"
case "$mode" in
  build|--prepare-tools|--install-existing) ;;
  *) printf 'Usage: bash scripts/install_depthai.sh [--prepare-tools|--install-existing]\n' >&2; exit 2 ;;
esac

# Keep newer build tools private to this checkout; do not replace Ubuntu's
# CMake or install Python packages into the ROS interpreter.
if [[ "$mode" == --prepare-tools ]]; then
  python3 -m venv "$sdk_dir/.build-tools"
  "$sdk_dir/.build-tools/bin/python" -m pip install --only-binary=:all: \
    'cmake==3.30.1' 'ninja==1.11.1.4'
fi
if [[ -x "$sdk_dir/.build-tools/bin/cmake" ]]; then
  export PATH="$sdk_dir/.build-tools/bin:$PATH"
fi

# A finished SDK can be installed at the fixed prefix without reconfiguring
# its project or invoking vcpkg. This never clones/downloads SDK source.
if [[ "$mode" == --install-existing ]]; then
  if [[ ! -f "$sdk_dir/build/cmake_install.cmake" ]]; then
    printf 'No configured SDK install script in %s/build. Use --prepare-tools to build the existing source.\n' "$sdk_dir" >&2
    exit 1
  fi
  cmake --install "$sdk_dir/build" --prefix "$sdk_dir/install"
  printf '\nExisting SDK installed in %s\n' "$sdk_dir/install"
  exit 0
fi

if [[ "$(uname -s)" == Linux ]]; then
  # vcpkg's bundled tool extraction can fail on aarch64. Provide a supported
  # host CMake/Ninja instead of letting vcpkg download another copy.
  cmake_version="$(cmake --version)"
  if [[ ! "$cmake_version" =~ cmake\ version\ ([0-9]+)\.([0-9]+)\.([0-9]+) ]]; then
    printf 'Cannot determine CMake version.\n' >&2
    exit 1
  fi
  if (( BASH_REMATCH[1] < 3 ||
      (BASH_REMATCH[1] == 3 && BASH_REMATCH[2] < 30) ||
      (BASH_REMATCH[1] == 3 && BASH_REMATCH[2] == 30 && BASH_REMATCH[3] < 1) )); then
    printf 'CMake 3.30.1+ is required for this vcpkg build; found %s\nRun: bash scripts/install_depthai.sh --prepare-tools\nUbuntu prerequisite: sudo apt install python3-venv build-essential ninja-build curl zip unzip tar pkg-config\n' "${cmake_version%%$'\n'*}" >&2
    exit 1
  fi
  for build_tool in ninja make cc c++ git curl zip unzip tar pkg-config; do
    if ! command -v "$build_tool" >/dev/null 2>&1; then
      printf 'Missing build tool: %s\nInstall Ubuntu build dependencies listed in DEPTHAI_SETUP.md.\n' "$build_tool" >&2
      exit 1
    fi
  done
  unset VCPKG_FORCE_DOWNLOADED_BINARIES
  export VCPKG_FORCE_SYSTEM_BINARIES=1
fi
export VCPKG_MAX_CONCURRENCY="${VCPKG_MAX_CONCURRENCY:-${DEPTHAI_BUILD_JOBS:-2}}"

cmake -S "$sdk_dir" -B "$sdk_dir/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DDEPTHAI_OPENCV_SUPPORT=ON \
  -DCMAKE_INSTALL_PREFIX="$sdk_dir/install"
cmake --build "$sdk_dir/build" --parallel "${DEPTHAI_BUILD_JOBS:-2}"
cmake --install "$sdk_dir/build"
printf '\nWorkspace DepthAI SDK installed in %s\n' "$sdk_dir/install"
