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

cmake -S "$sdk_dir" -B "$sdk_dir/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DDEPTHAI_OPENCV_SUPPORT=ON \
  -DCMAKE_INSTALL_PREFIX="$sdk_dir/install"
cmake --build "$sdk_dir/build" --parallel "${DEPTHAI_BUILD_JOBS:-2}"
cmake --install "$sdk_dir/build"
printf '\nWorkspace DepthAI SDK installed in %s\n' "$sdk_dir/install"
