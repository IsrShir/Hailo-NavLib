#!/bin/bash
# Laptop: cross-compile OpenVINS + the replay runner (VIO/openvins/runner) for the Hailo-15L (aarch64).
#   build_h15l.sh [build dir (default build_h15l)] [open_vins source (default: the VIO/openvins/open_vins submodule)]
# Env: H15L_SDK (default ~/h15l_sdk_v1.13/sdk), CERES_DIR (default ~/h15l_sdk_v1.13/ceres_install/lib/cmake/Ceres),
#      JOBS (default 4; more than ~4 can run WSL out of memory).
# Output: <build dir>/vio_replay (unstripped; deploy.sh strips it).
set -e
BUILD=${1:-build_h15l}
OV_SRC=${2:-$(cd "$(dirname "$0")" && pwd)/open_vins}
[ -f "$OV_SRC/ov_msckf/src/core/VioManager.h" ] || { echo "no OpenVINS source at $OV_SRC (git submodule update --init)" >&2; exit 1; }
H15L_SDK=${H15L_SDK:-$HOME/h15l_sdk_v1.13/sdk}
CERES_DIR=${CERES_DIR:-$HOME/h15l_sdk_v1.13/ceres_install/lib/cmake/Ceres}
RUNNER=$(cd "$(dirname "$0")/runner" && pwd)
# a build folder configured for another source path (e.g. before a repo move) cannot be reused by CMake: start it fresh
if [ -f "$BUILD/CMakeCache.txt" ] && ! grep -qx "CMAKE_HOME_DIRECTORY:INTERNAL=$RUNNER" "$BUILD/CMakeCache.txt"; then
  echo "build folder $BUILD was configured for another source path - starting it fresh"
  rm -rf "${BUILD:?}"
fi
. "$H15L_SDK/environment-setup-armv8a-poky-linux"
cmake -S "$RUNNER" -B "$BUILD" -DOV_SRC="$(cd "$OV_SRC" && pwd)" -DOV_OPENCV_MANUAL=ON \
  -DCeres_DIR="$CERES_DIR" \
  "-DCMAKE_CXX_FLAGS_RELEASE=-O3 -g -DNDEBUG -mcpu=cortex-a53 -fno-signed-zeros -fno-math-errno -funroll-loops"
make -C "$BUILD" -j"${JOBS:-4}" vio_replay
echo "built $BUILD/vio_replay"
