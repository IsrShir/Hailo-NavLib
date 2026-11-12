#!/bin/bash
# Laptop: build everything the board needs and install it on the board.
#   ./setup.sh [board]       board = ssh name (default h15); if it is not reachable, only the build runs
#   ./setup.sh --no-deploy   build only
# Steps (each one is skipped if already done):
#   1. Ceres for the board            openvins/build_ceres.sh
#   2. OpenVINS source                git submodule openvins/open_vins (official rpng/open_vins, pinned commit)
#   3. OpenVINS runner for the board  openvins/build_h15l.sh
#   4. recorder for the board         recorder/build.sh
#   5. install on the board           openvins/deploy.sh + recorder/deploy.sh
# Env: H15L_SDK (default ~/h15l_sdk_v1.13/sdk), CERES_PREFIX (default ~/h15l_sdk_v1.13/ceres_install),
#      NAVLIB_BUILD (default ~/navlib_build: build output)
set -e
T=${1:-h15}
ROOT=$(cd "$(dirname "$0")" && pwd)
export H15L_SDK=${H15L_SDK:-$HOME/h15l_sdk_v1.13/sdk}
CERES_PREFIX=${CERES_PREFIX:-$HOME/h15l_sdk_v1.13/ceres_install}
export CERES_DIR=$CERES_PREFIX/lib/cmake/Ceres
BUILD=${NAVLIB_BUILD:-$HOME/navlib_build}
step() { echo; echo "=== $*"; }

[ -f "$H15L_SDK/environment-setup-armv8a-poky-linux" ] || {
  echo "Hailo-15L SDK not found at $H15L_SDK (set H15L_SDK). Install the H15L Yocto SDK first." >&2; exit 1; }
mkdir -p "$BUILD"

step "1/5 Ceres -> $CERES_PREFIX"
if [ -f "$CERES_DIR/CeresConfig.cmake" ]; then echo "already built"; else "$ROOT/openvins/build_ceres.sh" "$CERES_PREFIX"; fi

step "2/5 OpenVINS source -> openvins/open_vins (submodule)"
git -C "$ROOT" submodule update --init openvins/open_vins
git -C "$ROOT/openvins/open_vins" log -1 --format="rpng/open_vins %h %ad" --date=short

step "3/5 OpenVINS runner -> $BUILD/build_h15l/vio_replay"
"$ROOT/openvins/build_h15l.sh" "$BUILD/build_h15l" > "$BUILD/build_h15l.log" 2>&1 \
  || { tail -20 "$BUILD/build_h15l.log"; exit 1; }
echo "built (log: $BUILD/build_h15l.log)"

step "4/5 recorder -> recorder/vio_rec"
"$ROOT/recorder/build.sh" > /dev/null && echo "built"

if [ "$T" = --no-deploy ]; then
  step "5/5 install on the board: skipped (--no-deploy)"
elif ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$T" true 2>/dev/null; then
  step "5/5 install on the board: skipped, '$T' not reachable (connect it and run ./setup.sh $T again)"
else
  step "5/5 install on $T"
  "$ROOT/openvins/deploy.sh" "$BUILD/build_h15l/vio_replay" "$T"
  "$ROOT/recorder/deploy.sh" "$T" > /dev/null && echo "recorder deployed"
fi
