#!/bin/bash
# Laptop: install the OpenVINS runner and everything it needs on the board.
#   deploy.sh <runner binary (build dir)/vio_replay> [ssh target (default h15)]
# Board layout (NAVLIB_BOARD, default /home/root/navlib), same relative layout as the repo so the
# settings in openvins/config/ find calibration/hw_setups/ by relative path:
#   bin/vio_replay   lib/ (runtime libs the board image lacks)   openvins/{run.sh,config/}   calibration/hw_setups/
#   euroc/ (benchmark scripts)   data/ (put datasets here)   res/ (results)
# Env: H15L_SDK (default ~/h15l_sdk_v1.13/sdk), NAVLIB_BOARD, BIN_NAME (default vio_replay)
set -e
RUNNER=${1:?usage: $0 <vio_replay> [ssh target]}
T=${2:-h15}
H15L_SDK=${H15L_SDK:-$HOME/h15l_sdk_v1.13/sdk}
NAVLIB_BOARD=${NAVLIB_BOARD:-/home/root/navlib}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SYSROOT=$H15L_SDK/sysroots/armv8a-poky-linux
STAGE=$(mktemp -d)
mkdir -p "$STAGE/bin" "$STAGE/lib" "$STAGE/openvins" "$STAGE/calibration" "$STAGE/euroc"
. "$H15L_SDK/environment-setup-armv8a-poky-linux"
$STRIP -o "$STAGE/bin/${BIN_NAME:-vio_replay}" "$RUNNER"
# The H15L image has OpenCV and Boost, but not these libraries the runner links against.
for l in libopencv_video.so.405 libboost_filesystem.so.1.78.0 libboost_atomic.so.1.78.0; do
  cp -L "$SYSROOT/usr/lib/$l" "$STAGE/lib/"
done
cp -r "$ROOT/openvins/config" "$ROOT/openvins/run.sh" "$STAGE/openvins/"
cp -r "$ROOT/calibration/hw_setups" "$STAGE/calibration/"
cp "$ROOT"/euroc/board_*.sh "$STAGE/euroc/"
tar czf - -C "$STAGE" . | ssh "$T" "mkdir -p $NAVLIB_BOARD/data $NAVLIB_BOARD/res && tar xzf - -C $NAVLIB_BOARD &&
  rm -rf $NAVLIB_BOARD/sensors $NAVLIB_BOARD/bin/run_euroc_mono $NAVLIB_BOARD/analysis"  # older layouts
rm -r "$STAGE"
echo "deployed to $T:$NAVLIB_BOARD  - run: ssh $T $NAVLIB_BOARD/openvins/run.sh drone <dataset> <out dir>"
