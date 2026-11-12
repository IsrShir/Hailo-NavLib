#!/bin/bash
# Laptop: copy the recorder to the board.
#   ./deploy.sh [ssh target (default h15)]
# Env: NAVLIB_BOARD (default /home/root/navlib) -> recorder in $NAVLIB_BOARD/recorder, recordings in $NAVLIB_BOARD/rec
set -e
cd "$(dirname "$0")"
T=${1:-h15}
B=${NAVLIB_BOARD:-/home/root/navlib}
[ -x vio_rec ] || ./build.sh
tar czf - vio_rec imu_setup.sh record.sh imu_selftest.sh to_asl.sh | ssh "$T" "mkdir -p $B/recorder $B/rec && tar xzf - -C $B/recorder && ls -l $B/recorder"
