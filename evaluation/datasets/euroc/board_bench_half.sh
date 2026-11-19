#!/bin/sh
# Board: EuRoC V1_01 mono at half resolution (downsample_cameras: 376x240).
B=$(cd "$(dirname "$0")/../../.." && pwd); NAVLIB=${NAVLIB:-$B}; DATA=${DATA:-$NAVLIB/data}; RES=${RES:-$NAVLIB/res/bench}
RUN=$NAVLIB/VIO/openvins/run.sh
for run in "reduced 1" "reduced 4" "default 4"; do
  set -- $run
  $RUN euroc_$1 "$DATA/V1_01_easy" "$RES/V1_01_$1_half_$2thr" num_opencv_threads=$2 downsample_cameras=true
done
