#!/bin/sh
# Board: EuRoC V1_01 mono, euroc_reduced / euroc_default x 1 / 4 threads.
# Env: NAVLIB (board root, default ..), DATA (datasets, default $NAVLIB/data), RES (default $NAVLIB/res/bench)
B=$(cd "$(dirname "$0")/.." && pwd); NAVLIB=${NAVLIB:-$B}; DATA=${DATA:-$NAVLIB/data}; RES=${RES:-$NAVLIB/res/bench}
RUN=$NAVLIB/openvins/run.sh
for cfg in reduced default; do
  for thr in 1 4; do
    $RUN euroc_$cfg "$DATA/V1_01_easy" "$RES/V1_01_${cfg}_${thr}thr" num_opencv_threads=$thr
  done
done
