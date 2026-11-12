#!/bin/sh
# Board: one-setting-at-a-time ablation from euroc_default on V1_01 mono, 1 thread.
# Report: analysis/timing_report.py $RES  (time per stage; for time inside tracking use a profiler, e.g. perf)
B=$(cd "$(dirname "$0")/.." && pwd); NAVLIB=${NAVLIB:-$B}; DATA=${DATA:-$NAVLIB/data}; RES=${RES:-$NAVLIB/res/ablation}
run() { tag=$1; shift; $NAVLIB/openvins/run.sh euroc_default "$DATA/V1_01_easy" "$RES/$tag" num_opencv_threads=1 "$@"; }
run A_default
run B_half_res     downsample_cameras=true
run C_slam25       max_slam=25
run D_slam0        max_slam=0
run E_pts100       num_pts=100
run F_clones7      max_clones=7
run G_reduced      num_pts=100 max_clones=7 max_slam=25
run H_reduced_half num_pts=100 max_clones=7 max_slam=25 downsample_cameras=true
run I_default_cap20 max_msckf_in_update=20
run J_default_cap10 max_msckf_in_update=10
