#!/bin/sh
# Board: lower camera rates by dropping frames (15 Hz: every 4th, 10 Hz: every 2nd) on V1_01 and MH_05.
# Creates <seq>_<hz>hz datasets next to the originals (frame list only; images and IMU are linked).
B=$(cd "$(dirname "$0")/.." && pwd); NAVLIB=${NAVLIB:-$B}; DATA=${DATA:-$NAVLIB/data}; RES=${RES:-$NAVLIB/res/bench}
RUN=$NAVLIB/openvins/run.sh
for seq in V1_01_easy MH_05_difficult; do
  D=$DATA/$seq
  for hz in 15 10; do
    R=$DATA/${seq}_${hz}hz; mkdir -p "$R/cam0"; ln -sfn "$D/imu0" "$R/imu0"; ln -sfn "$D/cam0/data" "$R/cam0/data"
    awk -v hz=$hz 'NR==1 {print; next} {i++; if ((hz==15) ? (i%4!=0) : (i%2==1)) print}' "$D/cam0/data.csv" > "$R/cam0/data.csv"
    for cfg in reduced default; do
      for thr in 1 4; do
        $RUN euroc_$cfg "$R" "$RES/${seq}_${cfg}_${hz}hz_${thr}thr" num_opencv_threads=$thr
      done
    done
  done
done
