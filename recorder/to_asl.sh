#!/bin/sh
# Board (or laptop): turn a vio_rec recording into the EuRoC ASL folder the OpenVINS runner reads, in place:
#   to_asl.sh <recording dir> [--dqbuf]   -> <recording dir>/asl/{cam0/data.csv, cam0/data/<ns>.pgm, imu0/data.csv}
# Images are symlinks to <recording dir>/cam/ (nothing is copied). IMU rows are on gyro timestamps with the
# accelerometer linearly interpolated onto them, in rad/s and m/s^2 - the same output as analysis/rec2asl.py
# (which also prints a timing report). Camera timestamps: V4L2 (default) or the dequeue time (--dqbuf).
set -e
REC=$(cd "${1:?usage: $0 <recording dir> [--dqbuf]}" && pwd)
TSCOL=2; [ "$2" = --dqbuf ] && TSCOL=3
OUT=$REC/asl
[ -e "$OUT" ] && { echo "$OUT exists" >&2; exit 1; }
mkdir -p "$OUT/cam0/data" "$OUT/imu0"

# sorted, duplicate stamps dropped: "t x y z" (raw), scale from the "# scale=" header
imu_sorted() { grep -v '^#' "$1" | sort -t, -k1,1n -s | awk -F, '$1 != last { print $1, $2, $3, $4; last = $1 }'; }
GS=$(sed -n 's/^# scale=//p' "$REC/gyro.csv"); AS=$(sed -n 's/^# scale=//p' "$REC/accel.csv")
imu_sorted "$REC/accel.csv" > "$OUT/imu0/.accel"
imu_sorted "$REC/gyro.csv" | awk -v gs="$GS" -v as="$AS" -v af="$OUT/imu0/.accel" '
  BEGIN {
    n = 0; while ((getline line < af) > 0) { split(line, f, " "); n++; T[n] = f[1]; X[n] = f[2] * as; Y[n] = f[3] * as; Z[n] = f[4] * as }
    print "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
    j = 1
  }
  {
    t = $1 + 0
    if (t < T[1] || t > T[n]) next                    # only inside the accel span
    while (j < n - 1 && T[j + 1] <= t) j++            # T[j] <= t < T[j+1] (or the last pair)
    if (t <= T[j]) { ax = X[j]; ay = Y[j]; az = Z[j] }
    else if (t >= T[j + 1]) { ax = X[j + 1]; ay = Y[j + 1]; az = Z[j + 1] }
    else { r = (t - T[j]) / (T[j + 1] - T[j]); ax = X[j] + r * (X[j + 1] - X[j]); ay = Y[j] + r * (Y[j + 1] - Y[j]); az = Z[j] + r * (Z[j + 1] - Z[j]) }
    printf "%s,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", $1, $2 * gs, $3 * gs, $4 * gs, ax, ay, az
  }' > "$OUT/imu0/data.csv"
rm "$OUT/imu0/.accel"

# camera: <ts>.pgm -> ../../../cam/<file>, strictly increasing timestamps
echo "#timestamp [ns],filename" > "$OUT/cam0/data.csv"
grep -v '^#' "$REC/cam.csv" | awk -F, -v c="$TSCOL" '$c + 0 > last { print $c, $5; last = $c + 0 }' |
  while read -r t f; do
    ln -s "../../../cam/$f" "$OUT/cam0/data/$t.pgm"
    echo "$t,$t.pgm" >> "$OUT/cam0/data.csv"
  done
echo "$OUT: $(($(wc -l < "$OUT/cam0/data.csv") - 1)) frames, $(($(wc -l < "$OUT/imu0/data.csv") - 1)) IMU rows"
