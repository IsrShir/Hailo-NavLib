#!/bin/sh
# Board-side: record one camera + IMU session for OpenVINS.
#   ./record.sh <name> [seconds] [extra vio_rec args, e.g. -W 1920 -H 1080 -f 20 -k 3]
# Output: $REC_DIR/<name>/ (default: rec/ next to this folder), including <name>/asl/ - the EuRoC folder that
# VIO/openvins/run.sh reads (made by to_asl.sh at the end, images as symlinks). fetch.sh copies it to the laptop.
# Delayed start (e.g. to unplug Ethernet first), from the laptop:
#   ssh -n h15 '(cd /home/root/navlib/recorder && setsid nohup sh -c "sleep 10; ./record.sh walk3 120 -W 768 -H 432 -k 1") > /dev/null 2>&1 < /dev/null &'
# Hold the board still for the first ~3 s (OpenVINS static init) and the last ~2 s.
set -e
cd "$(dirname "$0")"
NAME=${1:?usage: $0 <name> [seconds] [vio_rec args]}
SECS=${2:-60}
[ $# -ge 2 ] && shift 2 || shift 1
OUT=${REC_DIR:-$(cd .. && pwd)/rec}/$NAME
[ -e "$OUT" ] && { echo "$OUT exists" >&2; exit 1; }

# isp_media_server always holds /dev/video0 on these images; anything else (drone-follow,
# "Hailo Tiling App", a stale vio_rec) means the camera is taken.
busy=""
for p in $(fuser /dev/video0 2>/dev/null); do
  c=$(cat /proc/$p/comm 2>/dev/null)
  case "$c" in isp_media_serv*) ;; *) busy="$busy $p($c)" ;; esac
done
[ -n "$busy" ] && { echo "/dev/video0 in use by:$busy - stop it first" >&2; exit 1; }

mkdir -p "$OUT"
./imu_setup.sh on > "$OUT/imu_setup_before.txt"
cat "$OUT/imu_setup_before.txt"
v4l2-ctl -d /dev/video0 --all > "$OUT/v4l2_all.txt" 2>&1 || true
trap './imu_setup.sh off > /dev/null' EXIT INT TERM

echo "recording $SECS s -> $OUT  (keep still for 3 s)"
./vio_rec -o "$OUT" -s "$SECS" "$@"
./imu_setup.sh show > "$OUT/imu_setup_after.txt"
./to_asl.sh "$OUT"
du -sh "$OUT"
