#!/bin/sh
# Board: run OpenVINS (vio_replay) on one EuRoC-format dataset with one settings file.
#   run.sh <settings> <dataset dir> <out dir> [key=value ...]
#     settings  name in openvins/config/ (e.g. drone, euroc_reduced) or a path to a .yaml
#     key=value overrides a top-level setting, e.g. num_opencv_threads=1 max_clones=10
# Writes <out>/estimator_config.yaml (the exact settings used, sensor paths rewritten relative to <out> so the
# calibration/hw_setups/ files are used in place, not copied), traj.txt, stats.csv, timing_*.txt, log.txt.
# Env: NAVLIB  board root (default: the folder above openvins/)
#      BIN     runner binary (default $NAVLIB/bin/vio_replay)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
NAVLIB=${NAVLIB:-$(dirname "$HERE")}
BIN=${BIN:-$NAVLIB/bin/vio_replay}
[ $# -ge 3 ] || { sed -n '2,6p' "$0"; exit 2; }
CFG=$1; DATA=$2; OUT=$3; shift 3
[ -f "$CFG" ] || CFG=$NAVLIB/openvins/config/$CFG.yaml
[ -f "$CFG" ] || { echo "settings not found: $1" >&2; exit 1; }
[ -f "$DATA/cam0/data.csv" ] && [ -f "$DATA/imu0/data.csv" ] || { echo "not a EuRoC ASL dataset: $DATA" >&2; exit 1; }
DATA=$(cd "$DATA" && pwd)
CFGDIR=$(cd "$(dirname "$CFG")" && pwd)
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
S=$OUT/estimator_config.yaml
cp "$CFG" "$S"
# relative_config_imu(cam) are relative to the settings file, and OpenVINS always prefixes that file's folder (even
# to absolute paths): rewrite them relative to $OUT, where the copy lives, so they still point at the calibration/hw_setups/ files
for k in relative_config_imu relative_config_imucam; do
  rel=$(sed -n "s/^$k: *\"\{0,1\}\([^\"]*\)\"\{0,1\}.*/\1/p" "$S")
  abs=$(cd "$CFGDIR/$(dirname "$rel")" && pwd)/$(basename "$rel")
  [ -f "$abs" ] || { echo "$k -> $abs not found" >&2; exit 1; }
  sed -i "s#^$k:.*#$k: \"$(realpath --relative-to="$OUT" "$abs")\"#" "$S"
done
for kv in "$@"; do
  k=${kv%%=*}; v=${kv#*=}
  grep -q "^$k:" "$S" || { echo "unknown setting: $k" >&2; exit 1; }
  sed -i "s#^$k:.*#$k: $v#" "$S"
done
export LD_LIBRARY_PATH=$NAVLIB/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
cd "$OUT"
TIME=""; [ -x /usr/bin/time ] && TIME="/usr/bin/time -v"
$TIME "$BIN" "$S" "$DATA" traj.txt stats.csv > log.txt 2>&1
echo "$(basename "$OUT"): $(grep RESULT log.txt | sed 's/.*RESULT/RESULT/')"
grep -q 'RESULT frames=[1-9]' log.txt || { echo "no frames processed - see $OUT/log.txt" >&2; exit 1; }
