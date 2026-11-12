#!/bin/bash
# Laptop: copy a recording from the board (for analysis or backup; running OpenVINS does not need this) and
# print its timing report (dropped frames, intervals, IMU rate, start stillness).
#   ./fetch.sh <name> [ssh target (default h15)]  -> $NAVLIB_DATA/<name>/ (raw + asl/)
# Env: NAVLIB_DATA (default ~/navlib_data), NAVLIB_BOARD (default /home/root/navlib)
set -e
N=${1:?usage: $0 <name> [ssh target]}
T=${2:-h15}
DATA=${NAVLIB_DATA:-$HOME/navlib_data}
B=${NAVLIB_BOARD:-/home/root/navlib}
mkdir -p "$DATA"
ssh "$T" "tar czf - -C $B/rec $N" | tar xzf - -C "$DATA"
[ -d "$DATA/$N/asl" ] || "$(dirname "$0")/to_asl.sh" "$DATA/$N"
python3 "$(dirname "$0")/../analysis/rec2asl.py" --report-only "$DATA/$N"
