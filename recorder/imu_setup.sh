#!/bin/sh
# Configure the LSM6DSR (st_lsm6dsx) IIO buffers for VIO recording on the H15L.
#   imu_setup.sh on     # ODR, full scale, monotonic timestamps, scan elements, buffer on
#   imu_setup.sh off    # disable buffers (back to sysfs polling)
#   imu_setup.sh show   # print current state only
# Env overrides: ODR (Hz, default 208), GYRO_SCALE / ACCEL_SCALE (a value from
# in_*_scale_available; default: leave as is), BUF_LEN (samples, 512), WATERMARK (samples, 8).
set -e
ODR=${ODR:-208}
BUF_LEN=${BUF_LEN:-512}
WATERMARK=${WATERMARK:-8}

find_dev() {  # $1 = gyro|accel
  for d in /sys/bus/iio/devices/iio:device*; do
    case "$(cat "$d/name")" in *_"$1") echo "$d"; return 0 ;; esac
  done
  echo "no IIO device named *_$1" >&2
  exit 1
}
GYRO=$(find_dev gyro)
ACCEL=$(find_dev accel)

w() {  # write if the attribute exists, report what stuck
  if [ -e "$1" ]; then
    echo "$2" > "$1" 2>/dev/null || echo "  WARN: could not write $2 to $1" >&2
  else
    echo "  (no $1)" >&2
  fi
}

rd() { tr -d '\0' < "$1" 2>/dev/null; }   # current_timestamp_clock ends with a NUL byte
show() {
  for d in "$GYRO" "$ACCEL"; do
    echo "== $(cat "$d/name") ($d)"
    echo "  sampling_frequency:   $(cat "$d/sampling_frequency" 2>/dev/null)"
    for s in "$d"/in_*_scale; do echo "  $(basename "$s"): $(cat "$s")"; done
    echo "  timestamp_clock:      $(rd "$d/current_timestamp_clock")"
    echo "  buffer enable/length/watermark: $(cat "$d/buffer/enable" 2>/dev/null)/$(cat "$d/buffer/length" 2>/dev/null)/$(cat "$d/buffer/watermark" 2>/dev/null)"
    printf "  scan elements on:     "
    for e in "$d"/scan_elements/*_en; do [ "$(cat "$e")" = 1 ] && printf "%s " "$(basename "$e" _en)"; done
    echo
  done
  irq=$(awk '/lsm6dsx/ {print $1}' /proc/interrupts | tr -d :)
  [ -n "$irq" ] && echo "== IRQ $irq count: $(awk -v i="$irq:" '$1==i {s=0; for (k=2;k<=5;k++) s+=$k; print s}' /proc/interrupts)"
}

off() {
  for d in "$GYRO" "$ACCEL"; do w "$d/buffer/enable" 0; done
}

on() {
  off
  for d in "$GYRO" "$ACCEL"; do
    w "$d/sampling_frequency" "$ODR"
    # Must be set before the buffer is enabled: the driver anchors its hardware
    # FIFO timestamps to this clock. Default is realtime, which can jump.
    w "$d/current_timestamp_clock" monotonic
    for e in "$d"/scan_elements/*_en; do w "$e" 1; done
    w "$d/buffer/length" "$BUF_LEN"
    w "$d/buffer/watermark" "$WATERMARK"
  done
  [ -n "$GYRO_SCALE" ] && w "$GYRO/in_anglvel_scale" "$GYRO_SCALE"
  [ -n "$ACCEL_SCALE" ] && w "$ACCEL/in_accel_scale" "$ACCEL_SCALE"
  for d in "$GYRO" "$ACCEL"; do w "$d/buffer/enable" 1; done
}

case "${1:-show}" in
  on) on; show ;;
  off) off; show ;;
  show) show ;;
  *) echo "usage: $0 on|off|show" >&2; exit 2 ;;
esac
