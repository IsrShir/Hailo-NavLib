#!/bin/sh
# LSM6DSR self-test (accel + gyro) via the st_lsm6dsx sysfs "selftest" attribute.
# Keep the board still. Restores ODR, full scale and selftest mode afterwards.
# Pass criteria (LSM6DSR datasheet): accel |delta| 40..1700 mg at +-4 g,
# gyro |delta| 150..700 dps at +-2000 dps; sign follows the push (+ for positive-sign).
set -e
find_dev() {
  for d in /sys/bus/iio/devices/iio:device*; do
    case "$(cat "$d/name")" in *_"$1") echo "$d"; return 0 ;; esac
  done
  echo "no IIO device *_$1" >&2; exit 1
}
N=${N:-20}

# $1 dev  $2 channel prefix (in_accel|in_anglvel)  $3 odr  $4 scale  $5 unit mult  $6 unit  $7 lo  $8 hi
run() {
  d=$1; ch=$2
  [ "$(cat "$d/buffer/enable")" = 1 ] && { echo "$(cat "$d/name"): buffer in use, skipping" >&2; return 1; }
  o_odr=$(cat "$d/sampling_frequency"); o_sc=$(cat "$d/${ch}_scale"); o_st=$(cat "$d/selftest")
  echo "$3" > "$d/sampling_frequency"; echo "$4" > "$d/${ch}_scale"; sc=$(cat "$d/${ch}_scale")
  avg() {
    sleep 0.3; x=0; y=0; z=0
    for i in $(seq "$N"); do
      x=$((x + $(cat "$d/${ch}_x_raw"))); y=$((y + $(cat "$d/${ch}_y_raw"))); z=$((z + $(cat "$d/${ch}_z_raw")))
    done
    echo "$((x / N)) $((y / N)) $((z / N))"
  }
  echo normal-mode > "$d/selftest"; n1=$(avg)
  echo positive-sign > "$d/selftest"; p=$(avg)
  echo normal-mode > "$d/selftest"; n2=$(avg)
  echo negative-sign > "$d/selftest"; m=$(avg)
  echo normal-mode > "$d/selftest"; n3=$(avg)
  echo "$o_st" > "$d/selftest"; echo "$o_sc" > "$d/${ch}_scale"; echo "$o_odr" > "$d/sampling_frequency"
  echo "== $(cat "$d/name")  (scale $sc, pass |delta| $7..$8 $6)"
  echo "$n1 $p $n2 $m $n3" | awk -v sc="$sc" -v k="$5" -v u="$6" -v lo="$7" -v hi="$8" '{
    split("x y z", ax, " "); fail = 0
    for (a = 1; a <= 3; a++) {
      n = ($a + $(a+6) + $(a+12)) / 3
      dp = ($(a+3) - n) * sc * k; dm = ($(a+9) - n) * sc * k
      okp = (dp >= lo && dp <= hi); okm = (-dm >= lo && -dm <= hi)
      if (!okp || !okm) fail = 1
      printf "  %s: +push %+8.1f %s %s   -push %+8.1f %s %s   (normal drift %d LSB)\n", ax[a], dp, u, okp ? "ok" : "FAIL", dm, u, okm ? "ok" : "FAIL", $(a+12) - $a
    }
    print fail ? "  RESULT: FAIL" : "  RESULT: PASS"
  }'
}

# Gyro: one-shot sysfs reads power the gyro up per read and it has not settled (~70 ms
# turn-on), which gives bogus deltas. Instead record the buffered stream with vio_rec
# while switching modes, and average each mode window (skipping 0.6 s after a switch).
run_gyro_buffered() {
  d=$1; here=$(dirname "$0"); tmp=/var/tmp/imu_selftest_gyro
  [ "$(cat "$d/buffer/enable")" = 1 ] && { echo "$(cat "$d/name"): buffer in use, skipping" >&2; return 1; }
  o_odr=$(cat "$d/sampling_frequency"); o_sc=$(cat "$d/in_anglvel_scale"); o_st=$(cat "$d/selftest")
  GS=$(tr ' ' '\n' < "$d/in_anglvel_scale_available" | sort -g | tail -1)
  ODR=208 GYRO_SCALE=$GS "$here/imu_setup.sh" on > /dev/null 2>&1
  rm -rf "$tmp"; "$here/vio_rec" -o "$tmp" -s 11 --no-cam > /dev/null & P=$!
  sleep 1; : > "$tmp.modes"
  for m in normal-mode positive-sign normal-mode negative-sign normal-mode; do
    echo "$(cut -d' ' -f1 /proc/uptime) $m" >> "$tmp.modes"; echo $m > "$d/selftest"; sleep 2
  done
  echo "$(cut -d' ' -f1 /proc/uptime) end" >> "$tmp.modes"
  wait $P
  "$here/imu_setup.sh" off > /dev/null 2>&1
  echo "$o_st" > "$d/selftest"; echo "$o_sc" > "$d/in_anglvel_scale"
  for x in /sys/bus/iio/devices/iio:device*; do echo "$o_odr" > "$x/sampling_frequency"; done
  echo "== $(cat "$d/name")  (scale $GS, pass |delta| 150..700 dps, buffered)"
  awk -F'[, ]' '
    FNR == NR { t[NR] = $1; m[NR] = $2; nm = NR; next }
    /^# scale=/ { split($0, s, "="); sc = s[2] * 57.29578; next }
    /^#/ { next }
    { ts = $1 / 1e9
      for (k = 1; k < nm; k++) if (ts > t[k] + 0.6 && ts < t[k+1] - 0.1) {
        n[k]++; for (a = 1; a <= 3; a++) sum[k, a] += $(a+1) } }
    END {
      split("x y z", ax, " "); fail = 0
      for (a = 1; a <= 3; a++) {
        for (k = 1; k <= 5; k++) mu[k] = n[k] ? sum[k, a] / n[k] * sc : 0
        base = (mu[1] + mu[3] + mu[5]) / 3; dp = mu[2] - base; dm = mu[4] - base
        okp = (dp >= 150 && dp <= 700); okm = (-dm >= 150 && -dm <= 700)
        if (!okp || !okm || !n[2] || !n[4]) fail = 1
        printf "  %s: +push %+8.1f dps %s   -push %+8.1f dps %s   (normal drift %+.2f dps)\n", ax[a], dp, okp ? "ok" : "FAIL", dm, okm ? "ok" : "FAIL", mu[5] - mu[1]
      }
      print fail ? "  RESULT: FAIL" : "  RESULT: PASS"
    }' "$tmp.modes" "$tmp/gyro.csv"
}

ACC=$(find_dev accel); GYR=$(find_dev gyro)
# accel: m/s^2 per LSB -> mg ; +-4 g = 0.001196411 (one-shot reads are fine for the accel)
run "$ACC" in_accel 52 0.001196411 101.97162 mg 40 1700
run_gyro_buffered "$GYR"
