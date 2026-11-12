#!/usr/bin/env python3
"""Convert a vio_rec recording (recorder/vio_rec.cpp) to an EuRoC ASL folder, and report timing.

  rec2asl.py <rec_dir> <asl_out> [--ts auto|v4l2|dqbuf] [--png]
  rec2asl.py --report-only <rec_dir>

The board does the same conversion with recorder/to_asl.sh (byte-identical output, images as symlinks);
use this one for the timing report or for PNG output (--png, for Kalibr).

Output layout (what runner/vio_replay.cpp and Kalibr's bagcreater read):
  <asl_out>/cam0/data.csv     #timestamp [ns],filename
  <asl_out>/cam0/data/<ns>.pgm|png
  <asl_out>/imu0/data.csv     #timestamp [ns],w_x,w_y,w_z [rad/s],a_x,a_y,a_z [m/s^2]
IMU rows are on gyro timestamps; accel is linearly interpolated onto them.
"""
import argparse
import os
import shutil
import sys

import numpy as np

V4L2_TS_MASK = 0xE000
V4L2_TS_MONOTONIC = 0x2000
V4L2_TSRC_MASK = 0x70000
V4L2_TSRC_SOE = 0x10000


def load_imu(path):
    scale = None
    with open(path) as f:
        for line in f:
            if line.startswith("# scale="):
                scale = float(line.split("=", 1)[1])
                break
    a = np.loadtxt(path, delimiter=",", comments="#", dtype=np.int64, ndmin=2)
    if scale is None or a.size == 0:
        sys.exit(f"{path}: empty or no scale header")
    a = a[np.argsort(a[:, 0], kind="stable")]
    a = a[np.concatenate(([True], np.diff(a[:, 0]) > 0))]  # drop duplicate stamps
    return a[:, 0], a[:, 1:4].astype(np.float64) * scale


def load_cam(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            seq, tv, tdq, flags, fn = line.strip().split(",")
            rows.append((int(seq), int(tv), int(tdq), int(flags, 16), fn))
    if not rows:
        sys.exit(f"{path}: no frames")
    return rows


def stats_ms(dt_ns):
    d = np.asarray(dt_ns, dtype=np.float64) / 1e6
    return (f"mean {d.mean():8.3f}  std {d.std():7.3f}  min {d.min():8.3f}  "
            f"p99 {np.percentile(d, 99):8.3f}  max {d.max():8.3f} ms")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("out", nargs="?")
    ap.add_argument("--report-only", action="store_true", help="print the timing report, write nothing")
    ap.add_argument("--ts", choices=["auto", "v4l2", "dqbuf"], default="auto",
                    help="camera timestamp source (auto: v4l2 if the driver says monotonic)")
    ap.add_argument("--png", action="store_true", help="write PNG instead of copying PGM (for Kalibr)")
    args = ap.parse_args()

    cam = load_cam(os.path.join(args.rec, "cam.csv"))
    tg, w = load_imu(os.path.join(args.rec, "gyro.csv"))
    ta, acc = load_imu(os.path.join(args.rec, "accel.csv"))

    seq = np.array([r[0] for r in cam])
    tv = np.array([r[1] for r in cam], dtype=np.int64)
    tdq = np.array([r[2] for r in cam], dtype=np.int64)
    flags = cam[0][3]
    ts_mono = (flags & V4L2_TS_MASK) == V4L2_TS_MONOTONIC
    src = "start-of-exposure" if (flags & V4L2_TSRC_MASK) == V4L2_TSRC_SOE else "end-of-frame"
    use = args.ts if args.ts != "auto" else ("v4l2" if ts_mono else "dqbuf")
    tc = tv if use == "v4l2" else tdq

    print(f"== camera: {len(cam)} frames, flags 0x{flags:x} "
          f"({'monotonic' if ts_mono else 'NOT monotonic'}, {src}); using {use} timestamps")
    gaps = np.diff(seq)
    print(f"  dropped frames (sequence gaps): {int((gaps - 1).clip(min=0).sum())}")
    print(f"  v4l2 interval   {stats_ms(np.diff(tv))}")
    print(f"  dqbuf interval  {stats_ms(np.diff(tdq))}")
    if ts_mono:
        print(f"  dqbuf - v4l2    {stats_ms(tdq - tv)}   <- std is the delivery jitter")
    print(f"== gyro:  {len(tg)} samples, interval {stats_ms(np.diff(tg))}")
    print(f"== accel: {len(ta)} samples, interval {stats_ms(np.diff(ta))}")
    j = np.searchsorted(ta, tg).clip(1, len(ta) - 1)
    near = np.minimum(np.abs(ta[j] - tg), np.abs(ta[j - 1] - tg))
    print(f"  |gyro - nearest accel| stamp: median {np.median(near) / 1e6:.3f} ms, max {near.max() / 1e6:.3f} ms")
    g_mean = np.linalg.norm(acc[: min(len(acc), 300)].mean(axis=0))
    print(f"  |accel| over first ~1.5 s: {g_mean:.3f} m/s^2 (expect ~9.81 if still)")
    print(f"== overlap: cam [{tc[0]}, {tc[-1]}], imu [{tg[0]}, {tg[-1]}] ns; "
          f"first frame {(tc[0] - tg[0]) / 1e6:+.1f} ms after first gyro")
    if tc[0] < tg[0] or abs(tc[0] - tg[0]) > 5e9:
        print("  WARN: camera and IMU stamps don't overlap sensibly - different clocks?")

    if args.report_only:
        return
    if not args.out:
        sys.exit("asl_out required (or --report-only)")
    # IMU on gyro stamps, inside the accel span
    keep = (tg >= ta[0]) & (tg <= ta[-1])
    tg, w = tg[keep], w[keep]
    a_i = np.column_stack([np.interp(tg, ta, acc[:, k]) for k in range(3)])

    os.makedirs(os.path.join(args.out, "cam0", "data"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "imu0"), exist_ok=True)
    with open(os.path.join(args.out, "imu0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for t, ww, aa in zip(tg, w, a_i):
            f.write(f"{t},{ww[0]:.9f},{ww[1]:.9f},{ww[2]:.9f},{aa[0]:.9f},{aa[1]:.9f},{aa[2]:.9f}\n")

    if args.png:
        import cv2
    ext = "png" if args.png else "pgm"
    with open(os.path.join(args.out, "cam0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],filename\n")
        last = -1
        for t, r in zip(tc, cam):
            if t <= last:
                continue
            last = t
            src_p = os.path.join(args.rec, "cam", r[4])
            dst = os.path.join(args.out, "cam0", "data", f"{t}.{ext}")
            if args.png:
                cv2.imwrite(dst, cv2.imread(src_p, cv2.IMREAD_GRAYSCALE))
            else:
                shutil.copyfile(src_p, dst)
            f.write(f"{t},{t}.{ext}\n")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
