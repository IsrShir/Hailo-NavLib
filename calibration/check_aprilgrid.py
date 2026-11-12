#!/usr/bin/env python3
"""Count Kalibr AprilGrid (tag36h11) detections in an ASL cam0 folder and show image coverage.

  check_aprilgrid.py <asl_dir> [--step 5] [--out coverage.png]
"""
import argparse
import glob
import importlib.util
import os

import cv2
import numpy as np

# same detector as the lens calibration (handles Kalibr's 2-bit tag border and corner X-junctions)
_spec = importlib.util.spec_from_file_location("grid", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    "calib_fisheye_aprilgrid.py"))
grid = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(grid)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asl")
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--out")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.asl, "cam0/data/*.p[gn][gm]")))[:: args.step]
    found = {f: keep for f, keep, _ in grid.detect(files, min_tags=1)}
    shape = cv2.imread(files[0], cv2.IMREAD_GRAYSCALE).shape
    counts, pts, blur = [], [], []
    for f in files:
        keep = found.get(f, [])
        counts.append(len(keep))
        if keep:
            pts.append(np.concatenate([c for _, c in keep]))
            blur.append(cv2.Laplacian(cv2.imread(f, cv2.IMREAD_GRAYSCALE), cv2.CV_64F).var())
    counts = np.array(counts)
    h, w = shape
    print(f"{len(files)} frames checked (every {args.step}th): "
          f"target seen in {(counts > 0).sum()} ({100 * (counts > 0).mean():.0f}%), "
          f">=12 tags in {(counts >= 12).sum()}, all 24 in {(counts == 24).sum()}, "
          f"median tags when seen {np.median(counts[counts > 0]) if (counts > 0).any() else 0:.0f}")
    if pts:
        P = np.concatenate(pts)
        g = np.zeros((4, 6), int)
        for x, y in P:
            g[min(int(y / h * 4), 3), min(int(x / w * 6), 5)] += 1
        print("tag corners per image cell (4 rows x 6 cols; corners/edges matter most):")
        for r in g:
            print("   " + " ".join(f"{v:6d}" for v in r))
        print(f"sharpness (Laplacian var) on target frames: median {np.median(blur):.0f}, min {np.min(blur):.0f}")
        if args.out:
            canvas = np.full((h, w, 3), 255, np.uint8)
            for x, y in P:
                cv2.circle(canvas, (int(x), int(y)), 1, (200, 60, 0), -1)
            cv2.imwrite(args.out, canvas)
            print("coverage image ->", args.out)


if __name__ == "__main__":
    main()
