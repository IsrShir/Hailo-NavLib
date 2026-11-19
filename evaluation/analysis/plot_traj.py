#!/usr/bin/env python3
"""Plot an OpenVINS trajectory (traj.txt from vio_replay) without ground truth.

  plot_traj.py <traj.txt> <out.png> [--title T] [--max-dist 15]

Left: 3D path (equal axes, coloured by time, start/end marked). Right: distance from
the start over time, with an optional reference line (e.g. the known furthest distance).
"""
import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("out")
    ap.add_argument("--title", default="")
    ap.add_argument("--max-dist", type=float, help="reference line on the distance plot [m]")
    args = ap.parse_args()

    T = np.loadtxt(args.traj, comments="#")
    t = T[:, 0] - T[0, 0]
    d = T[:, 1:4] - T[0, 1:4]
    dist = np.linalg.norm(d, axis=1)
    hdist = np.linalg.norm(d[:, :2], axis=1)
    L = np.linalg.norm(np.diff(d, axis=0), axis=1).sum()

    fig = plt.figure(figsize=(14, 6))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    sc = ax.scatter(d[:, 0], d[:, 1], d[:, 2], c=t, s=3, cmap="viridis")
    ax.plot(d[:, 0], d[:, 1], d[:, 2], color="0.6", lw=0.5)
    ax.scatter(*d[0], color="green", s=60, label="start")
    ax.scatter(*d[-1], color="red", marker="x", s=80, label="end")
    span = (d.max(0) - d.min(0)).max() / 2
    mid = (d.max(0) + d.min(0)) / 2
    ax.set_xlim(mid[0] - span, mid[0] + span)
    ax.set_ylim(mid[1] - span, mid[1] + span)
    ax.set_zlim(mid[2] - span, mid[2] + span)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend(loc="upper left")
    ax.set_title(args.title or args.traj)
    fig.colorbar(sc, ax=ax, shrink=0.6, label="t [s]")

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(t, dist, label="3D distance from start")
    ax2.plot(t, hdist, label="horizontal distance from start")
    ax2.plot(t, d[:, 2], label="height (z) from start")
    if args.max_dist:
        ax2.axhline(args.max_dist, color="k", ls="--", lw=1, label=f"stated max {args.max_dist:g} m")
    ax2.grid(alpha=0.3)
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("[m]")
    ax2.legend()
    ax2.set_title(f"path {L:.1f} m · max dist {dist.max():.2f} m · end-start {dist[-1]:.2f} m")
    plt.tight_layout()
    plt.savefig(args.out, dpi=110)
    print(f"path {L:.1f} m, max dist {dist.max():.2f} m (horiz {hdist.max():.2f}) at {t[dist.argmax()]:.1f} s, "
          f"end-start {dist[-1]:.2f} m, z range [{d[:, 2].min():.2f}, {d[:, 2].max():.2f}] m -> {args.out}")


if __name__ == "__main__":
    main()
