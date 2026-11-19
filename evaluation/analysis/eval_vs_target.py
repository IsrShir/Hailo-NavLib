#!/usr/bin/env python3
"""Compare an OpenVINS trajectory with target-frame camera poses (cam_imu_targetcal.py --save-poses).

  eval_vs_target.py <traj.txt> <gt_poses.txt> [--out plot.png] [--title T]

VIO positions (IMU, translation-only extrinsic assumed ~0) are matched to GT frames by timestamp and
aligned with a rigid SE3 (Umeyama, no scale). Reports ATE and the scale a sim3 fit would need.
"""
import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def umeyama(X, Y, scale):
    """s, R, t minimising |Y - (s R X + t)|."""
    mx, my = X.mean(0), Y.mean(0)
    Xc, Yc = X - mx, Y - my
    U, S, Vt = np.linalg.svd(Yc.T @ Xc / len(X))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / Xc.var(0).sum() if scale else 1.0
    return s, R, my - s * R @ mx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("gt")
    ap.add_argument("--out")
    ap.add_argument("--title", default="")
    args = ap.parse_args()
    V = np.loadtxt(args.traj, comments="#")
    G = np.loadtxt(args.gt, comments="#")
    _, ui = np.unique(V[:, 0], return_index=True)
    V = V[ui]
    idx = np.searchsorted(V[:, 0], G[:, 0])
    ok = (idx < len(V)) & (np.abs(V[np.clip(idx, 0, len(V) - 1), 0] - G[:, 0]) < 0.005)
    X, Y, t = V[idx[ok], 1:4], G[ok, 1:4], G[ok, 0] - G[0, 0]
    _, R, tr = umeyama(X, Y, False)
    E = Y - (X @ R.T + tr)
    s, *_ = umeyama(X, Y, True)
    e = np.linalg.norm(E, axis=1)
    path = np.linalg.norm(np.diff(Y, axis=0), axis=1).sum()
    print(f"{ok.sum()} matched frames over {t[-1]:.0f} s, GT path {path:.2f} m, extent {np.ptp(Y, 0).round(2)} m")
    print(f"ATE (SE3) rmse {np.sqrt((e ** 2).mean()) * 100:.1f} cm, median {np.median(e) * 100:.1f} cm, "
          f"max {e.max() * 100:.1f} cm; sim3 scale {s:.3f}")
    if args.out:
        A = X @ R.T + tr
        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        for k, n in enumerate("xyz"):
            ax[0].plot(t, Y[:, k], "-", lw=1.5, label=f"target {n}")
            ax[0].plot(t, A[:, k], "--", lw=1, label=f"VIO {n}")
        ax[0].set_xlabel("t [s]"); ax[0].set_ylabel("[m]"); ax[0].legend(ncol=2, fontsize=8); ax[0].grid(alpha=0.3)
        ax[0].set_title(args.title or args.traj)
        ax[1].plot(t, e * 100)
        ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("position error [cm]"); ax[1].grid(alpha=0.3)
        ax[1].set_title(f"ATE rmse {np.sqrt((e ** 2).mean()) * 100:.1f} cm (path {path:.1f} m)")
        plt.tight_layout(); plt.savefig(args.out, dpi=110)
        print("->", args.out)


if __name__ == "__main__":
    main()
