#!/usr/bin/env python3
"""IMU-only dead reckoning from a VIO run's initial state (no camera), for comparison with the VIO path.

  imu_only_deadreckon.py <asl_dir> <vio_traj.txt> --bg gx gy gz --ba ax ay az [--out plot.png]

Starts at the first VIO pose (orientation from the JPL quaternion, position 0, velocity 0) and
integrates gyro (orientation) and accel (velocity, position) with the given biases and g = 9.81.
"""
import argparse

import numpy as np


def expm(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def jpl_to_R_ItoG(q):
    x, y, z, w = q
    R_GtoI = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)],
                       [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
                       [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]])
    return R_GtoI.T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asl")
    ap.add_argument("traj")
    ap.add_argument("--bg", type=float, nargs=3, default=[0, 0, 0])
    ap.add_argument("--ba", type=float, nargs=3, default=[0, 0, 0])
    ap.add_argument("--out")
    ap.add_argument("--label", default="")
    args = ap.parse_args()
    imu = np.loadtxt(args.asl + "/imu0/data.csv", delimiter=",", comments="#")
    V = np.loadtxt(args.traj, comments="#")
    t0 = V[0, 0]
    ti = imu[:, 0] * 1e-9
    m = ti >= t0
    ti, w, a = ti[m], imu[m, 1:4] - args.bg, imu[m, 4:7] - args.ba
    R = jpl_to_R_ItoG(V[0, 4:8])
    g = np.array([0, 0, -9.81])
    p, v = np.zeros(3), np.zeros(3)
    P = [p.copy()]
    for k in range(len(ti) - 1):
        dt = ti[k + 1] - ti[k]
        acc = R @ a[k] + g
        p = p + v * dt + 0.5 * acc * dt * dt
        v = v + acc * dt
        R = R @ expm(w[k] * dt)
        P.append(p.copy())
    P = np.array(P)
    tt = ti - t0
    for s in [1, 2, 5, 10, 20, 30, 60, tt[-1]]:
        k = min(np.searchsorted(tt, s), len(tt) - 1)
        print(f"  t={tt[k]:6.1f} s: IMU-only position {np.round(P[k], 2)} m, |p| {np.linalg.norm(P[k]):9.2f} m")
    vd = V[:, 1:4] - V[0, 1:4]
    if args.out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(14, 6))
        ax[0].plot(vd[:, 0], vd[:, 1], label="VIO (camera + IMU)")
        n = np.searchsorted(tt, 15)
        ax[0].plot(P[:n, 0], P[:n, 1], label="IMU only, first 15 s")
        ax[0].set_aspect("equal"); ax[0].grid(alpha=.3); ax[0].legend(); ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
        ax[0].set_title("top view (IMU-only cut at 15 s)")
        ax[1].semilogy(tt[1:], np.linalg.norm(P[1:], axis=1), label="IMU only: distance from start")
        tv = V[:, 0] - t0
        ax[1].semilogy(tv[1:], np.linalg.norm(vd[1:], axis=1) + 1e-3, label="VIO: distance from start")
        ax[1].grid(alpha=.3, which="both"); ax[1].legend(); ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("[m] (log)")
        ax[1].set_title("IMU-only drift " + args.label)
        plt.tight_layout(); plt.savefig(args.out, dpi=110)
        print("->", args.out)


if __name__ == "__main__":
    main()
