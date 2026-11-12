#!/usr/bin/env python3
"""Camera-IMU extrinsics + time offset from a Kalibr AprilGrid recording (a stopgap until Kalibr).

  cam_imu_targetcal.py <asl_dir> --lens lens.yaml [--win 1.0] [--min-tags 8]

1. Camera pose per frame: tags (calib_fisheye_aprilgrid.detect) + cv2.fisheye.solvePnP with the
   lens model (its 1920x1080 intrinsics, scaled to the recording's resolution).
2. Rotation R_ItoC and time shift: camera rotation between consecutive target frames vs the gyro
   rotation over the same (shifted) interval, Kabsch on rotation vectors, shift grid-searched.
3. Lever arm, gravity and accel bias: linear least squares. In windows of --win seconds the IMU
   position from double-integrated accel (rotated by the vision+gyro orientation) must match the
   vision position of the IMU (camera position + R_TC * lever arm). Per-window p0, v0 are free.

Conventions in the printed result:
  R_ItoC   maps IMU-frame vectors into the camera frame; R_CtoI = R_ItoC^T
  p_IinC   IMU origin in the camera frame;  p_CinI = -R_CtoI p_IinC (camera origin in IMU frame)
  T_imu_cam (OpenVINS kalibr_imucam_chain) = [R_CtoI p_CinI]
  shift    t_imu = t_cam + shift  -> timeshift_cam_imu
"""
import argparse
import glob
import importlib.util
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("grid", os.path.join(HERE, "calib_fisheye_aprilgrid.py"))
grid = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(grid)


def expm(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def kabsch(A, B):
    """R minimising sum |B_k - R A_k|^2."""
    U, _, Vt = np.linalg.svd(A.T @ B)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T


def load_lens(path, w, h):
    v = {}
    for line in open(path):
        if ":" in line and not line.startswith("#"):
            k, s = line.split(":", 1)
            v[k.strip()] = s.strip()
    num = lambda s: np.array([float(x) for x in s.strip("[]").split(",")])
    fx, fy, cx, cy = num(v["intrinsics_1920x1080"])
    s = w / 1920
    K = np.array([[fx * s, 0, (cx + 0.5) * s - 0.5], [0, fy * h / 1080, (cy + 0.5) * h / 1080 - 0.5], [0, 0, 1]])
    return K, num(v["distortion_coeffs"])


class Gyro:
    """Cumulative orientation R_{I0 I(t)} with in-sample interpolation."""

    def __init__(self, t, w):
        self.t, self.w = t, w
        R = [np.eye(3)]
        for i in range(len(t) - 1):
            R.append(R[-1] @ expm(w[i] * (t[i + 1] - t[i])))
        self.R = np.array(R)

    def at(self, t):
        i = int(np.clip(np.searchsorted(self.t, t) - 1, 0, len(self.t) - 2))
        return self.R[i] @ expm(self.w[i] * (t - self.t[i]))

    def rel(self, t1, t2):
        return self.at(t1).T @ self.at(t2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asl")
    ap.add_argument("--lens", required=True)
    ap.add_argument("--win", type=float, default=1.0, help="window length for the lever-arm fit [s]")
    ap.add_argument("--min-tags", type=int, default=8)
    ap.add_argument("--save-poses", help="write target-frame camera poses: t[s] px py pz qx qy qz qw (Hamilton, R_TC)")
    ap.add_argument("--span", default="0.15,0.6", help="time span of a rotation pair [s] min,max")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.asl, "cam0/data/*.p[gn][gm]")))
    dets = [d for d in grid.detect(files) if len(d[1]) >= args.min_tags]
    h, w = dets[0][2]
    K, D = load_lens(args.lens, w, h)
    imu = np.loadtxt(os.path.join(args.asl, "imu0/data.csv"), delimiter=",", comments="#")
    t0 = imu[0, 0]
    ti, wi, ai = (imu[:, 0] - t0) * 1e-9, imu[:, 1:4], imu[:, 4:7]
    gy = Gyro(ti, wi)

    # 1. camera poses in the target frame
    tc, R_TC, p_TC, rep = [], [], [], []
    for f, keep, _ in dets:
        o = np.concatenate([np.c_[grid.tag_obj(i), np.zeros(4)] for i, _ in keep]).reshape(1, -1, 3)
        p = np.concatenate([c for _, c in keep]).reshape(1, -1, 2)
        ok, rv, tv = cv2.fisheye.solvePnP(o, p, K, D)
        if not ok:
            continue
        pr, _ = cv2.fisheye.projectPoints(o, rv, tv, K, D)
        e = np.sqrt(np.mean(np.sum((pr - p) ** 2, axis=2)))
        if e > 3:
            continue
        R = cv2.Rodrigues(rv)[0]
        tc.append((int(os.path.basename(f)[:-4]) - t0) * 1e-9)
        R_TC.append(R.T)
        p_TC.append(-R.T @ tv.ravel())
        rep.append(e)
    tc, R_TC, p_TC = np.array(tc), np.array(R_TC), np.array(p_TC)
    if args.save_poses:
        with open(args.save_poses, "w") as fo:
            fo.write("# t[s, camera clock] p_TC xyz [m] q_TC xyzw (Hamilton)\n")
            for t_, R_, p_ in zip(tc, R_TC, p_TC):
                rv = cv2.Rodrigues(R_)[0][:, 0]
                th = np.linalg.norm(rv)
                q = np.r_[np.sin(th / 2) * rv / max(th, 1e-12), np.cos(th / 2)]
                fo.write(f"{t_ + t0 * 1e-9:.6f} " + " ".join(f"{v:.6f}" for v in np.r_[p_, q]) + "\n")
    print(f"{len(files)} frames {w}x{h}, {len(tc)} target poses (PnP RMS median {np.median(rep):.2f} px), "
          f"camera-target distance median {np.median(np.linalg.norm(p_TC, axis=1)):.2f} m")

    # 2. rotation + time shift from consecutive pose pairs
    # pairs a few frames apart: larger rotations, so pose noise matters less (but < ~60 deg)
    lo, hi = map(float, args.span.split(","))
    pairs = [(i, j) for i in range(len(tc)) for j in range(i + 1, len(tc)) if lo <= tc[j] - tc[i] <= hi]
    B = np.array([cv2.Rodrigues(R_TC[i].T @ R_TC[j])[0][:, 0] for i, j in pairs])
    keep = (np.linalg.norm(B, axis=1) > np.radians(3)) & (np.linalg.norm(B, axis=1) < np.radians(60))
    pairs = [p for p, k in zip(pairs, keep) if k]
    B = B[keep]
    inl = np.ones(len(B), bool)

    def fit_rot(shift, mask=None):
        m = inl if mask is None else mask
        A = np.array([cv2.Rodrigues(gy.rel(tc[i] + shift, tc[j] + shift))[0][:, 0] for i, j in pairs])
        R = kabsch(A[m], B[m])
        r = np.linalg.norm(B - A @ R.T, axis=1)
        return R, np.degrees(np.median(r[m])), A, r

    shifts = np.arange(-0.08, 0.0801, 0.002)
    for it in range(3):  # grid search, then drop outlier pairs (> 3x median) and search again
        s = shifts[int(np.argmin([fit_rot(x)[1] for x in shifts]))]
        fine = np.arange(s - 0.003, s + 0.0031, 0.0002)
        shift = fine[int(np.argmin([fit_rot(x)[1] for x in fine]))]
        R_ItoC, med, A, r = fit_rot(shift)
        inl = r < 3 * np.radians(med)
    R_ItoC, med, A, r = fit_rot(shift)
    nA, nB = np.linalg.norm(A[inl], axis=1), np.linalg.norm(B[inl], axis=1)
    print(f"rotation: {inl.sum()}/{len(pairs)} pairs ({np.degrees(np.median(nB)):.0f} deg median), shift "
          f"{shift * 1000:+.1f} ms, median residual {med:.3f} deg, |cam|/|gyro| {nB.sum() / nA.sum():.4f}")

    # 3. lever arm, gravity, accel bias. Unknowns: c=p_IinC (3), g_T (3), b (3), per window p0, v0.
    R_CtoI = R_ItoC.T
    rows, rhs, nwin = [], [], 0
    k = 0
    blocks = []
    while k < len(tc):
        kw = [j for j in range(k, len(tc)) if tc[j] - tc[k] <= args.win]
        if len(kw) >= 6:
            blocks.append(kw)
        k = kw[-1] + 1
    nb = len(blocks)
    for b_i, kw in enumerate(blocks):
        ts = tc[kw[0]] + shift  # window start on the IMU clock
        te = tc[kw[-1]] + shift
        m = (ti >= ts) & (ti <= te)
        tt = np.r_[ts, ti[m], te]
        R0 = R_TC[kw[0]] @ R_ItoC  # R_TI at window start (vision), propagated with the gyro
        RT = np.array([R0 @ gy.rel(ts, x) for x in tt])
        aa = np.array([np.interp(x, ti, ai[:, j]) for j in range(3) for x in tt]).reshape(3, -1).T
        fa = np.einsum("nij,nj->ni", RT, aa)
        dt = np.diff(tt)
        # cumulative double integrals (trapezoid) of R a and of R (for the bias term)
        def dint(x):
            v = np.concatenate([np.zeros((1,) + x.shape[1:]), np.cumsum(0.5 * (x[1:] + x[:-1]) * dt.reshape(-1, *([1] * (x.ndim - 1))), axis=0)])
            return np.concatenate([np.zeros((1,) + x.shape[1:]), np.cumsum(0.5 * (v[1:] + v[:-1]) * dt.reshape(-1, *([1] * (x.ndim - 1))), axis=0)])
        SA, SR = dint(fa), dint(RT)
        for j in kw:
            tau = tc[j] + shift - ts
            n = int(np.argmin(np.abs(tt - (tc[j] + shift))))
            row = np.zeros((3, 9 + 6 * nb))
            row[:, 0:3] = R_TC[j]  # + R_TC c
            row[:, 3:6] = -0.5 * tau ** 2 * np.eye(3)  # - 1/2 tau^2 g
            row[:, 6:9] = SR[n]  # + SR b
            row[:, 9 + 6 * b_i: 12 + 6 * b_i] = -np.eye(3)  # - p0
            row[:, 12 + 6 * b_i: 15 + 6 * b_i] = -tau * np.eye(3)  # - v0 tau
            rows.append(row)
            rhs.append(SA[n] - p_TC[j])
        nwin += 1
    Am, bm = np.concatenate(rows), np.concatenate(rhs)
    x, *_ = np.linalg.lstsq(Am, bm, rcond=None)
    res = (Am @ x - bm).reshape(-1, 3)
    cov = np.linalg.inv(Am.T @ Am) * (res ** 2).sum() / max(len(bm) - len(x), 1)
    sd = np.sqrt(np.diag(cov))
    c, g, bias = x[0:3], x[3:6], x[6:9]
    p_CinI = -R_CtoI @ c

    np.set_printoptions(precision=4, suppress=True)
    print(f"lever-arm fit: {nwin} windows of <= {args.win:g} s, position residual RMS "
          f"{1000 * np.sqrt((res ** 2).sum(1).mean()):.1f} mm")
    print(f"  |g| {np.linalg.norm(g):.3f} m/s^2 (9.81 expected; a different value = accel scale error)")
    print(f"  accel bias {bias} +- {sd[6:9]} m/s^2")
    print("\n=== result ===")
    print(f"timeshift_cam_imu (t_imu = t_cam + shift): {shift:+.4f} s")
    print("R_ItoC (IMU -> camera):\n", R_ItoC)
    print("R_CtoI (camera -> IMU, T_imu_cam rotation):\n", R_CtoI)
    for k, n in enumerate("xyz"):
        print(f"  camera {n} axis in IMU frame: {np.round(R_CtoI[:, k], 3)}")
    print(f"p_IinC (IMU origin in camera frame): {c} +- {sd[0:3]} m")
    print(f"p_CinI (camera origin in IMU frame, T_imu_cam translation): {p_CinI} m")
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R_CtoI, p_CinI
    print("T_imu_cam:")
    for r in T:
        print("  - [" + ", ".join(f"{v:.6f}" for v in r) + "]")


if __name__ == "__main__":
    main()
