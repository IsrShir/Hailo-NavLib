#!/usr/bin/env python3
"""Camera-IMU rotation + time offset from a Kalibr AprilGrid recording (a stopgap until Kalibr).

  cam_imu_targetcal.py <asl_dir> --imucam calibration/hw_setups/<setup>/kalibr_imucam_chain.yaml [--min-tags 8]

1. Camera pose per frame: tags (calib_fisheye_aprilgrid.detect) + cv2.fisheye.solvePnP with the
   lens model from --imucam (cam0, scaled to the recording's resolution).
2. Rotation R_ItoC and time shift: camera rotation between consecutive target frames vs the gyro
   rotation over the same (shifted) interval, Kabsch on rotation vectors, shift grid-searched.
The translation (camera-IMU lever arm) is not estimated: it is left at 0 and OpenVINS refines it online.

Conventions in the printed result:
  R_ItoC   maps IMU-frame vectors into the camera frame; R_CtoI = R_ItoC^T
  T_imu_cam (OpenVINS kalibr_imucam_chain) = [R_CtoI 0]
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
    """cam0 intrinsics + distortion from a kalibr_imucam_chain.yaml, scaled to a w x h image."""
    v = {}
    for line in open(path):
        line = line.split("#")[0]
        if ":" in line:
            k, s = line.split(":", 1)
            v.setdefault(k.strip(), s.strip())  # first occurrence = cam0
    num = lambda s: np.array([float(x) for x in s.strip("[]").split(",")])
    fx, fy, cx, cy = num(v["intrinsics"])
    sx, sy = w / num(v["resolution"])[0], h / num(v["resolution"])[1]
    K = np.array([[fx * sx, 0, (cx + 0.5) * sx - 0.5], [0, fy * sy, (cy + 0.5) * sy - 0.5], [0, 0, 1]])
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
    ap.add_argument("--imucam", required=True, help="kalibr_imucam_chain.yaml with the cam0 lens model")
    ap.add_argument("--min-tags", type=int, default=8)
    ap.add_argument("--save-poses", help="write target-frame camera poses: t[s] px py pz qx qy qz qw (Hamilton, R_TC)")
    ap.add_argument("--span", default="0.15,0.6", help="time span of a rotation pair [s] min,max")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.asl, "cam0/data/*.p[gn][gm]")))
    dets = [d for d in grid.detect(files) if len(d[1]) >= args.min_tags]
    h, w = dets[0][2]
    K, D = load_lens(args.imucam, w, h)
    imu = np.loadtxt(os.path.join(args.asl, "imu0/data.csv"), delimiter=",", comments="#")
    t0 = imu[0, 0]
    ti, wi = (imu[:, 0] - t0) * 1e-9, imu[:, 1:4]
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

    R_CtoI = R_ItoC.T

    np.set_printoptions(precision=4, suppress=True)
    print("\n=== result ===")
    print(f"timeshift_cam_imu (t_imu = t_cam + shift): {shift:+.4f} s")
    print("R_ItoC (IMU -> camera):\n", R_ItoC)
    print("R_CtoI (camera -> IMU, T_imu_cam rotation):\n", R_CtoI)
    for k, n in enumerate("xyz"):
        print(f"  camera {n} axis in IMU frame: {np.round(R_CtoI[:, k], 3)}")
    T = np.eye(4)
    T[:3, :3] = R_CtoI
    print("T_imu_cam (translation 0, not estimated):")
    for r in T:
        print("  - [" + ", ".join(f"{v:.6f}" for v in r) + "]")


if __name__ == "__main__":
    main()
