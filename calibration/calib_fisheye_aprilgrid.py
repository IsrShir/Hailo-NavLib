#!/usr/bin/env python3
"""Preliminary equidistant (Kalibr pinhole-equi / OpenCV fisheye) intrinsics from Kalibr AprilGrid frames.

  calib_fisheye_aprilgrid.py <asl_dir> [<asl_dir> ...] [--scale-to 768x432]
                             [--imucam calibration/hw_setups/<setup>/kalibr_imucam_chain.yaml]

--imucam writes the cam0 lens model (scaled to --scale-to) into that file: an existing file keeps its
T_imu_cam and time shift, a new one starts with identity T_imu_cam (then run cam_imu_targetcal.py).

Detection: OpenCV aruco tag36h11, 2x upscaled below 1200 px width (the Kalibr grid's touching corner
squares hurt OpenCV on small tags). Grid layout: 6x4 tags, ids row-major; corner order in tag_obj(). Not a replacement for Kalibr, a first estimate.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

COLS, ROWS, TAG, SP = 6, 4, 0.030, 0.3


def tag_obj(i):
    # board frame x right, y up; id 0 bottom-left, ids row-major. The PDF draws each tag rotated 180 deg
    # from OpenCV's canonical corner order, so corner 0 is the tag's bottom-right (checked on a frame).
    r, c = divmod(i, COLS)
    p = TAG * (1 + SP)
    x0, y0 = c * p, r * p
    return np.array([[x0 + TAG, y0], [x0, y0], [x0, y0 + TAG], [x0 + TAG, y0 + TAG]], np.float64)


def detect(files, min_tags=8):
    prm = cv2.aruco.DetectorParameters()
    prm.markerBorderBits = 2  # Kalibr tags have a 2-bit black border (outer square = tagSize)
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), prm)
    out = []
    for f in files:
        im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        f_up = 2 if im.shape[1] < 1200 else 1  # small images: upscale so the tags have enough pixels
        corners, ids, _ = det.detectMarkers(cv2.resize(im, None, fx=f_up, fy=f_up, interpolation=cv2.INTER_CUBIC))
        if ids is None:
            continue
        keep = []
        for i, c in zip(ids.ravel(), corners):
            if i >= COLS * ROWS:
                continue
            c = (c.reshape(4, 2) / f_up).astype(np.float32)
            # the true grid point is the X-junction with the small corner square; aruco's quad corner sits
            # a few px inside the tag. cornerSubPix snaps to the saddle (window ~one 3 mm bit).
            bit = np.linalg.norm(c[1] - c[0]) / 10
            win = int(np.clip(bit * 1.0, 3, 14))  # data bits start 2 bits in, so one bit is safe
            c = cv2.cornerSubPix(im, c.reshape(-1, 1, 2), (win, win), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 40, 0.01)).reshape(4, 2)
            keep.append((int(i), c.astype(np.float64)))
        if len(keep) >= min_tags:
            out.append((f, keep, im.shape))
    return out


def build(dets):
    O, I = [], []
    for _, keep, _ in dets:
        o = np.concatenate([np.c_[tag_obj(i), np.zeros(4)] for i, _ in keep])
        p = np.concatenate([c for _, c in keep])
        O.append(o.reshape(1, -1, 3).astype(np.float64))  # OpenCV 5 fisheye wants 1xNx3 / 1xNx2
        I.append(p.reshape(1, -1, 2).astype(np.float64))
    return O, I


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asl", nargs="+")
    ap.add_argument("--scale-to", default="768x432")
    ap.add_argument("--imucam", help="kalibr_imucam_chain.yaml to write the cam0 lens model into")
    args = ap.parse_args()
    files = sorted(sum([glob.glob(os.path.join(a, "cam0/data/*.p[gn][gm]")) for a in args.asl], []))
    dets = detect(files)
    h, w = dets[0][2]
    print(f"{len(files)} frames, {len(dets)} with >=8 tags, image {w}x{h}")

    O, I = build(dets)
    # layout sanity check: homography residual is a few px (lens distortion) if ids/corners map right
    e = [np.median(np.linalg.norm(cv2.perspectiveTransform(o[0, :, None, :2], cv2.findHomography(o[0, :, :2], i[0])[0])[:, 0] - i[0], axis=1))
         for o, i in zip(O, I)]
    print(f"  layout check: median homography residual {np.median(e):.2f} px")

    f0 = 4.063e-3 / 2.0e-6 * w / 3840  # datasheet EFL / IMX678 pixel pitch, scaled from 4K
    K = np.array([[f0, 0, w / 2], [0, f0, h / 2], [0, 0, 1]], np.float64)
    D = np.zeros(4)
    flags = cv2.CALIB_RECOMPUTE_EXTRINSIC | cv2.CALIB_FIX_SKEW | cv2.CALIB_USE_INTRINSIC_GUESS  # OpenCV 5 names
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)
    for it in range(4):  # calibrate, drop worst frames, recalibrate
        rms, K, D, rv, tv = cv2.fisheye.calibrate(O, I, (w, h), K, D, flags=flags, criteria=crit)
        errs = []
        for o, i, r, t in zip(O, I, rv, tv):
            pr, _ = cv2.fisheye.projectPoints(o, r, t, K, D)
            errs.append(np.sqrt(np.mean(np.sum((pr - i) ** 2, axis=2))))
        errs = np.array(errs)
        print(f"  pass {it}: {len(O)} frames, RMS {rms:.3f} px, per-frame RMS median {np.median(errs):.3f} max {errs.max():.3f}")
        if it < 3:
            k = errs < max(3 * np.median(errs), 1.0)
            O = [o for o, m in zip(O, k) if m]
            I = [i for i, m in zip(I, k) if m]

    allp = np.concatenate([i[0] for i in I])
    th_obs = np.degrees(np.arctan(np.linalg.norm(cv2.fisheye.undistortPoints(allp.reshape(1, -1, 2), K, D)[0], axis=1)))
    print(f"\n{w}x{h}: fx {K[0,0]:.3f} fy {K[1,1]:.3f} cx {K[0,2]:.3f} cy {K[1,2]:.3f}  k {D.ravel()}")
    corner = cv2.fisheye.undistortPoints(np.array([[[0, h / 2], [w, h / 2], [0, 0]]], np.float64), K, D)[0]
    ang = np.degrees(np.arctan(np.linalg.norm(corner, axis=1)))
    print(f"  HFOV ~{ang[0] + ang[1]:.1f} deg, corner half-angle {ang[2]:.1f} deg, observed half-angle p50/p99/max {np.percentile(th_obs, 50):.1f}/{np.percentile(th_obs, 99):.1f}/{th_obs.max():.1f} deg")

    sw, sh = map(int, args.scale_to.split("x"))
    s = sw / w
    Ks = K.copy()
    Ks[0, 0] *= s; Ks[1, 1] *= s
    Ks[0, 2] = (K[0, 2] + 0.5) * s - 0.5
    Ks[1, 2] = (K[1, 2] + 0.5) * (sh / h) - 0.5
    print(f"{sw}x{sh}: fx {Ks[0,0]:.4f} fy {Ks[1,1]:.4f} cx {Ks[0,2]:.4f} cy {Ks[1,2]:.4f} (same k)")
    if args.imucam:
        write_imucam(args.imucam, {
            "camera_model": "pinhole",
            "distortion_model": "equidistant",
            "intrinsics": f"[{Ks[0,0]:.4f}, {Ks[1,1]:.4f}, {Ks[0,2]:.4f}, {Ks[1,2]:.4f}]",
            "distortion_coeffs": f"[{', '.join(f'{v:.8f}' for v in D.ravel())}]",
            "resolution": f"[{sw}, {sh}]"})
        print(f"-> {args.imucam} (cam0 lens model; source: {', '.join(args.asl)}, {len(O)} frames, RMS {rms:.3f} px "
              f"at {w}x{h} - note it in the file's header comment)")


def write_imucam(path, vals):
    """Set the cam0 lens keys of a kalibr_imucam_chain.yaml, keeping everything else (T_imu_cam, time shift)."""
    if not os.path.exists(path):
        with open(path, "w") as f:
            f.write("%YAML:1.0\ncam0:\n  T_imu_cam: # R_CtoI | p_CinI (identity until cam_imu_targetcal.py)\n"
                    "    - [1.0, 0.0, 0.0, 0.0]\n    - [0.0, 1.0, 0.0, 0.0]\n    - [0.0, 0.0, 1.0, 0.0]\n"
                    "    - [0.0, 0.0, 0.0, 1.0]\n  cam_overlaps: []\n"
                    + "".join(f"  {k}: {v}\n" for k, v in vals.items())
                    + "  timeshift_cam_imu: 0.0\n  rostopic: /cam0/image_raw\n")
        return
    lines, seen = open(path).read().splitlines(keepends=True), set()
    for n, line in enumerate(lines):
        k = line.split(":", 1)[0].strip()
        if line.startswith("  ") and not line.startswith("   ") and k in vals and k not in seen:  # first = cam0
            lines[n] = f"  {k}: {vals[k]}\n"
            seen.add(k)
    missing = [k for k in vals if k not in seen]
    if missing:
        sys.exit(f"{path}: no cam0 key(s) {missing}")
    open(path, "w").writelines(lines)


if __name__ == "__main__":
    main()
