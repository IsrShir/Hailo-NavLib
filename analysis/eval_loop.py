#!/usr/bin/env python3
"""Accuracy of a VIO run on a walk that ends where it started (no ground-truth trajectory needed).

  eval_loop.py <traj.txt> [--asl <dataset dir>] [--same-spot 32-33.5,67.5-70.5] [--out plot.png] [--title T]

Times are seconds from the dataset's first camera frame with --asl, else from the first VIO pose.

Prints:
  path length (horizontal) and the loop error = horizontal distance between the end and the start position,
  in m and in % of the path; the height difference end - start.
  With --asl: the still periods, found from the IMU (gyro and accelerometer quiet for >= 1 s). OpenVINS starts
  at the first motion after the still start, so the start position is the first 0.3 s of VIO; the end position is
  averaged over the final still period - so a walk should start and end standing still on the same spot.
  With --same-spot A-B,C-D: two time intervals in which the drone stood on the same spot (e.g. a mark passed on
  the way out and on the way back). Prints the distance between VIO's two positions there, and - meaningful for an
  out-and-back walk - the heading and length of the leg start->spot against the leg spot->end (reversed): a heading
  difference means the error is mostly in yaw, a length difference means it is mostly in scale.
Plot: top view coloured by time (start, end, still periods marked) and distance / height from the start over time
(still periods shaded).
"""
import argparse
import os

import numpy as np


def load_traj(path):
    d = np.loadtxt(path, comments='#')
    _, i = np.unique(d[:, 0], return_index=True)  # drop repeated timestamps
    return d[i, 0], d[i, 1:4]


def still_periods(asl, t0, min_len=1.0, win=0.5, gyro_max=0.08, acc_std_max=0.3):
    """Still intervals [(start, end)] in seconds from t0, from imu0/data.csv (rad/s, m/s^2)."""
    imu = np.loadtxt(os.path.join(asl, 'imu0', 'data.csv'), delimiter=',', comments='#')
    t = imu[:, 0] * 1e-9 - t0
    g = np.linalg.norm(imu[:, 1:4], axis=1)
    a = np.linalg.norm(imu[:, 4:7], axis=1)
    fs = (len(t) - 1) / (t[-1] - t[0])
    n = max(int(win * fs), 3)
    k = np.ones(n) / n
    g_mean = np.convolve(g, k, 'same')
    a_mean = np.convolve(a, k, 'same')
    a_std = np.sqrt(np.maximum(np.convolve(a * a, k, 'same') - a_mean ** 2, 0))
    still = (g_mean < gyro_max) & (a_std < acc_std_max)
    out, s = [], None
    for ti, st in zip(t, still):
        if st and s is None:
            s = ti
        elif not st and s is not None:
            if ti - s >= min_len:
                out.append((s, ti))
            s = None
    if s is not None and t[-1] - s >= min_len:
        out.append((s, t[-1]))
    return out


def mean_in(t, p, a, b):
    m = (t >= a) & (t <= b)
    if not m.any():
        raise SystemExit(f'no VIO poses between {a:.1f} and {b:.1f} s')
    return p[m].mean(axis=0)


def heading(v):
    return np.degrees(np.arctan2(v[1], v[0]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('traj')
    ap.add_argument('--asl', help='dataset folder the run used (cam0/, imu0/): time base and still periods')
    ap.add_argument('--same-spot', help='two time intervals at the same spot, e.g. 32-33.5,67.5-70.5')
    ap.add_argument('--out', help='plot (PNG)')
    ap.add_argument('--title', default=None)
    a = ap.parse_args()

    t_abs, p = load_traj(a.traj)
    if a.asl:
        t0 = np.loadtxt(os.path.join(a.asl, 'cam0', 'data.csv'), delimiter=',', comments='#', usecols=0)[0] * 1e-9
    else:
        t0 = t_abs[0]
    t = t_abs - t0
    p = p - p[0]
    path = np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1).sum()
    name = a.title or os.path.basename(os.path.dirname(os.path.abspath(a.traj)))

    stills = still_periods(a.asl, t0) if a.asl else []
    # OpenVINS starts when the still start ends (first motion): start = first 0.3 s of VIO.
    # End = the final still period if the recording ends standing still, else the last 0.5 s.
    stills_vio = [(s, e) for s, e in stills if s > t[0] and e > t[0] + 0.2]
    start_iv = (t[0], t[0] + 0.3)
    end_iv = stills_vio[-1] if stills_vio and stills_vio[-1][1] >= t[-1] - 1.0 else (t[-1] - 0.5, t[-1])
    P0, PE = mean_in(t, p, *start_iv), mean_in(t, p, *end_iv)
    loop = np.linalg.norm((PE - P0)[:2])

    print(f'{name}: {len(t)} poses, VIO from {t[0]:.1f} to {t[-1]:.1f} s')
    if stills:
        print('still periods [s]: ' + ', '.join(f'{s:.1f}-{e:.1f}' for s, e in stills))
        print(f'start averaged over {start_iv[0]:.1f}-{start_iv[1]:.1f} s, end over {end_iv[0]:.1f}-{end_iv[1]:.1f} s')
    print(f'path {path:.1f} m (horizontal), max distance from start {np.linalg.norm(p[:, :2], axis=1).max():.1f} m')
    print(f'loop error {loop:.2f} m = {100 * loop / path:.1f}% of the path; height end - start {PE[2] - P0[2]:+.2f} m')

    if a.same_spot:
        (a1, b1), (a2, b2) = [tuple(float(x) for x in iv.split('-')) for iv in a.same_spot.split(',')]
        A, B = mean_in(t, p, a1, b1), mean_in(t, p, a2, b2)
        m = (t >= (a1 + b1) / 2) & (t <= (a2 + b2) / 2)
        between = np.linalg.norm(np.diff(p[m, :2], axis=0), axis=1).sum()
        out_leg, back_leg = (A - P0)[:2], (PE - B)[:2]
        dh = (heading(-back_leg) - heading(out_leg) + 180) % 360 - 180
        dl = 100 * (np.linalg.norm(back_leg) / np.linalg.norm(out_leg) - 1)
        print(f'same spot: the two visits are {np.linalg.norm((B - A)[:2]):.2f} m apart after {between:.1f} m of path')
        print(f'out-and-back: leg start->spot {np.linalg.norm(out_leg):.2f} m at {heading(out_leg):.1f} deg, '
              f'leg spot->end {np.linalg.norm(back_leg):.2f} m at {heading(-back_leg):.1f} deg (reversed) '
              f'-> heading differs {dh:+.1f} deg, length {dl:+.1f}%')

    if a.out:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 6.5), gridspec_kw=dict(width_ratios=[1, 1.3], wspace=0.3))
        seg = np.stack([p[:-1, :2], p[1:, :2]], axis=1)
        lc = LineCollection(seg, array=t[:-1], cmap='viridis', linewidth=3)
        ax.add_collection(lc)
        ax.plot(*P0[:2], 'o', color='#1aa874', ms=11, label='start', zorder=5)
        ax.plot(*PE[:2], 's', color='#e8622c', ms=9, label='end', zorder=5)
        placed = []
        for s, e in [iv for iv in stills_vio if iv != end_iv]:
            c = mean_in(t, p, s, e)
            ax.plot(*c[:2], 'D', color='#555555', ms=6, zorder=5)
            near = sum(np.linalg.norm(c[:2] - q) < 1.5 for q in placed)  # stack labels of nearby stops
            placed.append(c[:2])
            ax.annotate(f'still {s:.0f}-{e:.0f} s', c[:2], textcoords='offset points', xytext=(8, 4 - 14 * near),
                        fontsize=8)
        ax.set_aspect('equal')
        ax.autoscale()
        ax.margins(0.08)
        ax.grid(alpha=0.3)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.legend(loc='best', frameon=False)
        ax.set_title(f'top view - path {path:.1f} m, end {loop:.2f} m from start', fontsize=11)
        fig.colorbar(lc, ax=ax, orientation='horizontal', shrink=0.6, pad=0.1, aspect=30, label='t [s]')
        ax2.plot(t, np.linalg.norm(p[:, :2] - P0[:2], axis=1), color='#2a78c8', lw=2,
                 label='horizontal distance from start')
        ax2.plot(t, p[:, 2] - P0[2], color='#e8622c', lw=2, label='height from start')
        for s, e in stills:
            ax2.axvspan(s, e, color='#888888', alpha=0.15, lw=0)
        ax2.set_xlabel('t [s]' + (' from the first camera frame' if a.asl else ''))
        ax2.set_ylabel('[m]')
        ax2.grid(alpha=0.3)
        ax2.legend(frameon=False)
        ax2.set_title('distance and height' + (' (shaded: standing still)' if stills else ''), fontsize=11)
        for x in (ax, ax2):
            x.spines[['top', 'right']].set_visible(False)
        fig.suptitle(f'{name}: loop error {loop:.2f} m ({100 * loop / path:.1f}% of {path:.1f} m)', x=0.02,
                     ha='left')
        fig.savefig(a.out, dpi=110, bbox_inches='tight')
        print(f'plot: {a.out}')


if __name__ == '__main__':
    main()
