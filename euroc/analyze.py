#!/usr/bin/env python3
"""Laptop: one summary line per EuRoC run (time per stage, fps, CPU, ATE vs ground truth) for a folder of runs.

  analyze.py <results dir> [--gt <ground-truth dir>]

Each run is a subfolder written by openvins/run.sh (timing_wall.txt, traj.txt, stats.csv); its name must start
with the sequence, e.g. V1_01_reduced_1thr or MH_05_default_4thr. Ground truth: <gt dir>/<sequence>.txt
(default: the files that ship with OpenVINS, openvins/open_vins/ov_data/euroc_mav/). ATE = position RMSE after a
yaw + translation alignment (as OpenVINS' ov_eval "posyaw"), computed by analysis/plot_traj_vs_gt.py; no ROS needed.
"""
import argparse
import os
import statistics as st
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'analysis'))
from plot_traj_vs_gt import ate  # noqa: E402

STAGES = ['trk', 'prop', 'msckf', 'slam', 'slamDel', 'marg']


def load_csv(path):
    with open(path) as f:
        return [[float(x) for x in l.split(',')[1:]] for l in f if l.strip() and not l.startswith('#')]


def cpu_ms(stats):
    """Mean CPU time (all threads) of the frames that ran an update, from stats.csv."""
    if not os.path.isfile(stats):
        return None
    with open(stats) as f:
        hdr = f.readline().lstrip('# ').strip().split(',')
        if 'cpu_ms' not in hdr:
            return None
        rows = [l.strip().split(',') for l in f if l.strip() and not l.startswith('#')]
    c = [float(r[hdr.index('cpu_ms')]) for r in rows if r[hdr.index('updated')] == '1']
    return st.mean(c) if c else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('results')
    ap.add_argument('--gt', default=os.path.join(ROOT, 'openvins', 'open_vins', 'ov_data', 'euroc_mav'))
    a = ap.parse_args()
    seqs = sorted(f[:-4] for f in os.listdir(a.gt) if f.endswith('.txt'))
    for tag in sorted(os.listdir(a.results)):
        d = os.path.join(a.results, tag)
        seq = next((s for s in seqs if tag.startswith(s[:5])), None)  # V1_01_reduced -> V1_01_easy
        if not seq or not os.path.isfile(os.path.join(d, 'timing_wall.txt')):
            continue
        w = load_csv(os.path.join(d, 'timing_wall.txt'))
        if not w:
            print(f'{tag:24} (no timed frames)')
            continue
        tot = [r[-1] * 1000 for r in w]
        m = st.mean(tot)
        p99 = sorted(tot)[int(0.99 * len(tot))]
        c = cpu_ms(os.path.join(d, 'stats.csv'))
        traj = os.path.join(d, 'traj.txt')
        e = f'{ate(os.path.join(a.gt, seq + ".txt"), traj)[0]:.3f}' if os.path.isfile(traj) else '?'
        stages = ' '.join(f'{n}={st.mean(r[i] * 1000 for r in w):.1f}' for i, n in enumerate(STAGES)) \
            if len(w[0]) == 7 else '(no SLAM columns)'
        print(f'{tag:24} n={len(tot):5} wall={m:6.1f}ms p99={p99:6.1f} FPS={1000 / m:5.1f} '
              f'cpu={f"{c:6.1f}ms" if c else "?":>8} ATE={e}m | {stages}')


if __name__ == '__main__':
    main()
