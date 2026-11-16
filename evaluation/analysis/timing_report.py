#!/usr/bin/env python3
"""Mean time per OpenVINS stage [ms] for every run in a folder, one line per run.

  timing_report.py <results dir>        (each run in its own subfolder with timing_wall.txt, as run.sh writes it)

Stages: tracking, propagation, msckf (MSCKF update), slam (SLAM update), slam_delayed (new SLAM landmarks),
retri_marg (re-triangulation + marginalization), total. Runs with max_slam 0 have no SLAM columns (shown as 0).
For the time inside tracking (KLT, pyramid, detection, ...) use a profiler such as perf.
"""
import os
import statistics as st
import sys

STAGES = ['tracking', 'propagation', 'msckf', 'slam', 'slam_delayed', 'retri_marg', 'total']


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    root = sys.argv[1]
    for tag in sorted(os.listdir(root)):
        f = os.path.join(root, tag, 'timing_wall.txt')
        if not os.path.isfile(f):
            continue
        with open(f) as fh:
            hdr = fh.readline()
            rows = [[float(x) for x in l.split(',')[1:]] for l in fh if l.strip() and not l.startswith('#')]
        if not rows:
            print(f'{tag:24} (no timed frames)')
            continue
        cols = STAGES if 'slam update' in hdr else ['tracking', 'propagation', 'msckf', 'retri_marg', 'total']
        m = {s: st.mean(r[i] * 1000 for r in rows) for i, s in enumerate(cols)}
        print(f'{tag:24} ' + ' '.join(f'{s}={m.get(s, 0.0):.1f}' for s in STAGES))


if __name__ == '__main__':
    main()
