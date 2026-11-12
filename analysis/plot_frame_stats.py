#!/usr/bin/env python3
"""Per-frame OpenVINS feature statistics (stats.csv from vio_replay) over time.

  plot_frame_stats.py <stats.csv> <out.png> [--shade 30-35,45-55] [--title T] [--smooth 1.0]

Small multiples on one time axis (seconds from the first frame): active tracks; MSCKF features used
(older stats files also had offered/triangulated); SLAM landmarks (in state, added, removed); processing time.
Optional shaded intervals mark segments of interest. Also prints per-segment means.
"""
import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # categorical slots 1-4 (dataviz reference palette)
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stats")
    ap.add_argument("out")
    ap.add_argument("--shade", default="")
    ap.add_argument("--title", default="")
    ap.add_argument("--smooth", type=float, default=1.0, help="moving-average window [s] for counts")
    a = ap.parse_args()
    names = open(a.stats).readline().lstrip("# ").strip().split(",")
    D = np.loadtxt(a.stats, delimiter=",", comments="#")
    col = {n: D[:, i] for i, n in enumerate(names)}
    t = col["cam_ts_s"] - col["cam_ts_s"][0]
    upd = col["updated"] > 0
    k = max(1, int(round(a.smooth * len(t) / (t[-1] - t[0]))))

    def sm(x, m=None):
        x = np.where(m, x, np.nan) if m is not None else x.astype(float)
        v = np.convolve(np.nan_to_num(x), np.ones(k), "same")
        n = np.convolve(~np.isnan(x), np.ones(k), "same")
        return np.where(n > 0, v / np.maximum(n, 1), np.nan)

    shades = [tuple(map(float, s.split("-"))) for s in a.shade.split(",") if s]
    fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    def present(*pairs):  # (label, column, mask) for the columns this stats file has
        return [(lab, col[c], m) for lab, c, m in pairs if c in col]
    added = "slam_added" if "slam_added" in col else "slam_delayed_added"
    panels = [
        ("active tracks (features tracked in the image)", present(("active tracks", "active_tracks", None))),
        ("MSCKF per update frame", present(("offered (after cap)", "msckf_in", upd), ("triangulated", "msckf_triangulated", upd),
                                           ("used (passed chi2)", "msckf_used", upd))),
        ("SLAM landmarks", present(("in state", "slam_in_state", upd), ("updated (passed chi2)", "slam_updated", upd),
                                   ("added this frame", added, upd), ("removed this frame", "slam_removed", upd))),
        ("processing time per frame [ms]", present(("wall", "proc_ms", None), ("CPU, all threads", "cpu_ms", None))),
    ]
    for axi, (title, series) in zip(ax, panels):
        for (s0, s1) in shades:
            axi.axvspan(s0, s1, color="#f0efec", zorder=0)
        for j, (lab, x, m) in enumerate(series):
            # dashed on top so a series identical to the previous one stays visible
            axi.plot(t, sm(x, m), color=C[j], lw=2, ls="--" if j == len(series) - 1 and j > 0 else "-", label=lab, zorder=2 + j)
        axi.set_title(title, loc="left", fontsize=11, color=INK, pad=18 if len(series) > 1 else 6)
        axi.grid(color=GRID, lw=0.8)
        axi.tick_params(colors=INK2)
        for sp in ("top", "right"):
            axi.spines[sp].set_visible(False)
        if len(series) > 1:
            axi.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), fontsize=9, frameon=False, ncol=len(series))
        axi.set_ylim(bottom=0)
    ax[3].axhline(1000 / 24, color=INK2, ls="--", lw=1)
    ax[3].text(t[-1], 1000 / 24, " 41.7 ms (24 fps)", va="bottom", ha="right", color=INK2, fontsize=9)
    ax[3].set_xlabel("t from first frame [s]", color=INK2)
    fig.suptitle(a.title or a.stats, x=0.01, ha="left", fontsize=13, color=INK)
    fig.text(0.99, 0.985, f"lines: {a.smooth:g} s moving average" + ("; shaded: segments of interest" if shades else ""),
             ha="right", va="top", color=INK2, fontsize=9)
    plt.tight_layout(rect=(0, 0, 1, 0.97))
    plt.savefig(a.out, dpi=110)
    print("->", a.out)

    show = [c for c in ("active_tracks", "msckf_in", "msckf_triangulated", "msckf_used", "slam_in_state", "slam_updated",
                        "proc_ms", "cpu_ms") if c in col]
    per_frame = {"active_tracks", "proc_ms", "cpu_ms"}  # all frames; the rest: update frames only

    def seg_stats(m):
        return [np.mean(col[c][m if c in per_frame else m & upd]) for c in show]
    print("segment            " + "  ".join(f"{c[:10]:>10}" for c in show))
    inside = np.zeros(len(t), bool)
    for (s0, s1) in shades:
        m = (t >= s0) & (t < s1)
        inside |= m
        print(f"{s0:5.0f}-{s1:<5.0f}          " + "  ".join(f"{v:10.1f}" for v in seg_stats(m)))
    if shades:
        print("other frames       " + "  ".join(f"{v:10.1f}" for v in seg_stats(~inside)))
    print("all frames         " + "  ".join(f"{v:10.1f}" for v in seg_stats(np.ones(len(t), bool))))


if __name__ == "__main__":
    main()
