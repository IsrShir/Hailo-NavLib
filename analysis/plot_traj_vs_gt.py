# 3D trajectory plots: estimates (posyaw-aligned to ground truth, like ov_eval) vs ground truth, e.g. EuRoC.
# usage: plot_traj_vs_gt.py <ground_truth.txt> <out_dir> <label>=<traj> [<label>=<traj> ...]
#   ground truth and trajectories: "t x y z ..." per line (TUM-like; '#' comments), t in seconds
#   writes one plot per trajectory plus <out_dir>/overview.png; prints the ATE per trajectory
import sys, os, numpy as np

def load(path):
    d = np.loadtxt(path, comments="#")
    return d[:, 0], d[:, 1:4]

def associate(te, tg, max_dt=0.02):
    idx = np.clip(np.searchsorted(tg, te), 1, len(tg) - 1)
    left, right = tg[idx - 1], tg[idx]
    j = np.where(np.abs(te - left) < np.abs(te - right), idx - 1, idx)
    ok = np.abs(tg[j] - te) < max_dt
    return np.nonzero(ok)[0], j[ok]

def align_posyaw(pe, pg):
    # yaw + translation that best maps estimate onto ground truth (4-DoF Umeyama, as ov_eval "posyaw")
    me, mg = pe.mean(0), pg.mean(0)
    a, b = pe - me, pg - mg
    yaw = np.arctan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return lambda p: (p - me) @ Rz.T + mg

def prepare(gt_file, path):
    """Ground truth, aligned estimate, per-pose error and ATE (RMSE, m) of one trajectory."""
    tg, pg = load(gt_file)
    te, pe = load(path)
    ie, ig = associate(te, tg)
    f = align_posyaw(pe[ie], pg[ig])
    pa = f(pe[ie])
    err = np.linalg.norm(pa - pg[ig], axis=1)
    return pg, pa, err, np.sqrt(np.mean(err ** 2))


def ate(gt_file, path):
    """ATE (RMSE of position after yaw + translation alignment, m) and the number of matched poses."""
    _, _, err, rmse = prepare(gt_file, path)
    return rmse, len(err)

def draw(ax, seq, label, pg, pa, err, ate, vmax):
    ax.plot(pg[:, 0], pg[:, 1], pg[:, 2], color="0.55", lw=1.2, label="ground truth")
    sc = ax.scatter(pa[:, 0], pa[:, 1], pa[:, 2], c=err, cmap="viridis", vmin=0, vmax=vmax, s=2, label="estimate")
    ax.scatter(*pg[0], color="green", s=30, marker="o"); ax.scatter(*pg[-1], color="red", s=30, marker="s")
    ax.set_title(f"{label}\nATE {ate:.3f} m", fontsize=9)
    ax.set_xlabel("x [m]", fontsize=8); ax.set_ylabel("y [m]", fontsize=8); ax.set_zlabel("z [m]", fontsize=8)
    ax.tick_params(labelsize=7)
    rng = np.ptp(pg, axis=0); ax.set_box_aspect(np.maximum(rng, 0.3 * rng.max()))
    return sc

def main():
    if len(sys.argv) < 4:
        sys.exit("usage: plot_traj_vs_gt.py <ground_truth.txt> <out_dir> <label>=<traj> [<label>=<traj> ...]")
    GT_FILE, OUT = sys.argv[1], sys.argv[2]
    RUNS = [a.split("=", 1) for a in sys.argv[3:]]
    SEQ = os.path.splitext(os.path.basename(GT_FILE))[0]
    import matplotlib  # only for the plots: ate() works without it
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT, exist_ok=True)
    seq = SEQ
    data = [(lbl, *prepare(GT_FILE, p)) for lbl, p in RUNS]
    vmax = max(np.percentile(d[3], 99) for d in data)
    for lbl, pg, pa, err, ate in data:
        print(f"{seq:16} {lbl:40} ATE={ate:.3f} n={len(err)}")
        fig = plt.figure(figsize=(7, 5.5)); ax = fig.add_subplot(111, projection="3d")
        sc = draw(ax, seq, f"{seq} · {lbl}", pg, pa, err, ate, vmax)
        fig.colorbar(sc, ax=ax, shrink=0.6, label="position error [m]"); ax.legend(loc="upper left", fontsize=7)
        fn = "".join(c if c.isalnum() or c in "-_." else "_" for c in lbl) + ".png"
        fig.tight_layout(); fig.savefig(f"{OUT}/{fn}", dpi=130); plt.close(fig)
    n = len(data); cols = min(3, n); rows = int(np.ceil(n / cols))
    fig = plt.figure(figsize=(5.2 * cols, 4.4 * rows))
    for k, (lbl, pg, pa, err, ate) in enumerate(data):
        sc = draw(fig.add_subplot(rows, cols, k + 1, projection="3d"), seq, lbl, pg, pa, err, ate, vmax)
    fig.subplots_adjust(left=0.0, right=0.88, bottom=0.02, top=0.92, wspace=0.0, hspace=0.12)
    fig.colorbar(sc, cax=fig.add_axes([0.91, 0.3, 0.015, 0.4]), label="position error [m]")
    fig.suptitle(f"{seq} - estimate vs ground truth (grey); colour = position error", fontsize=12, y=0.98)
    fig.savefig(f"{OUT}/overview.png", dpi=110); plt.close(fig)


if __name__ == "__main__":
    main()
