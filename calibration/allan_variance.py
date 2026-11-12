#!/usr/bin/env python3
"""IMU noise from a static IMU-only recording (vio_rec --no-cam, board still for 2-3 h): Allan deviation per axis
and the Kalibr / OpenVINS noise parameters.

  allan_variance.py <recording dir> [--out allan.png] [--skip-min 10]

Reads <rec>/gyro.csv and <rec>/accel.csv (raw counts + "# scale=" header, as written by vio_rec).
Per axis, on the log-log Allan deviation curve sigma(tau) (details in analyse()):
  white noise  N (slope -1/2 line, value at tau = 1 s)      -> *_noise_density
  random walk  K (slope +1/2 line, value at tau = 3 s)      -> *_random_walk   (upper bound)
  bias instability B = min(sigma) / 0.664
Units: gyro N in rad/s/sqrt(Hz), K in rad/s^2/sqrt(Hz); accel N in m/s^2/sqrt(Hz), K in m/s^3/sqrt(Hz).
Kalibr's imu.yaml takes these (the "continuous-time" densities); in practice they are inflated (x2-x10) to cover
unmodelled effects (vibration, temperature, scale/misalignment).
"""
import argparse
import os

import numpy as np


def load(path):
    scale = None
    with open(path) as f:
        for line in f:
            if line.startswith('# scale='):
                scale = float(line.split('=')[1])
                break
    d = np.loadtxt(path, delimiter=',', comments='#')
    t = d[:, 0] * 1e-9
    keep = np.r_[True, np.diff(t) > 0]  # drop duplicate stamps
    return t[keep], d[keep, 1:4] * scale


def adev(x, fs, n_tau=120):
    """Overlapping Allan deviation of a rate signal x (N x 3) sampled at fs."""
    n = len(x)
    theta = np.vstack([np.zeros((1, x.shape[1])), np.cumsum(x, axis=0)]) / fs
    m = np.unique(np.logspace(0, np.log10((n - 1) / 2), n_tau).astype(int))
    tau = m / fs
    out = np.empty((len(m), x.shape[1]))
    for i, mi in enumerate(m):
        d = theta[2 * mi:] - 2 * theta[mi:-mi] + theta[:-2 * mi]
        out[i] = np.sqrt((d ** 2).mean(axis=0) / (2 * tau[i] ** 2))
    return tau, out


def analyse(tau, s, t_total):
    """Per axis: N = fixed slope -1/2 line fitted over tau 0.1..1 s, read at tau = 1 s (above the sensor's low-pass
    filter, below the bias wander); K = the largest slope +1/2 line that stays under the curve for tau <= T/10
    (an upper bound: with a few hours of data the +1/2 region is often barely reached); B = min(sigma) / 0.664."""
    res = []
    w = (tau >= 0.1) & (tau <= 1.0)
    r = tau <= t_total / 10
    for a in range(s.shape[1]):
        n = 10 ** np.mean(np.log10(s[w, a]) + 0.5 * np.log10(tau[w]))
        ks = s[r, a] * np.sqrt(3 / tau[r])
        res.append(dict(N=n, K=ks.min(), K_tau=tau[r][np.argmin(ks)], B=s[r, a].min() / 0.664,
                        B_tau=tau[r][np.argmin(s[r, a])]))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rec')
    ap.add_argument('--out', default=None, help='plot (default <rec>/allan.png)')
    ap.add_argument('--skip-min', type=float, default=10, help='drop the first minutes (warm-up), default 10')
    a = ap.parse_args()

    sensors = {}
    for name in ('gyro', 'accel'):
        t, x = load(os.path.join(a.rec, name + '.csv'))
        keep = t >= t[0] + a.skip_min * 60
        t, x = t[keep], x[keep]
        fs = (len(t) - 1) / (t[-1] - t[0])
        tau, s = adev(x, fs)
        sensors[name] = (t, x, fs, tau, s, analyse(tau, s, t[-1] - t[0]))
        print(f'{name}: {len(t)} samples, {(t[-1] - t[0]) / 3600:.2f} h, {fs:.2f} Hz, '
              f'mean {np.round(x.mean(0), 5)}')

    units = {'gyro': ('rad/s/sqrt(Hz)', 'rad/s^2/sqrt(Hz)', 'rad/s'), 'accel': ('m/s^2/sqrt(Hz)', 'm/s^3/sqrt(Hz)', 'm/s^2')}
    print()
    for name, (_, _, _, _, _, res) in sensors.items():
        u = units[name]
        print(f'{name:5s} axis  N [{u[0]}]   K [{u[1]}]   bias instability [{u[2]}] (at tau)')
        for ax, r in zip('xyz', res):
            print(f'      {ax}     {r["N"]:.3e}            {r["K"]:.3e}              {r["B"]:.3e} ({r["B_tau"]:.0f} s)   K at tau {r["K_tau"]:.0f} s')
    g, ac = sensors['gyro'][5], sensors['accel'][5]
    print('\nKalibr imu.yaml (worst axis, not inflated):')
    print(f'  gyroscope_noise_density:     {max(r["N"] for r in g):.3e}')
    print(f'  gyroscope_random_walk:       {max(r["K"] for r in g):.3e}')
    print(f'  accelerometer_noise_density: {max(r["N"] for r in ac):.3e}')
    print(f'  accelerometer_random_walk:   {max(r["K"] for r in ac):.3e}')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.8))
    colors = ['#2a78c8', '#d8742a', '#3a9a52']
    for axp, name in zip(axs, ('gyro', 'accel')):
        _, _, _, tau, s, res = sensors[name]
        for i, (ax, r) in enumerate(zip('xyz', res)):
            axp.loglog(tau, s[:, i], color=colors[i], lw=2, label=f'{ax}  N={r["N"]:.2e}  K={r["K"]:.2e}')
            axp.loglog(tau, r['N'] / np.sqrt(tau), color=colors[i], lw=1, ls=':')
            axp.loglog(tau, r['K'] * np.sqrt(tau / 3), color=colors[i], lw=1, ls='--')
        axp.set_xlabel('tau [s]')
        axp.set_ylabel(f'Allan deviation [{units[name][2]}]')
        axp.set_title(f'{name} (dotted: white noise fit, dashed: random walk fit)', fontsize=10)
        axp.grid(True, which='both', alpha=0.25)
        axp.legend(fontsize=8, loc='lower left')
    fig.suptitle(f'Allan deviation: {os.path.basename(os.path.abspath(a.rec))}')
    fig.tight_layout()
    out = a.out or os.path.join(a.rec, 'allan.png')
    fig.savefig(out, dpi=120)
    print(f'\nplot: {out}')


if __name__ == '__main__':
    main()
