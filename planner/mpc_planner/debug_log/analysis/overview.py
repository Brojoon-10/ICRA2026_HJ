"""One-shot overview of a debug_log run. Saves PNGs next to meta.json.

Usage:
    python3 overview.py /path/to/run_dir
"""

from __future__ import annotations

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from load_run import load_run


def main(run_dir: str):
    run = load_run(run_dir)
    df = run.ticks
    out = run.dir
    state = run.meta.get('state', '?')

    # ---- 1. solve_ms CDF ---------------------------------------------------
    fig, ax = plt.subplots()
    vals = np.sort(df['solve_ms'].values)
    if vals.size:
        cdf = np.linspace(0, 1, vals.size, endpoint=False)
        ax.plot(vals, cdf)
        p50 = float(np.percentile(vals, 50))
        p95 = float(np.percentile(vals, 95))
        p99 = float(np.percentile(vals, 99))
        ax.axvline(p50, ls='--', lw=0.7, label='p50=%.1f' % p50)
        ax.axvline(p95, ls='--', lw=0.7, color='orange', label='p95=%.1f' % p95)
        ax.axvline(p99, ls='--', lw=0.7, color='red', label='p99=%.1f' % p99)
    ax.set_xlabel('solve_ms')
    ax.set_ylabel('CDF')
    ax.set_title('[%s] solve_ms CDF (N=%d)' % (state, len(df)))
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'solve_ms_cdf.png'), dpi=120)
    plt.close(fig)

    # ---- 2. tier timeline --------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 2.5))
    ax.step(df['idx'], df['tier'], where='post', lw=0.8)
    ax.set_yticks([0, 1, 2, 3])
    ax.set_ylabel('tier')
    ax.set_xlabel('tick idx')
    counts = df['tier'].value_counts().to_dict()
    ax.set_title('[%s] fallback tier timeline (counts=%s)' % (state, counts))
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'tier_timeline.png'), dpi=120)
    plt.close(fig)

    # ---- 3. curvature (꼬불거림) ------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot(df['idx'], df['traj_kappa_rms'], lw=0.7, label='traj κ RMS')
    ax.plot(df['idx'], df['traj_kappa_max'], lw=0.7, alpha=0.6,
            label='traj κ max')
    ax.plot(df['idx'], df['ref_kappa_rms'], lw=0.7, ls='--',
            label='ref κ RMS', alpha=0.6)
    ax.set_xlabel('tick idx')
    ax.set_ylabel('|κ| (rad/m)')
    ax.set_title('[%s] curvature over time' % state)
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'curvature.png'), dpi=120)
    plt.close(fig)

    # ---- 4. IPOPT status pie ----------------------------------------------
    fig, ax = plt.subplots()
    pie = df['ipopt_status'].value_counts()
    ax.pie(pie.values, labels=pie.index, autopct='%1.1f%%', startangle=90)
    ax.set_title('[%s] IPOPT status (N=%d)' % (state, len(df)))
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'ipopt_status_pie.png'), dpi=120)
    plt.close(fig)

    # ---- 5. slack_max histogram -------------------------------------------
    fig, ax = plt.subplots()
    ax.hist(df['slack_max'].values, bins=50)
    ax.set_yscale('log')
    ax.set_xlabel('slack_max')
    ax.set_ylabel('count (log)')
    ax.set_title('[%s] slack_max distribution' % state)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'slack_hist.png'), dpi=120)
    plt.close(fig)

    # ---- 6. summary txt ----------------------------------------------------
    summary = []
    summary.append('run: %s' % run.dir)
    summary.append('state: %s  git: %s' % (state, run.meta.get('git_sha', '-')))
    summary.append('N ticks: %d' % len(df))
    if len(df):
        summary.append('solve_ms  p50=%.2f  p95=%.2f  p99=%.2f  max=%.2f' % (
            np.percentile(df['solve_ms'], 50),
            np.percentile(df['solve_ms'], 95),
            np.percentile(df['solve_ms'], 99),
            df['solve_ms'].max(),
        ))
        for t in (0, 1, 2, 3):
            n = int((df['tier'] == t).sum())
            summary.append('  tier %d: %d  (%.1f%%)' % (
                t, n, 100.0 * n / max(len(df), 1)))
        summary.append('npz saved: %d / %d' % (
            int(df['npz_path'].astype(bool).sum()), len(df)))
        summary.append('traj κ max    max=%.2f  p95=%.2f' % (
            df['traj_kappa_max'].max(),
            np.percentile(df['traj_kappa_max'], 95)))
        summary.append('slack_max     max=%.3f  p95=%.3f' % (
            df['slack_max'].max(),
            np.percentile(df['slack_max'], 95)))
    text = '\n'.join(summary)
    with open(os.path.join(out, 'overview.txt'), 'w') as f:
        f.write(text)
    print(text)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('usage: overview.py <run_dir>')
        sys.exit(1)
    main(sys.argv[1])
