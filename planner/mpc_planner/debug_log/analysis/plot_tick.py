"""Plot a single tick: ref centerline + boundaries + MPC traj + obstacles.

Usage:
    python3 plot_tick.py <run_dir> <tick_idx>
    python3 plot_tick.py <run_dir> worst-kappa
    python3 plot_tick.py <run_dir> worst-slack
"""

from __future__ import annotations

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from load_run import load_run


def _resolve_idx(df, token: str):
    if token.isdigit():
        return int(token)
    if token == 'worst-kappa':
        sub = df[df['npz_path'].astype(bool)]
        if sub.empty:
            raise ValueError('no npz-saved ticks')
        return int(sub.loc[sub['traj_kappa_max'].idxmax(), 'idx'])
    if token == 'worst-slack':
        sub = df[df['npz_path'].astype(bool)]
        if sub.empty:
            raise ValueError('no npz-saved ticks')
        return int(sub.loc[sub['slack_max'].idxmax(), 'idx'])
    raise ValueError('unknown selector: %s' % token)


def main(run_dir: str, token: str):
    run = load_run(run_dir)
    idx = _resolve_idx(run.ticks, token)
    tick = run.ticks[run.ticks['idx'] == idx].iloc[0]
    z = run.npz_by_idx(idx)

    ref_center = z['ref_center_points']
    ref_left = z['ref_left_points'] if 'ref_left_points' in z.files else None
    ref_right = z['ref_right_points'] if 'ref_right_points' in z.files else None
    traj = z['trajectory'] if 'trajectory' in z.files else None
    initial = z['initial_state'] if 'initial_state' in z.files else None
    obs = z['obs_arr'] if 'obs_arr' in z.files else None

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.plot(ref_center[:, 0], ref_center[:, 1], 'k.-', lw=1.0, ms=3,
            label='ref center')
    if ref_left is not None:
        ax.plot(ref_left[:, 0], ref_left[:, 1], color='0.6', lw=0.7)
    if ref_right is not None:
        ax.plot(ref_right[:, 0], ref_right[:, 1], color='0.6', lw=0.7)
    if traj is not None:
        ax.plot(traj[:, 0], traj[:, 1], 'g.-', lw=1.8, ms=4, label='MPC traj')
    if initial is not None:
        ax.plot(initial[0], initial[1], 'r*', ms=14, label='ego')
    if obs is not None and obs.size:
        for o in range(obs.shape[0]):
            if np.any(obs[o, :, 2] > 0.0):
                ax.plot(obs[o, :, 0], obs[o, :, 1], 'rx-', lw=1.0, ms=6,
                        label='obs %d' % o if o < 3 else None)

    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ttl = ('tick %d  tier=%d  %s\n'
           'solve=%.1fms  κ_max=%.2f  slack=%.3f  obs=%s' % (
               idx, int(tick['tier']), tick['ipopt_status'],
               tick['solve_ms'], tick['traj_kappa_max'],
               tick['slack_max'], tick['obs_tag']))
    ax.set_title(ttl)
    ax.legend(loc='best')
    fig.tight_layout()
    out = os.path.join(run.dir, 'tick_%06d.png' % idx)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print('saved', out)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('usage: plot_tick.py <run_dir> <idx|worst-kappa|worst-slack>')
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
