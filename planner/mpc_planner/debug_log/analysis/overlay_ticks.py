"""Overlay N consecutive ticks in a single frame to spot tick-to-tick
oscillation ("꼬불거림"). All trajectories are drawn in world coords so
shifting across ticks is expected — what we look for is conflicting
predictions crossing back and forth.

Usage:
    python3 overlay_ticks.py <run_dir> <start_idx> <count>
"""

from __future__ import annotations

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from load_run import load_run


def main(run_dir: str, start: int, count: int):
    run = load_run(run_dir)
    df = run.ticks
    have = df['npz_path'].astype(bool)
    window = df[(df['idx'] >= start) & (df['idx'] < start + count) & have]
    if window.empty:
        print('no npz-saved ticks in [%d,%d)' % (start, start + count))
        return

    fig, ax = plt.subplots(figsize=(9, 8))
    cmap = plt.get_cmap('viridis')
    n = len(window)
    for i, (_, row) in enumerate(window.iterrows()):
        idx = int(row['idx'])
        try:
            z = run.npz_by_idx(idx)
        except Exception:
            continue
        if 'trajectory' not in z.files:
            continue
        traj = z['trajectory']
        color = cmap(i / max(n - 1, 1))
        ax.plot(traj[:, 0], traj[:, 1], '-', color=color, lw=1.0, alpha=0.8,
                label='tick %d' % idx if i % max(n // 6, 1) == 0 else None)

    # Ego trail from the CSV for the same window
    trail = df[(df['idx'] >= start) & (df['idx'] < start + count)]
    if not trail.empty:
        ax.plot(trail['car_x'], trail['car_y'], 'k.-', lw=1.2, ms=4,
                label='ego trail')

    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    ax.set_title('overlay: ticks [%d, %d) (%d shown)' % (
        start, start + count, n))
    fig.tight_layout()
    out = os.path.join(run.dir, 'overlay_%06d_%d.png' % (start, count))
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print('saved', out)


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print('usage: overlay_ticks.py <run_dir> <start_idx> <count>')
        sys.exit(1)
    main(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))
