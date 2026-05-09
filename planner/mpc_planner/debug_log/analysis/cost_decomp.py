"""Per-term cost contribution for a tick (post-hoc reproduction).

Usage:
    python3 cost_decomp.py <run_dir> <idx>
    python3 cost_decomp.py <run_dir> worst-kappa
"""

from __future__ import annotations

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from load_run import load_run


def main(run_dir: str, token: str):
    run = load_run(run_dir)
    if token == 'worst-kappa':
        sub = run.ticks[run.ticks['npz_path'].astype(bool)]
        idx = int(sub.loc[sub['traj_kappa_max'].idxmax(), 'idx'])
    else:
        idx = int(token)
    z = run.npz_by_idx(idx)
    if 'cost_breakdown_keys' not in z.files:
        print('no cost breakdown saved for idx', idx)
        return
    keys = [str(k) for k in z['cost_breakdown_keys']]
    vals = np.asarray(z['cost_breakdown_vals'], dtype=np.float64)

    fig, ax = plt.subplots(figsize=(8, 4))
    order = np.argsort(vals)[::-1]
    ax.bar([keys[i] for i in order], [vals[i] for i in order])
    ax.set_title('cost breakdown  tick=%d  total=%.3f' % (idx, vals.sum()))
    ax.set_ylabel('contribution')
    for i, j in enumerate(order):
        ax.text(i, vals[j], '%.2f' % vals[j], ha='center', va='bottom',
                fontsize=8)
    ax.grid(alpha=0.3, axis='y')
    fig.tight_layout()
    out = os.path.join(run.dir, 'cost_decomp_%06d.png' % idx)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print('saved', out)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('usage: cost_decomp.py <run_dir> <idx|worst-kappa>')
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
