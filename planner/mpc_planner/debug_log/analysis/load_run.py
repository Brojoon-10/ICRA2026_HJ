"""Utility: open a debug_log run directory.

Usage:
    from load_run import load_run
    run = load_run('/path/to/runs/<ts>_<state>_<sha>')
    run.meta                # dict from meta.json
    run.ticks               # pandas.DataFrame
    run.npz(idx)            # dict-like from the idx-th tick's npz (if saved)
    run.npz_indices()       # ticks that have a saved npz
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass

import numpy as np
import pandas as pd


@dataclass
class Run:
    dir: str
    meta: dict
    ticks: pd.DataFrame

    def npz_indices(self):
        return self.ticks.index[self.ticks['npz_path'].astype(bool)].tolist()

    def npz_by_idx(self, idx):
        row = self.ticks[self.ticks['idx'] == idx]
        if row.empty:
            raise KeyError('No tick with idx=%d' % idx)
        rel = row['npz_path'].iloc[0]
        if not rel:
            raise KeyError('No npz saved for idx=%d' % idx)
        return np.load(os.path.join(self.dir, rel), allow_pickle=True)


def load_run(run_dir: str) -> Run:
    run_dir = os.path.abspath(run_dir)
    meta_path = os.path.join(run_dir, 'meta.json')
    with open(meta_path, 'r') as f:
        meta = json.load(f)
    ticks = pd.read_csv(os.path.join(run_dir, 'ticks.csv'))
    return Run(dir=run_dir, meta=meta, ticks=ticks)


if __name__ == '__main__':
    import sys
    run = load_run(sys.argv[1])
    print('run:', run.dir)
    print('meta keys:', list(run.meta.keys()))
    print('ticks: %d rows, cols=%s' % (len(run.ticks), list(run.ticks.columns)))
    print('npz ticks:', len(run.npz_indices()))
