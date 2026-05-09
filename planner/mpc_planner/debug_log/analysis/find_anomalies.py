"""Filter ticks.csv by condition, print offending tick indices.

Usage:
    python3 find_anomalies.py <run_dir> [--kappa=2.0] [--slack=0.2] [--tier=1]
                                         [--not-succeeded] [--top=20]
"""

from __future__ import annotations

import argparse
import os
import sys

from load_run import load_run


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--kappa', type=float, default=None,
                    help='min traj_kappa_max to keep')
    ap.add_argument('--slack', type=float, default=None,
                    help='min slack_max to keep')
    ap.add_argument('--tier', type=int, default=None,
                    help='min tier (1/2/3)')
    ap.add_argument('--not-succeeded', action='store_true',
                    help='IPOPT != Solve_Succeeded')
    ap.add_argument('--top', type=int, default=20)
    args = ap.parse_args()

    run = load_run(args.run_dir)
    df = run.ticks
    mask = df['idx'] > 0  # start with all True
    if args.kappa is not None:
        mask &= (df['traj_kappa_max'] >= args.kappa)
    if args.slack is not None:
        mask &= (df['slack_max'] >= args.slack)
    if args.tier is not None:
        mask &= (df['tier'] >= args.tier)
    if args.not_succeeded:
        mask &= (~df['ipopt_status'].isin(
            ['Solve_Succeeded', 'Solved_To_Acceptable_Level']))

    hits = df[mask].sort_values('traj_kappa_max', ascending=False).head(args.top)
    cols = ['idx', 'tier', 'ipopt_status', 'solve_ms', 'slack_max',
            'traj_kappa_max', 'ego_n', 'obs_tag', 'npz_path']
    if hits.empty:
        print('no hits')
    else:
        print(hits[cols].to_string(index=False))
        print('\n%d hits (of %d)' % (len(hits), mask.sum()))


if __name__ == '__main__':
    main()
