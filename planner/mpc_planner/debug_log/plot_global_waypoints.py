#!/usr/bin/env python3
"""### HJ : Visualize /global_waypoints.psi_rad continuity.

Subscribes once to /global_waypoints, then saves a PNG with:
  - (x, y) scatter colored by index with quiver arrows showing psi_rad heading
  - psi_rad vs s  (line + points; jumps > pi/4 between neighbors are flagged)
  - unwrapped(psi_rad) vs s  (should be ~monotonic for a closed loop)
  - finite-diff psi computed from (x,y) positions, overlaid on stored psi_rad
    (mismatch = the stored psi is inconsistent with point ordering)

Usage:
    rosrun mpc_planner plot_global_waypoints.py          # save next to this script
    python3 plot_global_waypoints.py --out /tmp/gw.png
"""

from __future__ import annotations

import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import rospy
from f110_msgs.msg import WpntArray


def extract(msg):
    n = len(msg.wpnts)
    s = np.zeros(n)
    x = np.zeros(n)
    y = np.zeros(n)
    psi = np.zeros(n)
    for i, w in enumerate(msg.wpnts):
        s[i] = w.s_m
        x[i] = w.x_m
        y[i] = w.y_m
        psi[i] = w.psi_rad
    return s, x, y, psi


def plot_all(s, x, y, psi, out_path):
    # --- jump detection on stored psi ---
    dpsi_raw = np.diff(psi)
    dpsi_wrapped = (dpsi_raw + np.pi) % (2 * np.pi) - np.pi
    big_jumps = np.where(np.abs(dpsi_wrapped) > np.pi / 4)[0]  # > 45 deg step
    # unwrapped psi
    psi_unw = np.unwrap(psi)

    # --- finite-diff tangent from (x,y) ---
    dx = np.gradient(x)
    dy = np.gradient(y)
    psi_fd = np.arctan2(dy, dx)
    psi_fd_unw = np.unwrap(psi_fd)

    # diff between stored and finite-diff
    delta = (psi - psi_fd + np.pi) % (2 * np.pi) - np.pi
    mismatch_idx = np.where(np.abs(delta) > np.pi / 4)[0]

    fig = plt.figure(figsize=(26, 16))
    gs = fig.add_gridspec(3, 2, height_ratios=[3.0, 1.2, 1.2])

    # Panel 1: FULL (x,y) + heading arrows (BIG)
    ax = fig.add_subplot(gs[0, 0])
    span = max(x.max() - x.min(), y.max() - y.min())
    L = span * 0.08  # 8% of track span per arrow — big and visible
    ax.plot(x, y, '-', c='lightgrey', lw=1.0, zorder=0)
    ax.scatter(x, y, c=np.arange(len(x)), cmap='viridis', s=10, zorder=1)
    step = max(len(x) // 30, 1)  # only ~30 arrows
    ii = np.arange(0, len(x), step)
    # stored psi — thick red
    ax.quiver(x[ii], y[ii], np.cos(psi[ii]) * L, np.sin(psi[ii]) * L,
              angles='xy', scale_units='xy', scale=1.0, width=0.006,
              headwidth=4, headlength=5, headaxislength=4,
              color='red', alpha=0.95, label='stored psi_rad', zorder=3)
    # finite-diff — thin green, offset so both are visible
    ax.quiver(x[ii], y[ii], np.cos(psi_fd[ii]) * L * 0.85,
              np.sin(psi_fd[ii]) * L * 0.85,
              angles='xy', scale_units='xy', scale=1.0, width=0.003,
              headwidth=4, headlength=5, headaxislength=4,
              color='green', alpha=0.9, label='finite-diff tangent', zorder=2)
    if len(big_jumps):
        ax.scatter(x[big_jumps], y[big_jumps], s=400, facecolors='none',
                   edgecolors='red', linewidth=3.0,
                   label=f'psi jump >45° ({len(big_jumps)})', zorder=4)
        for j in big_jumps:
            ax.annotate(f'idx {j}\npsi={np.degrees(psi[j]):.0f}°',
                        (x[j], y[j]), fontsize=10, color='red',
                        xytext=(10, 10), textcoords='offset points',
                        bbox=dict(boxstyle='round', fc='white', ec='red'))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title('FULL map: red=stored psi_rad  green=finite-diff tangent   '
                 '(arrows ~8% of track span)', fontsize=12)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.legend(loc='best', fontsize=11)

    # Panel 1b: ZOOM with every-point arrows, BIG
    axz = fig.add_subplot(gs[0, 1])
    focus_idx = big_jumps if len(big_jumps) else np.array([0])
    center = int(focus_idx[0])
    # For idx=0 (seam) show ±20 around the seam including both ends
    win = 20
    if center <= win:
        # at start — also pull in the end of the track
        seg_front = np.arange(0, center + win + 1)
        seg_back = np.arange(len(x) - win, len(x))
        sel = np.concatenate([seg_back, seg_front])
    elif center >= len(x) - win:
        seg_front = np.arange(0, win)
        seg_back = np.arange(center - win, len(x))
        sel = np.concatenate([seg_back, seg_front])
    else:
        sel = np.arange(center - win, center + win + 1)
    xz = x[sel]; yz = y[sel]; psiz = psi[sel]; psifdz = psi_fd[sel]
    Lz = max(xz.max() - xz.min(), yz.max() - yz.min(), 1e-3) * 0.15
    axz.plot(xz, yz, '-', c='lightgrey', lw=1.2, zorder=0)
    axz.scatter(xz, yz, c=np.arange(len(sel)), cmap='viridis', s=50, zorder=1)
    axz.quiver(xz, yz, np.cos(psiz) * Lz, np.sin(psiz) * Lz,
               angles='xy', scale_units='xy', scale=1.0, width=0.008,
               headwidth=4, headlength=5, headaxislength=4,
               color='red', alpha=0.95, label='stored psi', zorder=3)
    axz.quiver(xz, yz, np.cos(psifdz) * Lz * 0.85,
               np.sin(psifdz) * Lz * 0.85,
               angles='xy', scale_units='xy', scale=1.0, width=0.004,
               headwidth=4, headlength=5, headaxislength=4,
               color='green', alpha=0.9, label='finite-diff', zorder=2)
    for ii_sub, j in enumerate(sel):
        if j in big_jumps or ii_sub % 3 == 0:
            axz.annotate(str(int(j)), (xz[ii_sub], yz[ii_sub]),
                         fontsize=8, alpha=0.85,
                         xytext=(4, 4), textcoords='offset points')
    axz.scatter(x[focus_idx], y[focus_idx], s=600, facecolors='none',
                edgecolors='red', linewidth=3.5, zorder=4)
    axz.set_aspect('equal')
    axz.grid(True, alpha=0.3)
    axz.set_title(f'ZOOM around idx {center} (seam area, ±{win} pts)',
                  fontsize=12)
    axz.set_xlabel('x [m]')
    axz.set_ylabel('y [m]')
    axz.legend(loc='best', fontsize=11)

    # Panel 2: psi vs s (wrapped + jumps)
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(s, psi, '-', c='C0', lw=0.7, label='psi_rad (stored, wrapped)')
    ax.plot(s, psi_fd, '-', c='C2', lw=0.7, alpha=0.7, label='finite-diff (wrapped)')
    if len(big_jumps):
        for j in big_jumps:
            ax.axvline(s[j], color='red', alpha=0.3, lw=0.5)
    ax.grid(True, alpha=0.3)
    ax.set_title(f'psi_rad vs s  (red lines = jumps >45°, n={len(big_jumps)})')
    ax.set_xlabel('s [m]')
    ax.set_ylabel('psi [rad]')
    ax.legend(loc='best')

    # Panel 3: unwrapped psi vs s
    ax = fig.add_subplot(gs[1, 1])
    ax.plot(s, psi_unw, '-', c='C0', lw=1.0, label='stored psi_rad (unwrapped)')
    ax.plot(s, psi_fd_unw, '-', c='C2', lw=1.0, alpha=0.7, label='finite-diff (unwrapped)')
    # linear trend
    if len(s) > 2:
        slope = (psi_unw[-1] - psi_unw[0]) / (s[-1] - s[0])
        ax.plot(s, psi_unw[0] + slope * (s - s[0]), '--',
                c='grey', lw=0.6, label=f'linear (slope={slope:.3f})')
    ax.grid(True, alpha=0.3)
    ax.set_title('unwrapped psi vs s  (should be smooth & ~monotonic)')
    ax.set_xlabel('s [m]')
    ax.set_ylabel('psi [rad]')
    ax.legend(loc='best')

    # Panel 4: stored - finite_diff difference
    ax = fig.add_subplot(gs[2, :])
    ax.plot(s, np.degrees(delta), '-', c='C3', lw=0.8)
    ax.axhline(0, color='k', lw=0.5)
    ax.axhline(45, color='orange', lw=0.5, linestyle='--')
    ax.axhline(-45, color='orange', lw=0.5, linestyle='--')
    ax.grid(True, alpha=0.3)
    ax.set_title(f'(stored psi) − (finite-diff psi) [deg]  '
                 f'| mismatches >45°: {len(mismatch_idx)}')
    ax.set_xlabel('s [m]')
    ax.set_ylabel('Δpsi [deg]')

    fig.suptitle(
        f'global_waypoints sanity check — n={len(s)}  s∈[{s[0]:.1f}, {s[-1]:.1f}]',
        fontsize=13)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    print(f'[OK] saved {out_path}')
    print(f'  psi jumps >45°: {len(big_jumps)}  '
          f'(indices: {big_jumps.tolist()[:20]}{"..." if len(big_jumps)>20 else ""})')
    print(f'  stored ≠ finite-diff >45°: {len(mismatch_idx)}  '
          f'(indices: {mismatch_idx.tolist()[:20]}{"..." if len(mismatch_idx)>20 else ""})')
    if len(mismatch_idx):
        print('  sample mismatches (idx, s, stored[deg], fd[deg], delta[deg]):')
        for j in mismatch_idx[:10]:
            print(f'    {j:4d}  s={s[j]:7.3f}  stored={np.degrees(psi[j]):+7.2f}  '
                  f'fd={np.degrees(psi_fd[j]):+7.2f}  delta={np.degrees(delta[j]):+7.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/global_waypoints')
    ap.add_argument('--timeout', type=float, default=5.0)
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'global_waypoints_check.png'))
    args = ap.parse_args()

    rospy.init_node('mpc_global_waypoints_plot', anonymous=True, disable_signals=True)
    rospy.loginfo('waiting for %s (timeout %.1fs)...', args.topic, args.timeout)
    try:
        msg = rospy.wait_for_message(args.topic, WpntArray, timeout=args.timeout)
    except rospy.ROSException:
        print('[ERR] no message on %s' % args.topic, file=sys.stderr)
        sys.exit(1)

    s, x, y, psi = extract(msg)
    print(f'[OK] received {len(s)} waypoints')
    plot_all(s, x, y, psi, args.out)


if __name__ == '__main__':
    main()
