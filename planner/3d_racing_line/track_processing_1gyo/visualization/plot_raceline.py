#!/usr/bin/env python3
"""Visualize full pipeline result: track boundaries + racing line + profiles.

Usage:
    python visualization/plot_raceline.py \
        --track data/smoothed_track_data/localization_bridge_2_3d_smoothed.csv \
        --raceline data/global_racing_lines/localization_bridge_2_3d_rc_car_10th_timeoptimal.csv

    # Also overlay raw boundary CSV:
    python visualization/plot_raceline.py \
        --track data/smoothed_track_data/localization_bridge_2_3d_smoothed.csv \
        --raceline data/global_racing_lines/localization_bridge_2_3d_rc_car_10th_timeoptimal.csv \
        --raw data/raw_track_data/localization_bridge_2_bounds_3d.csv \
        --gen data/3d_track_data/localization_bridge_2_3d.csv
"""
import argparse
import os
import sys
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from matplotlib.patches import FancyArrowPatch
import matplotlib.colors as mcolors

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from track3D import Track3D

# ─────────────────────────────────────────────────────────────────────────────
def _close(arr):
    """Append first element to close loop for plotting."""
    return np.append(arr, arr[0])


def _speed_lc(x, y, v, lw=3.0, cmap='RdYlGn'):
    """Create a LineCollection colored by speed."""
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap=cmap, linewidth=lw)
    lc.set_array(v)
    lc.set_clim(v.min(), v.max())
    return lc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--track',    required=True, help='Smoothed 3D track CSV')
    parser.add_argument('--raceline', required=True, help='Raceline CSV')
    parser.add_argument('--raw',      default=None,  help='Raw boundary CSV (optional, for comparison)')
    parser.add_argument('--gen',      default=None,  help='Gen3D track CSV before smoothing (optional)')
    args = parser.parse_args()

    # ── Load smoothed track & raceline ───────────────────────────────────────
    track = Track3D(path=args.track)
    left_b, right_b = track.get_track_bounds()

    df      = pd.read_csv(args.raceline, sep=',')
    s_opt   = df['s_opt'].to_numpy()
    v_opt   = df['v_opt'].to_numpy()
    n_opt   = df['n_opt'].to_numpy()
    chi_opt = df['chi_opt'].to_numpy()
    ax_opt  = df['ax_opt'].to_numpy()
    ay_opt  = df['ay_opt'].to_numpy()
    jx_opt  = df['jx_opt'].to_numpy()
    jy_opt  = df['jy_opt'].to_numpy()
    laptime = df['laptime'].iloc[0]

    nv = track.get_normal_vector_numpy(theta=track.theta, mu=track.mu, phi=track.phi)
    rl_x = track.x + nv[0] * n_opt
    rl_y = track.y + nv[1] * n_opt

    # ── Load optional raw / gen3d data ───────────────────────────────────────
    raw_data = None
    if args.raw and os.path.exists(args.raw):
        with open(args.raw) as f:
            rows = list(csv.DictReader(f))
        raw_data = {
            'lx': np.array([float(r['left_bound_x']) for r in rows]),
            'ly': np.array([float(r['left_bound_y']) for r in rows]),
            'rx': np.array([float(r['right_bound_x']) for r in rows]),
            'ry': np.array([float(r['right_bound_y']) for r in rows]),
        }

    gen_data = None
    if args.gen and os.path.exists(args.gen):
        gen_track = Track3D(path=args.gen)
        gl, gr = gen_track.get_track_bounds()
        gen_data = {'left': gl, 'right': gr, 'x': gen_track.x, 'y': gen_track.y}

    # ════════════════════════════════════════════════════════════════════════
    #  FIGURE 1: Track map with racing line (speed-colored)
    # ════════════════════════════════════════════════════════════════════════
    fig1 = plt.figure('Track + Racing Line', figsize=(12, 10))
    gs = gridspec.GridSpec(1, 2, width_ratios=[40, 1], wspace=0.03)
    ax_map = fig1.add_subplot(gs[0])
    ax_cb  = fig1.add_subplot(gs[1])

    # Track boundary fill (light gray between walls)
    from matplotlib.patches import Polygon
    left_xy  = np.column_stack([_close(left_b[0]),  _close(left_b[1])])
    right_xy = np.column_stack([_close(right_b[0]), _close(right_b[1])])
    track_poly = np.vstack([left_xy, right_xy[::-1]])
    ax_map.add_patch(Polygon(track_poly, closed=True, fc='#e8e8e8', ec='none', zorder=0))

    # Boundaries
    ax_map.plot(_close(left_b[0]),  _close(left_b[1]),  color='#333', lw=1.8, zorder=2)
    ax_map.plot(_close(right_b[0]), _close(right_b[1]), color='#333', lw=1.8, zorder=2)

    # Centerline (faint)
    ax_map.plot(_close(track.x), _close(track.y), '--', color='#999', lw=0.6, zorder=1)

    # Racing line colored by speed
    lc = _speed_lc(rl_x, rl_y, v_opt, lw=3.5, cmap='RdYlGn')
    ax_map.add_collection(lc)
    plt.colorbar(lc, cax=ax_cb, label='Speed [m/s]')

    # Start marker
    ax_map.plot(rl_x[0], rl_y[0], 'o', color='white', ms=8, mec='black', mew=2, zorder=10)
    ax_map.annotate('START', (rl_x[0], rl_y[0]), fontsize=8, fontweight='bold',
                    xytext=(8, 8), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.2', fc='yellow', ec='k', alpha=0.9))

    ax_map.set_aspect('equal')
    ax_map.set_title(f'Laptime: {laptime:.3f}s  |  V: {v_opt.min():.1f}~{v_opt.max():.1f} m/s', fontsize=13, fontweight='bold')
    ax_map.grid(True, alpha=0.15)
    ax_map.set_xlabel('x [m]')
    ax_map.set_ylabel('y [m]')
    fig1.tight_layout()

    # ════════════════════════════════════════════════════════════════════════
    #  FIGURE 2: Profiles (speed, offset, accel, jerk)
    # ════════════════════════════════════════════════════════════════════════
    fig2, axes = plt.subplots(5, 1, figsize=(14, 10), sharex=True, num='Profiles')

    # Speed
    ax_v = axes[0]
    ax_v.fill_between(s_opt, 0, v_opt, alpha=0.15, color='C0')
    ax_v.plot(s_opt, v_opt, color='C0', lw=1.5)
    ax_v.set_ylabel('V [m/s]')
    ax_v.set_ylim(bottom=0)

    # Lateral offset
    ax_n = axes[1]
    ax_n.fill_between(s_opt, 0, n_opt, where=(n_opt >= 0), alpha=0.15, color='C2')
    ax_n.fill_between(s_opt, 0, n_opt, where=(n_opt < 0),  alpha=0.15, color='C3')
    ax_n.plot(s_opt, n_opt, color='C2', lw=1.2)
    ax_n.axhline(0, color='k', lw=0.4)
    # Track width bounds
    ax_n.fill_between(s_opt, track.w_tr_left,  alpha=0.06, color='blue',  label='left limit')
    ax_n.fill_between(s_opt, track.w_tr_right, alpha=0.06, color='red',   label='right limit')
    ax_n.set_ylabel('n [m]')
    ax_n.legend(fontsize=7, ncol=2, loc='upper right')

    # Heading angle chi
    ax_chi = axes[2]
    ax_chi.plot(s_opt, np.degrees(chi_opt), color='C4', lw=1.2)
    ax_chi.axhline(0, color='k', lw=0.4)
    ax_chi.set_ylabel(r'$\chi$ [deg]')

    # Accelerations
    ax_a = axes[3]
    ax_a.plot(s_opt, ax_opt, color='C0', lw=1.2, label=r'$a_x$ (long)')
    ax_a.plot(s_opt, ay_opt, color='C3', lw=1.2, label=r'$a_y$ (lat)')
    ax_a.axhline(0, color='k', lw=0.4)
    ax_a.set_ylabel(r'a [m/s²]')
    ax_a.legend(fontsize=7, ncol=2, loc='upper right')

    # Jerk
    ax_j = axes[4]
    ax_j.plot(s_opt, jx_opt, color='C0', lw=1.0, alpha=0.8, label=r'$j_x$')
    ax_j.plot(s_opt, jy_opt, color='C3', lw=1.0, alpha=0.8, label=r'$j_y$')
    ax_j.axhline(0, color='k', lw=0.4)
    ax_j.set_ylabel(r'j [m/s³]')
    ax_j.set_xlabel('s [m]')
    ax_j.legend(fontsize=7, ncol=2, loc='upper right')

    for ax in axes:
        ax.grid(True, alpha=0.2)
        ax.tick_params(labelsize=8)

    fig2.suptitle(f'Racing Line Profiles  —  Laptime {laptime:.3f}s', fontsize=13, fontweight='bold')
    fig2.tight_layout()

    # ════════════════════════════════════════════════════════════════════════
    #  FIGURE 3: Pipeline comparison (raw → gen3d → smoothed) if data given
    # ════════════════════════════════════════════════════════════════════════
    if raw_data or gen_data:
        n_panels = 1 + bool(gen_data) + 1  # raw(optional) + gen(optional) + smoothed
        panels = []
        if raw_data:
            panels.append(('Raw boundary', raw_data))
        if gen_data:
            panels.append(('Gen3D (before smoothing)', gen_data))
        panels.append(('Smoothed (final)', None))

        fig3, ax3s = plt.subplots(1, len(panels), figsize=(7 * len(panels), 8), num='Pipeline Comparison')
        if len(panels) == 1:
            ax3s = [ax3s]

        for idx, (title, data) in enumerate(panels):
            ax = ax3s[idx]

            if title == 'Raw boundary' and data:
                ax.plot(_close(data['lx']), _close(data['ly']), 'b-', lw=1.5, label='Left')
                ax.plot(_close(data['rx']), _close(data['ry']), 'r-', lw=1.5, label='Right')
                cx = (data['lx'] + data['rx']) / 2
                cy = (data['ly'] + data['ry']) / 2
                ax.plot(_close(cx), _close(cy), 'k--', lw=0.6, alpha=0.4)
                # Cross-bars
                for i in range(0, len(data['lx']), max(1, len(data['lx']) // 30)):
                    ax.plot([data['lx'][i], data['rx'][i]], [data['ly'][i], data['ry'][i]],
                            '-', color='green', lw=0.4, alpha=0.5)

            elif title.startswith('Gen3D') and data:
                ax.plot(_close(data['left'][0]),  _close(data['left'][1]),  'b-', lw=1.5, label='Left')
                ax.plot(_close(data['right'][0]), _close(data['right'][1]), 'r-', lw=1.5, label='Right')
                ax.plot(_close(data['x']), _close(data['y']), 'k--', lw=0.6, alpha=0.4)
                for i in range(0, len(data['x']), max(1, len(data['x']) // 30)):
                    ax.plot([data['left'][0][i], data['right'][0][i]],
                            [data['left'][1][i], data['right'][1][i]],
                            '-', color='green', lw=0.4, alpha=0.5)

            else:  # Smoothed + racing line
                left_xy  = np.column_stack([_close(left_b[0]),  _close(left_b[1])])
                right_xy = np.column_stack([_close(right_b[0]), _close(right_b[1])])
                poly = np.vstack([left_xy, right_xy[::-1]])
                ax.add_patch(Polygon(poly, closed=True, fc='#e8e8e8', ec='none', zorder=0))
                ax.plot(_close(left_b[0]),  _close(left_b[1]),  color='#333', lw=1.5, label='Left')
                ax.plot(_close(right_b[0]), _close(right_b[1]), color='#333', lw=1.5, label='Right')
                lc2 = _speed_lc(rl_x, rl_y, v_opt, lw=2.5, cmap='RdYlGn')
                ax.add_collection(lc2)
                plt.colorbar(lc2, ax=ax, shrink=0.6, label='V [m/s]')

            ax.set_aspect('equal')
            ax.set_title(title, fontsize=12, fontweight='bold')
            ax.legend(fontsize=8, loc='upper left')
            ax.grid(True, alpha=0.15)

        fig3.suptitle('Pipeline: Raw → Gen3D → Smoothed + Raceline', fontsize=14, fontweight='bold')
        fig3.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()
