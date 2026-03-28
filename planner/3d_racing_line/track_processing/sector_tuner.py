#!/usr/bin/env python3
"""
Sector Tuner — standalone GUI for defining track sectors and assigning surface friction.
Based on UNICORN's sector_slicing.py (https://github.com/HMCL-UNIST/unicorn-racing-stack)
Adapted for offline 3D racing line planning pipeline (no ROS dependency).

Generates:
  - sector_friction.yaml : sector boundaries + friction_scale per sector

For velocity tuning per sector, run velocity_tuner.py after this tool.

Usage:
  python track_processing/sector_tuner.py
"""

import os
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from matplotlib.patches import Arrow
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize

# ============================================================
# Settings
# ============================================================
params = {
    'track_name': 'experiment_3d_2_3d_smoothed.csv',
}

# Available surface types and their default friction scales
SURFACE_DEFAULTS = {
    'carpet': 1.0,
    'concrete': 0.7,
    'bridge': 0.5,
}

# Paths
dir_path = os.path.dirname(os.path.abspath(__file__))
data_path = os.path.join(dir_path, '..', 'data')
track_path = os.path.join(data_path, 'smoothed_track_data')
out_path = os.path.join(data_path, 'sector_config')


class SectorTuner:
    """
    Matplotlib GUI for slicing a track into sectors and assigning surface types.
    Adapted from UNICORN's SectorSlicer.
    """

    def __init__(self, track_csv_path: str, out_dir: str):
        # Load track data
        self.track = pd.read_csv(track_csv_path)
        self.x = self.track['x_m'].values
        self.y = self.track['y_m'].values
        self.z = self.track['z_m'].values
        self.s = self.track['s_m'].values
        self.n_points = len(self.s)

        self.out_dir = out_dir
        os.makedirs(self.out_dir, exist_ok=True)

        # Sector state
        self.sector_pnts = [0]  # sector always starts at 0
        self.sector_surfaces = []  # surface type per sector
        self.glob_slider_idx = 0
        self.current_surface = 'concrete'

    # ----------------------------------------------------------
    # GUI
    # ----------------------------------------------------------
    def run(self):
        """Launch the sector tuner GUI."""
        fig = plt.figure(figsize=(12, 8))
        fig.canvas.manager.set_window_title('Sector Tuner')

        # Layout: track plot + controls
        ax_track = fig.add_axes([0.05, 0.35, 0.55, 0.60])
        ax_slider = fig.add_axes([0.05, 0.22, 0.55, 0.04])
        ax_select = fig.add_axes([0.05, 0.12, 0.25, 0.06])
        ax_undo = fig.add_axes([0.35, 0.12, 0.25, 0.06])
        ax_finish = fig.add_axes([0.05, 0.03, 0.55, 0.06])
        ax_radio = fig.add_axes([0.68, 0.35, 0.28, 0.25])
        ax_info = fig.add_axes([0.68, 0.65, 0.28, 0.30])
        ax_info.axis('off')

        # Track plot
        self._draw_track(ax_track)

        # Start arrow (same as UNICORN)
        arr_par = {
            'x': self.x[0],
            'dx': 10 * (self.x[1] - self.x[0]),
            'y': self.y[0],
            'dy': 10 * (self.y[1] - self.y[0]),
            'color': 'gray',
            'width': 0.05,
        }
        ax_track.add_artist(Arrow(**arr_par))

        # Slider (same structure as UNICORN)
        slider = Slider(ax_slider, 'Point idx', 0, self.n_points - 1, valinit=0, valfmt='%d')

        def update_s(val):
            self.glob_slider_idx = int(slider.val)
            self._draw_track(ax_track)
            self._draw_info(ax_info)
            fig.canvas.draw_idle()

        slider.on_changed(update_s)

        # Surface type selector
        radio = RadioButtons(ax_radio, list(SURFACE_DEFAULTS.keys()), active=0)
        ax_radio.set_title('Surface type')

        def on_surface_change(label):
            self.current_surface = label

        radio.on_clicked(on_surface_change)

        # Select button (same as UNICORN)
        btn_select = Button(ax_select, 'Add Sector Point')

        def select_s(event):
            idx = self.glob_slider_idx
            if idx not in self.sector_pnts:
                # Record surface for the sector that just ended
                self.sector_surfaces.append(self.current_surface)
                self.sector_pnts.append(idx)
                self._draw_track(ax_track)
                self._draw_info(ax_info)
                fig.canvas.draw_idle()

        btn_select.on_clicked(select_s)

        # Undo button
        btn_undo = Button(ax_undo, 'Undo Last')

        def undo_s(event):
            if len(self.sector_pnts) > 1:
                self.sector_pnts.pop()
                if self.sector_surfaces:
                    self.sector_surfaces.pop()
                self._draw_track(ax_track)
                self._draw_info(ax_info)
                fig.canvas.draw_idle()

        btn_undo.on_clicked(undo_s)

        # Finish button (same as UNICORN)
        btn_finish = Button(ax_finish, 'Done — Save YAML')

        def finish(event):
            # Last sector surface
            self.sector_surfaces.append(self.current_surface)
            # Sectors always end at end
            self.sector_pnts.append(self.n_points - 1)
            # Eliminate duplicates
            # (keep order, remove dups while keeping corresponding surfaces)
            seen = set()
            unique_pnts = []
            unique_surfs = []
            for i, p in enumerate(self.sector_pnts):
                if p not in seen:
                    seen.add(p)
                    unique_pnts.append(p)
                    if i < len(self.sector_surfaces):
                        unique_surfs.append(self.sector_surfaces[i])
            self.sector_pnts = unique_pnts
            self.sector_surfaces = unique_surfs
            plt.close()

        btn_finish.on_clicked(finish)

        # Initial info draw
        self._draw_info(ax_info)
        plt.show()

        # After GUI closes, save
        self._save_yamls()

    def _draw_track(self, ax):
        """Redraw the track plot with z-height colormap and sector overlay."""
        ax.cla()

        # z값 컬러코딩된 트랙 (LineCollection)
        points = np.array([self.x, self.y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        norm = Normalize(vmin=self.z.min(), vmax=self.z.max())
        lc = LineCollection(segments, cmap='coolwarm', norm=norm, linewidths=4, alpha=0.6)
        lc.set_array(self.z[:-1])
        ax.add_collection(lc)

        # colorbar (최초 1회만 생성)
        if not hasattr(self, '_cbar'):
            self._cbar = plt.colorbar(lc, ax=ax, pad=0.02, shrink=0.8)
            self._cbar.set_label('z [m] (height)')
        else:
            self._cbar.update_normal(lc)

        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', 'datalim')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('Track (color=z height) — slider to move, then "Add Sector Point"')
        # autoscale for LineCollection
        ax.set_xlim(self.x.min() - 0.5, self.x.max() + 0.5)
        ax.set_ylim(self.y.min() - 0.5, self.y.max() + 0.5)

        # Color existing sectors (overlay)
        sector_colors = {'concrete': 'blue', 'carpet': 'orange', 'bridge': 'red'}
        for i in range(len(self.sector_pnts) - 1):
            start = self.sector_pnts[i]
            end = self.sector_pnts[i + 1]
            surf = self.sector_surfaces[i] if i < len(self.sector_surfaces) else 'concrete'
            ax.plot(self.x[start:end + 1], self.y[start:end + 1],
                    color=sector_colors.get(surf, 'gray'), linewidth=6, alpha=0.3,
                    label=f'S{i}: {surf}')

        # Sector boundary points
        if len(self.sector_pnts) > 0:
            ax.scatter(self.x[self.sector_pnts], self.y[self.sector_pnts],
                       c='red', s=60, zorder=5, marker='x')

        # Current slider position
        idx = self.glob_slider_idx
        ax.scatter(self.x[idx], self.y[idx], c='green', s=80, zorder=6,
                   edgecolors='black', linewidths=1)

        if len(self.sector_pnts) > 1:
            ax.legend(fontsize=8, loc='upper right')

    def _draw_info(self, ax):
        """Draw sector info text."""
        ax.cla()
        ax.axis('off')
        idx = self.glob_slider_idx
        lines = [
            f'Current point: {idx}',
            f's = {self.s[idx]:.2f} m',
            f'x = {self.x[idx]:.3f}, y = {self.y[idx]:.3f}',
            f'z = {self.z[idx]:.3f} m  {"<< HIGH" if self.z[idx] > 0.1 else ""}',
            f'',
            f'Sectors defined: {max(0, len(self.sector_pnts) - 1)}',
        ]
        for i in range(len(self.sector_pnts) - 1):
            surf = self.sector_surfaces[i] if i < len(self.sector_surfaces) else '?'
            lines.append(f'  S{i}: [{self.sector_pnts[i]}→{self.sector_pnts[i+1]}] {surf}')
        ax.text(0, 1, '\n'.join(lines), transform=ax.transAxes,
                fontsize=10, verticalalignment='top', fontfamily='monospace')

    # ----------------------------------------------------------
    # YAML export
    # ----------------------------------------------------------
    def _save_yamls(self):
        """Save sector_friction.yaml."""
        n_sectors = len(self.sector_pnts) - 1
        if n_sectors < 1:
            print('No sectors defined. Skipping YAML generation.')
            return

        # --- sector_friction.yaml ---
        friction_dict = {'n_sectors': n_sectors}
        for i in range(n_sectors):
            surface = self.sector_surfaces[i] if i < len(self.sector_surfaces) else 'concrete'
            friction_dict[f'Sector{i}'] = {
                'start': int(self.sector_pnts[i]),
                'end': int(self.sector_pnts[i + 1]),
                'surface': surface,
                'friction_scale': SURFACE_DEFAULTS.get(surface, 1.0),
            }

        friction_path = os.path.join(self.out_path, 'sector_friction.yaml')
        with open(friction_path, 'w') as f:
            yaml.dump(friction_dict, f, sort_keys=False, default_flow_style=False)
        print(f'Saved: {friction_path}')

        print(f'\nDefined {n_sectors} sectors:')
        for i in range(n_sectors):
            surf = self.sector_surfaces[i] if i < len(self.sector_surfaces) else '?'
            fric = SURFACE_DEFAULTS.get(surf, 1.0)
            print(f'  Sector{i}: [{self.sector_pnts[i]}→{self.sector_pnts[i+1]}] '
                  f'surface={surf}, friction={fric}')

    @property
    def out_path(self):
        return self.out_dir


if __name__ == '__main__':
    track_csv = os.path.join(track_path, params['track_name'])
    tuner = SectorTuner(track_csv_path=track_csv, out_dir=out_path)
    tuner.run()
