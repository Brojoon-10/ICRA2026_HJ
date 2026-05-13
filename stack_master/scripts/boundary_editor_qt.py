#!/usr/bin/env python3
"""Boundary editor with multi-selection (PyQt5 + pyqtgraph).

Standalone tool for heuristic boundary adjustment. Supports both:
  - Raw boundary CSV  (9 cols: right_bound_x/y/z, left_bound_x/y/z, center_x/y/z)
  - 3D track CSV      (15 cols: s_m, x/y/z, theta/mu/phi, ..., w_right, w_left, ...)

Constraints are preserved by reusing the recompute logic from
interactive_track_editor.py: tangent/normal/widths/arc-length re-derived on save.

Controls (active layer = the layer you act on; toggle with L / R / C keys):
  Left click on a point   : toggle that point in selection
  Shift + left click      : add to selection
  Left drag on empty area : rubber-band box select on active layer
  Drag a SELECTED point   : translate whole selection
                              (Shift held = also translate paired layer = width-preserving)
                              (Ctrl held  = project translation onto local track normal)
  Right click             : context menu — delete (on a point), or pose_smooth.
                              pose_smooth pivots on the point you right-clicked,
                              or on the last point you dragged. Pick R from the
                              slider / 10/20/30 presets / custom prompt, then Apply.
                              With "Width-preserving" on, the per-index delta is
                              also added to the paired boundary and the center
                              so widths are preserved (Ctrl variant projects
                              onto the local track normal).
  Backspace               : delete all selected indices
  D key                   : downsample selection (keep every Nth, prompt for N)
  U key                   : undo
  S key                   : save (backs up original to .bak on first save)
  Esc                     : clear selection
  L / R / C               : set active layer to Left / Right / Center
  P                       : toggle paired-layer view (also draw the non-active boundary)
"""

import os
import sys
import csv
import json
import argparse
import shutil
import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg


pg.setConfigOptions(antialias=True, useOpenGL=False)


# ----------------------------- PCD helpers -----------------------------

def find_matching_pcd(csv_path):
    """Look for a PCD inside the same map folder as the CSV.

    ### HJ : race_stack layout — everything for one map lives in
    ###      stack_master/maps/<map>/. We search only that folder, by name:
    ###        <map_dir>/<stem>.pcd                (e.g. test_gazebo_wall.pcd)
    ###        <map_dir>/<stripped>.pcd            (e.g. test_gazebo_wall.pcd
    ###                                              after stripping _bounds_3d)
    ###        <map_dir>/wall.pcd                  (older race_stack convention)
    ###      No legacy `pcd/` subfolder fallback — simpler & matches the
    ###      "everything in the map folder" rule.

    Returns the first existing path or None.
    """
    csv_path = os.path.abspath(csv_path)
    map_dir = os.path.dirname(csv_path)
    stem = os.path.splitext(os.path.basename(csv_path))[0]
    stripped = stem
    for suf in ('_3d_smoothed', '_bounds_3d', '_3d', '_smoothed'):
        if stripped.endswith(suf):
            stripped = stripped[: -len(suf)]
            break

    for name in (stem, stripped, 'wall'):
        cand = os.path.join(map_dir, name + '.pcd')
        if os.path.exists(cand):
            return cand
    return None


def load_pcd_xy(path, max_points=200000):
    """Minimal PCD reader that returns Nx2 float XY samples.

    Supports ASCII and binary PCD with float32 x/y/z fields. Falls back to
    open3d if the header layout is unusual. Down-samples by a stride so the
    pyqtgraph scatter stays responsive (≤ `max_points` points).
    """
    with open(path, 'rb') as f:
        header_bytes = b''
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f'Truncated PCD header: {path}')
            header_bytes += line
            if line.strip().lower().startswith(b'data '):
                break
        header_text = header_bytes.decode('latin-1')
        body_offset = f.tell()
        data_kind = header_text.strip().split()[-1].lower()

    # Parse minimal fields.
    fields, sizes, types_, counts = [], [], [], []
    points_count = None
    for line in header_text.splitlines():
        s = line.strip()
        if s.lower().startswith('fields'):
            fields = s.split()[1:]
        elif s.lower().startswith('size'):
            sizes = [int(x) for x in s.split()[1:]]
        elif s.lower().startswith('type'):
            types_ = s.split()[1:]
        elif s.lower().startswith('count'):
            counts = [int(x) for x in s.split()[1:]]
        elif s.lower().startswith('points'):
            points_count = int(s.split()[1])

    if not fields or 'x' not in fields or 'y' not in fields:
        raise ValueError(f'PCD missing x/y fields: {path}')

    ix, iy = fields.index('x'), fields.index('y')

    if data_kind == 'ascii':
        arr = np.loadtxt(path, skiprows=len(header_bytes.splitlines()), dtype=np.float32,
                         comments=None)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        xy = arr[:, [ix, iy]].astype(np.float32)
    elif data_kind == 'binary':
        # Build numpy dtype from fields/sizes/types/counts.
        dtype_parts = []
        if not counts:
            counts = [1] * len(fields)
        if not types_:
            types_ = ['F'] * len(fields)
        if not sizes:
            sizes = [4] * len(fields)
        type_map = {('F', 4): 'f4', ('F', 8): 'f8',
                    ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4', ('I', 8): 'i8',
                    ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4', ('U', 8): 'u8'}
        for f_name, sz, tp, cnt in zip(fields, sizes, types_, counts):
            np_kind = type_map.get((tp.upper(), sz))
            if np_kind is None:
                raise ValueError(f'Unsupported PCD field {f_name} type={tp} size={sz}')
            if cnt == 1:
                dtype_parts.append((f_name, np_kind))
            else:
                dtype_parts.append((f_name, np_kind, (cnt,)))
        dtype = np.dtype(dtype_parts)
        with open(path, 'rb') as f:
            f.seek(body_offset)
            buf = f.read()
        arr = np.frombuffer(buf, dtype=dtype, count=points_count or -1)
        xy = np.column_stack([arr['x'].astype(np.float32),
                              arr['y'].astype(np.float32)])
    else:
        raise ValueError(f'Unsupported PCD DATA kind "{data_kind}" in {path}')

    # Drop non-finite rows.
    mask = np.isfinite(xy).all(axis=1)
    xy = xy[mask]

    # Stride downsample to keep the scatter snappy.
    if len(xy) > max_points:
        stride = max(1, len(xy) // max_points)
        xy = xy[::stride]
    return xy


# ----------------------------- IO helpers -----------------------------

BOUND_HEADER_9 = [
    'right_bound_x', 'right_bound_y', 'right_bound_z',
    'left_bound_x', 'left_bound_y', 'left_bound_z',
    'center_x', 'center_y', 'center_z',
]
BOUND_HEADER_6 = [
    'right_bound_x', 'right_bound_y', 'right_bound_z',
    'left_bound_x', 'left_bound_y', 'left_bound_z',
]


def load_track_csv(path):
    """Return dict with: n, left, right, center (Nx3), fmt, header, raw_data."""
    with open(path) as f:
        header = f.readline().strip().split(',')
    data = np.genfromtxt(path, delimiter=',', skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    out = {'header': header, 'raw_data': data, 'n': len(data)}
    if header and header[0] == 's_m':
        out['fmt'] = '3d_track'
        x, y, z = data[:, 1], data[:, 2], data[:, 3]
        theta, mu, phi = data[:, 4], data[:, 5], data[:, 6]
        w_right, w_left = data[:, 10], data[:, 11]
        ct, st = np.cos(theta), np.sin(theta)
        cm, sm = np.cos(mu), np.sin(mu)
        cp, sp = np.cos(phi), np.sin(phi)
        nx = ct * sm * sp - st * cp
        ny = st * sm * sp + ct * cp
        nz = cm * sp
        out['center'] = np.column_stack([x, y, z])
        out['left']   = np.column_stack([x + w_left * nx, y + w_left * ny, z + w_left * nz])
        out['right']  = np.column_stack([x + w_right * nx, y + w_right * ny, z + w_right * nz])
        out['has_center_col'] = True
    elif header and header[0] == 'right_bound_x':
        out['fmt'] = 'boundary'
        out['right'] = data[:, 0:3].copy()
        out['left']  = data[:, 3:6].copy()
        if data.shape[1] >= 9:
            out['center'] = data[:, 6:9].copy()
            out['has_center_col'] = True
        else:
            out['center'] = (out['left'] + out['right']) / 2.0
            out['has_center_col'] = False
    else:
        out['fmt'] = 'boundary'
        out['right'] = data[:, 0:3].copy()
        out['left']  = data[:, 3:6].copy()
        out['center'] = (out['left'] + out['right']) / 2.0
        out['has_center_col'] = False
    out['dirty']  = np.zeros(out['n'], dtype=bool)
    out['orig_n'] = out['n']
    return out


def save_track_csv(path, state, csv_path_for_backup=None):
    """Persist state back to CSV in its original format. Backs up once.

    For 3D track format, if no row is marked dirty AND the row count is
    unchanged, the raw data is written verbatim (zero drift). Otherwise the
    full recompute is done (s_m, Euler angles, widths, derivatives)."""
    if csv_path_for_backup and not os.path.exists(csv_path_for_backup + '.bak'):
        shutil.copy2(csv_path_for_backup, csv_path_for_backup + '.bak')

    n = state['n']
    if state['fmt'] == '3d_track':
        dirty = state.get('dirty')
        unchanged_n = (state.get('orig_n') == n)
        if dirty is not None and unchanged_n and not bool(dirty.any()):
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(state['header'])
                for row in state['raw_data']:
                    w.writerow([f"{v:.6f}" for v in row])
            return

        from scipy.spatial.transform import Rotation
        center = state['center']; left = state['left']; right = state['right']
        cx, cy, cz = center[:, 0], center[:, 1], center[:, 2]

        tang = np.diff(center, axis=0)
        tang = tang / np.linalg.norm(tang, axis=1, keepdims=True)
        tang = np.vstack([tang, tang[-1:]])

        left_dir = left - center
        left_dist = np.linalg.norm(left_dir, axis=1, keepdims=True)
        left_dist = np.where(left_dist < 1e-9, 1.0, left_dist)
        normal = left_dir / left_dist
        ortho = np.cross(tang, normal, axis=1)
        ortho = ortho / np.linalg.norm(ortho, axis=1, keepdims=True)

        rot = np.stack((tang, normal, ortho), axis=1)
        euler = -Rotation.from_matrix(rot).as_euler('zyx')
        euler = np.unwrap(euler, axis=0)
        theta, mu, phi = euler[:, 0], euler[:, 1], euler[:, 2]

        ct, st = np.cos(theta), np.sin(theta)
        cm, sm = np.cos(mu), np.sin(mu)
        cp, sp = np.cos(phi), np.sin(phi)
        nx = ct * sm * sp - st * cp
        ny = st * sm * sp + ct * cp
        nz = cm * sp

        w_left = ((left[:, 0] - cx) * nx + (left[:, 1] - cy) * ny + (left[:, 2] - cz) * nz)
        w_right = ((right[:, 0] - cx) * nx + (right[:, 1] - cy) * ny + (right[:, 2] - cz) * nz)

        ds = np.sqrt(np.sum(np.diff(center, axis=0) ** 2, axis=1))
        s_m = np.concatenate([[0], np.cumsum(ds)])
        deuler = np.diff(euler, axis=0) / np.maximum(ds, 1e-9)[:, None]
        deuler = np.vstack([deuler, deuler[-1:]])

        # Resize raw_data if length changed (deletions)
        raw = state['raw_data']
        if raw.shape[0] != n:
            new = np.zeros((n, raw.shape[1]))
            new[:, 12:] = 0.0
            raw = new
        else:
            raw = raw.copy()
        raw[:, 0] = s_m
        raw[:, 1] = cx; raw[:, 2] = cy; raw[:, 3] = cz
        raw[:, 4] = theta; raw[:, 5] = mu; raw[:, 6] = phi
        raw[:, 7:10] = deuler
        raw[:, 10] = w_right
        raw[:, 11] = w_left

        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(state['header'])
            for row in raw:
                w.writerow([f"{v:.6f}" for v in row])
    else:
        header = BOUND_HEADER_9 if state.get('has_center_col', True) else BOUND_HEADER_6
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            for i in range(n):
                row = [
                    f"{state['right'][i,0]:.6f}", f"{state['right'][i,1]:.6f}", f"{state['right'][i,2]:.6f}",
                    f"{state['left'][i,0]:.6f}",  f"{state['left'][i,1]:.6f}",  f"{state['left'][i,2]:.6f}",
                ]
                if state.get('has_center_col', True):
                    row += [f"{state['center'][i,0]:.6f}", f"{state['center'][i,1]:.6f}", f"{state['center'][i,2]:.6f}"]
                w.writerow(row)


# ----------------------------- Wheel-scrollable menu -----------------------------

class WheelMenu(QtWidgets.QMenu):
    """QMenu that forwards wheel events to a target QSpinBox.

    Lets the user tweak a value (e.g. pose_smooth R) by scrolling the wheel
    anywhere over the open menu — useful when the menu's QSlider isn't
    accepting drag events. Holding Shift multiplies the step.
    """

    def __init__(self, parent=None, target_spinbox=None, base_step=1, shift_step=5):
        super().__init__(parent)
        self._wheel_target = target_spinbox
        self._wheel_base = base_step
        self._wheel_shift = shift_step

    def set_wheel_target(self, spinbox, base_step=1, shift_step=5):
        self._wheel_target = spinbox
        self._wheel_base = base_step
        self._wheel_shift = shift_step

    def wheelEvent(self, ev):
        spin = self._wheel_target
        if spin is None:
            super().wheelEvent(ev); return
        delta = ev.angleDelta().y()
        if delta == 0:
            super().wheelEvent(ev); return
        step = self._wheel_shift if (ev.modifiers() & QtCore.Qt.ShiftModifier) else self._wheel_base
        sign = 1 if delta > 0 else -1
        new_val = int(np.clip(spin.value() + sign * step, spin.minimum(), spin.maximum()))
        spin.setValue(new_val)
        ev.accept()


# ----------------------------- Custom plot widget -----------------------------

class EditorPlot(pg.PlotWidget):
    """PlotWidget with rubber-band selection on left-drag in empty area
    and selection-translation on left-drag of a selected point."""

    sigBoxSelect    = QtCore.pyqtSignal(object, object)  # (rect_in_data, modifiers)
    sigPointClicked = QtCore.pyqtSignal(int, object)      # (index, modifiers)
    sigContextMenu  = QtCore.pyqtSignal(object, object)  # (idx_or_None, global_pos)
    sigDragStart    = QtCore.pyqtSignal(int, object, object)  # (idx, scene_pos, modifiers)
    sigDragMove     = QtCore.pyqtSignal(object, object)       # (delta_data_xy, modifiers)
    sigDragEnd      = QtCore.pyqtSignal()

    PICK_R_PIX = 10  # screen pixels

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackground('w')
        self.setAspectLocked(True)
        self.showGrid(x=True, y=True, alpha=0.3)
        self.getPlotItem().setMenuEnabled(False)

        # State
        self._mode = None  # None | 'box' | 'drag'
        self._press_scene = None
        self._press_data = None
        self._press_modifiers = None
        self._drag_idx = None
        self._last_drag_data = None

        # Rubber band rect overlay
        self._rubber = QtWidgets.QRubberBand(QtWidgets.QRubberBand.Rectangle, self.viewport())

        # Hit-test callback set by main window
        self.hit_test = None  # function(scene_pos, pix_radius) -> idx or None

    def _scene_to_data(self, scene_pos):
        vb = self.getPlotItem().getViewBox()
        return vb.mapSceneToView(scene_pos)

    def mousePressEvent(self, ev):
        if ev.button() == QtCore.Qt.LeftButton:
            scene_pos = self.mapToScene(ev.pos())
            mods = ev.modifiers()
            idx = self.hit_test(scene_pos, self.PICK_R_PIX) if self.hit_test else None
            self._press_scene = ev.pos()
            self._press_data = self._scene_to_data(scene_pos)
            self._press_modifiers = mods
            if idx is not None:
                self._mode = 'maybe_drag'
                self._drag_idx = idx
                self._last_drag_data = self._press_data
                ev.accept()
                return
            else:
                self._mode = 'box'
                self._rubber.setGeometry(QtCore.QRect(self._press_scene, QtCore.QSize()))
                self._rubber.show()
                ev.accept()
                return
        elif ev.button() == QtCore.Qt.RightButton:
            scene_pos = self.mapToScene(ev.pos())
            idx = self.hit_test(scene_pos, self.PICK_R_PIX) if self.hit_test else None
            self.sigContextMenu.emit(idx, ev.globalPos())
            ev.accept()
            return
        elif ev.button() == QtCore.Qt.MiddleButton:
            # let pyqtgraph handle pan
            super().mousePressEvent(ev)
            return
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self._mode == 'box':
            rect = QtCore.QRect(self._press_scene, ev.pos()).normalized()
            self._rubber.setGeometry(rect)
            ev.accept()
            return
        if self._mode == 'maybe_drag':
            if (ev.pos() - self._press_scene).manhattanLength() > 3:
                self._mode = 'drag'
                self.sigDragStart.emit(self._drag_idx, self._press_scene, self._press_modifiers)
            else:
                ev.accept()
                return
        if self._mode == 'drag':
            scene_pos = self.mapToScene(ev.pos())
            data_pt = self._scene_to_data(scene_pos)
            delta = (data_pt.x() - self._last_drag_data.x(),
                     data_pt.y() - self._last_drag_data.y())
            self._last_drag_data = data_pt
            self.sigDragMove.emit(delta, ev.modifiers())
            ev.accept()
            return
        super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if ev.button() == QtCore.Qt.LeftButton:
            if self._mode == 'box':
                self._rubber.hide()
                rel_data = self._scene_to_data(self.mapToScene(ev.pos()))
                x0, x1 = sorted([self._press_data.x(), rel_data.x()])
                y0, y1 = sorted([self._press_data.y(), rel_data.y()])
                rect = QtCore.QRectF(x0, y0, x1 - x0, y1 - y0)
                self.sigBoxSelect.emit(rect, self._press_modifiers)
                self._mode = None
                ev.accept()
                return
            if self._mode == 'maybe_drag':
                # treat as click
                self.sigPointClicked.emit(self._drag_idx, self._press_modifiers)
                self._mode = None
                self._drag_idx = None
                ev.accept()
                return
            if self._mode == 'drag':
                self.sigDragEnd.emit()
                self._mode = None
                self._drag_idx = None
                ev.accept()
                return
        super().mouseReleaseEvent(ev)


# ----------------------------- Pipeline worker (Step 3 + Step 4) -----------------------------

class _PipelineThread(QtCore.QThread):
    """Background thread that runs run_pipeline.sh's Step 3 (gen3D) and/or
    Step 4 (smooth) using `Track3D` directly.

    `steps` is a list of (kind, in_path, out_path) tuples where kind is
    one of {'gen3d', 'smooth'}. Emits `progress(str)` between steps,
    `done(str)` on success and `failed(str)` on exception. The owner must
    keep a strong reference to the thread until `finished` fires.
    """

    progress = QtCore.pyqtSignal(str)
    done = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, track3d_cls, steps, weights, step_size, parent=None):
        super().__init__(parent)
        self._cls = track3d_cls
        self._steps = list(steps)
        self._weights = dict(weights)
        self._step_size = float(step_size)

    def run(self):
        import time
        import subprocess
        try:
            outs = []
            t_total = time.time()
            for kind, in_path, out_path in self._steps:
                t0 = time.time()
                if kind == 'regen_center':
                    # Step 2.9 from run_pipeline.sh — interactive matplotlib window
                    # with `Midpoint reg` and `Tangent⊥Rib` sliders. Blocks until
                    # the user clicks Save & Close.
                    here = os.path.dirname(os.path.abspath(__file__))
                    script = os.path.join(here, 'regen_center.py')
                    ### HJ : sanity — script must exist before subprocess fails
                    ###      with a useless exit code 2.
                    if not os.path.isfile(script):
                        raise RuntimeError(
                            f'regen_center.py missing at {script}. '
                            f'Place it next to boundary_editor_qt.py.')
                    self.progress.emit(
                        'regen_center: adjust reg / tangent⊥rib sliders, then Save & Close …')
                    ### HJ : capture stdout/stderr so the real reason for any
                    ###      non-zero exit (missing module, X display, the
                    ###      script's own argparse complaint, etc.) reaches
                    ###      the user instead of a bare "exited with code N".
                    proc = subprocess.run(
                        [sys.executable, script, in_path],
                        capture_output=True, text=True)
                    if proc.returncode != 0:
                        tail = (proc.stderr or proc.stdout or '').strip()
                        if len(tail) > 1200:
                            tail = '…' + tail[-1200:]
                        raise RuntimeError(
                            f'regen_center.py exited with code '
                            f'{proc.returncode}\n\n{tail}')
                elif kind == 'gen3d':
                    self.progress.emit(
                        f'gen3D: {os.path.basename(in_path)} → {os.path.basename(out_path)} …')
                    self._cls().generate_3d_from_3d_track_bounds(
                        path=in_path, out_path=out_path,
                        ignore_banking=False, visualize=False)
                elif kind == 'smooth':
                    self.progress.emit(
                        f'smooth: {os.path.basename(in_path)} → {os.path.basename(out_path)}  '
                        '(NLP, ~tens of seconds) …')
                    self._cls().smooth_track(
                        out_path=out_path, weights=self._weights,
                        in_path=in_path, step_size=self._step_size,
                        visualize=False)
                else:
                    raise ValueError(f'Unknown pipeline step: {kind}')
                outs.append((kind, out_path, time.time() - t0))
            lines = [f'  {kind}: {out_path}  ({dt:.1f}s)'
                     for kind, out_path, dt in outs]
            self.done.emit(
                f'{len(outs)} step(s) in {time.time() - t_total:.1f}s\n'
                + '\n'.join(lines))
        except Exception as e:
            import traceback
            self.failed.emit(f'{type(e).__name__}: {e}\n\n{traceback.format_exc()}')


# ----------------------------- Main window -----------------------------

class BoundaryEditor(QtWidgets.QMainWindow):
    LAYER_COLORS = {
        'left':   (40, 180, 80),
        'right':  (220, 60, 60),
        'center': (60, 100, 220),
    }

    def __init__(self, csv_path):
        super().__init__()
        self.csv_path = csv_path
        self.state = load_track_csv(csv_path)
        self.history = []  # list of (left, right, center, raw_data) tuples
        self.selected = set()  # indices into current layer arrays
        self.active = 'left'
        self.show_other_layers = True

        # pub_track-style smoothing (applied on demand from right-click menu).
        self.smooth_resolution = 10   # neighbors on each side; pub_track default is 10/20/30
        self.width_preserve = False   # propagate per-index delta to paired layer + center (off by default so a local edit stays local)
        # Auto-align rib⊥tangent: when ON, every drag end or pose_smooth Apply
        # triggers a global rib realignment. That's convenient but means a
        # local edit can shift the pairing far away — by default we leave it
        # OFF so per-point tweaks only move what the user actually touched.
        # The 'Full Realign' button always runs the global pass on demand.
        self.auto_align_rib = False
        self._last_drag_idx = None    # pivot for "smooth around last-moved point"

        # Wrap-around: enabled automatically when the CSV looks like a closed loop
        # (last point ≈ first point on every layer). Effective length skips the
        # duplicate tail point so modular indexing matches pub_track's behaviour.
        self.wrap_around, self._n_effective = self._detect_closed_loop()

        # Drag-time bookkeeping.
        self._drag_base = None
        self._cached_normals_xy = None

        # Active preview pivot — set while the right-click menu is open so that
        # slider/spinbox/wheel changes update the highlighted window in place.
        self._preview_pivot = None

        self.setWindowTitle(f'Boundary Editor — {os.path.basename(csv_path)}')
        self.resize(1400, 900)

        self._build_ui()
        # Permanent status-bar label for ⊥err mean/max — survives showMessage()
        # calls that report transient actions (drag end, smoothing, etc.).
        self.perp_err_label = QtWidgets.QLabel()
        self.perp_err_label.setStyleSheet(
            'QLabel { padding: 0 8px; font-weight: 600; }')
        self.statusBar().addPermanentWidget(self.perp_err_label)
        # Optional same-name PCD overlay (drawn behind boundaries). Missing → silent.
        self._load_optional_pcd()
        self._configure_pipeline_button()
        self._pipeline_thread = None  # active QThread, if any
        self._refresh()
        self.statusBar().showMessage(self._status_text())

    # ---------- UI ----------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        v = QtWidgets.QVBoxLayout(central)
        v.setContentsMargins(4, 4, 4, 4)

        # Toolbar row
        h = QtWidgets.QHBoxLayout()
        self.layer_combo = QtWidgets.QComboBox()
        self.layer_combo.addItems(['left', 'right', 'center'])
        self.layer_combo.currentTextChanged.connect(self._set_active)
        h.addWidget(QtWidgets.QLabel('Active layer:'))
        h.addWidget(self.layer_combo)

        self.show_other_cb = QtWidgets.QCheckBox('Show other layers')
        self.show_other_cb.setChecked(True)
        self.show_other_cb.toggled.connect(self._toggle_other_layers)
        h.addWidget(self.show_other_cb)

        self.show_ribs_cb = QtWidgets.QCheckBox('Show ribs (L↔R)')
        self.show_ribs_cb.setChecked(True)
        self.show_ribs_cb.toggled.connect(lambda _: self._refresh())
        h.addWidget(self.show_ribs_cb)

        self.width_cb = QtWidgets.QCheckBox('Width-preserving')
        self.width_cb.setChecked(self.width_preserve)
        self.width_cb.setToolTip(
            'When ON, dragging a boundary point and applying pose_smooth\n'
            'also move the paired boundary and the centerline by the same\n'
            'per-index delta, so the track widths stay the same.')
        self.width_cb.toggled.connect(self._set_width_preserve)
        h.addWidget(self.width_cb)

        self.wrap_cb = QtWidgets.QCheckBox('Wrap-around (closed loop)')
        self.wrap_cb.setChecked(self.wrap_around)
        self.wrap_cb.setToolTip(
            'Treat the CSV as a closed loop: pose_smooth indexes wrap modulo N.\n'
            'Auto-detected by comparing the first and last point on every layer;\n'
            'turn off if the CSV is an open segment.')
        self.wrap_cb.toggled.connect(self._set_wrap_around)
        h.addWidget(self.wrap_cb)

        self.align_cb = QtWidgets.QCheckBox('Auto-align rib⊥tangent')
        self.align_cb.setChecked(self.auto_align_rib)
        self.align_cb.setToolTip(
            'ON: after every drag end and pose_smooth Apply, run a global\n'
            'rib⊥tangent realignment automatically. Convenient but means a\n'
            'local edit can shift far-away pairings.\n'
            'OFF (default): per-point edits stay local. Use the Full Realign\n'
            'button to run the global pass on demand.')
        self.align_cb.toggled.connect(self._set_auto_align)
        h.addWidget(self.align_cb)

        self.btn_full_realign = QtWidgets.QPushButton('Full Realign')
        self.btn_full_realign.setToolTip(
            'Globally rebuild the rib⊥tangent pairing across the whole track:\n'
            ' both boundaries are resampled along each other\'s normals in a\n'
            ' fixed-point loop. Shapes are preserved (linear resample on the\n'
            ' existing polylines). Use after large edits or before pipeline.')
        self.btn_full_realign.clicked.connect(self._full_realign)
        h.addWidget(self.btn_full_realign)

        self.heat_cb = QtWidgets.QCheckBox('Show ⊥err heat')
        self.heat_cb.setChecked(True)
        self.heat_cb.setToolTip(
            'Color the centerline by |tangent⊥rib| angle in degrees.\n'
            'Green = small error, red = large. The 8 worst offenders are\n'
            'circled in yellow so you can find them at a glance.')
        self.heat_cb.toggled.connect(lambda _=False: self._refresh())
        h.addWidget(self.heat_cb)

        h.addStretch(1)
        h.addWidget(QtWidgets.QLabel('  Selected:'))
        self.sel_label = QtWidgets.QLabel('0')
        h.addWidget(self.sel_label)

        btn_save = QtWidgets.QPushButton('Save (S)')
        btn_save.clicked.connect(self._save)
        h.addWidget(btn_save)

        # Run the downstream pipeline (Step 3 gen3D when needed, then Step 4 smooth).
        # Disabled when the current CSV is already a smoothed output.
        self.btn_run_pipeline = QtWidgets.QPushButton('Run Pipeline')
        self.btn_run_pipeline.setToolTip(
            'Re-run the post-edit pipeline on the current CSV:\n'
            '  • *_bounds_3d.csv → regen_center (Step 2.9, interactive) →\n'
            '                      gen3D (Step 3) → smooth (Step 4)\n'
            '  • *_3d.csv         → smooth (Step 4)\n'
            '  • *_smoothed.csv   → (already the final output; button disabled)')
        self.btn_run_pipeline.clicked.connect(self._on_run_pipeline)
        h.addWidget(self.btn_run_pipeline)

        btn_undo = QtWidgets.QPushButton('Undo (U)')
        btn_undo.clicked.connect(self._undo)
        h.addWidget(btn_undo)

        btn_decim = QtWidgets.QPushButton('Downsample (D)')
        btn_decim.clicked.connect(self._downsample_selection)
        h.addWidget(btn_decim)

        btn_del = QtWidgets.QPushButton('Delete (⌫)')
        btn_del.clicked.connect(self._delete_selection)
        h.addWidget(btn_del)
        v.addLayout(h)

        # Plot
        self.plot = EditorPlot()
        v.addWidget(self.plot, 1)

        # Plot items
        # Optional PCD background — drawn first so boundaries render on top.
        self.scatter_pcd = pg.ScatterPlotItem(
            pen=None,
            brush=pg.mkBrush(160, 160, 160, 120),
            size=2,
            pxMode=True,
        )
        self.scatter_pcd.setZValue(-10)
        self.plot.addItem(self.scatter_pcd)

        self.line_left   = self.plot.plot([], [], pen=pg.mkPen(self.LAYER_COLORS['left'],   width=1.5))
        self.line_right  = self.plot.plot([], [], pen=pg.mkPen(self.LAYER_COLORS['right'],  width=1.5))
        self.line_center = self.plot.plot([], [], pen=pg.mkPen(self.LAYER_COLORS['center'], width=1.0, style=QtCore.Qt.DashLine))

        # Rib lines as a single ScatterPlotItem-like polyline batch using PlotCurveItem
        self.ribs = pg.PlotCurveItem(pen=pg.mkPen(160, 160, 160, 80, width=0.5), connect='pairs')
        self.plot.addItem(self.ribs)

        self.scatter_left   = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(self.LAYER_COLORS['left']),   size=6)
        self.scatter_right  = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(self.LAYER_COLORS['right']),  size=6)
        self.scatter_center = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(self.LAYER_COLORS['center']), size=4)
        self.scatter_sel    = pg.ScatterPlotItem(pen=pg.mkPen('y', width=1.5), brush=None, size=12)
        # ⊥err heat overlay on the centerline: filled markers colored by the
        # |tangent⊥rib| angle (degrees). Sits above the regular center scatter
        # but below the selection rim.
        self.scatter_err_heat = pg.ScatterPlotItem(pen=None, size=10, pxMode=True)
        # Big yellow rings on the top-K worst offenders so they're easy to find
        # even when zoomed out.
        self.scatter_err_top  = pg.ScatterPlotItem(
            pen=pg.mkPen((255, 200, 0), width=2.5), brush=None, size=22, pxMode=True)
        # Window preview shown while the right-click menu is open.
        # Magenta filled markers over the [pivot-R, pivot+R] window on the
        # active layer; the pivot itself uses a slightly larger crosshair.
        self.scatter_preview = pg.ScatterPlotItem(
            pen=pg.mkPen((200, 30, 200), width=1.2),
            brush=pg.mkBrush(200, 30, 200, 120), size=10)
        self.scatter_preview_pivot = pg.ScatterPlotItem(
            pen=pg.mkPen((200, 30, 200), width=2.0),
            brush=None, size=18, symbol='+')
        for it in (self.scatter_left, self.scatter_right, self.scatter_center,
                   self.scatter_err_heat, self.scatter_err_top,
                   self.scatter_sel, self.scatter_preview, self.scatter_preview_pivot):
            self.plot.addItem(it)

        # Plot signals
        self.plot.hit_test = self._hit_test
        self.plot.sigBoxSelect.connect(self._on_box_select)
        self.plot.sigPointClicked.connect(self._on_point_click)
        self.plot.sigContextMenu.connect(self._on_context_menu)
        self.plot.sigDragStart.connect(self._on_drag_start)
        self.plot.sigDragMove.connect(self._on_drag_move)
        self.plot.sigDragEnd.connect(self._on_drag_end)

    # ---------- helpers ----------
    def _layer_arr(self, name=None):
        return self.state[name or self.active]

    def _load_optional_pcd(self):
        """Best-effort overlay of the matching PCD.

        ### HJ : PCD is optional — if the map folder doesn't have a matching
        ###      .pcd file (or it's unreadable), tell the user via the status
        ###      bar but never block the rest of the editor. Every other
        ###      feature (boundary editing, pipeline, save) works without it.
        """
        path = find_matching_pcd(self.csv_path)
        if path is None:
            map_dir = os.path.dirname(os.path.abspath(self.csv_path))
            self.statusBar().showMessage(
                f'No PCD overlay found in {map_dir} — editor runs without it.')
            return
        try:
            xy = load_pcd_xy(path)
        except Exception as e:
            self.statusBar().showMessage(
                f'PCD found but skipped ({os.path.basename(path)}): {e}')
            return
        if len(xy) == 0:
            self.statusBar().showMessage(
                f'PCD {os.path.basename(path)} contained 0 points — no overlay.')
            return
        self.scatter_pcd.setData(xy[:, 0], xy[:, 1])
        # Keep PCD visible but unobtrusive; user still operates the CSV layers.
        self.statusBar().showMessage(
            f'PCD overlay: {os.path.basename(path)}  ({len(xy)} pts)')

    def _detect_closed_loop(self):
        """Return (wrap_enabled, n_effective).

        A CSV is treated as a closed loop when, for every layer, the first and
        last XY point are within 1/10 of the median segment length. When the
        last point is essentially a duplicate of the first we drop it from the
        modular index (n_effective = n - 1) so wrap math doesn't divide by zero.
        """
        n = self.state['n']
        if n < 4:
            return False, n
        thr_factor = 0.1  # gap must be < 10% of median step to count as duplicate
        n_eff = n
        for layer in ('left', 'right', 'center'):
            P = self.state[layer]
            seg = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)
            med = np.median(seg) if len(seg) else 0.0
            if med <= 0:
                return False, n
            gap = np.linalg.norm(P[0, :2] - P[-1, :2])
            if gap > thr_factor * med:
                # Layers disagree on closure → safer not to wrap.
                return False, n
        # All layers close — assume last point duplicates the first.
        return True, n - 1

    def _wrap_idx(self, i):
        """Wrap an index into [0, n_effective) when wrap is on; otherwise clamp."""
        if self.wrap_around:
            return int(i) % self._n_effective
        return int(np.clip(i, 0, self.state['n'] - 1))

    def _window_indices_for_preview(self, pivot, R):
        """Compute the index list that pose_smooth would touch — same logic
        as _apply_smoothing_at but without modifying anything. Returns
        (pivot_idx, window_idx_array, clamped_R) or (None, None, 0) if R<1.
        """
        n = self.state['n']
        R = int(max(1, R))
        if self.wrap_around:
            n_eff = self._n_effective
            R = min(R, (n_eff - 1) // 2)
            if R < 1:
                return None, None, 0
            idx_pivot = int(pivot) % n_eff
            window = np.array([(idx_pivot - R + i) % n_eff
                               for i in range(2 * R + 1)], dtype=int)
        else:
            R = min(R, pivot, n - 1 - pivot)
            if R < 1:
                return None, None, 0
            idx_pivot = int(pivot)
            window = np.arange(idx_pivot - R, idx_pivot + R + 1, dtype=int)
        return idx_pivot, window, R

    def _show_window_preview(self, pivot, R):
        """Highlight the [pivot-R, pivot+R] window on the active layer."""
        idx_pivot, window, R_eff = self._window_indices_for_preview(pivot, R)
        if window is None:
            self._clear_window_preview()
            return
        arr = self._layer_arr()
        self.scatter_preview.setData(arr[window, 0], arr[window, 1])
        self.scatter_preview_pivot.setData([arr[idx_pivot, 0]], [arr[idx_pivot, 1]])
        self.statusBar().showMessage(
            f'Preview: {self.active} window around idx={idx_pivot} R={R_eff} '
            f'({2*R_eff+1} pts){" [wrap]" if self.wrap_around else ""}')

    def _clear_window_preview(self):
        self.scatter_preview.setData([], [])
        self.scatter_preview_pivot.setData([], [])

    def _set_active(self, name):
        self.active = name
        self.selected.clear()
        self._refresh()

    def _toggle_other_layers(self, on):
        self.show_other_layers = on
        self._refresh()

    def _status_text(self):
        s = self.state
        return (f'{s["fmt"]}  N={s["n"]}  active={self.active}  '
                f'selected={len(self.selected)}  '
                f'file={os.path.basename(self.csv_path)}')

    def _compute_perp_err_deg(self):
        """Same metric as regen_center.py — angle between (R-L) and the
        centerline tangent, in degrees, per index. Wrap-aware. Returns None
        when there aren't enough points to define a tangent.
        """
        L = self.state['left']; R = self.state['right']; C = self.state['center']
        N = self.state['n']
        if N < 3:
            return None
        # Tangent at i via central diff (wrap-aware).
        if self.wrap_around:
            n_eff = max(2, min(self._n_effective, N))
            base = C[:n_eff, :2]
            fwd = np.roll(base, -1, axis=0) - np.roll(base, 1, axis=0)
            fn = np.linalg.norm(fwd, axis=1) + 1e-9
            rib = R[:n_eff, :2] - L[:n_eff, :2]
            rn = np.linalg.norm(rib, axis=1) + 1e-9
            cos_tr = (fwd[:, 0] * rib[:, 0] + fwd[:, 1] * rib[:, 1]) / (fn * rn)
        else:
            xy = C[:, :2]
            fwd = np.empty_like(xy)
            fwd[1:-1] = xy[2:] - xy[:-2]
            fwd[0]  = xy[1] - xy[0]
            fwd[-1] = xy[-1] - xy[-2]
            fn = np.linalg.norm(fwd, axis=1) + 1e-9
            rib = R[:, :2] - L[:, :2]
            rn = np.linalg.norm(rib, axis=1) + 1e-9
            cos_tr = (fwd[:, 0] * rib[:, 0] + fwd[:, 1] * rib[:, 1]) / (fn * rn)
        return np.degrees(np.abs(np.arcsin(np.clip(cos_tr, -1, 1))))

    def _push_history(self):
        self.history.append((
            self.state['left'].copy(),
            self.state['right'].copy(),
            self.state['center'].copy(),
            self.state['raw_data'].copy(),
            self.state['dirty'].copy(),
            self.state['n'],
        ))
        if len(self.history) > 50:
            self.history.pop(0)

    def _undo(self):
        if not self.history:
            self.statusBar().showMessage('Nothing to undo')
            return
        L, R, C, raw, dirty, n = self.history.pop()
        self.state['left'] = L; self.state['right'] = R; self.state['center'] = C
        self.state['raw_data'] = raw; self.state['dirty'] = dirty; self.state['n'] = n
        self.selected = {i for i in self.selected if i < n}
        self._refresh()
        self.statusBar().showMessage(f'Undo. {self._status_text()}')

    # ---------- rendering ----------
    def _refresh(self):
        L = self.state['left']; R = self.state['right']; C = self.state['center']

        if self.show_other_layers or self.active == 'left':
            self.line_left.setData(L[:, 0], L[:, 1])
            self.scatter_left.setData(L[:, 0], L[:, 1])
        else:
            self.line_left.setData([], []); self.scatter_left.setData([], [])

        if self.show_other_layers or self.active == 'right':
            self.line_right.setData(R[:, 0], R[:, 1])
            self.scatter_right.setData(R[:, 0], R[:, 1])
        else:
            self.line_right.setData([], []); self.scatter_right.setData([], [])

        if self.show_other_layers or self.active == 'center':
            self.line_center.setData(C[:, 0], C[:, 1])
            self.scatter_center.setData(C[:, 0], C[:, 1])
        else:
            self.line_center.setData([], []); self.scatter_center.setData([], [])

        # Ribs
        if self.show_ribs_cb.isChecked():
            xs = np.empty(2 * self.state['n']); ys = np.empty(2 * self.state['n'])
            xs[0::2] = L[:, 0]; xs[1::2] = R[:, 0]
            ys[0::2] = L[:, 1]; ys[1::2] = R[:, 1]
            self.ribs.setData(xs, ys)
        else:
            self.ribs.setData([], [])

        # Selection highlight on active layer
        active_arr = self._layer_arr()
        if self.selected:
            idxs = np.array(sorted(self.selected), dtype=int)
            self.scatter_sel.setData(active_arr[idxs, 0], active_arr[idxs, 1])
        else:
            self.scatter_sel.setData([], [])

        # ⊥err heat overlay + permanent label.
        self._refresh_perp_err_overlay()

        self.sel_label.setText(str(len(self.selected)))
        self.statusBar().showMessage(self._status_text())

    def _refresh_perp_err_overlay(self):
        """Update the centerline color-heat by ⊥err and the permanent label
        in the status bar. Called from _refresh on every edit."""
        err = self._compute_perp_err_deg()
        if err is None:
            self.perp_err_label.setText('')
            self.scatter_err_heat.setData([], [])
            self.scatter_err_top.setData([], [])
            return
        mean, mx = float(err.mean()), float(err.max())
        worst_i = int(np.argmax(err))
        self.perp_err_label.setText(
            f'⊥err mean={mean:.2f}°   max={mx:.2f}°   '
            f'worst@idx {worst_i}')

        # Heat overlay on centerline (toggle).
        if not getattr(self, 'heat_cb', None) or not self.heat_cb.isChecked():
            self.scatter_err_heat.setData([], [])
            self.scatter_err_top.setData([], [])
            return
        C = self.state['center']
        # err length may be n_eff (= len(C) - 1 in closed-loop CSVs); we only
        # color the points we have data for.
        n_err = len(err)
        # Color ramp: green at 0°, yellow at 4°, red at 15°+.
        vmax = max(15.0, float(mx))
        colors = self._err_to_brushes(err, vmax)
        spots = [
            {'pos': (C[i, 0], C[i, 1]), 'brush': colors[i]}
            for i in range(n_err)
        ]
        self.scatter_err_heat.setData(spots=spots)
        # Highlight the K worst offenders (only on indices we have err for).
        K = min(8, n_err)
        if n_err > K:
            top_idx = np.argpartition(-err, K - 1)[:K]
        else:
            top_idx = np.arange(n_err)
        # Threshold: don't ring anything below 4° (cosmetic).
        top_idx = [int(i) for i in top_idx if err[i] >= 4.0 and i < len(C)]
        if top_idx:
            self.scatter_err_top.setData(C[top_idx, 0], C[top_idx, 1])
        else:
            self.scatter_err_top.setData([], [])

    @staticmethod
    def _err_to_brushes(err, vmax):
        """Map ⊥err (degrees) to a green→yellow→red gradient brush per point."""
        if vmax < 1e-6:
            vmax = 1.0
        t = np.clip(err / vmax, 0.0, 1.0)
        # piecewise: 0→0.5 green→yellow, 0.5→1 yellow→red
        r = np.where(t < 0.5, t * 2.0, 1.0)
        g = np.where(t < 0.5, 1.0, 1.0 - (t - 0.5) * 2.0)
        b = np.zeros_like(t)
        return [pg.mkBrush(int(r[i] * 255), int(g[i] * 255), int(b[i] * 255), 200)
                for i in range(len(err))]

    # ---------- hit-test ----------
    def _hit_test(self, scene_pos, pix_radius):
        """Find nearest point in active layer within pix_radius pixels of scene_pos.
        Returns index or None."""
        vb = self.plot.getPlotItem().getViewBox()
        data_pt = vb.mapSceneToView(scene_pos)
        arr = self._layer_arr()
        if len(arr) == 0:
            return None
        dx = arr[:, 0] - data_pt.x()
        dy = arr[:, 1] - data_pt.y()
        d2 = dx * dx + dy * dy
        i = int(np.argmin(d2))
        # convert pix_radius to data units using the view's pixel size
        psize = vb.viewPixelSize()
        # use larger of x/y pixel size
        thresh = (max(psize) * pix_radius) ** 2
        if d2[i] <= thresh:
            return i
        return None

    # ---------- selection ----------
    def _on_box_select(self, rect, modifiers):
        arr = self._layer_arr()
        x0, y0, x1, y1 = rect.left(), rect.top(), rect.right(), rect.bottom()
        mask = ((arr[:, 0] >= x0) & (arr[:, 0] <= x1) &
                (arr[:, 1] >= y0) & (arr[:, 1] <= y1))
        new = set(np.flatnonzero(mask).tolist())
        if modifiers & QtCore.Qt.ShiftModifier:
            self.selected |= new
        elif modifiers & QtCore.Qt.ControlModifier:
            self.selected ^= new
        else:
            self.selected = new
        self._refresh()

    def _on_point_click(self, idx, modifiers):
        if modifiers & QtCore.Qt.ShiftModifier:
            self.selected.add(idx)
        elif modifiers & QtCore.Qt.ControlModifier:
            self.selected ^= {idx}
        else:
            self.selected = {idx}
        self._refresh()

    def _on_point_right_click(self, idx):
        self._push_history()
        self._delete_indices([idx])
        self.statusBar().showMessage(f'Deleted index {idx}. {self._status_text()}')

    # ---------- context menu ----------
    def _on_context_menu(self, idx, global_pos):
        """Unified right-click menu — mirrors pub_track's pose_smooth flow.

        Pivot resolution:
          - right-click on a point  → that point is the pivot
          - right-click on empty    → fall back to the last point you dragged

        Scrolling the mouse wheel anywhere over the menu tweaks the R value
        (hold Shift for a larger step).
        """
        menu = WheelMenu(self)

        # Pick the pivot for any smoothing action launched from this menu.
        pivot = idx if idx is not None else self._last_drag_idx

        if idx is not None:
            act_del = menu.addAction(f'Delete point {idx}')
            act_del.triggered.connect(lambda _=False, i=idx: self._on_point_right_click(i))
            menu.addSeparator()

        # Pivot header
        if pivot is None:
            ### HJ : QMenu.addAction(QAction) returns None on some PyQt5
            ###      builds (e.g. Qt 5.12), so chaining .setEnabled(False)
            ###      crashed with AttributeError. Match the safe two-step
            ###      pattern used by the `pivot is not None` branch below.
            hdr_none = QtWidgets.QAction(
                'pose_smooth — drag a point first or right-click one to pick a pivot',
                menu)
            hdr_none.setEnabled(False)
            menu.addAction(hdr_none)
        else:
            hdr = QtWidgets.QAction(f'pose_smooth pivot: idx={pivot}', menu)
            hdr.setEnabled(False)
            menu.addAction(hdr)

            # Width-preserve toggle (carries the paired layer + center).
            act_wp = menu.addAction('Width-preserving (paired + center)')
            act_wp.setCheckable(True)
            act_wp.setChecked(self.width_preserve)
            act_wp.toggled.connect(self._set_width_preserve)

            # Resolution slider — pub_track's `resolution` (each-side neighbor count).
            r_action = self._make_slider_action(
                menu, label='Resolution R (each side)',
                minimum=1, maximum=60, value=self.smooth_resolution,
                on_change=self._set_smooth_resolution,
                fmt=lambda v: f'R={v}  (2R+1={2*v+1} pts)')
            menu.addAction(r_action)
            # Mouse-wheel anywhere over the menu now adjusts R; Shift = bigger step.
            menu.set_wheel_target(r_action._spinbox, base_step=1, shift_step=5)

            # Apply with current slider value.
            act_apply = menu.addAction('Apply pose_smooth')
            act_apply.triggered.connect(
                lambda _=False, p=pivot: self._smooth_with_history(p, self.smooth_resolution))

            # Preset shortcuts that mirror pub_track's 10 / 20 / 30 submenu.
            preset_menu = menu.addMenu('Apply with preset')
            for r in (10, 20, 30):
                a = preset_menu.addAction(f'R = {r}  (2R+1={2*r+1} pts)')
                a.triggered.connect(lambda _=False, rr=r, p=pivot:
                                    self._smooth_with_history(p, rr))

            # Apply with explicit custom length prompt (pub_track's "idx" entry).
            act_custom = menu.addAction('Apply with custom R …')
            act_custom.triggered.connect(lambda _=False, p=pivot: self._apply_smoothing_custom(p))

            # Light up the smoothing window so the user can see what R covers.
            self._preview_pivot = pivot
            self._show_window_preview(pivot, self.smooth_resolution)

        try:
            menu.exec_(global_pos)
        finally:
            self._preview_pivot = None
            self._clear_window_preview()

    def _make_slider_action(self, parent_menu, label, minimum, maximum, value,
                            on_change, fmt='{}'):
        """Build a QWidgetAction with [label : <slider> : <spinbox>].

        Qt menus aggressively close on mouse release, which can swallow slider
        drags. Pairing the slider with a QSpinBox guarantees the user can
        always set the value precisely; the slider remains there for quick
        coarse changes. Both widgets stay synchronised.
        """
        w = QtWidgets.QWidget(parent_menu)
        h = QtWidgets.QHBoxLayout(w)
        h.setContentsMargins(8, 2, 8, 2)
        h.addWidget(QtWidgets.QLabel(label))

        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, w)
        slider.setMinimum(minimum); slider.setMaximum(maximum); slider.setValue(value)
        slider.setFixedWidth(160)
        slider.setFocusPolicy(QtCore.Qt.StrongFocus)
        slider.setTracking(True)
        h.addWidget(slider)

        spin = QtWidgets.QSpinBox(w)
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setFixedWidth(70)
        spin.setKeyboardTracking(True)
        h.addWidget(spin)

        # Optional preview label using the same `fmt` rule.
        def _fmt(v):
            return fmt(v) if callable(fmt) else fmt.format(v)
        readout = QtWidgets.QLabel(_fmt(value), w)
        readout.setMinimumWidth(120)
        h.addWidget(readout)

        # Forward changes from either widget, keep both in sync without echo.
        def _apply(v):
            readout.setText(_fmt(v))
            on_change(int(v))

        def _on_slider(v):
            if spin.value() != v:
                spin.blockSignals(True); spin.setValue(v); spin.blockSignals(False)
            _apply(v)

        def _on_spin(v):
            if slider.value() != v:
                slider.blockSignals(True); slider.setValue(v); slider.blockSignals(False)
            _apply(v)

        slider.valueChanged.connect(_on_slider)
        spin.valueChanged.connect(_on_spin)

        action = QtWidgets.QWidgetAction(parent_menu)
        action.setDefaultWidget(w)
        action._spinbox = spin   # expose for WheelMenu wiring
        return action

    def _set_width_preserve(self, on):
        on = bool(on)
        self.width_preserve = on
        # Keep the toolbar checkbox and the context-menu toggle in sync without
        # bouncing the signal back through us.
        if hasattr(self, 'width_cb') and self.width_cb.isChecked() != on:
            self.width_cb.blockSignals(True)
            self.width_cb.setChecked(on)
            self.width_cb.blockSignals(False)
        self.statusBar().showMessage(
            f'Width-preserving: {"ON" if on else "off"}. {self._status_text()}')

    def _set_auto_align(self, on):
        self.auto_align_rib = bool(on)
        # Pipeline button label depends on whether the editor will skip regen_center.
        if hasattr(self, 'btn_run_pipeline'):
            self._configure_pipeline_button()
        self.statusBar().showMessage(
            f'Auto-align rib⊥tangent: {"ON" if self.auto_align_rib else "off"}.')

    def _set_wrap_around(self, on):
        on = bool(on)
        self.wrap_around = on
        # If toggled on but we didn't detect a closed loop, fall back to N as
        # the effective length (treat last point as a real neighbor of first).
        if on and self._n_effective >= self.state['n']:
            self._n_effective = self.state['n']
        elif on and self._n_effective < self.state['n']:
            pass  # auto-detected closed loop already set this
        else:
            # turning off — keep n_effective consistent for next toggle but the
            # wrap path is gated by self.wrap_around anyway.
            pass
        if hasattr(self, 'wrap_cb') and self.wrap_cb.isChecked() != on:
            self.wrap_cb.blockSignals(True)
            self.wrap_cb.setChecked(on)
            self.wrap_cb.blockSignals(False)
        self.statusBar().showMessage(
            f'Wrap-around: {"ON" if on else "off"} '
            f'(N_eff={self._n_effective if on else self.state["n"]}). '
            f'{self._status_text()}')

    def _set_smooth_resolution(self, n):
        self.smooth_resolution = int(n)
        # Refresh the highlighted window if the context menu is currently open.
        if self._preview_pivot is not None:
            self._show_window_preview(self._preview_pivot, self.smooth_resolution)

    def _apply_smoothing_custom(self, pivot):
        """Prompt for a custom resolution value, then apply (pub_track's `idx` path)."""
        r, ok = QtWidgets.QInputDialog.getInt(
            self, 'pose_smooth — custom R',
            'Neighbors on each side (R). Total points = 2R+1:',
            value=self.smooth_resolution, min=1, max=200)
        if not ok:
            return
        self.smooth_resolution = int(r)
        self._smooth_with_history(pivot, r)

    def _smooth_with_history(self, pivot, resolution):
        """Wrap _apply_smoothing_at with a history snapshot for undo."""
        self._push_history()
        self._apply_smoothing_at(int(pivot), int(resolution))

    # ---------- drag ----------
    def _on_drag_start(self, idx, scene_pos, modifiers):
        if idx not in self.selected:
            # if dragging an unselected point, make it the selection
            self.selected = {idx}
            self._refresh()
        self._push_history()
        # cache local normals for Ctrl-mode (project to track normal)
        self._cached_normals_xy = self._compute_normals_xy()
        # remember the dragged index so right-click can smooth around it later
        self._drag_base = {'idx': int(idx)}

    def _compute_normals_xy(self):
        """Per-index 2D unit normal (perpendicular to centerline tangent)."""
        return self._unit_normals_of(self.state['center'])

    @staticmethod
    def _unit_normals_of(P, smooth_radius=2, wrap=False, n_eff=None):
        """Per-index 2D unit normal of any polyline P (Nx2 or Nx3).

        Smoothing: averages forward-differences over a window of width
        2*smooth_radius+1 so point-to-point noise in P doesn't make the
        normal flip in tight curves.

        Wrap: when True, the polyline is treated as a closed loop of length
        n_eff (defaults to len(P)). The last point may be a duplicate of the
        first — set n_eff to N-1 in that case so it isn't double-counted.
        Tangents and the smoothing window then wrap modulo n_eff.
        """
        n_pts = len(P)
        if n_pts < 2:
            return np.zeros((n_pts, 2))
        xy = P[:, :2]

        if wrap:
            n_eff = int(n_eff if n_eff is not None else n_pts)
            n_eff = max(2, min(n_eff, n_pts))
            base = xy[:n_eff]
            # Forward differences wrap: diffs[i] = base[(i+1) % n_eff] - base[i]
            diffs = np.roll(base, -1, axis=0) - base
            r = max(0, int(smooth_radius))
            if r > 0 and n_eff > 2 * r:
                pad = np.concatenate([diffs[-r:], diffs, diffs[:r]], axis=0)
                sm = np.zeros_like(diffs)
                for off in range(2 * r + 1):
                    sm += pad[off: off + n_eff]
                diffs_s = sm / (2 * r + 1)
            else:
                diffs_s = diffs
            # Tangent at i = 0.5*(diffs_s[i-1] + diffs_s[i]) (also wraps)
            t_eff = 0.5 * (np.roll(diffs_s, 1, axis=0) + diffs_s)
            t = np.empty((n_pts, 2))
            t[:n_eff] = t_eff
            if n_pts > n_eff:
                # duplicate tail point shares the head's tangent
                t[n_eff:] = t_eff[0]
        else:
            diffs = np.diff(xy, axis=0)  # (N-1) x 2
            r = max(0, int(smooth_radius))
            if r > 0 and len(diffs) > (2 * r):
                pad = np.pad(diffs, ((r, r), (0, 0)), mode='edge')
                sm = np.zeros_like(diffs)
                for off in range(2 * r + 1):
                    sm += pad[off: off + len(diffs)]
                diffs_s = sm / (2 * r + 1)
            else:
                diffs_s = diffs
            t = np.empty((n_pts, 2))
            t[1:-1] = 0.5 * (diffs_s[:-1] + diffs_s[1:])
            t[0]  = diffs_s[0]
            t[-1] = diffs_s[-1]
        norm = np.linalg.norm(t, axis=1, keepdims=True)
        norm = np.where(norm < 1e-9, 1.0, norm)
        t = t / norm
        return np.column_stack([-t[:, 1], t[:, 0]])

    # ---------- rib⊥tangent: realign paired boundary along active's normal ----------
    def _align_paired_to_active(self, source_layer=None, smooth_radius=2,
                                search_radius=20, width_ratio_limit=2.0):
        """Re-sample the paired boundary along the active layer's normals so
        that rib (R-L) ⊥ active tangent at every index.

        Method (no centerline assumption):
          - take `source_layer` (active by default) as the truth curve
          - at each index i, draw a ray from source[i] along source's local
            unit normal (both directions)
          - intersect the ray with the paired polyline (continuous, piecewise
            linear) and pick the intersection nearest to the previous paired
            point — this preserves topology around tight curves
          - that intersection becomes the new paired[i]

        Center is reconstructed as (L + R) / 2 to keep things consistent.
        Active is 'left' or 'right'; if the user is editing 'center' we
        instead re-align both L and R along the centerline's normal.
        Returns the number of indices whose paired point moved (0 = no-op).
        """
        src_name = source_layer or self.active
        kw = dict(smooth_radius=smooth_radius,
                  search_radius=search_radius,
                  width_ratio_limit=width_ratio_limit)
        if src_name == 'center':
            moved_l = self._align_polyline_along_normals(
                src='center', dst='left', **kw)
            moved_r = self._align_polyline_along_normals(
                src='center', dst='right', **kw)
            return moved_l + moved_r
        pair_name = 'right' if src_name == 'left' else 'left'
        moved = self._align_polyline_along_normals(
            src=src_name, dst=pair_name, **kw)
        # In raw-boundary CSVs, center is derived; in 3d-track format we
        # still keep it consistent with the new boundary pair before save.
        if self.state['fmt'] == 'boundary' or self.active in ('left', 'right'):
            self.state['center'] = (self.state['left'] + self.state['right']) / 2.0
        return moved

    def _align_polyline_along_normals(self, src, dst,
                                      search_radius=20,
                                      width_ratio_limit=2.0,
                                      smooth_radius=2):
        """Move `dst` polyline so that for every i, dst[i] sits on the ray
        from src[i] along src's normal. `dst` curve shape is preserved
        (resampled along its existing polyline). Returns moved-index count.

        Wrap-aware: when self.wrap_around is on, normals are computed with
        cyclic central differences and the ray-polyline search is allowed to
        cross the head/tail seam.
        """
        S = self.state[src]
        D_old = self.state[dst].copy()
        N = len(S)
        if N < 2 or len(D_old) < 2:
            return 0
        wrap = bool(self.wrap_around)
        n_eff = self._n_effective if wrap else N
        n_src = self._unit_normals_of(S, smooth_radius=smooth_radius,
                                      wrap=wrap, n_eff=n_eff)
        # Build dst polyline edges and arc-length parameterization. With
        # wrap-around, append the seam segment dst[n_eff-1]→dst[0] so the
        # closed loop is searched as a single polyline of (n_eff+1) points.
        if wrap and n_eff < N:
            # CSV already duplicates the tail; D_old[n_eff] == D_old[0] (~).
            pts = D_old[:n_eff + 1, :2]
        elif wrap:
            pts = np.vstack([D_old[:, :2], D_old[0:1, :2]])
        else:
            pts = D_old[:, :2]
        seg_vecs = np.diff(pts, axis=0)
        seg_lens = np.linalg.norm(seg_vecs, axis=1)
        s_cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
        new_dst = D_old.copy()
        moved = 0
        prev_seg = 0
        N_iter = n_eff if wrap else N
        for i in range(N_iter):
            origin = S[i, :2]
            direction = n_src[i]
            hit = self._intersect_ray_polyline(
                origin, direction, pts, s_cum, seg_lens,
                hint_seg=prev_seg, max_search=2 * search_radius + 1,
                window_only=True, wrap=wrap)
            if hit is None:
                continue
            pt_xy, seg_idx = hit
            # Width sanity: avoid collapsing / ballooning the rib (e.g. when
            # a tight U-turn brings the opposite stretch into the search window).
            old_width = np.linalg.norm(D_old[i, :2] - S[i, :2])
            new_width = np.linalg.norm(pt_xy - origin)
            if old_width > 1e-6 and (
                    new_width > old_width * width_ratio_limit
                    or new_width < old_width / width_ratio_limit):
                continue
            # Interpolate z using the same segment. Wrap maps seg_idx == n_eff
            # back to (n_eff-1, 0) for z lookup.
            j0 = seg_idx
            j1 = (seg_idx + 1)
            if wrap and j1 >= n_eff:
                j1 = 0
            t01 = (np.linalg.norm(pt_xy - D_old[j0, :2])
                   / max(seg_lens[seg_idx], 1e-9))
            z_new = D_old[j0, 2] + t01 * (D_old[j1, 2] - D_old[j0, 2])
            if np.linalg.norm(pt_xy - new_dst[i, :2]) > 1e-9:
                moved += 1
            new_dst[i, 0] = pt_xy[0]
            new_dst[i, 1] = pt_xy[1]
            new_dst[i, 2] = z_new
            prev_seg = seg_idx
        # Mirror the duplicate tail point to the head if needed.
        if wrap and n_eff < N:
            new_dst[n_eff:] = new_dst[0]
        self.state[dst] = new_dst
        if moved:
            self.state['dirty'][:] = True
        return moved

    @staticmethod
    def _intersect_ray_polyline(origin, direction, P, s_cum, seg_lens,
                                hint_seg=0, max_search=None,
                                window_only=False, wrap=False):
        """Find an intersection of the infinite line {origin + α·direction}
        (α ∈ ℝ, both directions) with polyline P (Nx2).

        Strategy: scan segments outward from `hint_seg`, keep the candidate
        with the smallest |α| (closest in normal distance). Returns
        (point_xy, segment_index) or None if nothing hits.

        When `window_only` is True, segments outside ±max_search/2 around
        `hint_seg` are not considered — this prevents the search from jumping
        to a far stretch of a closed-loop track.
        """
        n = len(P) - 1
        if n < 1:
            return None
        if max_search is None:
            max_search = n  # full scan; cheap enough for our scale (~280)
        best = None  # (abs_alpha, point, seg_idx)
        if window_only:
            half = max(1, (max_search - 1) // 2)
            if wrap:
                # Closed loop: window can wrap across the seam.
                candidates = [(j % n) for j in range(hint_seg - half,
                                                     hint_seg + half + 1)]
                order = sorted(set(candidates),
                               key=lambda j: min(abs(j - hint_seg),
                                                 n - abs(j - hint_seg)))
            else:
                lo = max(0, hint_seg - half)
                hi = min(n, hint_seg + half + 1)
                order = list(range(lo, hi))
                order.sort(key=lambda j: abs(j - hint_seg))
        else:
            # iterate segments by distance from hint_seg
            order = sorted(range(n), key=lambda j: abs(j - hint_seg))
            order = order[:max_search]
        for j in order:
            A = P[j]; B = P[j + 1]
            seg = B - A
            # Solve A + t·seg = origin + α·direction, for (t, α)
            #   [seg.x   -direction.x] [t]   [origin.x - A.x]
            #   [seg.y   -direction.y] [α] = [origin.y - A.y]
            M00, M01 = seg[0], -direction[0]
            M10, M11 = seg[1], -direction[1]
            det = M00 * M11 - M01 * M10
            if abs(det) < 1e-12:
                continue  # parallel
            r0 = origin[0] - A[0]
            r1 = origin[1] - A[1]
            t = (M11 * r0 - M01 * r1) / det
            alpha = (-M10 * r0 + M00 * r1) / det
            # Segment hit requires t ∈ [0, 1]; we allow tiny over/undershoot.
            if t < -1e-6 or t > 1.0 + 1e-6:
                continue
            t = float(np.clip(t, 0.0, 1.0))
            point = A + t * seg
            cand = (abs(alpha), point, j)
            if best is None or cand[0] < best[0]:
                best = cand
        if best is None:
            return None
        _, point, seg_idx = best
        return point, seg_idx

    def _on_drag_move(self, delta, modifiers):
        dx, dy = delta

        if not self.selected:
            return
        idxs = np.array(sorted(self.selected), dtype=int)

        # Determine direction (Ctrl: normal-only)
        if modifiers & QtCore.Qt.ControlModifier:
            normals = self._cached_normals_xy[idxs]
            # project (dx,dy) onto each point's normal
            proj = normals[:, 0] * dx + normals[:, 1] * dy
            shift_xy = normals * proj[:, None]
        else:
            shift_xy = np.tile([dx, dy], (len(idxs), 1))

        # Apply to active layer
        active = self._layer_arr()
        active[idxs, 0] += shift_xy[:, 0]
        active[idxs, 1] += shift_xy[:, 1]
        self.state['dirty'][idxs] = True

        # Width-preserving: paired layer + center follow the same shift. Honour
        # the toolbar/menu toggle, and treat Shift as a per-drag override so the
        # user can force it on even when the global toggle is off.
        carry_pair = self.width_preserve or bool(modifiers & QtCore.Qt.ShiftModifier)
        if carry_pair:
            pair_name = {'left': 'right', 'right': 'left', 'center': None}[self.active]
            if pair_name is not None:
                pair = self.state[pair_name]
                pair[idxs, 0] += shift_xy[:, 0]
                pair[idxs, 1] += shift_xy[:, 1]
            # also drag center along
            self.state['center'][idxs, 0] += shift_xy[:, 0]
            self.state['center'][idxs, 1] += shift_xy[:, 1]

        self._refresh()

    # ---------- pub_track-style 3-point cubic spline smoothing ----------
    @staticmethod
    def _heading_tangent(P, i):
        """Unit tangent at index i of a 2D polyline P (Nx2-or-more) via central diff.

        Used as the 'heading' tangent for BPoly.from_derivatives.
        """
        n = len(P)
        if n < 2:
            return np.array([1.0, 0.0])
        i_prev = max(i - 1, 0)
        i_next = min(i + 1, n - 1)
        if i_prev == i_next:
            return np.array([1.0, 0.0])
        v = P[i_next, :2] - P[i_prev, :2]
        m = np.linalg.norm(v)
        if m < 1e-9:
            return np.array([1.0, 0.0])
        return v / m

    @staticmethod
    def _sample_cubic_with_derivatives(P_minus, P_pivot, P_plus,
                                       T_minus, T_pivot, T_plus, resolution):
        """Port of pub_track's sampleCubicSplinesWithDerivative for a 2D position
        target (i.e. target == "Pose").

        Builds a cubic Hermite curve through three points with given tangents,
        using chord-length parameterisation, then resamples `2*resolution+1`
        equispaced points along it. Returns (2R+1, 2) ndarray.
        """
        from scipy.interpolate import BPoly  # scipy already in requirements.txt

        points = np.array([P_minus, P_pivot, P_plus], dtype=float)
        tangents = np.array([T_minus, T_pivot, T_plus], dtype=float)

        # Parametrize along chord length, identical to pub_track.
        dp = np.diff(points, axis=0)
        seg = np.linalg.norm(dp, axis=1)
        if seg.sum() < 1e-9:
            return np.tile(P_pivot, (2 * resolution + 1, 1))
        d = np.hstack([[0.0], np.cumsum(seg)])
        l = d[-1]
        n_samples = 2 * resolution + 1
        s = np.linspace(0.0, l, n_samples)

        out = np.zeros((n_samples, 2))
        for dim in range(2):
            yd = list(zip(points[:, dim], tangents[:, dim]))
            poly = BPoly.from_derivatives(d, yd)
            out[:, dim] = poly(s)
        return out

    def _apply_smoothing_at(self, anchor_idx, resolution, modifiers=QtCore.Qt.NoModifier):
        """Smooth the active layer in a window of half-width `resolution` around
        `anchor_idx` using the pub_track-style three-point cubic spline.

        The anchor and both window endpoints are taken from the *current* layer
        geometry (so a freshly dragged anchor sits exactly on the new curve).
        The 2R+1 reassigned points overwrite indices [anchor-R … anchor+R],
        which wrap around modulo N_eff when wrap_around is on (matches pub_track).

        If `width_preserve` is on, the per-index delta on the active layer is
        propagated to the paired layer and center (Ctrl modifier projects the
        delta onto the local track normal).
        """
        n = self.state['n']
        R = int(max(1, resolution))

        if self.wrap_around:
            n_eff = self._n_effective
            # Cap R so the window doesn't fully overlap itself.
            R = min(R, (n_eff - 1) // 2)
            if R < 1:
                self.statusBar().showMessage(
                    f'Smoothing skipped: not enough points for wrap (n_eff={n_eff}).')
                return
            idx_pivot = int(anchor_idx) % n_eff
            idx_minus = (idx_pivot - R) % n_eff
            idx_plus  = (idx_pivot + R) % n_eff
            window_idx = np.array([(idx_pivot - R + i) % n_eff
                                   for i in range(2 * R + 1)], dtype=int)
        else:
            # Clamp R so [anchor-R, anchor+R] stays within the array.
            max_R_left = anchor_idx
            max_R_right = n - 1 - anchor_idx
            R = min(R, max_R_left, max_R_right)
            if R < 1:
                self.statusBar().showMessage(
                    f'Smoothing skipped: anchor {anchor_idx} too close to the polyline end '
                    f'(enable Wrap-around for closed-loop CSVs).')
                return
            idx_pivot = anchor_idx
            idx_minus = anchor_idx - R
            idx_plus  = anchor_idx + R
            window_idx = np.arange(anchor_idx - R, anchor_idx + R + 1, dtype=int)

        active = self._layer_arr()
        base_active = active.copy()

        P_pivot  = active[idx_pivot, :2].copy()
        P_minus  = active[idx_minus, :2].copy()
        P_plus   = active[idx_plus,  :2].copy()

        # Heading tangents from current polyline geometry on the active layer.
        # On a closed loop we want central-diff to also wrap, so build them
        # locally here instead of going through _heading_tangent.
        def tan_at(i):
            if self.wrap_around:
                ip = (i - 1) % n_eff
                inx = (i + 1) % n_eff
            else:
                ip = max(i - 1, 0); inx = min(i + 1, n - 1)
            v = active[inx, :2] - active[ip, :2]
            m = np.linalg.norm(v)
            return v / m if m > 1e-9 else np.array([1.0, 0.0])
        T_pivot = tan_at(idx_pivot)
        T_minus = tan_at(idx_minus)
        T_plus  = tan_at(idx_plus)

        try:
            samples = self._sample_cubic_with_derivatives(
                P_minus, P_pivot, P_plus,
                T_minus, T_pivot, T_plus, R)
        except Exception as e:
            QtWidgets.QMessageBox.warning(
                self, 'Smoothing failed', f'BPoly interpolation error: {e}')
            return

        # Write samples back along the (possibly wrapped) window indices.
        active[window_idx, 0] = samples[:, 0]
        active[window_idx, 1] = samples[:, 1]
        self.state['dirty'][window_idx] = True

        # Constraint propagation: keep widths to the paired layer and center.
        if self.width_preserve and self.active in ('left', 'right'):
            d_xy = active[window_idx, :2] - base_active[window_idx, :2]
            if modifiers & QtCore.Qt.ControlModifier:
                normals = self._compute_normals_xy()[window_idx]
                proj = normals[:, 0] * d_xy[:, 0] + normals[:, 1] * d_xy[:, 1]
                d_prop = normals * proj[:, None]
            else:
                d_prop = d_xy
            pair_name = 'right' if self.active == 'left' else 'left'
            self.state[pair_name][window_idx, 0] += d_prop[:, 0]
            self.state[pair_name][window_idx, 1] += d_prop[:, 1]
            self.state['center'][window_idx, 0] += d_prop[:, 0]
            self.state['center'][window_idx, 1] += d_prop[:, 1]
        elif self.state['fmt'] == 'boundary' and self.active in ('left', 'right'):
            # No width preserve but keep center sane in raw-boundary format.
            self.state['center'] = (self.state['left'] + self.state['right']) / 2.0

        # On a closed loop with a duplicated tail point, keep the tail in sync
        # with the head so the polyline remains visually closed.
        if self.wrap_around and self._n_effective < n:
            for layer in ('left', 'right', 'center'):
                self.state[layer][-1] = self.state[layer][0]

        self._refresh()
        self.statusBar().showMessage(
            f'Smoothed {self.active} around idx={idx_pivot} with R={R} '
            f'({2*R+1} pts){" [wrap]" if self.wrap_around else ""}. '
            f'{self._status_text()}')
        # Auto-align rib⊥tangent right after pose_smooth (if enabled).
        if self.auto_align_rib:
            self._auto_align_after_edit('pose_smooth')

    def _auto_align_after_edit(self, source):
        """Run _align_paired_to_active and report. Safe to call after any edit."""
        try:
            moved = self._align_paired_to_active()
        except Exception as e:
            self.statusBar().showMessage(f'Auto-align skipped ({source}): {e}')
            return
        self._refresh()
        if moved:
            self.statusBar().showMessage(
                f'Auto-aligned rib⊥tangent after {source}: '
                f'{moved} index(es) re-paired.')

    # ---------- densify boundaries (point count only, shape preserved) ----------
    @staticmethod
    def _resample_polyline(P, n_new, wrap=False):
        """Resample a Nx3 polyline at `n_new` arc-length-uniform points.

        Curve shape is preserved exactly: new points lie on the existing
        polyline (linear interpolation between vertices). Returns a new
        Nx3 array; when wrap=True, distance includes the seam segment and
        the last new point coincides with the first.
        """
        P = np.asarray(P, dtype=float)
        n_old = len(P)
        if n_old < 2 or n_new < 2:
            return P.copy()
        if wrap:
            # Add the seam so arc-length covers the closed loop.
            P_ext = np.vstack([P, P[0:1]])
            seg = np.linalg.norm(np.diff(P_ext, axis=0), axis=1)
        else:
            P_ext = P
            seg = np.linalg.norm(np.diff(P_ext, axis=0), axis=1)
        s_cum = np.concatenate([[0.0], np.cumsum(seg)])
        s_total = s_cum[-1]
        if s_total < 1e-9:
            return np.tile(P[0:1], (n_new, 1))
        s_new = np.linspace(0.0, s_total, n_new)
        out = np.empty((n_new, P_ext.shape[1]))
        for d in range(P_ext.shape[1]):
            out[:, d] = np.interp(s_new, s_cum, P_ext[:, d])
        if wrap:
            out[-1] = out[0]  # tail = head exactly
        return out

    def _densify_all(self, n_new):
        """Resample left / right / center to `n_new` points each, preserving
        each polyline's shape. Updates state['n'], state['dirty'], state['raw_data'].
        Wrap state is preserved (closed-loop tracks stay closed).
        """
        n_old = self.state['n']
        if n_new == n_old:
            return
        wrap = bool(self.wrap_around)
        L = self._resample_polyline(self.state['left'],   n_new, wrap=wrap)
        R = self._resample_polyline(self.state['right'],  n_new, wrap=wrap)
        C = self._resample_polyline(self.state['center'], n_new, wrap=wrap)
        self.state['left']   = L
        self.state['right']  = R
        self.state['center'] = C
        # raw_data is laid out per-format; only used at save time. For boundary
        # format we don't need to maintain it (we recompute on save), so just
        # zero-fill at the new length. For 3d_track format we'd need to
        # resample each column individually — handled at save time anyway.
        ncols = self.state['raw_data'].shape[1] if self.state['raw_data'].ndim == 2 else 9
        self.state['raw_data'] = np.zeros((n_new, ncols))
        self.state['dirty'] = np.ones(n_new, dtype=bool)
        self.state['n'] = n_new
        # Selection indices may now be out of range — drop those.
        self.selected = {i for i in self.selected if 0 <= i < n_new}
        # Re-detect effective length for wrap.
        if wrap and n_new >= 2:
            self._n_effective = n_new - 1 if self._n_effective < n_old else n_new
        else:
            self._n_effective = n_new

    # ---------- Full Realign: global rib⊥tangent fixup ----------
    def _full_realign(self):
        """Globally resample one boundary along the other's normals so that
        rib⊥tangent holds everywhere — not only near the edited region.

        Monotone in max(⊥err): tries both source candidates (L→paired and
        R→paired), keeps the result that reduces max the most, and rejects
        both if they would increase max (state stays untouched). Repeated
        clicks therefore never grow max.
        """
        self._push_history()
        err = self._compute_perp_err_deg()
        if err is None:
            self.statusBar().showMessage('Full Realign: not enough points.')
            return
        before_max = float(err.max())
        before_mean = float(err.mean())

        # Snapshot once so we can try each candidate from the same starting state.
        snap = {
            'left':     self.state['left'].copy(),
            'right':    self.state['right'].copy(),
            'center':   self.state['center'].copy(),
            'raw_data': self.state['raw_data'].copy(),
            'dirty':    self.state['dirty'].copy(),
            'n':        self.state['n'],
            'n_eff':    self._n_effective,
        }
        n_base = snap['n']

        def restore_snap():
            self.state['left']     = snap['left'].copy()
            self.state['right']    = snap['right'].copy()
            self.state['center']   = snap['center'].copy()
            self.state['raw_data'] = snap['raw_data'].copy()
            self.state['dirty']    = snap['dirty'].copy()
            self.state['n']        = snap['n']
            self._n_effective      = snap['n_eff']

        # Staged candidate search. We minimise max(⊥err) but want to keep the
        # user's boundary shape close to what they laid out, so width changes
        # are bounded (`width_ratio_limit`) and the local pairing search is
        # capped (`search_radius`). Stages share state via snapshot rewinds and
        # accumulate the best candidate so far.
        #
        # Density factor (`dens`) lets us resample L/R at a higher point count
        # before aligning — curves are preserved (linear interpolation between
        # existing vertices) but the extra resolution often unlocks a lower
        # max(⊥err). Downstream smooth_track resamples uniformly anyway, so
        # variable N here is fine.
        def try_one(source, smooth_r, search_r, width_lim, dens, label):
            restore_snap()
            if dens != 1.0:
                self._densify_all(int(round(n_base * dens)))
            total_moved = 0
            prev = None
            for _ in range(4):
                m = self._align_paired_to_active(
                    source_layer=source, smooth_radius=smooth_r,
                    search_radius=search_r, width_ratio_limit=width_lim)
                total_moved += m
                err_now = self._compute_perp_err_deg()
                if err_now is None:
                    break
                sig = (round(float(err_now.mean()), 4),
                       round(float(err_now.max()), 4))
                if sig == prev:
                    break
                prev = sig
            err_now = self._compute_perp_err_deg()
            if err_now is None:
                return None
            return (
                float(err_now.max()), float(err_now.mean()), total_moved, label,
                {
                    'left':     self.state['left'].copy(),
                    'right':    self.state['right'].copy(),
                    'center':   self.state['center'].copy(),
                    'raw_data': self.state['raw_data'].copy(),
                    'dirty':    self.state['dirty'].copy(),
                    'n':        self.state['n'],
                    'n_eff':    self._n_effective,
                },
            )

        candidates = []
        # Stage 1: (source) × (smoothing radius) at native density.
        for source in ('left', 'right'):
            for r in (1, 2, 4, 8, 16):
                c = try_one(source, r, search_r=20, width_lim=1.3,
                            dens=1.0, label=f'{source}/r{r}')
                if c is not None:
                    candidates.append(c)

        # Stage 2: refine around the best stage-1 source — tighter / wider
        # search window and a relaxed width limit.
        if candidates:
            best_so_far = min(candidates, key=lambda c: (c[0], c[1]))
            best_src = best_so_far[3].split('/')[0]
            best_r = int(best_so_far[3].split('/')[1][1:])
            for sw in (10, 40):
                c = try_one(best_src, best_r, search_r=sw, width_lim=1.3,
                            dens=1.0,
                            label=f'{best_src}/r{best_r}/sw{sw}')
                if c is not None:
                    candidates.append(c)
            for wl in (1.6, 2.5):
                c = try_one(best_src, best_r, search_r=20, width_lim=wl,
                            dens=1.0,
                            label=f'{best_src}/r{best_r}/w{wl}')
                if c is not None:
                    candidates.append(c)

        # (A density-sweep stage was tried here but per-curve uniform resampling
        # scrambles rib pairing when L and R have different arc-lengths, so it
        # consistently regressed max(⊥err). Removed.)

        # Pick the candidate with the smallest max (tiebreak: smaller mean).
        candidates.sort(key=lambda c: (c[0], c[1]))
        best = candidates[0] if candidates else None
        tol = 1e-3  # accept neutral / improving but reject growth
        if best is None or best[0] > before_max + tol:
            # No improvement — leave the state as it was.
            restore_snap()
            self._refresh()
            self.statusBar().showMessage(
                f'Full Realign: no improvement available. '
                f'⊥err mean={before_mean:.2f}° max={before_max:.2f}°.')
            return

        # Apply the winning candidate.
        max_after, mean_after, total_moved, label, st = best
        self.state['left']     = st['left']
        self.state['right']    = st['right']
        self.state['center']   = st['center']
        self.state['raw_data'] = st['raw_data']
        self.state['dirty']    = st['dirty']
        self.state['n']        = st['n']
        self._n_effective      = st['n_eff']
        # drop selection that's now out of range (density may have changed N)
        self.selected = {i for i in self.selected if 0 <= i < st['n']}
        self._refresh()
        self.statusBar().showMessage(
            f'Full Realign [{label}]: {total_moved} updates.  '
            f'⊥err {before_mean:.2f}°/{before_max:.2f}° → '
            f'{mean_after:.2f}°/{max_after:.2f}° (mean/max)')

    def _on_drag_end(self):
        # In raw-boundary format, keep center = midpoint after a drag *unless*
        # the drag already moved center together with the boundary (width-preserve);
        # in that case (L+R)/2 happens to equal the already-shifted center, but
        # we skip the rewrite anyway to keep the code path explicit.
        if (not self.width_preserve
                and self.state['fmt'] == 'boundary'
                and self.active in ('left', 'right')):
            self.state['center'] = (self.state['left'] + self.state['right']) / 2.0
            self._refresh()
        # Auto-align rib⊥tangent on drag end (if enabled).
        if self.auto_align_rib:
            self._auto_align_after_edit('drag')
        # Remember the last-dragged point as the smoothing pivot, but drop the
        # full geometry snapshot (we only need the index later).
        if self._drag_base is not None:
            self._last_drag_idx = int(self._drag_base['idx'])
        self._drag_base = None

    # ---------- delete / decimate ----------
    def _delete_selection(self):
        if not self.selected:
            return
        self._push_history()
        self._delete_indices(sorted(self.selected))
        self.selected.clear()
        self.statusBar().showMessage(f'Deleted {self._status_text()}')

    def _delete_indices(self, idxs):
        keep = np.ones(self.state['n'], dtype=bool)
        keep[idxs] = False
        self.state['left']   = self.state['left'][keep]
        self.state['right']  = self.state['right'][keep]
        self.state['center'] = self.state['center'][keep]
        self.state['raw_data'] = self.state['raw_data'][keep]
        self.state['dirty']    = self.state['dirty'][keep]
        self.state['n'] = int(keep.sum())
        self.selected = {i for i in self.selected if i not in set(idxs)}
        # Re-index remaining selection
        old_to_new = -np.ones(len(keep), dtype=int)
        old_to_new[keep] = np.arange(self.state['n'])
        self.selected = {int(old_to_new[i]) for i in self.selected if old_to_new[i] >= 0}
        self._refresh()

    def _downsample_selection(self):
        if not self.selected:
            QtWidgets.QMessageBox.information(self, 'Downsample',
                'Select a range of points first (box-drag).')
            return
        n_keep_every, ok = QtWidgets.QInputDialog.getInt(
            self, 'Downsample selection',
            'Keep every Nth selected point (e.g., 2 = halve, 3 = third):',
            value=2, min=2, max=100)
        if not ok:
            return
        idxs = sorted(self.selected)
        # Within the selected indices, drop those NOT at every Nth position
        drop = [idxs[i] for i in range(len(idxs)) if (i % n_keep_every) != 0]
        if not drop:
            return
        self._push_history()
        self._delete_indices(drop)
        self.selected.clear()
        self._refresh()
        self.statusBar().showMessage(f'Downsampled (kept every {n_keep_every}). {self._status_text()}')

    # ---------- save ----------
    def _save(self):
        try:
            save_track_csv(self.csv_path, self.state, csv_path_for_backup=self.csv_path)
            self.statusBar().showMessage(f'Saved → {self.csv_path}  (backup: {self.csv_path}.bak)')
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, 'Save failed', str(e))

    # ---------- downstream pipeline (Step 3 + Step 4 from run_pipeline.sh) ----------
    PIPELINE_WEIGHTS = {
        'w_c': 1e5, 'w_l': 1e5, 'w_r': 1e5,
        'w_theta': 1e2, 'w_mu': 1e2, 'w_phi': 1e1,
        'w_nl': 1e-2, 'w_nr': 1e-2,
    }

    def _pipeline_paths(self):
        """Classify the current CSV and derive raw / gen3D / smoothed paths.

        Returns dict with keys: kind ('bounds_3d' | '3d' | 'smoothed' | 'unknown'),
        base (track name with all suffixes stripped), repo_root, raw_csv,
        gen3d_csv, smoothed_csv, params_json. Paths may not all exist; the
        caller decides which steps are needed.
        """
        path = os.path.abspath(self.csv_path)
        fname = os.path.basename(path)
        stem, _ = os.path.splitext(fname)
        kind = 'unknown'
        base = stem
        for suf, k in (('_3d_smoothed', 'smoothed'), ('_bounds_3d', 'bounds_3d'),
                       ('_3d', '3d')):
            if base.endswith(suf):
                kind = k
                base = base[: -len(suf)]
                break

        ### HJ : every artefact for one track lives in the same map folder
        ###      (stack_master/maps/<map>/). PCD, raw bounds CSV, 3D CSV,
        ###      smoothed CSV, params.json — all co-located with the input
        ###      CSV. No subtree lookup, no other roots.
        map_dir = os.path.dirname(path)

        return {
            'kind': kind, 'base': base, 'repo_root': map_dir,
            'raw_csv':       os.path.join(map_dir, f'{base}_bounds_3d.csv'),
            'gen3d_csv':     os.path.join(map_dir, f'{base}_3d.csv'),
            'smoothed_csv':  os.path.join(map_dir, f'{base}_3d_smoothed.csv'),
            'params_json':   os.path.join(map_dir, f'{base}_bounds_3d_params.json'),
        }

    def _configure_pipeline_button(self):
        """Enable/disable the Run Pipeline button based on the current CSV kind."""
        p = self._pipeline_paths()
        if p['kind'] == 'smoothed':
            self.btn_run_pipeline.setEnabled(False)
            self.btn_run_pipeline.setText('Run Pipeline (n/a)')
            self.btn_run_pipeline.setToolTip(
                'Current file is already a smoothed output — nothing downstream to run.')
        elif p['kind'] == 'bounds_3d':
            self.btn_run_pipeline.setEnabled(True)
            if self.auto_align_rib:
                self.btn_run_pipeline.setText('Run Pipeline (gen3D + smooth)')
            else:
                self.btn_run_pipeline.setText('Run Pipeline (regen + gen3D + smooth)')
        elif p['kind'] == '3d':
            self.btn_run_pipeline.setEnabled(True)
            self.btn_run_pipeline.setText('Run Pipeline (smooth)')
        else:
            self.btn_run_pipeline.setEnabled(False)
            self.btn_run_pipeline.setText('Run Pipeline (n/a)')
            self.btn_run_pipeline.setToolTip(
                'Could not classify the current CSV name (expected *_bounds_3d.csv '
                'or *_3d.csv).')

    def _on_run_pipeline(self):
        """Save then kick off Step 3 (when needed) + Step 4 in a background thread."""
        if self._pipeline_thread is not None and self._pipeline_thread.isRunning():
            QtWidgets.QMessageBox.information(
                self, 'Pipeline running', 'A pipeline run is already in progress.')
            return

        # Always persist the current edits first so the pipeline sees them.
        try:
            save_track_csv(self.csv_path, self.state, csv_path_for_backup=self.csv_path)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, 'Save failed', str(e))
            return

        # Lazy-import Track3D. pandas / casadi are required for the pipeline
        # but not for the editor itself, so we keep the error friendly.
        try:
            track3d_cls = self._import_track3d()
        except Exception as e:
            QtWidgets.QMessageBox.critical(
                self, 'Pipeline import failed',
                f'Could not import Track3D from scripts/track3D.py:\n{e}\n\n'
                'Make sure numpy, scipy, pandas, matplotlib, casadi are installed '
                'in this Python environment.')
            return

        p = self._pipeline_paths()
        if p['kind'] not in ('bounds_3d', '3d'):
            QtWidgets.QMessageBox.warning(
                self, 'Run Pipeline', 'Nothing to run for this CSV.')
            return

        # step_size: same fallback policy as run_pipeline.sh.
        step_size = 0.2
        if os.path.exists(p['params_json']):
            try:
                with open(p['params_json']) as f:
                    step_size = json.load(f).get('step_size', 0.2)
            except Exception:
                pass

        steps = []
        if p['kind'] == 'bounds_3d':
            # Step 2.9 (regen_center) is only needed if rib⊥tangent isn't already
            # enforced by the editor's auto-align. With auto-align ON every
            # edit keeps the constraint satisfied, so we can skip the
            # interactive matplotlib stop and head straight into gen3D + smooth.
            if not self.auto_align_rib:
                steps.append(('regen_center', p['raw_csv'], p['raw_csv']))
            steps.append(('gen3d', p['raw_csv'], p['gen3d_csv']))
            steps.append(('smooth', p['gen3d_csv'], p['smoothed_csv']))
        else:  # '3d'
            steps.append(('smooth', p['gen3d_csv'], p['smoothed_csv']))

        # Make sure output dirs exist (run_pipeline.sh creates these via mkdir -p).
        for _, _, out_path in steps:
            os.makedirs(os.path.dirname(out_path), exist_ok=True)

        self.btn_run_pipeline.setEnabled(False)
        self.statusBar().showMessage(
            f'Pipeline starting on {os.path.basename(self.csv_path)} …')

        thread = _PipelineThread(track3d_cls, steps,
                                 self.PIPELINE_WEIGHTS, step_size, parent=self)
        thread.progress.connect(self._on_pipeline_progress)
        thread.done.connect(self._on_pipeline_done)
        thread.failed.connect(self._on_pipeline_failed)
        # Keep a strong reference until the thread fully finishes; only after
        # finished do we clear the reference. No deleteLater — Qt cleans up via
        # parent=self when the editor closes.
        thread.finished.connect(self._pipeline_finished_cleanup)
        self._pipeline_thread = thread
        thread.start()

    @staticmethod
    def _import_track3d():
        """Import Track3D from the race_stack layout.

        ### HJ : Track3D lives at
        ###      planner/3d_gb_optimizer/global_line/src/track3D.py
        ###      in race_stack. We resolve that path via rospkg when
        ###      available, or fall back to the repo-relative path. No
        ###      other layouts are searched — everything is in race_stack.
        """
        candidates = []
        try:
            import rospkg
            opt_root = rospkg.RosPack().get_path('global_line_3d')
            candidates.append(os.path.join(opt_root, 'src'))
        except Exception:
            pass
        here = os.path.dirname(os.path.abspath(__file__))
        race_stack_root = os.path.dirname(os.path.dirname(here))
        candidates.append(os.path.join(
            race_stack_root, 'planner', '3d_gb_optimizer',
            'global_line', 'src'))

        last_err = None
        for d in candidates:
            if not os.path.isdir(d):
                continue
            if d not in sys.path:
                sys.path.insert(0, d)
            try:
                from track3D import Track3D  # noqa: F401
                return Track3D
            except Exception as e:
                last_err = e
                continue
        if last_err is None:
            raise ImportError(
                f'track3D.py not found under any of:\n  - ' +
                '\n  - '.join(candidates))
        raise last_err

    def _on_pipeline_progress(self, message):
        self.statusBar().showMessage(message)

    def _on_pipeline_done(self, summary):
        self.statusBar().showMessage(f'Pipeline finished — {summary}')
        QtWidgets.QMessageBox.information(self, 'Pipeline finished', summary)

    def _on_pipeline_failed(self, err):
        self.statusBar().showMessage(f'Pipeline failed: {err}')
        QtWidgets.QMessageBox.critical(self, 'Pipeline failed', err)

    def _pipeline_finished_cleanup(self):
        self._pipeline_thread = None
        # Re-enable button (text already reflects the current CSV kind).
        self.btn_run_pipeline.setEnabled(self.btn_run_pipeline.text() != 'Run Pipeline (n/a)')

    def closeEvent(self, ev):
        # Don't tear down with a live worker — that's the "QThread destroyed
        # while still running" abort.
        t = self._pipeline_thread
        if t is not None and t.isRunning():
            ans = QtWidgets.QMessageBox.question(
                self, 'Pipeline still running',
                'A pipeline run is still in progress. Wait for it to finish?',
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
            if ans == QtWidgets.QMessageBox.Yes:
                t.wait()
            else:
                # Best-effort: ask the thread to stop accepting new work and
                # block until its current step completes.
                t.requestInterruption()
                t.wait()
        super().closeEvent(ev)

    # ---------- key handling ----------
    def keyPressEvent(self, ev):
        k = ev.key()
        if k == QtCore.Qt.Key_S:
            self._save()
        elif k == QtCore.Qt.Key_U:
            self._undo()
        elif k == QtCore.Qt.Key_D:
            self._downsample_selection()
        elif k == QtCore.Qt.Key_Backspace:
            self._delete_selection()
        elif k == QtCore.Qt.Key_Escape:
            self.selected.clear(); self._refresh()
        elif k == QtCore.Qt.Key_L:
            self.layer_combo.setCurrentText('left')
        elif k == QtCore.Qt.Key_R:
            self.layer_combo.setCurrentText('right')
        elif k == QtCore.Qt.Key_C:
            self.layer_combo.setCurrentText('center')
        elif k == QtCore.Qt.Key_P:
            self.show_other_cb.toggle()
        else:
            super().keyPressEvent(ev)


# ----------------------------- main -----------------------------

### HJ : ROS-friendly entry — three ways to specify the CSV, in priority order:
###        1) `_csv:=<path>`     (rosrun private param) — explicit path wins
###        2) `_map:=<name>`     (rosrun private param) — resolves
###             $(stack_master)/maps/<name>/<name>_bounds_3d.csv
###        3) positional CSV     (`python3 boundary_editor_qt.py <path>`)
###             also accepted as a bare CLI tool for non-ROS use.
###      Examples:
###        rosrun stack_master boundary_editor_qt.py _map:=test_gazebo_wall
###        rosrun stack_master boundary_editor_qt.py _csv:=/abs/path.csv
###        roslaunch stack_master boundary_editor.launch map:=test_gazebo_wall
###        python3 boundary_editor_qt.py /abs/path.csv         (legacy)
def _resolve_csv_from_args():
    ### HJ : roslaunch / rosrun inject internal args like `__name:=`,
    ###      `__log:=`, `__ns:=` that argparse would choke on. Strip those
    ###      plus our private-param `_key:=value` tokens before handing the
    ###      rest to argparse, while pulling _map / _csv out as our own
    ###      private params.
    map_name = None
    csv_path = None
    leftover = []
    for tok in sys.argv[1:]:
        if tok.startswith('__'):
            continue   # ROS-internal (__name, __log, __ns, ...)
        if tok.startswith('_csv:='):
            val = tok.split(':=', 1)[1]
            ### HJ : roslaunch passes <param value=""/> as `_csv:=` (empty)
            ###      when the user didn't override — treat empty as "not set"
            ###      so we fall through to the _map: path.
            if val:
                csv_path = val
            continue
        if tok.startswith('_map:='):
            val = tok.split(':=', 1)[1]
            if val:
                map_name = val
            continue
        if tok.startswith('_') and ':=' in tok:
            ### HJ : unknown private param — ignore so future launch args
            ###      don't accidentally crash the editor.
            continue
        leftover.append(tok)

    ### HJ : fallback to positional argparse only when neither _csv nor _map
    ###      is given (preserves the original CLI for legacy non-ROS use).
    if csv_path is None and map_name is None:
        parser = argparse.ArgumentParser(
            description='Multi-select boundary editor (PyQt5 + pyqtgraph).')
        parser.add_argument(
            'csv', help='Track CSV (raw boundary or 3D track format).')
        args = parser.parse_args(leftover)
        csv_path = args.csv

    ### HJ : resolve via map name → $(stack_master)/maps/<map>/<map>_bounds_3d.csv
    if csv_path is None and map_name is not None:
        try:
            import rospkg
            stack_master_path = rospkg.RosPack().get_path('stack_master')
        except Exception as e:
            print(f'rospack stack_master lookup failed: {e}', file=sys.stderr)
            sys.exit(1)
        csv_path = os.path.join(
            stack_master_path, 'maps', map_name, f'{map_name}_bounds_3d.csv')

    return csv_path


def main():
    csv_path = _resolve_csv_from_args()
    if not csv_path or not os.path.exists(csv_path):
        print(f'File not found: {csv_path}', file=sys.stderr)
        sys.exit(1)
    print(f'[boundary_editor] opening: {csv_path}')

    app = QtWidgets.QApplication(sys.argv)
    win = BoundaryEditor(csv_path)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
