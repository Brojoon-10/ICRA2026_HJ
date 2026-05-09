"""### HJ : MPC debug logger (Tier A CSV + Tier B NPZ).

Writes one directory per run:

    runs/<YYYYMMDD_HHMMSS>_<state>_<git_sha>/
        meta.json              one-shot: params + solver settings + topics
        ticks.csv              one row per tick (headers stable)
        ticks/tick_000123.npz  per-tick full state (only when recorded)

The logger is built to be:
  * **always-on-safe** — Tier A CSV is a single append per tick, ~KB/min.
  * **anomaly-aware** — Tier B NPZ triggers on fallback tiers, bad IPOPT
    status, corridor overrun, or high-curvature trajectories. Or sampled
    every N ticks, or always, or never.
  * **crash-safe** — CSV is flushed every row; NPZ per-tick is atomic.

Node integration:
    logger = DebugLogger(cfg, params_snapshot)
    ...
    logger.log_tick(rec)   # rec = dict of fields (see TICK_FIELDS below)
    logger.close()         # on shutdown
"""

from __future__ import annotations

import csv
import json
import os
import subprocess
import time
from typing import Optional

import numpy as np


# ---- Tier A columns (pandas-friendly, stable order) -------------------------
# Keep this list append-only so existing ticks.csv parsers don't break.
TICK_FIELDS = [
    'idx', 't_ros', 't_wall',
    'state', 'tier', 'status',
    'ipopt_status', 'iter_count', 'solve_ms', 'slack_max', 'obj_value',
    'car_x', 'car_y', 'car_yaw', 'car_vx', 'car_vy',
    'ego_s', 'ego_n',
    'speed0', 'steer0',
    'speed_max', 'steer_max',
    'traj_kappa_rms', 'traj_kappa_max',
    'ref_kappa_rms', 'ref_kappa_max',
    'corridor_min_width', 'ego_in_corridor',
    'obs_tag', 'n_obs_active',
    'fail_streak', 'warm_used',
    # ### HJ : current weight snapshot — lets me correlate rqt slider moves
    # with trajectory quality changes without opening meta.json or events.jsonl.
    'w_contour', 'w_lag', 'w_velocity', 'v_bias_max',
    'w_dv', 'w_dsteering', 'w_slack',
    'max_speed', 'min_speed', 'max_steering', 'boundary_inflation',
    'w_obstacle', 'obstacle_sigma', 'w_wall', 'wall_safe',
    'ipopt_max_iter',
    'npz_path',
]


def _git_sha(repo_hint_path: str) -> str:
    try:
        out = subprocess.check_output(
            ['git', 'rev-parse', '--short=8', 'HEAD'],
            cwd=repo_hint_path, stderr=subprocess.DEVNULL, timeout=1.0)
        return out.decode('utf-8', errors='ignore').strip() or 'nosha'
    except Exception:
        return 'nosha'


def _curvature_rms_max(traj_xy: np.ndarray) -> tuple:
    """Finite-difference curvature from a (M, 2) planar path. Robust to short
    arrays — returns (0.0, 0.0) if M<3 or any segment is degenerate."""
    if traj_xy is None or traj_xy.shape[0] < 3:
        return 0.0, 0.0
    p = np.asarray(traj_xy, dtype=np.float64)
    d1 = p[1:] - p[:-1]
    seg = np.linalg.norm(d1, axis=1)
    if np.any(seg < 1e-6):
        return 0.0, 0.0
    heading = np.arctan2(d1[:, 1], d1[:, 0])
    dpsi = np.arctan2(np.sin(np.diff(heading)), np.cos(np.diff(heading)))
    ds = 0.5 * (seg[:-1] + seg[1:])
    kappa = dpsi / np.maximum(ds, 1e-6)
    return float(np.sqrt(np.mean(kappa ** 2))), float(np.max(np.abs(kappa)))


def _corridor_min_width(left: np.ndarray, right: np.ndarray) -> float:
    if left is None or right is None:
        return float('nan')
    w = np.linalg.norm(np.asarray(left) - np.asarray(right), axis=1)
    return float(np.min(w)) if w.size else float('nan')


def _cost_breakdown(ref_slice, trajectory, u_sol, obs_arr, params):
    """Post-hoc reproduction of per-term cost contributions at the returned
    solution. Branches on solver_backend so frenet_d (n-only NLP) doesn't
    get evaluated with the xy solver's Cartesian formulas (which would read
    obs_arr's [s_o, n_o] columns as [x_o, y_o] — garbage)."""
    if trajectory is None:
        return {}
    backend = str(params.get('solver_backend', 'xy')).lower()
    if backend == 'frenet_d':
        return _cost_breakdown_frenet_d(ref_slice, trajectory, obs_arr, params)
    return _cost_breakdown_xy(ref_slice, trajectory, u_sol, obs_arr, params)


def _cost_breakdown_xy(ref_slice, trajectory, u_sol, obs_arr, params):
    if u_sol is None:
        return {}
    N = u_sol.shape[0]
    half = 0.5
    w_contour = params['w_contour']
    w_lag = params['w_lag']
    w_velocity = params['w_velocity']
    v_bias = params['v_bias_max']
    w_dv = params['w_dv']
    w_dsteer = params['w_dsteering']
    w_steer_reg = params.get('w_steer_reg', 1e-3)
    sigma = params.get('obstacle_sigma', 0.35)
    inv2s2 = 1.0 / (2.0 * sigma * sigma)

    center = np.asarray(ref_slice['center_points'])
    ref_v = np.asarray(ref_slice['ref_v'])
    ref_dx = np.asarray(ref_slice['ref_dx'])
    ref_dy = np.asarray(ref_slice['ref_dy'])

    acc = dict(contour=0.0, lag=0.0, vel=0.0, steer_reg=0.0,
               dv=0.0, dsteer=0.0, slack=0.0, obs=0.0)
    for k in range(N):
        idx = min(k, len(center) - 1)
        t_angle = float(np.arctan2(ref_dy[idx], ref_dx[idx]))
        st_next = trajectory[k + 1]
        ex = st_next[0] - center[idx, 0]
        ey = st_next[1] - center[idx, 1]
        e_c = (np.sin(t_angle) * ex - np.cos(t_angle) * ey) / half
        e_l = (-np.cos(t_angle) * ex - np.sin(t_angle) * ey) / half
        acc['contour']   += w_contour * e_c ** 2
        acc['lag']       += w_lag * e_l ** 2
        acc['vel']       += w_velocity * ((u_sol[k, 0] - ref_v[idx]) / v_bias) ** 2
        acc['steer_reg'] += w_steer_reg * u_sol[k, 1] ** 2
        if k < N - 1:
            acc['dv']     += w_dv     * (u_sol[k + 1, 0] - u_sol[k, 0]) ** 2
            acc['dsteer'] += w_dsteer * (u_sol[k + 1, 1] - u_sol[k, 1]) ** 2
        if obs_arr is not None:
            for o in range(obs_arr.shape[0]):
                ox, oy, w_o = obs_arr[o, k, 0], obs_arr[o, k, 1], obs_arr[o, k, 2]
                dx = st_next[0] - ox
                dy = st_next[1] - oy
                acc['obs'] += float(w_o) * float(np.exp(-(dx * dx + dy * dy) * inv2s2))
    return {k: float(v) for k, v in acc.items()}


def _cost_breakdown_frenet_d(ref_slice, trajectory, obs_arr, params):
    """Mirror of FrenetDSolver._build_nlp objective at the returned n-profile.

    n_k is recovered by projecting (trajectory[k] - center[k]) onto the
    left-normal (-ref_dy, ref_dx). Cost terms:
      contour:  w_contour · α_c(k) · n_k²                       for k=0..N
      d1:       w_dv · (n_{k+1} - n_k)²                          for k=0..N-1
      d2:       w_dsteering · (n_{k+2} - 2 n_{k+1} + n_k)²       for k=0..N-2
      wall:     w_wall · [max(0, n_k - n_hi)² + max(0, -n_k - n_lo_neg)²]
                    where n_hi = d_left - inflation - wall_safe
                          n_lo_neg = d_right - inflation - wall_safe
      obs:      Σ_o w_o · exp(-((Δs)² + (Δn)²) / (2σ²))          per (k,o)
    Other xy-only terms (lag/vel/steer_reg/slack) are kept at 0.
    """
    traj = np.asarray(trajectory)
    center = np.asarray(ref_slice['center_points'])
    ref_dx = np.asarray(ref_slice['ref_dx'])
    ref_dy = np.asarray(ref_slice['ref_dy'])
    ref_s = np.asarray(ref_slice['ref_s'])
    d_left = np.asarray(ref_slice.get('d_left', np.zeros_like(ref_s)))
    d_right = np.asarray(ref_slice.get('d_right', np.zeros_like(ref_s)))
    Np1 = traj.shape[0]

    w_contour = float(params.get('w_contour', 2.0))
    w_dv = float(params.get('w_dv', 5.0))
    w_dsteer = float(params.get('w_dsteering', 20.0))
    ramp_start = float(params.get('contour_ramp_start', 1.0))
    sigma = float(params.get('obstacle_sigma', 0.5))
    inv_two_sigma2 = 1.0 / max(2.0 * sigma * sigma, 1e-6)
    w_wall = float(params.get('w_wall', 0.0))
    wall_safe = float(params.get('wall_safe', 0.15))
    inflation = float(params.get('boundary_inflation', 0.1))

    # Recover n_k = (traj - center) · (-dy, dx)
    disp_x = traj[:, 0] - center[:, 0]
    disp_y = traj[:, 1] - center[:, 1]
    n_sol = disp_x * (-ref_dy) + disp_y * ref_dx

    acc = dict(contour=0.0, lag=0.0, vel=0.0, steer_reg=0.0,
               dv=0.0, dsteer=0.0, slack=0.0, obs=0.0, wall=0.0)
    N = Np1 - 1
    for k in range(Np1):
        alpha_c = ramp_start + (1.0 - ramp_start) * (k / float(N) if N > 0 else 1.0)
        nk = float(n_sol[k])
        acc['contour'] += w_contour * alpha_c * nk * nk
        if w_wall > 0.0 and k < len(d_left) and k < len(d_right):
            n_hi = float(d_left[k]) - inflation - wall_safe
            n_lo_neg = float(d_right[k]) - inflation - wall_safe
            over_hi = max(0.0, nk - n_hi)
            over_lo = max(0.0, -nk - n_lo_neg)
            acc['wall'] += w_wall * (over_hi * over_hi + over_lo * over_lo)
        if obs_arr is not None:
            for o in range(obs_arr.shape[0]):
                s_o = float(obs_arr[o, k, 0])
                n_o = float(obs_arr[o, k, 1])
                w_o = float(obs_arr[o, k, 2])
                ds = float(ref_s[k]) - s_o
                dn = nk - n_o
                acc['obs'] += w_o * float(np.exp(-(ds * ds + dn * dn) * inv_two_sigma2))
    for k in range(N):
        d1 = float(n_sol[k + 1] - n_sol[k])
        acc['dv'] += w_dv * d1 * d1
    for k in range(N - 1):
        d2 = float(n_sol[k + 2] - 2.0 * n_sol[k + 1] + n_sol[k])
        acc['dsteer'] += w_dsteer * d2 * d2
    return {k: float(v) for k, v in acc.items()}


class DebugLogger:
    """Append-only per-tick logger with opt-in heavy NPZ."""

    TIER_MODES = ('never', 'every_n', 'on_anomaly', 'always')

    def __init__(self, cfg: dict, params_snapshot: dict,
                 repo_hint_path: Optional[str] = None):
        """
        cfg keys (all optional, sensible defaults):
            enable: bool
            dir: str                    root for runs/
            tier_b: str in TIER_MODES
            every_n: int                (tier_b='every_n')
            anomaly_kappa_max: float
            state: str                  informational
            node_name: str              informational
        """
        self.enable = bool(cfg.get('enable', False))
        self.mode = str(cfg.get('tier_b', 'on_anomaly'))
        if self.mode not in self.TIER_MODES:
            self.mode = 'on_anomaly'
        self.every_n = int(cfg.get('every_n', 10))
        self.k_thresh = float(cfg.get('anomaly_kappa_max', 3.0))

        self.state = str(cfg.get('state', 'unknown'))
        self.node_name = str(cfg.get('node_name', 'mpc'))

        self.params = dict(params_snapshot)  # shallow copy
        self._tick_idx = 0
        self._csv_fh = None
        self._csv_writer = None
        self._events_fh = None
        self._run_dir = None
        self._ticks_dir = None

        # ### HJ : rolling-window summary for live monitor.
        self._summary_window = int(cfg.get('summary_window', 50))
        self._summary_every = int(cfg.get('summary_every', 10))
        self._summary_buf = []  # list of small dicts (last N rows)

        if not self.enable:
            return

        root = os.path.expanduser(str(cfg.get('dir', './runs')))
        os.makedirs(root, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        sha = _git_sha(repo_hint_path or root)
        self._run_dir = os.path.join(root, '%s_%s_%s' % (ts, self.state, sha))
        self._ticks_dir = os.path.join(self._run_dir, 'ticks')
        os.makedirs(self._ticks_dir, exist_ok=True)

        meta = {
            'run_dir': self._run_dir,
            'git_sha': sha,
            'timestamp': ts,
            'state': self.state,
            'node_name': self.node_name,
            'tier_b_mode': self.mode,
            'every_n': self.every_n,
            'anomaly_kappa_max': self.k_thresh,
            'params': self.params,
        }
        with open(os.path.join(self._run_dir, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2, default=str)

        self._csv_fh = open(os.path.join(self._run_dir, 'ticks.csv'), 'w', newline='')
        self._csv_writer = csv.DictWriter(self._csv_fh, fieldnames=TICK_FIELDS,
                                          extrasaction='ignore')
        self._csv_writer.writeheader()
        self._csv_fh.flush()

        # Events file (config changes, run markers).
        self._events_fh = open(
            os.path.join(self._run_dir, 'events.jsonl'), 'w')
        self._events_fh.flush()

    # -------------------------------------------------------------------
    def _should_record_npz(self, row: dict) -> bool:
        if self.mode == 'never':
            return False
        if self.mode == 'always':
            return True
        if self.mode == 'every_n':
            return (self._tick_idx % max(self.every_n, 1)) == 0
        # on_anomaly
        if row.get('tier', 0) and row['tier'] >= 1:
            return True
        if row.get('ipopt_status', 'Solve_Succeeded') not in (
                'Solve_Succeeded', 'Solved_To_Acceptable_Level', '-'):
            return True
        k_max = row.get('traj_kappa_max', 0.0) or 0.0
        if k_max > self.k_thresh:
            return True
        if (row.get('slack_max', 0.0) or 0.0) > 0.1:
            return True
        if row.get('ego_in_corridor', 1) == 0:
            return True
        return False

    # -------------------------------------------------------------------
    def log_tick(self, rec: dict):
        """rec fields:
            required: state, tier, status, solve_ms, t_ros
            strongly recommended: ipopt_status, iter_count, slack_max,
                                  car_{x,y,yaw,vx,vy}, ego_{s,n},
                                  speed0, steer0, fail_streak, obs_tag,
                                  warm_used
            heavy (for Tier B, ignored in CSV):
                ref_slice (dict), trajectory, u_sol, obs_arr,
                initial_state, X0_init, u0_init, X0_post, u0_post,
                slack (array), obj_value
        """
        if not self.enable:
            return
        self._tick_idx += 1

        # --- derived metrics --------------------------------------------
        traj_xy = None
        ref_xy = None
        if rec.get('trajectory') is not None:
            traj_xy = np.asarray(rec['trajectory'])[:, :2]
        if rec.get('ref_slice') is not None:
            ref_xy = np.asarray(rec['ref_slice'].get('center_points'))

        traj_k_rms, traj_k_max = _curvature_rms_max(traj_xy)
        ref_k_rms, ref_k_max = _curvature_rms_max(ref_xy)

        corridor_w = float('nan')
        ego_in = 1
        if rec.get('ref_slice') is not None:
            rs = rec['ref_slice']
            corridor_w = _corridor_min_width(rs.get('left_points'),
                                             rs.get('right_points'))

        u_sol = rec.get('u_sol')
        speed_max = steer_max = 0.0
        if u_sol is not None and len(u_sol) > 0:
            u_sol_arr = np.asarray(u_sol)
            speed_max = float(np.max(np.abs(u_sol_arr[:, 0])))
            steer_max = float(np.max(np.abs(u_sol_arr[:, 1])))

        obs_arr = rec.get('obs_arr')
        n_obs_active = 0
        if obs_arr is not None:
            n_obs_active = int(np.sum(np.any(obs_arr[:, :, 2] > 0.0, axis=1)))

        # --- Tier A row --------------------------------------------------
        weights = rec.get('weights', {}) or {}
        row = {
            'idx': self._tick_idx,
            't_ros': rec.get('t_ros', 0.0),
            't_wall': time.time(),
            'state': rec.get('state', self.state),
            'tier': int(rec.get('tier', 0)),
            'status': rec.get('status', ''),
            'ipopt_status': rec.get('ipopt_status', '-'),
            'iter_count': int(rec.get('iter_count', -1)),
            'solve_ms': float(rec.get('solve_ms', 0.0)),
            'slack_max': float(rec.get('slack_max', 0.0)),
            'obj_value': float(rec.get('obj_value', 0.0)),
            'car_x': float(rec.get('car_x', 0.0)),
            'car_y': float(rec.get('car_y', 0.0)),
            'car_yaw': float(rec.get('car_yaw', 0.0)),
            'car_vx': float(rec.get('car_vx', 0.0)),
            'car_vy': float(rec.get('car_vy', 0.0)),
            'ego_s': float(rec.get('ego_s', 0.0)),
            'ego_n': float(rec.get('ego_n', 0.0)),
            'speed0': float(rec.get('speed0', 0.0)),
            'steer0': float(rec.get('steer0', 0.0)),
            'speed_max': speed_max,
            'steer_max': steer_max,
            'traj_kappa_rms': traj_k_rms,
            'traj_kappa_max': traj_k_max,
            'ref_kappa_rms': ref_k_rms,
            'ref_kappa_max': ref_k_max,
            'corridor_min_width': corridor_w,
            'ego_in_corridor': int(rec.get('ego_in_corridor', 1)),
            'obs_tag': str(rec.get('obs_tag', '')),
            'n_obs_active': n_obs_active,
            'fail_streak': int(rec.get('fail_streak', 0)),
            'warm_used': int(rec.get('warm_used', 0)),
            'w_contour':          float(weights.get('w_contour',    0.0)),
            'w_lag':              float(weights.get('w_lag',        0.0)),
            'w_velocity':         float(weights.get('w_velocity',   0.0)),
            'v_bias_max':         float(weights.get('v_bias_max',   0.0)),
            'w_dv':               float(weights.get('w_dv',         0.0)),
            'w_dsteering':        float(weights.get('w_dsteering',  0.0)),
            'w_slack':            float(weights.get('w_slack',      0.0)),
            'max_speed':          float(weights.get('max_speed',    0.0)),
            'min_speed':          float(weights.get('min_speed',    0.0)),
            'max_steering':       float(weights.get('max_steering', 0.0)),
            'boundary_inflation': float(weights.get('boundary_inflation', 0.0)),
            'w_obstacle':         float(weights.get('w_obstacle',     0.0)),
            'obstacle_sigma':     float(weights.get('obstacle_sigma', 0.0)),
            'w_wall':             float(weights.get('w_wall',         0.0)),
            'wall_safe':          float(weights.get('wall_safe',      0.0)),
            'ipopt_max_iter':     int(weights.get('ipopt_max_iter',   0)),
            'npz_path': '',
        }

        npz_path = ''
        if self._should_record_npz(row):
            npz_name = 'tick_%06d.npz' % self._tick_idx
            npz_path_abs = os.path.join(self._ticks_dir, npz_name)
            self._write_npz(npz_path_abs, rec)
            npz_path = os.path.join('ticks', npz_name)

        row['npz_path'] = npz_path

        try:
            self._csv_writer.writerow(row)
            self._csv_fh.flush()
        except Exception:
            # Never crash the planner on a logger issue.
            pass

        # Rolling summary buffer (last N rows) → live_summary.json
        self._summary_buf.append(row)
        if len(self._summary_buf) > self._summary_window:
            self._summary_buf = self._summary_buf[-self._summary_window:]
        if self._tick_idx % max(self._summary_every, 1) == 0:
            self._write_live_summary()

    # -------------------------------------------------------------------
    def _write_live_summary(self):
        """### HJ : Overwrite live_summary.json with rolling-window stats.

        Meant to be cat'd by the user or me during a live run, and also
        returned as a JSON string for the node to publish on ROS. Atomic
        write so a reader never sees a half-written file.
        """
        if not self._summary_buf or self._run_dir is None:
            return None
        buf = self._summary_buf
        arr = lambda k: np.asarray([r.get(k, 0.0) for r in buf], dtype=np.float64)

        def pct(a, p):
            return float(np.percentile(a, p)) if a.size else 0.0

        tier_arr = arr('tier')
        ipopt_ok = sum(1 for r in buf
                       if r.get('ipopt_status') in
                       ('Solve_Succeeded', 'Solved_To_Acceptable_Level'))
        summary = {
            'last_idx': int(buf[-1].get('idx', 0)),
            'window': len(buf),
            'state': buf[-1].get('state', '?'),
            'tier_ratio': {
                str(t): float((tier_arr == t).sum()) / len(buf)
                for t in (0, 1, 2, 3)
            },
            'ipopt_success_ratio': ipopt_ok / len(buf),
            'solve_ms': {
                'p50': pct(arr('solve_ms'), 50),
                'p95': pct(arr('solve_ms'), 95),
                'p99': pct(arr('solve_ms'), 99),
                'max': float(arr('solve_ms').max()),
            },
            'traj_kappa_max': {
                'p50': pct(arr('traj_kappa_max'), 50),
                'p95': pct(arr('traj_kappa_max'), 95),
                'max': float(arr('traj_kappa_max').max()),
            },
            'traj_kappa_rms_mean': float(arr('traj_kappa_rms').mean()),
            'slack_max_mean':      float(arr('slack_max').mean()),
            'speed0_mean':         float(arr('speed0').mean()),
            'ego_n_abs_p95':       pct(np.abs(arr('ego_n')), 95),
            'current_weights': {k: buf[-1].get(k, 0.0) for k in (
                'w_contour', 'w_lag', 'w_velocity', 'v_bias_max',
                'w_dv', 'w_dsteering', 'w_slack',
                'max_speed', 'min_speed', 'max_steering',
                'boundary_inflation', 'w_obstacle', 'obstacle_sigma',
                'w_wall', 'wall_safe', 'ipopt_max_iter')},
            'obs_tag': buf[-1].get('obs_tag', ''),
            't_wall': time.time(),
        }
        try:
            path = os.path.join(self._run_dir, 'live_summary.json')
            tmp = path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(summary, f, indent=2)
            os.replace(tmp, path)
        except Exception:
            pass
        return summary

    # -------------------------------------------------------------------
    def log_event(self, kind: str, payload: dict):
        """### HJ : Append a line to events.jsonl — rqt slider moves, run
        markers, etc. Read with e.g. `tail -f events.jsonl`."""
        if self._events_fh is None:
            return
        try:
            evt = {
                't_wall': time.time(),
                'idx': self._tick_idx,
                'kind': kind,
                'payload': payload,
            }
            self._events_fh.write(json.dumps(evt) + '\n')
            self._events_fh.flush()
        except Exception:
            pass

    # -------------------------------------------------------------------
    def get_live_summary(self):
        """External-facing: force a fresh summary computation (for ROS pub)."""
        return self._write_live_summary()

    # -------------------------------------------------------------------
    def _write_npz(self, path: str, rec: dict):
        payload = {}
        for key in ('trajectory', 'u_sol', 'obs_arr', 'initial_state',
                    'X0_init', 'u0_init', 'X0_post', 'u0_post', 'slack'):
            v = rec.get(key)
            if v is not None:
                payload[key] = np.asarray(v)
        rs = rec.get('ref_slice')
        if rs is not None:
            for k, v in rs.items():
                if v is None:
                    continue
                # Prefix with 'ref_' unless the key already starts with it,
                # so downstream code sees e.g. ref_v (not ref_ref_v).
                name = k if k.startswith('ref_') else 'ref_' + k
                payload[name] = np.asarray(v)

        breakdown = _cost_breakdown(rs, rec.get('trajectory'), rec.get('u_sol'),
                                    rec.get('obs_arr'), self.params)
        if breakdown:
            payload['cost_breakdown_keys'] = np.array(list(breakdown.keys()))
            payload['cost_breakdown_vals'] = np.array(list(breakdown.values()),
                                                      dtype=np.float64)

        # Scalars — keep a small dict as JSON inside the npz via 'meta' key
        meta = {
            'ipopt_status': rec.get('ipopt_status', '-'),
            'iter_count': int(rec.get('iter_count', -1)),
            'solve_ms': float(rec.get('solve_ms', 0.0)),
            'slack_max': float(rec.get('slack_max', 0.0)),
            'obj_value': float(rec.get('obj_value', 0.0)),
            'tier': int(rec.get('tier', 0)),
            'status': rec.get('status', ''),
            'state': rec.get('state', self.state),
            'fail_streak': int(rec.get('fail_streak', 0)),
            'warm_used': int(rec.get('warm_used', 0)),
            'obs_tag': str(rec.get('obs_tag', '')),
            't_ros': float(rec.get('t_ros', 0.0)),
        }
        payload['meta_json'] = np.array(json.dumps(meta))

        try:
            # ### HJ : np.savez_compressed auto-appends '.npz' if missing, so
            # using path+'.tmp' yields 'foo.npz.tmp.npz' and os.replace fails.
            # Keep '.npz' at the end so numpy leaves the name alone.
            tmp = path + '.tmp.npz'
            np.savez_compressed(tmp, **payload)
            os.replace(tmp, path)
        except Exception:
            pass

    # -------------------------------------------------------------------
    def close(self):
        # Final summary so the on-disk state matches the last tick.
        try:
            self._write_live_summary()
        except Exception:
            pass
        for fh_attr in ('_csv_fh', '_events_fh'):
            fh = getattr(self, fh_attr, None)
            if fh is not None:
                try:
                    fh.flush()
                    fh.close()
                except Exception:
                    pass
                setattr(self, fh_attr, None)
        self._csv_writer = None
