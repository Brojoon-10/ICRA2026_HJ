"""### HJ : sampling_planner_state_node 용 OT 튜닝 디버그 로거.

매 tick metric (CSV) + 비주기 이벤트 (JSONL) + 시작 시 메타/파라미터 스냅샷
(YAML) + (옵션) candidates/cost arrays npz dump 를 한 세션 폴더에 기록한다.

설계 포인트
------------
- ``tick.csv`` 는 line-buffered (buffering=1) → 매 tick (≤33ms @30Hz) 디스크 반영,
  ``tail -n N`` 으로 거의 실시간 분석 가능.
- ``events.jsonl`` 은 write 직후 명시 flush — param 변경 같은 비주기 이벤트는 즉시 보임.
- I/O 는 모두 try/except 로 감싸 logger 실패가 spin loop 을 죽이지 않게 함.
- 로그 호출 위치는 spin 스레드 한 곳 + dynreg/subscriber 콜백 (lock 으로 직렬화).
"""

import csv
import json
import os
import threading
import time
from datetime import datetime

import yaml


class DebugLogger:
    """OT 튜닝용 sampling_planner debug 로거.

    Per-session 폴더에 4개 산출물 + (옵션) snapshots/ 를 만든다::

        debug_log/<instance>_<timestamp>_session/
            tick.csv             ← 매 tick 1줄, ~44 컬럼
            events.jsonl         ← 비주기 이벤트
            params_snapshot.yaml ← 시작 시점 모든 param
            meta.yaml            ← node 메타 (git_sha, map, …)
            snapshots/           ← (옵션) tick_NNNNNN.npz
    """

    # ---- column schemas -------------------------------------------------
    DECISION_COLUMNS = [
        't_ros', 'tick_idx', 'state',
        'ego_s', 'ego_n', 'ego_v',
        'opp_count', 'opp_id_lead', 'opp_s_lead', 'opp_n_lead', 'opp_vs_lead', 'opp_pred_age_lead',
        'chosen_mode', 'prev_mode', 'mode_changed', 'lock_remain_s',
        'cost_follow', 'cost_ot_left', 'cost_ot_right',
        'best_idx_follow', 'best_idx_ot_left', 'best_idx_ot_right',
        'n_endpoint_chosen', 'v_endpoint_chosen',
        'total_progress_m',
        'kappa_max_chosen', 'kappa_dot_max_chosen', 'n_swing_chosen',
        'boundary_min_margin_chosen', 'prediction_min_dist_chosen',
        'n_valid_candidates', 'n_total_candidates',
        'calc_traj_ms', 'post_add_ms', 'total_tick_ms',
        # ### HJ : MPPI blending diagnostics — softmax effective sample size,
        # top-1 weight, temperature used, # of valid candidates blended.
        # 0/NaN if mppi_enable=False or _mppi_blend returned None this tick.
        'mppi_eff_n', 'mppi_top1_w', 'mppi_T_used', 'mppi_n_blend',
        'status',
    ]

    # rqt 슬라이더 변경이 즉시 다음 tick row 에 반영되도록 매 tick 같이 기록.
    PARAM_COLUMNS = [
        'p_raceline_w', 'p_velocity_w', 'p_prediction_w', 'p_continuity_w', 'p_boundary_w',
        'p_safety_margin_m', 'p_side_w', 'p_progress_w', 'p_opp_pred_ttl_s',
        'p_hys_ttl_s', 'p_hys_margin', 'p_kappa_dot_max',
        'p_filter_alpha', 'p_mppi_temp_rel', 'p_mppi_temporal_w', 'p_horizon',
    ]

    TICK_COLUMNS = DECISION_COLUMNS + PARAM_COLUMNS

    def __init__(self, base_dir, instance_name, state, rate_hz,
                 params_snapshot, meta, snapshot_every_n=0):
        """``base_dir`` 아래 ``<instance>_<timestamp>_session/`` 폴더 생성 후 핸들 open.

        Parameters
        ----------
        base_dir : str
            예: ``<pkg>/debug_log``. 없으면 makedirs.
        instance_name : str
            노드 이름 (예: ``/sampling_planner/overtake``). 폴더 이름 prefix 로 사용.
        state : str
            'overtake' / 'recovery' — meta.yaml 에 들어감.
        rate_hz : float
            spin rate, meta.yaml 에 들어감.
        params_snapshot : dict
            시작 시점 모든 dynreg + YAML 값. ``params_snapshot.yaml`` 로 dump.
        meta : dict
            git_sha / map / vehicle 등. ``meta.yaml`` 로 dump.
        snapshot_every_n : int
            >0 이면 그 주기로 candidates/cost npz 저장. 0 이면 비활성.
        """
        try:
            os.makedirs(base_dir, exist_ok=True)
        except Exception:
            pass

        ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        safe_name = (instance_name or 'sampling').strip('/').replace('/', '_')
        self.session_dir = os.path.join(base_dir, '{}_{}_session'.format(safe_name, ts))
        self._snapshot_dir = os.path.join(self.session_dir, 'snapshots')
        try:
            os.makedirs(self._snapshot_dir, exist_ok=True)
        except Exception:
            pass

        self._tick_csv_path = os.path.join(self.session_dir, 'tick.csv')
        self._events_path   = os.path.join(self.session_dir, 'events.jsonl')
        self._params_path   = os.path.join(self.session_dir, 'params_snapshot.yaml')
        self._meta_path     = os.path.join(self.session_dir, 'meta.yaml')

        self.snapshot_every_n = int(max(0, snapshot_every_n))

        self._lock = threading.Lock()
        self._closed = False
        self._tick_fh = None
        self._tick_writer = None
        self._events_fh = None

        # write meta + params snapshot up-front
        self._dump_yaml_safe(self._params_path, params_snapshot or {})
        meta_full = dict(meta or {})
        meta_full.setdefault('start_time_iso', ts)
        meta_full.setdefault('state', state)
        meta_full.setdefault('rate_hz', float(rate_hz))
        meta_full.setdefault('node_name', instance_name)
        self._dump_yaml_safe(self._meta_path, meta_full)

        # open CSV (line buffered) + JSONL handles
        try:
            self._tick_fh = open(self._tick_csv_path, 'w', buffering=1)
            self._tick_writer = csv.DictWriter(
                self._tick_fh, fieldnames=self.TICK_COLUMNS, extrasaction='ignore'
            )
            self._tick_writer.writeheader()
        except Exception:
            self._tick_fh = None
            self._tick_writer = None

        try:
            self._events_fh = open(self._events_path, 'w', buffering=1)
        except Exception:
            self._events_fh = None

        # session-start event marker (events.jsonl 첫 줄)
        self.log_event('session_start', {
            't': time.time(),
            'session_dir': self.session_dir,
            'state': state,
            'snapshot_every_n': self.snapshot_every_n,
        })

    # ---- public API ----------------------------------------------------
    def log_tick(self, fields):
        """매 tick 한 줄 append. ``fields`` 는 TICK_COLUMNS 의 부분집합 dict."""
        if self._closed or self._tick_writer is None:
            return
        try:
            with self._lock:
                self._tick_writer.writerow(fields)
        except Exception:
            pass

    def log_event(self, event_type, payload):
        """비주기 이벤트 기록. ``payload`` 는 임의 dict (직렬화 가능해야 함)."""
        if self._closed or self._events_fh is None:
            return
        try:
            evt = {'t': payload.get('t', time.time()), 'type': event_type}
            for k, v in (payload or {}).items():
                if k == 't':
                    continue
                evt[k] = v
            line = json.dumps(evt, default=_json_default)
            with self._lock:
                self._events_fh.write(line + '\n')
                self._events_fh.flush()
        except Exception:
            pass

    def log_snapshot(self, tick_idx, **arrays):
        """N tick 마다 candidates/cost arrays npz dump. ``snapshot_every_n=0`` 이면 noop."""
        if self._closed or self.snapshot_every_n <= 0:
            return
        try:
            if int(tick_idx) % self.snapshot_every_n != 0:
                return
            import numpy as np
            path = os.path.join(self._snapshot_dir, 'tick_{:06d}.npz'.format(int(tick_idx)))
            np.savez_compressed(path, **arrays)
        except Exception:
            pass

    def close(self):
        if self._closed:
            return
        try:
            self.log_event('session_end', {'t': time.time()})
        except Exception:
            pass
        with self._lock:
            self._closed = True
            try:
                if self._tick_fh is not None:
                    self._tick_fh.close()
            except Exception:
                pass
            try:
                if self._events_fh is not None:
                    self._events_fh.close()
            except Exception:
                pass

    # ---- helpers --------------------------------------------------------
    @staticmethod
    def _dump_yaml_safe(path, data):
        try:
            with open(path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        except Exception:
            pass


def _json_default(obj):
    """numpy scalar / array / Time 등 직렬화 안 되는 객체 fallback."""
    try:
        import numpy as np
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, (np.bool_,)):
            return bool(obj)
    except Exception:
        pass
    return str(obj)
