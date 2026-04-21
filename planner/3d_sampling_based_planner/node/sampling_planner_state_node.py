#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
### HJ : State-aware variant of sampling_planner_node.py.

Mirrors observation-mode SamplingPlannerNode but:
  * ~state ∈ {overtake, recovery, observe} routes the chosen trajectory to
    the state-machine-facing topic — OTWpntArray on ~out/otwpnts for overtake,
    WpntArray on ~out/wpnts for recovery; observe keeps only debug ~best_*.
  * All cost weights + filter / resample / MPPI toggles are exposed through
    dynamic_reconfigure (cfg/SamplingCost.cfg) so rqt can tune them live per
    instance, with save_params / reset_params one-shot triggers writing back
    to ~instance_yaml.
  * Tick-to-tick continuity via (a) continuity_weight post-added to the
    upstream cost_array before re-argmin, (b) 1-pole EMA on (d, V) between
    ticks, (c) optional uniform arc-length resampling on output.
  * Translucent "previous-best" marker published alongside current best so
    tick jitter is visible in RViz.

Upstream core (LocalSamplingPlanner / Track3D / GGManager) is reused unchanged
via import.
"""

import os
import sys
import time
import copy
import threading
import yaml
import numpy as np

import rospy
import rospkg
from std_msgs.msg import String, Float32, Float32MultiArray, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from f110_msgs.msg import WpntArray, Wpnt, OTWpntArray, PredictionArray, ObstacleArray
from dynamic_reconfigure.server import Server
from sampling_based_planner_3d.cfg import SamplingCostConfig


# -- Make upstream src importable -------------------------------------------------------------------
_PKG_DIR      = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_UPSTREAM_SRC = os.path.join(_PKG_DIR, 'src')
_SHARED_SRC   = os.path.abspath(os.path.join(_PKG_DIR, '..', '3d_gb_optimizer', 'global_line', 'src'))
for _p in (_UPSTREAM_SRC, _SHARED_SRC):
    if _p and os.path.isdir(_p) and _p not in sys.path:
        sys.path.append(_p)

from track3D import Track3D                              # noqa: E402
from ggManager import GGManager                          # noqa: E402
from sampling_based_planner import LocalSamplingPlanner  # noqa: E402


_VALID_STATES = ('overtake', 'recovery', 'observe')

# ### HJ : Phase 2 mode-aware overtake. Each mode picks its own best candidate via the
# same base cost_array + per-mode side/progress post-add. Hysteresis selects which mode
# the chosen trajectory comes from. Recovery/observe states bypass the loop entirely.
_OVERTAKE_MODES = ('follow', 'ot_left', 'ot_right')
# Default opponent prediction step. The publisher (3d_opp_prediction.py) hard-codes
# `self.dt = 0.02` and emits a Prediction[] of N points at i*dt offsets — kept in sync
# here so we don't need a per-message stride parameter.
_OPP_PRED_DT_DEFAULT = 0.02


class SamplingPlannerStateNode:

    # =============================================================================================
    # Init
    # =============================================================================================
    def __init__(self):
        rospy.init_node('sampling_planner_state_node', anonymous=False)

        # -- Role ---------------------------------------------------------------------------------
        state = str(rospy.get_param('~state', 'overtake')).lower().strip()
        if state not in _VALID_STATES:
            rospy.logwarn('[sampling][%s] invalid ~state=%r — falling back to "observe".',
                          rospy.get_name(), state)
            state = 'observe'
        self.state = state
        self.instance_yaml_path = rospy.get_param('~instance_yaml', '')

        # -- Paths --------------------------------------------------------------------------------
        # ### HJ : map_name은 3d_base_system.launch가 /map rosparam에 세팅함.
        # 런치에서 경로를 하드코딩하는 대신 /map을 읽어 경로를 동적 구성 (launch 인자 override 가능).
        self.track_csv_path      = rospy.get_param('~track_csv_path', '')
        self.gg_dir_path         = rospy.get_param('~gg_dir_path', '')
        self.vehicle_params_path = rospy.get_param('~vehicle_params_path', '')
        self.raceline_csv_path   = rospy.get_param('~raceline_csv_path', '')
        # ### HJ : vehicle_name is the 3D physics-model id, independent from racecar_version
        # (hardware id). No racecar_version fallback — empty means use default in path resolver.
        self.vehicle_name        = rospy.get_param('~vehicle_name', '')
        self._resolve_paths_from_map_rosparam()

        # -- Rates / IO ---------------------------------------------------------------------------
        self.rate_hz  = float(rospy.get_param('~rate_hz', 30.0))
        self.frame_id = rospy.get_param('~frame_id', 'map')

        # -- Upstream planner params --------------------------------------------------------------
        self.horizon             = float(rospy.get_param('~horizon', 2.0))
        self.num_samples         = int(rospy.get_param('~num_samples', 30))
        self.n_samples           = int(rospy.get_param('~n_samples', 11))
        self.v_samples           = int(rospy.get_param('~v_samples', 5))
        self.safety_distance     = float(rospy.get_param('~safety_distance', 0.20))
        self.gg_abs_margin       = float(rospy.get_param('~gg_abs_margin', 0.0))
        self.gg_margin_rel       = float(rospy.get_param('~gg_margin_rel', 0.0))
        self.friction_check_2d   = bool(rospy.get_param('~friction_check_2d', False))
        self.relative_generation = bool(rospy.get_param('~relative_generation', True))
        self.s_dot_min           = float(rospy.get_param('~s_dot_min', 1.5))
        self.kappa_thr           = float(rospy.get_param('~kappa_thr', 1.5))

        # -- Cost weights (live-tuneable) --------------------------------------------------------
        self.w_raceline   = float(rospy.get_param('~cost/raceline_weight',   0.1))
        self.w_velocity   = float(rospy.get_param('~cost/velocity_weight',   100.0))
        self.w_prediction = float(rospy.get_param('~cost/prediction_weight', 5000.0))
        self.w_continuity = float(rospy.get_param('~cost/continuity_weight', 50.0))
        self.w_boundary   = float(rospy.get_param('~cost/boundary_weight',   0.0))

        # -- Phase 2: mode-aware overtake + boundary safety --------------------------------------
        # ### HJ : safety_margin_m is the keep-out band INSIDE the half-width.
        # side / progress / hysteresis only act in state=overtake; recovery uses 0.
        self.safety_margin_m   = float(rospy.get_param('~cost/safety_margin_m',   0.15))
        self.w_side            = float(rospy.get_param('~cost/side_weight',       100.0))
        self.w_progress        = float(rospy.get_param('~cost/progress_weight',     5.0))
        self.opp_pred_ttl_s    = float(rospy.get_param('~cost/opp_pred_ttl_s',     0.5))
        self.hysteresis_ttl_s  = float(rospy.get_param('~hysteresis/ttl_s',        1.0))
        self.hysteresis_margin = float(rospy.get_param('~hysteresis/margin',       0.3))
        self.kappa_dot_max     = float(rospy.get_param('~hysteresis/kappa_dot_max', 5.0))

        # Mode hysteresis state — only used in state=overtake.
        self._active_mode      = None
        self._mode_lock_until  = rospy.Time(0)

        # Opponent prediction cache. Single-opponent today (PredictionArray.id is at the outer
        # level), but kept as a dict so multi-opponent extension is a drop-in change.
        self._opp_predictions  = {}     # id → {'t', 's', 'n', 'stamp'}
        self._opp_obstacles    = {}     # id → {'is_static', 's_var', 'd_var', 'vs', 'stamp'}
        self._opp_lock         = threading.Lock()

        # -- EMA between ticks -------------------------------------------------------------------
        self.filter_alpha = float(rospy.get_param('~filter_alpha', 0.7))

        # -- MPPI --------------------------------------------------------------------------------
        self.mppi_enable          = bool(rospy.get_param('~mppi/enable',          True))
        self.mppi_temperature_rel = float(rospy.get_param('~mppi/temperature_rel', 0.25))
        self.mppi_temporal_weight = float(rospy.get_param('~mppi/temporal_weight', 0.0))
        self._prev_blended_s = None
        self._prev_blended_n = None

        # -- Output resampling -------------------------------------------------------------------
        self.resample_enable = bool(rospy.get_param('~resample_enable', True))
        self.resample_ds     = float(rospy.get_param('~resample_ds_m',   0.1))

        # -- Previous-tick best (for continuity cost + EMA) ---------------------------------------
        # Stored in unwrapped / raw form (pre-filter, pre-resample) so subsequent-tick comparisons
        # measure the planner's *own* tick jitter rather than filtered output variance.
        self._prev_best_s     = None
        self._prev_best_n     = None
        self._prev_best_V     = None
        self._prev_best_xyz   = None   # (xs, ys, zs) for the translucent marker

        # -- OT defaults -------------------------------------------------------------------------
        self.ot_side_default = str(rospy.get_param('~ot_side_default', 'right'))

        # -- Vehicle params ----------------------------------------------------------------------
        with open(self.vehicle_params_path, 'r') as f:
            vehicle_yml = yaml.safe_load(f)
        self.params = dict(vehicle_yml)
        rospy.loginfo('[sampling][%s] loaded vehicle params from %s',
                      rospy.get_name(), self.vehicle_params_path)

        # -- Upstream core -----------------------------------------------------------------------
        rospy.loginfo('[sampling][%s] loading Track3D from %s', rospy.get_name(), self.track_csv_path)
        self.track = Track3D(path=self.track_csv_path)

        rospy.loginfo('[sampling][%s] loading gg-diagrams from %s', rospy.get_name(), self.gg_dir_path)
        self.gg = GGManager(gg_path=self.gg_dir_path, gg_margin=self.gg_margin_rel)

        self.planner = LocalSamplingPlanner(
            params=self.params,
            track_handler=self.track,
            gg_handler=self.gg,
        )
        rospy.loginfo('[sampling][%s] LocalSamplingPlanner ready (state=%s).',
                      rospy.get_name(), self.state)

        rospy.loginfo(
            '[sampling][%s][track3d] N=%d  s=[%.3f..%.3f]',
            rospy.get_name(), len(self.track.s),
            float(self.track.s[0]), float(self.track.s[-1]),
        )

        # -- Reference raceline ------------------------------------------------------------------
        self.raceline_dict = self._load_raceline_dict()
        _rs = self.raceline_dict['s']
        _rv = self.raceline_dict['V']
        rospy.loginfo(
            '[sampling][%s][raceline] N=%d  s=[%.3f..%.3f]  V=[%.2f..%.2f]',
            rospy.get_name(), len(_rs), float(_rs[0]), float(_rs[-1]),
            float(np.min(_rv)), float(np.max(_rv)),
        )

        # -- State caches -----------------------------------------------------------------------
        self._cur_x = None
        self._cur_y = None
        self._cur_z = None
        self._cur_vs = None
        self._prev_s_cent = None
        self._prev_xyz    = None
        self._prev_stamp  = None
        self.debug_jump_threshold_m = float(rospy.get_param('~debug_jump_threshold_m', 2.0))

        # -- Subscribers ------------------------------------------------------------------------
        rospy.Subscriber('/car_state/odom', Odometry, self._cb_odom, queue_size=1)
        # ### HJ : Phase 2 — opponent prediction wiring. Currently only one opponent is
        # published (3d_opp_prediction.py id-at-outer-level), but the cache keeps it keyed
        # so multi-opponent works with the same _build_prediction_dict.
        rospy.Subscriber('/opponent_prediction/obstacles_pred',
                         PredictionArray, self._cb_opp_pred, queue_size=2)
        rospy.Subscriber('/opponent_prediction/obstacles',
                         ObstacleArray, self._cb_opp_obs,  queue_size=2)

        # -- Publishers (debug — always) ---------------------------------------------------------
        self.pub_wpnts        = rospy.Publisher('~best_trajectory', WpntArray,   queue_size=1)
        self.pub_best_sample  = rospy.Publisher('~best_sample',     Path,        queue_size=1)
        self.pub_best_markers = rospy.Publisher('~best_sample/markers',     MarkerArray, queue_size=10)
        self.pub_vel_markers  = rospy.Publisher('~best_sample/vel_markers', MarkerArray, queue_size=10)
        self.pub_candidates   = rospy.Publisher('~candidates',      MarkerArray, queue_size=10)
        self.pub_prev_marker  = rospy.Publisher('~prev_best/markers', MarkerArray, queue_size=10)
        self.pub_status       = rospy.Publisher('~status',  String,  queue_size=1, latch=True)
        self.pub_timing       = rospy.Publisher('~timing_ms', Float32, queue_size=1)
        # ### HJ : Phase 2 debug topics — observable mode lock state for tuning.
        self.pub_active_mode  = rospy.Publisher('~active_mode', String, queue_size=1, latch=True)
        self.pub_mode_costs   = rospy.Publisher('~mode_costs',  Float32MultiArray, queue_size=1)

        # -- Publishers (role-specific) ---------------------------------------------------------
        if self.state == 'overtake':
            self.pub_out = rospy.Publisher('~out/otwpnts', OTWpntArray, queue_size=1)
        elif self.state == 'recovery':
            self.pub_out = rospy.Publisher('~out/wpnts',   WpntArray,   queue_size=1)
        else:
            self.pub_out = None

        # -- dynamic_reconfigure -----------------------------------------------------------------
        # Seed rqt sliders with the rosparam-loaded values — otherwise they snap to .cfg defaults
        # on first connect and silently overwrite whatever was in the YAML.
        # ### HJ : _gg_lock guards swaps of self.gg / self.planner.gg_handler from the dynreg
        # thread so the planner tick never observes a half-rebuilt handler.
        self._gg_lock = threading.Lock()
        self._suppress_dynreg_cb = False
        self._dyn_srv = Server(SamplingCostConfig, self._weight_cb)
        self._push_params_to_dynreg()

        self._publish_status('INIT_OK')
        rospy.loginfo(
            '[sampling][%s] ready — state=%s  rl=%.3f v=%.3f p=%.3f c=%.3f b=%.3f  '
            'α=%.2f  mppi=%s/%.3f/%.1f  resample=%s/%.2f',
            rospy.get_name(), self.state,
            self.w_raceline, self.w_velocity, self.w_prediction,
            self.w_continuity, self.w_boundary,
            self.filter_alpha,
            self.mppi_enable, self.mppi_temperature_rel, self.mppi_temporal_weight,
            self.resample_enable, self.resample_ds,
        )

    # =============================================================================================
    # Path resolution (/map rosparam)
    # =============================================================================================
    def _resolve_paths_from_map_rosparam(self):
        """### HJ : 경로 fallback 해석기.
        launch에서 ~track_csv_path / ~gg_dir_path / ~vehicle_params_path / ~raceline_csv_path를
        전부 하드코딩하는 대신, 비어 있으면 /map (3d_base_system.launch line 27)에서 map 이름을
        읽어 stack_master/maps/<map>/<map>_3d_* 및 global_line_3d/data/* 표준 경로로 동적 구성.

        우선순위: 명시 launch arg(~*_path) > /map rosparam 기반 자동 구성.
        """
        rp = rospkg.RosPack()

        need_paths = not all([self.track_csv_path, self.gg_dir_path, self.vehicle_params_path])
        if not need_paths and self.raceline_csv_path:
            return

        map_name = rospy.get_param('/map', '')
        if not map_name:
            map_name = rospy.get_param('~map_name', '')
        if not map_name:
            raise RuntimeError(
                '[sampling] cannot resolve paths: /map rosparam empty and ~map_name unset. '
                'Either launch 3d_base_system.launch first (sets /map) or pass ~map_name/~*_path.')

        # ### HJ : vehicle_name is a 3D physics-model id, separate from racecar_version
        # (hardware id). Default to 'rc_car_10th' when unset — NO racecar_version fallback.
        vehicle_name = self.vehicle_name or 'rc_car_10th'
        if not self.vehicle_name:
            rospy.logwarn("[sampling] ~vehicle_name unset — defaulting to 'rc_car_10th'. "
                          "Pass vehicle_name:=<id> on the launch cmdline to override.")
            self.vehicle_name = vehicle_name
        try:
            stack_master_dir = rp.get_path('stack_master')
            global_line_3d_dir = rp.get_path('global_line_3d')
        except rospkg.ResourceNotFound as e:
            raise RuntimeError('[sampling] rospack missing required package: %s' % e)

        map_dir = os.path.join(stack_master_dir, 'maps', map_name)

        if not self.track_csv_path:
            self.track_csv_path = os.path.join(map_dir, '%s_3d_smoothed.csv' % map_name)
        if not self.raceline_csv_path:
            self.raceline_csv_path = os.path.join(
                map_dir, '%s_3d_%s_timeoptimal.csv' % (map_name, vehicle_name))
        if not self.gg_dir_path:
            self.gg_dir_path = os.path.join(
                global_line_3d_dir, 'data', 'gg_diagrams', vehicle_name, 'velocity_frame')
        if not self.vehicle_params_path:
            self.vehicle_params_path = os.path.join(
                global_line_3d_dir, 'data', 'vehicle_params', 'params_%s.yml' % vehicle_name)

        rospy.loginfo('[sampling] resolved paths from /map=%s (vehicle=%s):', map_name, vehicle_name)
        rospy.loginfo('[sampling]   track    = %s', self.track_csv_path)
        rospy.loginfo('[sampling]   raceline = %s', self.raceline_csv_path)
        rospy.loginfo('[sampling]   gg_dir   = %s', self.gg_dir_path)
        rospy.loginfo('[sampling]   params   = %s', self.vehicle_params_path)

    # =============================================================================================
    # dynamic_reconfigure
    # =============================================================================================
    def _push_params_to_dynreg(self):
        """Push our current member values into the dynreg server so rqt's initial view matches
        the YAML-loaded params instead of the .cfg defaults."""
        self._suppress_dynreg_cb = True
        try:
            self._dyn_srv.update_configuration({
                'raceline_weight':      float(self.w_raceline),
                'velocity_weight':      float(self.w_velocity),
                'prediction_weight':    float(self.w_prediction),
                'continuity_weight':    float(self.w_continuity),
                'boundary_weight':      float(self.w_boundary),
                'filter_alpha':         float(self.filter_alpha),
                'mppi_enable':          bool(self.mppi_enable),
                'mppi_temperature':     float(self.mppi_temperature_rel),
                'mppi_temporal_weight': float(self.mppi_temporal_weight),
                'resample_enable':      bool(self.resample_enable),
                'resample_ds_m':        float(self.resample_ds),
                'horizon':              float(self.horizon),
                'num_samples':          int(self.num_samples),
                'n_samples':            int(self.n_samples),
                'v_samples':            int(self.v_samples),
                'safety_distance':      float(self.safety_distance),
                'gg_abs_margin':        float(self.gg_abs_margin),
                'friction_check_2d':    bool(self.friction_check_2d),
                's_dot_min':            float(self.s_dot_min),
                'kappa_thr':            float(self.kappa_thr),
                'relative_generation':  bool(self.relative_generation),
                'gg_margin_rel':        float(self.gg_margin_rel),
                # Phase 2
                'safety_margin_m':      float(self.safety_margin_m),
                'side_weight':          float(self.w_side),
                'progress_weight':      float(self.w_progress),
                'opp_pred_ttl_s':       float(self.opp_pred_ttl_s),
                'hysteresis_ttl_s':     float(self.hysteresis_ttl_s),
                'hysteresis_margin':    float(self.hysteresis_margin),
                'kappa_dot_max':        float(self.kappa_dot_max),
                'save_params':          False,
                'reset_params':         False,
            })
        finally:
            self._suppress_dynreg_cb = False

    def _weight_cb(self, config, level):
        # save / reset one-shot triggers
        if config.save_params:
            self._save_yaml(config)
            config.save_params = False
        if config.reset_params:
            # ### HJ : Mutate `config` in place and let the server publish it on return.
            # Calling self._dyn_srv.update_configuration() from inside the callback
            # re-enters _weight_cb and the outer return path then overwrites those
            # values with the pre-reload `config` — which is why rqt showed no change.
            new_cfg = self._reload_yaml()
            if new_cfg is not None:
                for k, v in new_cfg.items():
                    if hasattr(config, k):
                        setattr(config, k, v)
                rospy.loginfo('[sampling][%s][reset] reloaded YAML: %s',
                              rospy.get_name(), self.instance_yaml_path)
            config.reset_params = False

        # normal parameter updates — cost / filter / mppi / resample (hot, no rebuild)
        self.w_raceline          = float(config.raceline_weight)
        self.w_velocity          = float(config.velocity_weight)
        self.w_prediction        = float(config.prediction_weight)
        self.w_continuity        = float(config.continuity_weight)
        self.w_boundary          = float(config.boundary_weight)
        self.filter_alpha        = float(config.filter_alpha)
        self.mppi_enable         = bool(config.mppi_enable)
        self.mppi_temperature_rel = float(config.mppi_temperature)
        self.mppi_temporal_weight = float(config.mppi_temporal_weight)
        self.resample_enable     = bool(config.resample_enable)
        self.resample_ds         = float(config.resample_ds_m)

        # ### HJ : sampling grid / horizon / safety — forwarded per tick into
        # calc_trajectory(), so just overwriting self.* is enough. No rebuild.
        self.horizon             = float(config.horizon)
        self.num_samples         = int(config.num_samples)
        self.n_samples           = int(config.n_samples)
        self.v_samples           = int(config.v_samples)
        self.safety_distance     = float(config.safety_distance)
        self.gg_abs_margin       = float(config.gg_abs_margin)
        self.friction_check_2d   = bool(config.friction_check_2d)
        self.s_dot_min           = float(config.s_dot_min)
        self.kappa_thr           = float(config.kappa_thr)
        self.relative_generation = bool(config.relative_generation)

        # ### HJ : gg_margin_rel — baked into GGManager's CasADi interpolants at
        # construction time. Detect change and rebuild; swap handler into planner.
        new_gg_margin = float(config.gg_margin_rel)
        if abs(new_gg_margin - self.gg_margin_rel) > 1e-9:
            self._rebuild_gg_manager(new_gg_margin)

        # Phase 2 — boundary / mode-aware / hysteresis (all hot, no rebuild).
        self.safety_margin_m   = float(config.safety_margin_m)
        self.w_side            = float(config.side_weight)
        self.w_progress        = float(config.progress_weight)
        self.opp_pred_ttl_s    = float(config.opp_pred_ttl_s)
        self.hysteresis_ttl_s  = float(config.hysteresis_ttl_s)
        self.hysteresis_margin = float(config.hysteresis_margin)
        self.kappa_dot_max     = float(config.kappa_dot_max)

        if not self._suppress_dynreg_cb:
            rospy.loginfo_throttle(
                2.0,
                '[sampling][%s] tune rl=%.3f v=%.2f p=%.1f c=%.2f b=%.2f  α=%.2f  '
                'mppi=%s/%.3f/%.1f  resample=%s/%.2f  '
                'H=%.2f N=%d/%d/%d safe=%.2f gg_abs=%.2f f2d=%s sdotmin=%.2f κthr=%.2f rel=%s '
                'gg_rel=%.3f',
                rospy.get_name(),
                self.w_raceline, self.w_velocity, self.w_prediction,
                self.w_continuity, self.w_boundary, self.filter_alpha,
                self.mppi_enable, self.mppi_temperature_rel, self.mppi_temporal_weight,
                self.resample_enable, self.resample_ds,
                self.horizon, self.num_samples, self.n_samples, self.v_samples,
                self.safety_distance, self.gg_abs_margin, self.friction_check_2d,
                self.s_dot_min, self.kappa_thr, self.relative_generation,
                self.gg_margin_rel,
            )
        return config

    def _rebuild_gg_manager(self, new_gg_margin):
        """### HJ : Rebuild GGManager (= re-JIT CasADi interpolants) with a new relative
        margin and swap it into the live LocalSamplingPlanner. Runs on the dynreg thread.

        Blocks the planner tick via _gg_lock so calc_trajectory never sees a half-built
        handler. Typical cost: ~100-300 ms depending on the gg-diagram grid size.
        """
        t0 = rospy.Time.now()
        rospy.loginfo('[sampling][%s] rebuilding GGManager: gg_margin_rel %.3f → %.3f',
                      rospy.get_name(), self.gg_margin_rel, new_gg_margin)
        try:
            new_gg = GGManager(gg_path=self.gg_dir_path, gg_margin=new_gg_margin)
        except Exception as e:
            rospy.logerr('[sampling][%s] GGManager rebuild failed, keeping old: %s',
                         rospy.get_name(), e)
            return
        with self._gg_lock:
            self.gg = new_gg
            self.planner.gg_handler = new_gg
            self.gg_margin_rel = new_gg_margin
        dt_ms = (rospy.Time.now() - t0).to_sec() * 1000.0
        rospy.loginfo('[sampling][%s] GGManager rebuilt in %.1f ms', rospy.get_name(), dt_ms)

    def _save_yaml(self, config):
        if not self.instance_yaml_path:
            rospy.logwarn('[sampling][%s][save] ~instance_yaml not set — skipping',
                          rospy.get_name())
            return
        try:
            if os.path.exists(self.instance_yaml_path):
                with open(self.instance_yaml_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {}
            # cost weights
            data.setdefault('cost', {})
            data['cost']['raceline_weight']   = float(config.raceline_weight)
            data['cost']['velocity_weight']   = float(config.velocity_weight)
            data['cost']['prediction_weight'] = float(config.prediction_weight)
            data['cost']['continuity_weight'] = float(config.continuity_weight)
            data['cost']['boundary_weight']   = float(config.boundary_weight)
            # Phase 2 — mode-aware overtake fields live alongside the base cost block.
            data['cost']['safety_margin_m']   = float(config.safety_margin_m)
            data['cost']['side_weight']       = float(config.side_weight)
            data['cost']['progress_weight']   = float(config.progress_weight)
            data['cost']['opp_pred_ttl_s']    = float(config.opp_pred_ttl_s)
            data.setdefault('hysteresis', {})
            data['hysteresis']['ttl_s']         = float(config.hysteresis_ttl_s)
            data['hysteresis']['margin']        = float(config.hysteresis_margin)
            data['hysteresis']['kappa_dot_max'] = float(config.kappa_dot_max)
            # sampling grid / horizon / safety (top-level, matches YAML layout)
            data['horizon']             = float(config.horizon)
            data['num_samples']         = int(config.num_samples)
            data['n_samples']           = int(config.n_samples)
            data['v_samples']           = int(config.v_samples)
            data['safety_distance']     = float(config.safety_distance)
            data['gg_abs_margin']       = float(config.gg_abs_margin)
            data['gg_margin_rel']       = float(config.gg_margin_rel)
            data['friction_check_2d']   = bool(config.friction_check_2d)
            data['s_dot_min']           = float(config.s_dot_min)
            data['kappa_thr']           = float(config.kappa_thr)
            data['relative_generation'] = bool(config.relative_generation)
            # filter / mppi / resample
            data['filter_alpha'] = float(config.filter_alpha)
            data.setdefault('mppi', {})
            data['mppi']['enable']          = bool(config.mppi_enable)
            data['mppi']['temperature_rel'] = float(config.mppi_temperature)
            data['mppi']['temporal_weight'] = float(config.mppi_temporal_weight)
            data['resample_enable'] = bool(config.resample_enable)
            data['resample_ds_m']   = float(config.resample_ds_m)
            with open(self.instance_yaml_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            rospy.loginfo('[sampling][%s][save] YAML updated: %s',
                          rospy.get_name(), self.instance_yaml_path)
        except Exception as e:
            rospy.logerr('[sampling][%s][save] failed: %s', rospy.get_name(), e)

    def _reload_yaml(self):
        if not self.instance_yaml_path or not os.path.exists(self.instance_yaml_path):
            rospy.logwarn('[sampling][%s][reset] ~instance_yaml missing — skip',
                          rospy.get_name())
            return None
        try:
            with open(self.instance_yaml_path, 'r') as f:
                d = yaml.safe_load(f) or {}
            cost = d.get('cost', {}) or {}
            mppi = d.get('mppi', {}) or {}
            hyst = d.get('hysteresis', {}) or {}
            return {
                'raceline_weight':      float(cost.get('raceline_weight',   self.w_raceline)),
                'velocity_weight':      float(cost.get('velocity_weight',   self.w_velocity)),
                'prediction_weight':    float(cost.get('prediction_weight', self.w_prediction)),
                'continuity_weight':    float(cost.get('continuity_weight', self.w_continuity)),
                'boundary_weight':      float(cost.get('boundary_weight',   self.w_boundary)),
                'safety_margin_m':      float(cost.get('safety_margin_m',   self.safety_margin_m)),
                'side_weight':          float(cost.get('side_weight',       self.w_side)),
                'progress_weight':      float(cost.get('progress_weight',   self.w_progress)),
                'opp_pred_ttl_s':       float(cost.get('opp_pred_ttl_s',    self.opp_pred_ttl_s)),
                'hysteresis_ttl_s':     float(hyst.get('ttl_s',             self.hysteresis_ttl_s)),
                'hysteresis_margin':    float(hyst.get('margin',            self.hysteresis_margin)),
                'kappa_dot_max':        float(hyst.get('kappa_dot_max',     self.kappa_dot_max)),
                'filter_alpha':         float(d.get('filter_alpha', self.filter_alpha)),
                'mppi_enable':          bool(mppi.get('enable',          self.mppi_enable)),
                'mppi_temperature':     float(mppi.get('temperature_rel', self.mppi_temperature_rel)),
                'mppi_temporal_weight': float(mppi.get('temporal_weight', self.mppi_temporal_weight)),
                'resample_enable':      bool(d.get('resample_enable', self.resample_enable)),
                'resample_ds_m':        float(d.get('resample_ds_m',   self.resample_ds)),
                'horizon':              float(d.get('horizon',             self.horizon)),
                'num_samples':          int(d.get('num_samples',           self.num_samples)),
                'n_samples':            int(d.get('n_samples',             self.n_samples)),
                'v_samples':            int(d.get('v_samples',             self.v_samples)),
                'safety_distance':      float(d.get('safety_distance',     self.safety_distance)),
                'gg_abs_margin':        float(d.get('gg_abs_margin',       self.gg_abs_margin)),
                'friction_check_2d':    bool(d.get('friction_check_2d',    self.friction_check_2d)),
                's_dot_min':            float(d.get('s_dot_min',           self.s_dot_min)),
                'kappa_thr':            float(d.get('kappa_thr',           self.kappa_thr)),
                'relative_generation':  bool(d.get('relative_generation',  self.relative_generation)),
                'gg_margin_rel':        float(d.get('gg_margin_rel',       self.gg_margin_rel)),
                'save_params':          False,
                'reset_params':         False,
            }
        except Exception as e:
            rospy.logerr('[sampling][%s][reset] parse failed: %s', rospy.get_name(), e)
            return None

    # =============================================================================================
    # Helpers (copied from sampling_planner_node.py — identical logic)
    # =============================================================================================
    def _load_raceline_dict(self):
        if self.raceline_csv_path and os.path.exists(self.raceline_csv_path):
            try:
                import pandas as pd
                df = pd.read_csv(self.raceline_csv_path, comment='#')
                rospy.loginfo('[sampling][%s] loaded raceline CSV: %d rows, cols=%s',
                              rospy.get_name(), len(df), list(df.columns))

                def _pick(candidates, fallback=None):
                    low = {c.lower(): c for c in df.columns}
                    for k in candidates:
                        if k.lower() in low:
                            return df[low[k.lower()]].to_numpy()
                    return fallback

                s = _pick(['s_opt', 's_m', 's'])
                v = _pick(['v_opt', 'vx_mps', 'v', 'vs'])
                if s is None or v is None:
                    raise KeyError(
                        "required s/v columns not found in CSV; got {}".format(list(df.columns))
                    )
                n   = _pick(['n_opt', 'd_m', 'n'],      fallback=np.zeros_like(s))
                chi = _pick(['chi_opt', 'chi'],          fallback=np.zeros_like(s))
                ax  = _pick(['ax_opt', 'ax_mps2', 'ax'], fallback=np.zeros_like(s))
                ay  = _pick(['ay_opt', 'ay_mps2', 'ay'], fallback=np.zeros_like(s))

                # s[-1] pin — see sampling_planner_node.py for full rationale.
                L_track = float(self.track.s[-1])
                EPS_S   = 1e-6
                target_last = L_track - EPS_S
                if len(s) >= 2 and s[-1] != target_last:
                    rospy.logwarn(
                        '[sampling][%s] pinning raceline s[-1]: %.9f → %.9f  (track L=%.9f)',
                        rospy.get_name(), float(s[-1]), target_last, L_track,
                    )
                    s = s.copy()
                    s[-1] = target_last

                v_safe = np.clip(v, 1e-3, None)
                ds = np.diff(s, prepend=s[0])
                t  = np.cumsum(ds / v_safe)
                s_ddot = np.gradient(v, s)
                z = np.zeros_like(s)
                return {
                    't':      t,
                    's':      s,
                    's_dot':  v,
                    's_ddot': s_ddot,
                    'n':      n,
                    'n_dot':  z,
                    'n_ddot': z,
                    'V':      v,
                    'chi':    chi,
                    'ax':     ax,
                    'ay':     ay,
                }
            except Exception as e:
                rospy.logwarn('[sampling][%s] raceline CSV parse failed (%s) — centerline fallback.',
                              rospy.get_name(), e)

        rospy.logwarn('[sampling][%s] no raceline CSV — centerline + 5 m/s fallback.',
                      rospy.get_name())
        L = float(self.track.s[-1])
        s = np.linspace(0.0, L, 500)
        v = np.full_like(s, 5.0)
        t = s / 5.0
        z = np.zeros_like(s)
        return {
            't': t,
            's': s, 's_dot': v, 's_ddot': z,
            'n': z, 'n_dot': z, 'n_ddot': z,
            'V': v, 'chi': z, 'ax': z, 'ay': z,
        }

    def _publish_status(self, msg):
        self.pub_status.publish(String(data=msg))

    def _cb_odom(self, msg):
        self._cur_x  = float(msg.pose.pose.position.x)
        self._cur_y  = float(msg.pose.pose.position.y)
        self._cur_z  = float(msg.pose.pose.position.z)
        self._cur_vs = float(msg.twist.twist.linear.x)

    def _cb_opp_pred(self, msg):
        """### HJ : Cache one PredictionArray per opponent id. The publisher
        ([3d_opp_prediction.py](../../prediction/gp_traj_predictor/src/3d_opp_prediction.py))
        emits a fixed-stride Prediction[] (i*dt offsets, dt=0.02s) so we just
        materialise (t, s, n) arrays here. Stale predictions are dropped by
        opp_pred_ttl_s in _build_prediction_dict.
        """
        if not msg.predictions:
            return
        n = len(msg.predictions)
        t_arr = np.arange(n, dtype=np.float64) * _OPP_PRED_DT_DEFAULT
        s_arr = np.fromiter((p.pred_s for p in msg.predictions), dtype=np.float64, count=n)
        n_arr = np.fromiter((p.pred_d for p in msg.predictions), dtype=np.float64, count=n)
        with self._opp_lock:
            self._opp_predictions[int(msg.id)] = {
                't':     t_arr,
                's':     s_arr,
                'n':     n_arr,
                'stamp': msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now(),
            }

    def _cb_opp_obs(self, msg):
        """### HJ : Cache the static / dynamic flag and Frenet variances per opponent
        for future sigma-aware Gaussian inflation. Today this only seeds opp metadata
        — the cost weights ignore is_static, but mode hysteresis can use it later."""
        if not msg.obstacles:
            return
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        with self._opp_lock:
            for ob in msg.obstacles:
                self._opp_obstacles[int(ob.id)] = {
                    'is_static': bool(ob.is_static),
                    's_var':     float(getattr(ob, 's_var', 0.0)),
                    'd_var':     float(getattr(ob, 'd_var', 0.0)),
                    'vs':        float(getattr(ob, 'vs', 0.0)),
                    'stamp':     stamp,
                }

    def _build_prediction_dict(self):
        """Drop stale entries (>= opp_pred_ttl_s old) and return the upstream-friendly
        `{id: {'t', 's', 'n'}}` dict consumed by LocalSamplingPlanner.calc_trajectory.
        Returns an empty dict when no fresh predictions are available — preserves the
        existing observation-mode behavior in that case."""
        now = rospy.Time.now()
        ttl = max(0.05, float(self.opp_pred_ttl_s))
        out = {}
        with self._opp_lock:
            for opp_id, entry in list(self._opp_predictions.items()):
                age = (now - entry['stamp']).to_sec()
                if age >= ttl:
                    self._opp_predictions.pop(opp_id, None)
                    continue
                out[opp_id] = {'t': entry['t'], 's': entry['s'], 'n': entry['n']}
        return out

    def _cart_to_cl_frenet_exact(self, x, y, z, debug=False):
        """Project (x,y,z) onto Track3D centerline → (s_cent, n_cent).
        Identical to the observation node implementation."""
        xs    = np.asarray(self.track.x)
        ys    = np.asarray(self.track.y)
        zs    = np.asarray(self.track.z)
        s_arr = np.asarray(self.track.s)
        L = float(s_arr[-1])
        N = len(xs)

        d2 = (xs - x) ** 2 + (ys - y) ** 2 + (zs - z) ** 2
        i = int(np.argmin(d2))

        best_s, best_d2 = None, np.inf
        for ja, jb in (((i - 1) % N, i), (i, (i + 1) % N)):
            xa, ya, za = xs[ja], ys[ja], zs[ja]
            xb, yb, zb = xs[jb], ys[jb], zs[jb]
            dxab = xb - xa; dyab = yb - ya; dzab = zb - za
            len2 = dxab * dxab + dyab * dyab + dzab * dzab
            if len2 < 1e-12:
                continue
            t_seg = ((x - xa) * dxab + (y - ya) * dyab + (z - za) * dzab) / len2
            t_seg = max(0.0, min(1.0, t_seg))
            fx = xa + t_seg * dxab; fy = ya + t_seg * dyab; fz = za + t_seg * dzab
            dseg2 = (x - fx) ** 2 + (y - fy) ** 2 + (z - fz) ** 2
            if dseg2 < best_d2:
                best_d2 = dseg2
                sa = s_arr[ja]; sb = s_arr[jb]
                if sb < sa:
                    sb = sb + L
                best_s = sa + t_seg * (sb - sa)
                if best_s >= L:
                    best_s -= L

        theta = float(np.interp(best_s, s_arr, self.track.theta))
        mu    = float(np.interp(best_s, s_arr, self.track.mu))
        phi   = float(np.interp(best_s, s_arr, self.track.phi))
        normal = Track3D.get_normal_vector_numpy(theta, mu, phi)
        xc = float(np.interp(best_s, s_arr, xs))
        yc = float(np.interp(best_s, s_arr, ys))
        zc = float(np.interp(best_s, s_arr, zs))
        n  = float(np.dot(np.array([x - xc, y - yc, z - zc]), normal))
        return float(best_s), n

    # =============================================================================================
    # Continuity cost post-add (acts on upstream candidates + cost_array)
    # =============================================================================================
    def _apply_continuity_cost(self):
        """Post-add a continuity L2 penalty against the previous tick's best to
        self.planner.cost_array, re-run argmin, then rebuild self.planner.trajectory
        from the new best candidate slice.

        Returns True when the best idx changed (and trajectory was rebuilt)."""
        if self.w_continuity <= 0.0 or self._prev_best_s is None:
            return False
        cands    = getattr(self.planner, 'candidates', None)
        cost_arr = getattr(self.planner, 'cost_array', None)
        if cands is None or cost_arr is None:
            return False

        valid = np.asarray(cands['valid'], dtype=bool)
        if not valid.any():
            return False

        s_all = np.asarray(cands['s'], dtype=np.float64)
        n_all = np.asarray(cands['n'], dtype=np.float64)

        L = float(self.track.s[-1])
        m = int(min(self._prev_best_s.shape[0], s_all.shape[1]))
        if m < 2:
            return False

        # Wrap-aware s delta: treat shortest-arc difference.
        ds = s_all[:, :m] - self._prev_best_s[np.newaxis, :m]
        ds = np.where(ds >  L / 2.0, ds - L, ds)
        ds = np.where(ds < -L / 2.0, ds + L, ds)
        dn = n_all[:, :m] - self._prev_best_n[np.newaxis, :m]

        L2 = np.sum(ds * ds + dn * dn, axis=1) / float(m)  # normalise by horizon length
        new_cost = np.asarray(cost_arr, dtype=np.float64).copy() + self.w_continuity * L2
        self.planner.cost_array = new_cost

        old_best = int(self.planner.trajectory.get('optimal_idx', -1))
        masked = np.where(valid, new_cost, np.inf)
        new_best = int(np.argmin(masked))
        if new_best == old_best:
            return False

        self.planner.trajectory = self._extract_traj_from_candidate(new_best)
        return True

    # =============================================================================================
    # Boundary cost post-add (track-edge keep-out using Track3D half-widths)
    # =============================================================================================
    def _apply_boundary_cost(self):
        """### HJ : Phase 2 — penalise candidates whose |n| sits inside (half_w - safety_margin_m).
        Track3D stores `w_tr_left` (positive) and `w_tr_right` (negative-signed). For each
        candidate sample point we pick the half-width on the side n is on, integrate
        max(0, safety_margin - margin)² over the horizon, and add to cost_array."""
        if self.w_boundary <= 0.0:
            return False
        cands    = getattr(self.planner, 'candidates', None)
        cost_arr = getattr(self.planner, 'cost_array', None)
        if cands is None or cost_arr is None:
            return False
        valid = np.asarray(cands['valid'], dtype=bool)
        if not valid.any():
            return False

        s_all = np.asarray(cands['s'], dtype=np.float64)
        n_all = np.asarray(cands['n'], dtype=np.float64)
        t_all = np.asarray(cands['t'], dtype=np.float64)

        L = float(self.track.s[-1])
        s_track = np.asarray(self.track.s, dtype=np.float64)
        wL_arr  = np.asarray(self.track.w_tr_left, dtype=np.float64)
        wR_arr  = np.abs(np.asarray(self.track.w_tr_right, dtype=np.float64))
        s_mod   = np.mod(s_all, L)
        wL = np.interp(s_mod, s_track, wL_arr)
        wR = np.interp(s_mod, s_track, wR_arr)
        half_w = np.where(n_all >= 0.0, wL, wR)
        margin = half_w - np.abs(n_all)
        pen = np.maximum(0.0, float(self.safety_margin_m) - margin) ** 2  # (N, T)

        diff_t = np.diff(t_all, axis=1)
        pen_int = np.sum(pen[:, :-1] * diff_t, axis=1)

        new_cost = np.asarray(cost_arr, dtype=np.float64).copy() + self.w_boundary * pen_int
        self.planner.cost_array = new_cost

        old_best = int(self.planner.trajectory.get('optimal_idx', -1))
        masked = np.where(valid, new_cost, np.inf)
        new_best = int(np.argmin(masked))
        if new_best == old_best:
            return False
        self.planner.trajectory = self._extract_traj_from_candidate(new_best)
        return True

    # =============================================================================================
    # Phase 2 — mode-aware overtake extras
    # =============================================================================================
    def _compute_mode_extras(self, mode):
        """Per-candidate (side + progress) extras for a given OT mode.
        - follow:   target_n = raceline n at endpoint, no progress reward.
        - ot_left:  target_n = +0.5 * w_tr_left at endpoint, progress reward.
        - ot_right: target_n = -0.5 * |w_tr_right| at endpoint, progress reward.
        Returns shape (N_candidates,)."""
        cands = self.planner.candidates
        s_all = np.asarray(cands['s'], dtype=np.float64)
        n_all = np.asarray(cands['n'], dtype=np.float64)
        L = float(self.track.s[-1])
        s_track = np.asarray(self.track.s, dtype=np.float64)
        s_end = np.mod(s_all[:, -1], L)
        n_end = n_all[:, -1]

        if mode == 'follow':
            rl_s = np.asarray(self.raceline_dict['s'], dtype=np.float64)
            rl_n = np.asarray(self.raceline_dict['n'], dtype=np.float64)
            target_n = np.interp(s_end, rl_s, rl_n, period=L)
            prog_w = 0.0
        elif mode == 'ot_left':
            wL = np.interp(s_end, s_track, np.asarray(self.track.w_tr_left, dtype=np.float64))
            target_n = +0.5 * wL
            prog_w = float(self.w_progress)
        elif mode == 'ot_right':
            wR = np.interp(s_end, s_track, np.abs(np.asarray(self.track.w_tr_right, dtype=np.float64)))
            target_n = -0.5 * wR
            prog_w = float(self.w_progress)
        else:
            target_n = np.zeros_like(n_end)
            prog_w = 0.0

        side_pen = float(self.w_side) * (n_end - target_n) ** 2

        # Wrap-aware total Σs over horizon → reward (negative cost) for OT modes only.
        ds_total = s_all[:, -1] - s_all[:, 0]
        ds_total = np.where(ds_total < -L / 2.0, ds_total + L, ds_total)
        ds_total = np.where(ds_total >  L / 2.0, ds_total - L, ds_total)
        prog_reward = -prog_w * ds_total

        return side_pen + prog_reward

    def _select_mode_with_hysteresis(self, cost_per_mode):
        """Mode lock with TTL + relative cost margin. Returns the chosen mode name.
        cost_per_mode : dict[str, float]  (∞ for modes with no valid candidate)."""
        valid_modes = {m: c for m, c in cost_per_mode.items() if np.isfinite(c)}
        if not valid_modes:
            return self._active_mode  # may be None

        best_now = min(valid_modes, key=valid_modes.get)
        now = rospy.Time.now()

        if self._active_mode is None or self._active_mode not in valid_modes:
            self._active_mode = best_now
            self._mode_lock_until = now + rospy.Duration(float(self.hysteresis_ttl_s))
            return best_now

        if now < self._mode_lock_until:
            return self._active_mode

        if best_now == self._active_mode:
            return self._active_mode

        cur_cost = valid_modes[self._active_mode]
        new_cost = valid_modes[best_now]
        # Avoid sign-flip pathologies: only switch when the new mode strictly improves
        # by the configured margin and current cost is positive.
        if cur_cost > 0.0 and new_cost < (1.0 - float(self.hysteresis_margin)) * cur_cost:
            self._active_mode = best_now
            self._mode_lock_until = now + rospy.Duration(float(self.hysteresis_ttl_s))
            return best_now
        return self._active_mode

    def _publish_mode_debug(self, cost_per_mode, chosen_mode):
        mc = Float32MultiArray()
        mc.data = [
            float(cost_per_mode.get('follow',   float('inf'))),
            float(cost_per_mode.get('ot_left',  float('inf'))),
            float(cost_per_mode.get('ot_right', float('inf'))),
        ]
        self.pub_mode_costs.publish(mc)
        lock_remain = max(0.0, (self._mode_lock_until - rospy.Time.now()).to_sec())
        self.pub_active_mode.publish(String(data='%s|lock_remain=%.2fs' %
                                            (chosen_mode if chosen_mode else 'none', lock_remain)))

    def _extract_traj_from_candidate(self, idx):
        """Rebuild a trajectory dict from candidates[idx]. candidates has s/n/V/chi/ax/ay/kappa/t
        (no n_dot / s_dot) — sufficient for _publish_trajectory."""
        cand = self.planner.candidates
        traj = {
            'traj_cnt':    int(self.planner.traj_cnt),
            'optimal_idx': int(idx),
            't':     np.asarray(cand['t'][idx],     dtype=np.float64),
            's':     np.asarray(cand['s'][idx],     dtype=np.float64),
            'n':     np.asarray(cand['n'][idx],     dtype=np.float64),
            'V':     np.asarray(cand['V'][idx],     dtype=np.float64),
            'chi':   np.asarray(cand['chi'][idx],   dtype=np.float64),
            'ax':    np.asarray(cand['ax'][idx],    dtype=np.float64),
            'ay':    np.asarray(cand['ay'][idx],    dtype=np.float64),
            'kappa': np.asarray(cand['kappa'][idx], dtype=np.float64),
        }
        L = float(self.track.s[-1])
        s_mod = np.clip(np.mod(traj['s'], L), 1e-6, L - 1e-6)
        try:
            xyz = self.track.sn2cartesian(s=s_mod, n=traj['n'])
            traj['x'] = np.asarray(xyz[:, 0], dtype=np.float64)
            traj['y'] = np.asarray(xyz[:, 1], dtype=np.float64)
            traj['z'] = np.asarray(xyz[:, 2], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(2.0, '[sampling][%s] sn2cartesian rebuild failed: %s',
                                   rospy.get_name(), e)
            traj['x'] = np.zeros_like(traj['s'])
            traj['y'] = np.zeros_like(traj['s'])
            traj['z'] = np.zeros_like(traj['s'])
        return traj

    # =============================================================================================
    # Main loop
    # =============================================================================================
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self._cur_x is None:
                self._publish_status('WAITING_ODOM')
                rate.sleep()
                continue

            x_cur, y_cur, z_cur = self._cur_x, self._cur_y, self._cur_z
            v_cur = self._cur_vs or 0.0

            s_cent, n_cent = self._cart_to_cl_frenet_exact(x_cur, y_cur, z_cur)
            self._prev_s_cent = s_cent
            self._prev_xyz    = (x_cur, y_cur, z_cur)
            self._prev_stamp  = rospy.Time.now().to_sec()

            state = {
                's':      s_cent,
                'n':      n_cent,
                's_dot':  max(self.s_dot_min, v_cur),
                's_ddot': 0.0,
                'n_dot':  0.0,
                'n_ddot': 0.0,
            }
            # ### HJ : Phase 2 — feed cached opponent predictions into the upstream
            # gaussian-kernel cost. Empty dict preserves observation-only behavior.
            prediction = self._build_prediction_dict()

            t0 = time.time()
            try:
                # ### HJ : hold _gg_lock so a concurrent rqt-triggered GGManager rebuild
                # cannot swap self.planner.gg_handler mid-calc_trajectory.
                with self._gg_lock:
                    self.planner.calc_trajectory(
                        state=state,
                        prediction=prediction,
                        raceline=self.raceline_dict,
                        relative_generation=self.relative_generation,
                        n_samples=self.n_samples,
                        v_samples=self.v_samples,
                        horizon=self.horizon,
                        num_samples=self.num_samples,
                        safety_distance=self.safety_distance,
                        gg_abs_margin=self.gg_abs_margin,
                        friction_check_2d=self.friction_check_2d,
                        s_dot_min=self.s_dot_min,
                        kappa_thr=self.kappa_thr,
                        raceline_cost_weight=self.w_raceline,
                        velocity_cost_weight=self.w_velocity,
                        prediction_cost_weight=self.w_prediction,
                    )
                dt_ms = (time.time() - t0) * 1000.0
                self.pub_timing.publish(Float32(data=dt_ms))

                # Continuity post-add (modifies cost_array and possibly self.planner.trajectory).
                changed = self._apply_continuity_cost()
                # Boundary post-add (track-edge keep-out via Track3D half-widths).
                if self._apply_boundary_cost():
                    changed = True

                # ### HJ : Phase 2 — mode-aware OT selection. Computes per-mode total
                # cost (= base + side + progress) over the same candidate pool, picks
                # the mode via TTL/margin hysteresis, then rebuilds the trajectory
                # from the chosen mode's best idx. Only active in state=overtake.
                chosen_mode = None
                if self.state == 'overtake':
                    cands    = getattr(self.planner, 'candidates', None)
                    cost_arr = getattr(self.planner, 'cost_array', None)
                    if cands is not None and cost_arr is not None:
                        valid = np.asarray(cands['valid'], dtype=bool)
                        if valid.any():
                            base_cost = np.asarray(cost_arr, dtype=np.float64)
                            cost_per_mode = {}
                            best_idx_per_mode = {}
                            mode_cost_arrays = {}
                            for mode in _OVERTAKE_MODES:
                                extras = self._compute_mode_extras(mode)
                                mc = base_cost + extras
                                mode_cost_arrays[mode] = mc
                                masked = np.where(valid, mc, np.inf)
                                bi = int(np.argmin(masked))
                                best_idx_per_mode[mode] = bi
                                cost_per_mode[mode] = float(masked[bi])

                            chosen_mode = self._select_mode_with_hysteresis(cost_per_mode)
                            if chosen_mode is not None and chosen_mode in mode_cost_arrays:
                                self.planner.cost_array = mode_cost_arrays[chosen_mode]
                                bi = best_idx_per_mode[chosen_mode]
                                if bi != int(self.planner.trajectory.get('optimal_idx', -1)):
                                    self.planner.trajectory = self._extract_traj_from_candidate(bi)
                                    changed = True
                            self._publish_mode_debug(cost_per_mode, chosen_mode)

                traj = self.planner.trajectory
                if traj and 'x' in traj and len(traj['x']) > 0:
                    if self.mppi_enable:
                        blended = self._mppi_blend()
                        if blended is not None:
                            traj = blended
                    self._publish_trajectory(traj)
                    self._publish_candidates(traj.get('optimal_idx', -1))
                    if changed:
                        self._publish_status('CONTINUITY_SWITCH')
                    else:
                        self._publish_status('OK')
                else:
                    self._publish_status('NO_FEASIBLE')

            except Exception as e:
                rospy.logerr_throttle(2.0, '[sampling][%s] calc_trajectory failed: %s',
                                     rospy.get_name(), e)
                self._publish_status('EXCEPTION:' + type(e).__name__)

            rate.sleep()

    # =============================================================================================
    # Publishing — trajectory  (EMA + resample + role-specific emission)
    # =============================================================================================
    def _publish_trajectory(self, traj):
        header = Header()
        header.stamp    = rospy.Time.now()
        header.frame_id = self.frame_id

        L = float(self.track.s[-1])

        # -- Unwrap raw s -----------------------------------------------------------------------
        s_raw = np.asarray(traj['s'], dtype=np.float64).copy()
        n_arr = np.asarray(traj['n'], dtype=np.float64).copy()
        V_arr = np.asarray(traj['V'], dtype=np.float64).copy()
        ax_arr = np.asarray(traj['ax'], dtype=np.float64).copy()
        kappa_arr = np.asarray(traj['kappa'], dtype=np.float64).copy()

        ds = np.diff(s_raw)
        wrap_adj = np.cumsum(
            np.where(ds < -L / 2.0,  L, 0.0) +
            np.where(ds >  L / 2.0, -L, 0.0)
        )
        s_unwr = s_raw.copy()
        s_unwr[1:] += wrap_adj

        # -- Backward-step truncation (upstream guard) -----------------------------------------
        BACKWARD_THR = -0.20
        ds_unwr = np.diff(s_unwr)
        bad = np.where(ds_unwr < BACKWARD_THR)[0]
        if len(bad) > 0:
            cut = int(bad[0]) + 1
            s_unwr    = s_unwr[:cut]
            n_arr     = n_arr[:cut]
            V_arr     = V_arr[:cut]
            ax_arr    = ax_arr[:cut]
            kappa_arr = kappa_arr[:cut]

        # -- Snapshot raw-unwrapped best (pre-filter) for next-tick continuity/EMA baseline ----
        s_raw_snap = s_unwr.copy()
        n_raw_snap = n_arr.copy()
        V_raw_snap = V_arr.copy()

        # -- EMA on (n, V) vs previous-tick raw best -------------------------------------------
        if (self._prev_best_n is not None and
                0.0 < self.filter_alpha < 1.0 and
                len(self._prev_best_n) > 0 and
                len(n_arr) > 0):
            a = self.filter_alpha
            m = int(min(len(self._prev_best_n), len(n_arr)))
            if m > 0:
                n_arr[:m] = a * n_arr[:m] + (1.0 - a) * self._prev_best_n[:m]
                V_arr[:m] = a * V_arr[:m] + (1.0 - a) * self._prev_best_V[:m]

        # -- Uniform arc-length resampling ------------------------------------------------------
        if self.resample_enable and len(s_unwr) >= 2 and self.resample_ds > 1e-3:
            s0, s1 = float(s_unwr[0]), float(s_unwr[-1])
            if (s1 - s0) > self.resample_ds:
                s_uniform = np.arange(s0, s1, self.resample_ds)
                if len(s_uniform) == 0 or s_uniform[-1] < s1 - 1e-6:
                    s_uniform = np.append(s_uniform, s1)
                n_arr     = np.interp(s_uniform, s_unwr, n_arr)
                V_arr     = np.interp(s_uniform, s_unwr, V_arr)
                ax_arr    = np.interp(s_uniform, s_unwr, ax_arr)
                kappa_arr = np.interp(s_uniform, s_unwr, kappa_arr)
                s_unwr    = s_uniform

        # -- Cartesian recomputation (consistent with filtered/resampled (s,n)) ---------------
        s_for_cart = np.clip(np.mod(s_unwr, L), 1e-6, L - 1e-6)
        try:
            xyz = self.track.sn2cartesian(s=s_for_cart, n=n_arr)
            xs_arr = np.asarray(xyz[:, 0], dtype=np.float64)
            ys_arr = np.asarray(xyz[:, 1], dtype=np.float64)
            zs_arr = np.asarray(xyz[:, 2], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(2.0, '[sampling][%s] sn2cartesian failed: %s',
                                   rospy.get_name(), e)
            n_pts  = len(s_unwr)
            xs_arr = np.asarray(traj['x'][:n_pts], dtype=np.float64)
            ys_arr = np.asarray(traj['y'][:n_pts], dtype=np.float64)
            zs_arr = np.asarray(traj['z'][:n_pts], dtype=np.float64)

        # -- Cart-gap truncation -------------------------------------------------------------
        if len(xs_arr) >= 2:
            cart_d = np.sqrt(np.diff(xs_arr) ** 2 + np.diff(ys_arr) ** 2 + np.diff(zs_arr) ** 2)
            big = np.where(cart_d > 3.0)[0]
            if len(big) > 0:
                k = int(big[0]) + 1
                xs_arr    = xs_arr[:k]
                ys_arr    = ys_arr[:k]
                zs_arr    = zs_arr[:k]
                s_unwr    = s_unwr[:k]
                n_arr     = n_arr[:k]
                V_arr     = V_arr[:k]
                ax_arr    = ax_arr[:k]
                kappa_arr = kappa_arr[:k]

        if len(xs_arr) == 0:
            return

        # -- Build WpntArray --------------------------------------------------------------------
        wp_arr = WpntArray()
        wp_arr.header = header
        for i in range(len(xs_arr)):
            w = Wpnt()
            w.id          = i
            w.s_m         = float(s_unwr[i])
            w.d_m         = float(n_arr[i])
            w.x_m         = float(xs_arr[i])
            w.y_m         = float(ys_arr[i])
            if hasattr(w, 'z_m'):
                w.z_m = float(zs_arr[i])
            w.vx_mps      = float(V_arr[i])
            w.ax_mps2     = float(ax_arr[i])
            w.kappa_radpm = float(kappa_arr[i])
            wp_arr.wpnts.append(w)

        # -- Publish: debug (always) ------------------------------------------------------------
        self.pub_wpnts.publish(wp_arr)

        path = Path()
        path.header = header
        for i in range(len(xs_arr)):
            ps = PoseStamped()
            ps.header = header
            ps.pose.position.x = float(xs_arr[i])
            ps.pose.position.y = float(ys_arr[i])
            ps.pose.position.z = float(zs_arr[i])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_best_sample.publish(path)

        self._publish_best_markers(header, xs_arr, ys_arr, zs_arr, V_arr)
        self._publish_prev_best_marker(header)

        # -- Publish: role-specific output ------------------------------------------------------
        if self.state == 'overtake' and self.pub_out is not None:
            ot = OTWpntArray()
            ot.header           = header
            ot.last_switch_time = rospy.Time.now()
            ot.side_switch      = False
            # Dominant side of the chosen trajectory (sign of mean n).
            mean_n = float(np.mean(n_arr)) if len(n_arr) > 0 else 0.0
            if mean_n >= 0.0:
                ot.ot_side = 'left'
            else:
                ot.ot_side = 'right'
            ot.ot_line = 'sampling'
            ot.wpnts = wp_arr.wpnts
            self.pub_out.publish(ot)
        elif self.state == 'recovery' and self.pub_out is not None:
            self.pub_out.publish(wp_arr)

        # -- Update previous-tick best for next iteration --------------------------------------
        self._prev_best_s   = s_raw_snap
        self._prev_best_n   = n_raw_snap
        self._prev_best_V   = V_raw_snap
        self._prev_best_xyz = (xs_arr.copy(), ys_arr.copy(), zs_arr.copy())

    # =============================================================================================
    # Markers
    # =============================================================================================
    def _publish_best_markers(self, header, xs_arr, ys_arr, zs_arr, V_arr):
        n_pts = len(xs_arr)
        if n_pts == 0:
            return
        vx_vals = [float(v) for v in V_arr[:n_pts]]
        vx_min = min(vx_vals) if vx_vals else 0.0
        vx_max = max(vx_vals) if vx_vals else 1.0

        def _vel_color(vx):
            t = (vx - vx_min) / (vx_max - vx_min) if vx_max > vx_min else 0.5
            return (max(0.0, min(1.0, 1.0 - 2.0 * (t - 0.5))),
                    max(0.0, min(1.0, 2.0 * t)),
                    0.0)

        loc_markers = MarkerArray()
        clr = Marker(); clr.header = header; clr.action = Marker.DELETEALL
        loc_markers.markers.append(clr)
        for i in range(n_pts):
            mk = Marker()
            mk.header = header
            mk.type = Marker.SPHERE
            mk.id = i + 1
            mk.scale.x = mk.scale.y = mk.scale.z = 0.15
            mk.color.a = 1.0
            mk.color.r, mk.color.g, mk.color.b = _vel_color(vx_vals[i])
            mk.pose.position.x = float(xs_arr[i])
            mk.pose.position.y = float(ys_arr[i])
            mk.pose.position.z = float(zs_arr[i])
            mk.pose.orientation.w = 1.0
            loc_markers.markers.append(mk)
        self.pub_best_markers.publish(loc_markers)

        VEL_SCALE = 0.1317
        vel_markers = MarkerArray()
        clr2 = Marker(); clr2.header = header; clr2.action = Marker.DELETEALL
        vel_markers.markers.append(clr2)
        for i in range(n_pts):
            mk = Marker()
            mk.header = header
            mk.type = Marker.CYLINDER
            mk.id = i + 1
            mk.scale.x = mk.scale.y = 0.1
            height = max(vx_vals[i] * VEL_SCALE, 0.02)
            mk.scale.z = height
            mk.color.a = 0.7
            mk.color.r, mk.color.g, mk.color.b = _vel_color(vx_vals[i])
            mk.pose.position.x = float(xs_arr[i])
            mk.pose.position.y = float(ys_arr[i])
            mk.pose.position.z = float(zs_arr[i]) + height * 0.5
            mk.pose.orientation.w = 1.0
            vel_markers.markers.append(mk)
        self.pub_vel_markers.publish(vel_markers)

    def _publish_prev_best_marker(self, header):
        """Translucent gray LINE_STRIP of the previous tick's best — eyeball jitter in RViz."""
        ma = MarkerArray()
        clr = Marker(); clr.header = header; clr.action = Marker.DELETEALL
        ma.markers.append(clr)
        if self._prev_best_xyz is not None:
            xs_p, ys_p, zs_p = self._prev_best_xyz
            mk = Marker()
            mk.header = header
            mk.ns = 'prev_best'
            mk.id = 1
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale.x = 0.05
            mk.color.a = 0.45
            mk.color.r = 0.8; mk.color.g = 0.8; mk.color.b = 0.8
            mk.pose.orientation.w = 1.0
            for i in range(len(xs_p)):
                p = Point(); p.x = float(xs_p[i]); p.y = float(ys_p[i]); p.z = float(zs_p[i])
                mk.points.append(p)
            ma.markers.append(mk)
        self.pub_prev_marker.publish(ma)

    # =============================================================================================
    # Candidate-samples publishing (gray fan) — copied verbatim from sampling_planner_node.py
    # =============================================================================================
    def _publish_candidates(self, optimal_idx):
        cands = getattr(self.planner, 'candidates', None)
        if cands is None:
            return
        s_all     = np.asarray(cands['s'])
        n_all     = np.asarray(cands['n'])
        valid_all = np.asarray(cands['valid'], dtype=bool)
        N = s_all.shape[0]

        header = Header()
        header.stamp    = rospy.Time.now()
        header.frame_id = self.frame_id

        ma = MarkerArray()
        clr = Marker(); clr.header = header; clr.action = Marker.DELETEALL
        ma.markers.append(clr)

        L = float(self.track.s[-1])
        drawn = 0
        best_marker = None
        _s_arr  = np.asarray(self.track.s)
        _x_arr  = np.asarray(self.track.x)
        _y_arr  = np.asarray(self.track.y)
        _z_arr  = np.asarray(self.track.z)
        _th_arr = np.asarray(self.track.theta)

        for i in range(N):
            is_best  = (i == optimal_idx)
            is_valid = bool(valid_all[i])

            s_row = s_all[i]
            n_row = n_all[i]
            ds = np.diff(s_row)
            wrap_adj = np.cumsum(
                np.where(ds < -L / 2.0,  L, 0.0) +
                np.where(ds >  L / 2.0, -L, 0.0)
            )
            s_unwr = s_row.copy().astype(np.float64)
            s_unwr[1:] += wrap_adj
            s_mod = np.clip(s_unwr % L, 1e-6, L - 1e-6)

            xc = np.interp(s_mod, _s_arr, _x_arr)
            yc = np.interp(s_mod, _s_arr, _y_arr)
            zc = np.interp(s_mod, _s_arr, _z_arr)
            th = np.interp(s_mod, _s_arr, _th_arr)
            xs_ = xc - np.sin(th) * n_row
            ys_ = yc + np.cos(th) * n_row
            zs_ = zc

            mk = Marker()
            mk.header = header
            mk.ns     = 'candidates'
            mk.id     = drawn + 1
            mk.type   = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.pose.orientation.w = 1.0

            if is_best:
                mk.scale.x = 0.07
                mk.color.r = 1.0; mk.color.g = 0.1; mk.color.b = 0.1
                mk.color.a = 1.0
            elif is_valid:
                mk.scale.x = 0.03
                mk.color.r = 0.1; mk.color.g = 0.1; mk.color.b = 0.1
                mk.color.a = 0.45
            else:
                mk.scale.x = 0.015
                mk.color.r = 0.6; mk.color.g = 0.6; mk.color.b = 0.6
                mk.color.a = 0.25

            for k in range(len(xs_)):
                p = Point(); p.x = float(xs_[k]); p.y = float(ys_[k]); p.z = float(zs_[k])
                mk.points.append(p)

            if is_best:
                best_marker = mk
            else:
                ma.markers.append(mk)
            drawn += 1

        if best_marker is not None:
            ma.markers.append(best_marker)
        self.pub_candidates.publish(ma)

    # =============================================================================================
    # MPPI-style weighted blending  (copied from sampling_planner_node.py, uses the continuity-
    # adjusted cost_array so blending inherits the continuity bias for free)
    # =============================================================================================
    def _mppi_blend(self):
        cands = getattr(self.planner, 'candidates', None)
        cost_arr = getattr(self.planner, 'cost_array', None)
        if cands is None or cost_arr is None:
            return None

        valid = np.asarray(cands['valid'], dtype=bool)
        if valid.sum() == 0:
            return None

        cost = np.asarray(cost_arr, dtype=np.float64).copy()
        if self.mppi_temporal_weight > 0.0 and self._prev_blended_s is not None:
            s_arr = np.asarray(cands['s'])
            n_arr = np.asarray(cands['n'])
            m = min(self._prev_blended_s.shape[0], s_arr.shape[1])
            ds = s_arr[:, :m] - self._prev_blended_s[:m]
            dn = n_arr[:, :m] - self._prev_blended_n[:m]
            tempo = np.sum(ds * ds + dn * dn, axis=1)
            cost = cost + self.mppi_temporal_weight * tempo

        valid_costs = cost[valid]
        c_min = float(valid_costs.min())
        c_max = float(valid_costs.max())
        c_range = max(c_max - c_min, 1e-6)
        T = max(self.mppi_temperature_rel * c_range, 1e-6)

        w = np.exp(-(valid_costs - c_min) / T)
        w /= w.sum()

        L = float(self.track.s[-1])
        s_valid = np.asarray(cands['s'])[valid].astype(np.float64).copy()
        ds_valid = np.diff(s_valid, axis=1)
        wrap_adj = np.cumsum(
            np.where(ds_valid < -L / 2.0,  L, 0.0) +
            np.where(ds_valid >  L / 2.0, -L, 0.0),
            axis=1,
        )
        s_valid[:, 1:] += wrap_adj
        anchor = float(np.mean(s_valid[:, 0]))
        for k in range(s_valid.shape[0]):
            while s_valid[k, 0] - anchor > L / 2.0:
                s_valid[k] -= L
            while s_valid[k, 0] - anchor < -L / 2.0:
                s_valid[k] += L

        blended = {
            'traj_cnt': self.planner.traj_cnt,
            'optimal_idx': int(np.arange(valid.size)[valid][int(np.argmax(w))]),
        }
        blended['s'] = np.sum(w[:, None] * s_valid, axis=0)
        for key in ('n', 'V', 'chi', 'ax', 'ay', 'kappa', 't'):
            arr = np.asarray(cands[key])
            if arr.ndim == 1:
                continue
            blended[key] = np.sum(w[:, None] * arr[valid], axis=0)

        try:
            s_for_cart = np.clip(np.mod(blended['s'], L), 1e-6, L - 1e-6)
            xyz = self.track.sn2cartesian(s=s_for_cart, n=blended['n'])
            blended['x'] = np.asarray(xyz[:, 0], dtype=np.float64)
            blended['y'] = np.asarray(xyz[:, 1], dtype=np.float64)
            blended['z'] = np.asarray(xyz[:, 2], dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(2.0, '[sampling][%s][mppi] cartesian failed: %s',
                                   rospy.get_name(), e)
            return None

        self._prev_blended_s = np.asarray(blended['s'], dtype=np.float64).copy()
        self._prev_blended_n = np.asarray(blended['n'], dtype=np.float64).copy()
        return blended


def main():
    node = SamplingPlannerStateNode()
    node.spin()


if __name__ == '__main__':
    main()
