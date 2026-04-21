#!/usr/bin/env python3
"""Rolling-horizon overtake planner — ROS1 Noetic node (20Hz).

Parallel companion to planner/sqp_planner/src/sqp_avoidance_node.py:
  - Subscribes to the same upstream topics.
  - Instead of one-shot SQP at OVERTAKE entry, re-solves every cycle with
    shifted warm-start + regularization + GG-based velocity profile +
    safety/performance abort.
  - Publishes refined OTWpntArray to /planner/rolling/otwpnts; launch-file
    remaps it onto /planner/avoidance/otwpnts so state_machine is unchanged.

Design doc: IY_docs/rolling_horizon_overtake_planner.md
Implementation plan: ~/.claude/plans/casadi-radiant-elephant.md
"""

from __future__ import annotations

import os
import sys
import time
from copy import deepcopy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from scipy.interpolate import PchipInterpolator
import rospy
from std_msgs.msg import Bool, Float32, Float32MultiArray, Header
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
from rospkg import RosPack

import trajectory_planning_helpers as tph
from ccma import CCMA
from f110_msgs.msg import (
    Wpnt, WpntArray, Obstacle, ObstacleArray,
    OTWpntArray, OpponentTrajectory, OppWpnt, BehaviorStrategy,
)
from frenet_converter.frenet_converter import FrenetConverter

from sqp_casadi import CasadiSQPSolver, SQPProblem
from warm_start import shift_solution, analytic_fallback
from velocity_profiler import VelocityProfiler
from abort_checker import AbortChecker, AbortConfig, AbortReason


class OvertakingIYNode:

    def __init__(self):
        rospy.init_node('overtaking_iy_node')

        # ---- params -------------------------------------------------------
        pp = rospy.get_param
        self.rate_hz = pp('~rate_hz', 20.0)
        self.rate = rospy.Rate(self.rate_hz)
        self.dt = 1.0 / self.rate_hz

        self.racecar_version = os.environ.get('CAR_NAME', pp('~racecar_version', 'SRX1'))

        # SQP / shape params (mirrors sqp_avoidance_node.py defaults)
        self.lookahead = pp('~lookahead', 15.0)
        self.width_car = pp('~width_car', 0.30)
        self.evasion_dist = pp('~evasion_dist', 0.65)
        self.spline_bound_mindist = pp('~spline_bound_mindist', 0.20)
        self.avoidance_resolution = int(pp('~avoidance_resolution', 20))
        self.back_to_raceline_before = pp('~back_to_raceline_before', 5.0)
        self.back_to_raceline_after = pp('~back_to_raceline_after', 5.0)
        self.obs_traj_tresh = pp('~obs_traj_tresh', 1.5)

        # Rolling-horizon specific
        self.lambda_reg = pp('~regularization/lambda_reg', 1.0)
        self.homotopy_lock = bool(pp('~regularization/homotopy_lock', True))
        self.shift_lookahead_s = pp('~warm_start/shift_lookahead_s', 0.05)

        # SQP cost weights (tune via yaml to shape overtake path)
        self.lambda_smooth = pp('~weights/lambda_smooth', 500.0)
        self.lambda_start_heading = pp('~weights/lambda_start_heading', 1000.0)
        self.lambda_apex_bias = pp('~weights/lambda_apex_bias', 0.0)
        self.lambda_side = pp('~weights/lambda_side', 300.0)
        self.lambda_jerk = pp('~weights/lambda_jerk', 0.0)
        ## IY : online mode — fixed time-based horizon + soft terminal, bypasses
        ##      ot_section / smart_static gates. Off by default (legacy behavior).
        self.online_mode = bool(pp('~rolling_mode', False))
        self.T_horizon = pp('~T_horizon', 1.0)
        self.s_min_horizon = pp('~s_min_horizon', 3.0)
        self.lambda_term = pp('~weights/lambda_term', 0.0)
        ## IY : raceline tail for local_wpnts continuity (Phase 3).
        #       append raceline beyond overtake horizon with cubic-blended seam.
        self.tail_len_m = pp('~tail_len_m', 8.0)
        self.tail_smooth_m = pp('~tail_smooth_m', 1.5)

        # Abort
        self.abort = AbortChecker(AbortConfig(
            performance_margin_s=pp('~abort/performance_margin_s', 0.1),
            consecutive_cycles=int(pp('~abort/consecutive_cycles', 3)),
            cooldown_s=pp('~abort/cooldown_s', 2.0),
            safety_sigma_multiplier=pp('~abort/safety_sigma_multiplier', 3.0),
        ))
        ### IY : hold-last-valid fallback (toggleable)
        self.hold_last_valid = bool(pp('~fallback/hold_last_valid', False))
        self.max_fail_hold_cycles = int(pp('~fallback/max_fail_hold_cycles', 5))
        self.fail_streak = 0

        self.measure = rospy.get_param('/measure', False)

        # ---- state caches -------------------------------------------------
        self.frenet_state = Odometry()
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_v = 0.0
        self.cur_yaw = 0.0
        self.cur_s = 0.0
        self.current_d = 0.0

        self.scaled_wpnts = None
        self.scaled_wpnts_msg = WpntArray()
        self.scaled_vmax = None
        self.scaled_max_idx = None
        self.scaled_max_s = None
        self.scaled_delta_s = None

        self.wpnts_updated = None
        self.max_s_updated = None
        self.max_idx_updated = None

        self.obs = ObstacleArray()
        self.obs_perception = ObstacleArray()
        self.obs_predict = ObstacleArray()
        self.avoid_static_obs = True

        self.opponent_waypoints = []
        self.max_opp_idx = None
        self.opponent_wpnts_sm = None
        self.opponent_wpnts_d = None
        self.opponent_wpnts_dvar = None
        self.opponent_wpnts_vs = None

        self.ot_section_check = False
        self.smart_static_active = False
        self.local_wpnts = None

        self.ccma = CCMA(w_ma=10, w_cc=3)
        self.global_waypoints = None
        self.converter = None

        # Solver + rolling state
        self.solver = CasadiSQPSolver()
        self.prev_d = None      # np.ndarray
        self.prev_s = None      # np.ndarray
        self.last_ot_side = ''  # 'left' | 'right'
        self.last_desired_side = 'any'
        ### IY : previous cycle total solve time — used to adaptively shift
        ### start_av forward so RViz-visible path matches ego at render time.
        self.last_solve_ms = 0.0

        # Velocity profiler (loads ggv CSVs once)
        cfg_dir = os.path.join(RosPack().get_path('stack_master'), 'config')
        self.vp = VelocityProfiler(cfg_dir, self.racecar_version)

        # ---- topics -------------------------------------------------------
        # Subscribers — mirror sqp_avoidance_node.py:88-98
        rospy.Subscriber('/tracking/obstacles', ObstacleArray, self._obs_perception_cb)
        rospy.Subscriber('/opponent_prediction/obstacles', ObstacleArray, self._obs_prediction_cb)
        rospy.Subscriber('/car_state/odom_frenet', Odometry, self._state_frenet_cb)
        rospy.Subscriber('/car_state/odom', Odometry, self._state_cartesian_cb)
        rospy.Subscriber('/global_waypoints_scaled', WpntArray, self._scaled_wpnts_cb)
        rospy.Subscriber('/behavior_strategy', BehaviorStrategy, self._behavior_cb)
        rospy.Subscriber('/global_waypoints', WpntArray, self._gb_cb)
        rospy.Subscriber('/global_waypoints_updated', WpntArray, self._updated_wpnts_cb)
        rospy.Subscriber('/opponent_trajectory', OpponentTrajectory, self._opp_traj_cb)
        rospy.Subscriber('/ot_section_check', Bool, self._ot_section_cb)
        rospy.Subscriber('/planner/avoidance/smart_static_active', Bool, self._smart_static_cb)

        # Publishers — remapped by launch onto /planner/avoidance/*
        self.evasion_pub = rospy.Publisher('/planner/rolling/otwpnts', OTWpntArray, queue_size=10)
        self.mrks_pub = rospy.Publisher('/planner/rolling/markers', MarkerArray, queue_size=10)
        self.merger_pub = rospy.Publisher('/planner/rolling/merger', Float32MultiArray, queue_size=10)
        self.debug_pub = rospy.Publisher('/planner/rolling/debug', Float32MultiArray, queue_size=10)
        # diagnostic: data = [obs_center_min, obs_center_max, obs_center_std,
        #                     n_constraints, n_considered, cur_s, start_av, end_av,
        #                     d_init_std, d_opt_std]
        self.diag_pub = rospy.Publisher('/planner/rolling/diag', Float32MultiArray, queue_size=10)
        if self.measure:
            self.measure_pub = rospy.Publisher('/planner/rolling/latency', Float32, queue_size=10)

        self.converter = self._init_converter()
        rospy.loginfo('[OvertakingIY] initialized. rate=%.1fHz car=%s lambda_reg=%.3f',
                      self.rate_hz, self.racecar_version, self.lambda_reg)

        ## IY : state_machine hyst override — force latest overtake path refresh.
        self._state_dr_client = None
        self._state_dr_original_hyst = None
        if self.online_mode:
            self._override_state_hyst_timer()
            rospy.on_shutdown(self._restore_state_hyst_timer)
        ## IY : end

    ## IY : state_machine hyst override helpers (Phase 3 P2).
    def _override_state_hyst_timer(self):
        """Force state_machine to refresh cached overtake path every tick."""
        try:
            import dynamic_reconfigure.client
            ns = '/dyn_planners/dynamic_avoidance_planner'
            self._state_dr_client = dynamic_reconfigure.client.Client(
                ns, timeout=3.0)
            self._state_dr_original_hyst = rospy.get_param(
                ns + '/hyst_timer_sec', None)
            self._state_dr_client.update_configuration(
                {'hyst_timer_sec': 0.01})
            rospy.loginfo(
                '[OvertakingIY] state_machine hyst_timer_sec -> 0.01 '
                '(orig=%s)', str(self._state_dr_original_hyst))
        except Exception as e:   # noqa: BLE001
            rospy.logwarn('[OvertakingIY] dyn_recfg override failed: %s', e)

    def _restore_state_hyst_timer(self):
        """Restore state_machine hyst_timer_sec on node shutdown."""
        try:
            if (self._state_dr_client is not None
                    and self._state_dr_original_hyst is not None):
                self._state_dr_client.update_configuration(
                    {'hyst_timer_sec': self._state_dr_original_hyst})
                rospy.loginfo(
                    '[OvertakingIY] restored hyst_timer_sec=%s',
                    str(self._state_dr_original_hyst))
        except Exception:   # noqa: BLE001
            pass
    ## IY : end

    # =====================================================================
    # Callbacks (mirror sqp_avoidance_node.py)
    # =====================================================================
    def _obs_perception_cb(self, data: ObstacleArray):
        self.obs_perception = data
        self.obs_perception.obstacles = [o for o in data.obstacles if o.is_static]
        if self.avoid_static_obs:
            self.obs.header = data.header
            self.obs.obstacles = self.obs_perception.obstacles + self.obs_predict.obstacles

    def _obs_prediction_cb(self, data: ObstacleArray):
        self.obs_predict = data
        self.obs = self.obs_predict
        if self.avoid_static_obs:
            self.obs.obstacles = self.obs.obstacles + self.obs_perception.obstacles
        rospy.loginfo_throttle(2.0,
            '[OvertakingIY] pred_cb: n_pred=%d n_perc_static=%d n_total=%d',
            len(self.obs_predict.obstacles),
            len(self.obs_perception.obstacles),
            len(self.obs.obstacles))

    def _state_frenet_cb(self, data: Odometry):
        self.frenet_state = data
        q = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.cur_yaw = yaw

    def _state_cartesian_cb(self, msg: Odometry):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_v = msg.twist.twist.linear.x

    def _gb_cb(self, data: WpntArray):
        self.global_waypoints = np.array([[w.x_m, w.y_m, w.z_m] for w in data.wpnts])

    def _scaled_wpnts_cb(self, data: WpntArray):
        self.scaled_wpnts = np.array([[w.s_m, w.d_m] for w in data.wpnts])
        self.scaled_wpnts_msg = data
        vmax = np.max(np.array([w.vx_mps for w in data.wpnts]))
        if self.scaled_vmax != vmax:
            self.scaled_vmax = float(vmax)
            self.scaled_max_idx = data.wpnts[-1].id
            self.scaled_max_s = data.wpnts[-1].s_m
            self.scaled_delta_s = data.wpnts[1].s_m - data.wpnts[0].s_m
        # fallback: if /global_waypoints_updated never publishes (no publisher
        # in this launch), reuse scaled as the "updated" waypoints
        if self.wpnts_updated is None:
            self.wpnts_updated = data.wpnts[:-1]
            self.max_s_updated = self.wpnts_updated[-1].s_m
            self.max_idx_updated = self.wpnts_updated[-1].id

    def _updated_wpnts_cb(self, data: WpntArray):
        self.wpnts_updated = data.wpnts[:-1]
        self.max_s_updated = self.wpnts_updated[-1].s_m
        self.max_idx_updated = self.wpnts_updated[-1].id

    def _behavior_cb(self, data: BehaviorStrategy):
        self.local_wpnts = np.array([[w.s_m, w.d_m] for w in data.local_wpnts])

    def _opp_traj_cb(self, data: OpponentTrajectory):
        self.opponent_waypoints = data.oppwpnts
        if len(data.oppwpnts) == 0:
            return
        self.max_opp_idx = len(data.oppwpnts) - 1
        self.opponent_wpnts_sm = np.array([w.s_m for w in data.oppwpnts])
        self.opponent_wpnts_d = np.array([w.d_m for w in data.oppwpnts])
        self.opponent_wpnts_dvar = np.array([w.d_var for w in data.oppwpnts])
        self.opponent_wpnts_vs = np.array([w.proj_vs_mps for w in data.oppwpnts])

    def _ot_section_cb(self, data: Bool):
        self.ot_section_check = data.data

    def _smart_static_cb(self, data: Bool):
        self.smart_static_active = data.data

    # =====================================================================
    # Utilities
    # =====================================================================
    def _init_converter(self) -> FrenetConverter:
        rospy.wait_for_message('/global_waypoints', WpntArray)
        conv = FrenetConverter(self.global_waypoints[:, 0],
                               self.global_waypoints[:, 1],
                               self.global_waypoints[:, 2])
        rospy.loginfo('[OvertakingIY] FrenetConverter initialized')
        return conv

    def _more_space(self, obstacle: Obstacle, gb_wpnts, gb_idxs):
        left_mean = np.mean([gb_wpnts[i].d_left for i in gb_idxs])
        right_mean = np.mean([gb_wpnts[i].d_right for i in gb_idxs])
        left_gap = abs(left_mean - obstacle.d_left)
        right_gap = abs(right_mean + obstacle.d_right)
        min_space = self.evasion_dist + self.spline_bound_mindist
        if right_gap > min_space and left_gap < min_space:
            apex = obstacle.d_right - self.evasion_dist
            return 'right', min(apex, 0.0)
        if left_gap > min_space and right_gap < min_space:
            apex = obstacle.d_left + self.evasion_dist
            return 'left', max(apex, 0.0)
        cand_l = obstacle.d_left + self.evasion_dist
        cand_r = obstacle.d_right - self.evasion_dist
        if abs(cand_l) <= abs(cand_r):
            return 'left', max(cand_l, 0.0)
        return 'right', min(cand_r, 0.0)

    @staticmethod
    def _group_objects(obstacles):
        agg = deepcopy(obstacles[0])
        for o in obstacles:
            agg.d_left = max(agg.d_left, o.d_left)
            agg.d_right = min(agg.d_right, o.d_right)
            agg.s_start = min(agg.s_start, o.s_start)
            agg.s_end = max(agg.s_end, o.s_end)
        agg.s_center = (agg.s_start + agg.s_end) / 2.0
        return agg

    def _clear_markers(self):
        mrks = MarkerArray()
        del_mrk = Marker(header=Header(stamp=rospy.Time.now()))
        del_mrk.ns = 'rolling_path'
        del_mrk.action = Marker.DELETEALL
        mrks.markers = [del_mrk]
        self.mrks_pub.publish(mrks)

    def _publish_empty_otwpnts(self, clear_markers=True):
        msg = OTWpntArray(header=Header(stamp=rospy.Time.now(), frame_id='map'))
        msg.wpnts = []
        self.evasion_pub.publish(msg)
        if clear_markers:
            self._clear_markers()

    ### IY : failure handler — publish empty only if hold disabled or streak exceeded
    def _handle_failure(self, dry_run: bool, is_abort: bool = False) -> None:
        if is_abort or not self.hold_last_valid:
            self.prev_d = None
            self.prev_s = None
            self.fail_streak = 0
            if not dry_run:
                self._publish_empty_otwpnts()
            return
        self.fail_streak += 1
        if self.fail_streak >= self.max_fail_hold_cycles:
            self.prev_d = None
            self.prev_s = None
            self.fail_streak = 0
            if not dry_run:
                self._publish_empty_otwpnts()

    def _publish_debug(self, solve_ms: float, success: bool,
                       t_ot: float, t_trail: float, abort_reason: AbortReason):
        msg = Float32MultiArray()
        flag_map = {AbortReason.NONE: 0.0,
                    AbortReason.SAFETY: 1.0,
                    AbortReason.PERFORMANCE: 2.0}
        msg.data = [solve_ms,
                    1.0 if success else 0.0,
                    t_ot,
                    t_trail,
                    flag_map[abort_reason]]
        self.debug_pub.publish(msg)

    def _visualize(self, s_arr, d_arr, x_arr, y_arr, v_arr, dry_run=False):
        if len(s_arr) == 0:
            return
        mrks = MarkerArray()
        del_mrk = Marker(header=Header(stamp=rospy.Time.now()))
        del_mrk.ns = 'rolling_path'
        del_mrk.action = Marker.DELETEALL
        mrks.markers.append(del_mrk)
        vmax = self.scaled_vmax if self.scaled_vmax else 1.0
        ttl = rospy.Duration(max(0.2, 3.0 / self.rate_hz))
        for i in range(len(s_arr)):
            m = Marker(header=Header(stamp=rospy.Time.now(), frame_id='map'))
            m.ns = 'rolling_path'
            m.type = m.CYLINDER
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = max(v_arr[i] / vmax, 0.05)
            m.color.a = 0.45 if dry_run else 1.0
            if dry_run:
                m.color.r, m.color.g, m.color.b = 1.0, 0.55, 0.0   # orange ghost
            else:
                m.color.r, m.color.g, m.color.b = 0.10, 0.70, 0.85  # cyan active
            m.id = i
            m.pose.position.x = x_arr[i]
            m.pose.position.y = y_arr[i]
            m.pose.position.z = v_arr[i] / vmax / 2.0
            m.pose.orientation.w = 1.0
            m.lifetime = ttl
            mrks.markers.append(m)
        self.mrks_pub.publish(mrks)

    def _publish_status_text(self, n_obs, ot_check, smart_static,
                             solve_ms, solve_ok, side, abort_reason, dry_run):
        if self.cur_x == 0.0 and self.cur_y == 0.0:
            return
        mrks = MarkerArray()
        m = Marker(header=Header(stamp=rospy.Time.now(), frame_id='map'))
        m.type = m.TEXT_VIEW_FACING
        m.id = 999
        m.ns = 'rolling_status'
        m.pose.position.x = self.cur_x
        m.pose.position.y = self.cur_y
        m.pose.position.z = 1.2
        m.pose.orientation.w = 1.0
        m.scale.z = 0.3
        m.color.a = 1.0
        m.lifetime = rospy.Duration(1.0)
        if abort_reason != AbortReason.NONE:
            m.color.r, m.color.g, m.color.b = 1.0, 0.2, 0.2
        elif dry_run:
            m.color.r, m.color.g, m.color.b = 1.0, 0.6, 0.1
        elif solve_ok:
            m.color.r, m.color.g, m.color.b = 0.2, 1.0, 0.4
        else:
            m.color.r, m.color.g, m.color.b = 0.8, 0.8, 0.8
        mode = ('ABORT:' + abort_reason.value) if abort_reason != AbortReason.NONE \
            else ('ACTIVE' if (solve_ok and not dry_run) else
                  ('DRY' if dry_run else 'IDLE'))
        m.text = (f'{mode} | obs={n_obs} ot={int(ot_check)} static={int(smart_static)} '
                  f'solve={solve_ms:.1f}ms side={side}')
        mrks.markers = [m]
        self.mrks_pub.publish(mrks)

    # =====================================================================
    # Main loop
    # =====================================================================
    def _prewarm_solver(self):
        """Force CasADi to build the NLP so the first OT cycle isn't ~500ms.

        Uses a typical (n_knots, n_obs) shape; the resulting warm-start cache
        is discarded immediately via reset_warmstart() so real cycles still
        start from prev_solution=None semantics.
        """
        n = int(self.avoidance_resolution)
        ## IY : fixed-slot obs — fill middle third, rest 0.
        obs_center = np.zeros(n)
        obs_min = np.zeros(n)
        obs_center[n // 3 : 2 * n // 3] = 0.1
        obs_min[n // 3 : 2 * n // 3] = 0.55
        try:
            _, info = self.solver.solve(SQPProblem(
                n_knots=n, delta_s=0.5,
                d_init=np.zeros(n), current_d=0.0,
                bounds_lower=np.full(n, -1.0), bounds_upper=np.full(n, 1.0),
                obs_center_d=obs_center,
                obs_min_dist=obs_min,
                desired_side='right', kappa_limit=0.5,
                lambda_reg=self.lambda_reg,
                lambda_smooth=self.lambda_smooth,
                lambda_start_heading=self.lambda_start_heading,
                lambda_apex_bias=self.lambda_apex_bias,
                lambda_side=self.lambda_side,
            ))
            self.solver.reset_warmstart()
            rospy.loginfo('[OvertakingIY] solver prewarmed (status=%s, iters=%d)',
                          info.get('status'), info.get('iter_count', 0))
        except Exception as exc:   # noqa: BLE001
            rospy.logwarn('[OvertakingIY] prewarm failed (non-fatal): %s', exc)

    def loop(self):
        rospy.loginfo('[OvertakingIY] waiting for upstream topics...')
        rospy.wait_for_message('/global_waypoints_scaled', WpntArray)
        rospy.wait_for_message('/car_state/odom', Odometry)
        rospy.wait_for_message('/behavior_strategy', BehaviorStrategy)
        self._prewarm_solver()
        rospy.loginfo('[OvertakingIY] ready')

        while not rospy.is_shutdown():
            t0 = time.perf_counter()

            # snapshot
            obs_now = deepcopy(self.obs)
            fr = self.frenet_state
            self.current_d = fr.pose.pose.position.y
            self.cur_s = fr.pose.pose.position.x
            rospy.loginfo_throttle(2.0,
                '[OvertakingIY] loop: cur_s=%.2f n_obs=%d max_s=%s wpnts_upd=%s',
                self.cur_s, len(obs_now.obstacles),
                str(self.scaled_max_s is not None),
                str(self.wpnts_updated is not None))

            # filter considered obstacles (copy of sqp_avoidance_node.py:211-215)
            sorted_obs = sorted(obs_now.obstacles, key=lambda o: o.s_start)
            considered = []
            if self.scaled_max_s is not None:
                for o in sorted_obs:
                    if (abs(o.d_center) < self.obs_traj_tresh
                            and (o.s_start - self.cur_s) % self.scaled_max_s < self.lookahead):
                        considered.append(o)

            have_inputs = (self.scaled_max_s is not None
                           and self.wpnts_updated is not None)
            have_obs = len(considered) > 0
            if have_obs and not have_inputs:
                rospy.logwarn_throttle(2.0,
                    '[OvertakingIY] have_obs=%d have_inputs=0 → solve SKIPPED '
                    '(scaled_max_s=%s wpnts_updated=%s)',
                    len(considered),
                    str(self.scaled_max_s is not None),
                    str(self.wpnts_updated is not None))
            # diag: why is _rolling_step not firing?
            if not have_obs and len(sorted_obs) > 0:
                o0 = sorted_obs[0]
                ds = ((o0.s_start - self.cur_s) % self.scaled_max_s
                      if self.scaled_max_s is not None else -1.0)
                rospy.loginfo_throttle(2.0,
                    '[OvertakingIY] filter cut: n_raw=%d d=%.2f ds=%.2f (tresh=%.2f look=%.2f)',
                    len(sorted_obs), o0.d_center, ds,
                    self.obs_traj_tresh, self.lookahead)
            ## IY : online mode bypasses ot_section / smart_static gates.
            if self.online_mode:
                publish_to_sm = (have_inputs and have_obs)
            else:
                publish_to_sm = (have_inputs and have_obs
                                 and self.ot_section_check
                                 and not self.smart_static_active)
            dry_run = (have_inputs and have_obs and not publish_to_sm)

            # leave-OT cleanup: drop warm-start when SM publish is off
            if not publish_to_sm and self.prev_d is not None:
                self.solver.reset_warmstart()
                self.prev_d = None
                self.prev_s = None
                self.abort.reset()

            if not publish_to_sm:
                self._publish_empty_otwpnts(clear_markers=not dry_run)

            solve_ok = False
            t_ot = 0.0
            t_trail = 0.0
            abort_reason = AbortReason.NONE
            side = self.last_desired_side
            try:
                if have_obs and have_inputs:
                    solve_ok, t_ot, t_trail, abort_reason, side = \
                        self._rolling_step(considered, dry_run=dry_run)
            except Exception as exc:   # noqa: BLE001
                rospy.logwarn('[OvertakingIY] rolling_step raised: %s', exc)
                self.solver.reset_warmstart()
                self._handle_failure(dry_run)

            solve_ms = (time.perf_counter() - t0) * 1000.0
            ### IY : feed latency to next cycle for adaptive start_av shift.
            ### Only update on success — a 600ms failure must not poison the
            ### next cycle's dt_solve (caused CCMA 3-point crashes).
            if solve_ok:
                self.last_solve_ms = solve_ms
            self._publish_debug(solve_ms, solve_ok, t_ot, t_trail, abort_reason)
            self._publish_status_text(
                n_obs=len(considered),
                ot_check=self.ot_section_check,
                smart_static=self.smart_static_active,
                solve_ms=solve_ms, solve_ok=solve_ok,
                side=side, abort_reason=abort_reason, dry_run=dry_run)
            if self.measure:
                self.measure_pub.publish(Float32(time.perf_counter() - t0))
            self.rate.sleep()

    # ---------------------------------------------------------------------
    def _rolling_step(self, considered_obs, dry_run=False):
        """One cycle: build RoC, SQP refine d(s), FB velocity, abort check, publish.

        If dry_run=True, skip publishing /planner/rolling/otwpnts to state_machine
        and draw candidate markers in ghost color instead. Used to preview what
        the planner would produce outside the OT section.

        Returns (success, t_ot, t_trail, abort_reason, side_str).
        """
        # ---- build RoC knots and bounds (mirrors sqp_avoidance_node.sqp_solver) ----
        ### IY : cluster first so gb_idxs/side/extension all decide per-cluster
        dyn_obs = sorted([o for o in considered_obs if not o.is_static],
                         key=lambda o: o.s_start)
        static_obs = [o for o in considered_obs if o.is_static]
        dyn_swept = []
        if len(dyn_obs) >= 1:
            gap_thresh = 0.5  # meters; s-gap between distinct opponents
            groups = [[dyn_obs[0]]]
            for o in dyn_obs[1:]:
                if o.s_start - groups[-1][-1].s_end <= gap_thresh:
                    groups[-1].append(o)
                else:
                    groups.append([o])
            for grp in groups:
                if len(grp) == 1:
                    dyn_swept.append(grp[0])
                else:
                    swept = self._group_objects(grp)
                    swept.is_static = False
                    dyn_swept.append(swept)
        clusters = static_obs + dyn_swept
        n_clusters = len(clusters)

        ### IY : gb_idxs = union of per-cluster s-ranges (two separate clusters
        ### no longer pull in the empty gap between them)
        gb_idx_set = set()
        for c in clusters:
            i_start = int(np.abs(self.scaled_wpnts[:, 0] - c.s_start).argmin())
            i_end = int(np.abs(self.scaled_wpnts[:, 0] - c.s_end).argmin())
            span = (i_end - i_start) % self.scaled_max_idx
            for k in range(span + 1):
                gb_idx_set.add((i_start + k) % self.scaled_max_idx)
        gb_idxs = np.array(sorted(gb_idx_set), dtype=int)
        if len(gb_idxs) < 20:
            # fallback: contiguous 20-knot window around first cluster center
            s_c = clusters[0].s_center if hasattr(clusters[0], 's_center') \
                else 0.5 * (clusters[0].s_start + clusters[0].s_end)
            gb_idxs = np.array([int(s_c / self.scaled_delta_s + i) % self.scaled_max_idx
                                for i in range(20)], dtype=int)

        ### IY : per-cluster side decision; aggregates gone
        cluster_sides = []  # list of (cluster, side_str, apex)
        for c in clusters:
            c_side, c_apex = self._more_space(c, self.scaled_wpnts_msg.wpnts, gb_idxs)
            cluster_sides.append((c, c_side, c_apex))

        # initial_apex from ego-nearest cluster (drives warm-start direction)
        def _s_gap(c):
            return (c.s_start - self.cur_s) % self.scaled_max_s
        nearest_idx = int(np.argmin([_s_gap(c) for c, _, _ in cluster_sides]))
        first_side = cluster_sides[nearest_idx][1]
        initial_apex = cluster_sides[nearest_idx][2]

        # homotopy lock applies only to the first (ego-nearest) cluster's side
        if self.homotopy_lock and self.last_desired_side in ('left', 'right'):
            first_side = self.last_desired_side
        self.last_desired_side = first_side
        # single string 'side' kept for debug/status UIs; reflects first cluster
        side = first_side

        kappas = np.array([self.scaled_wpnts_msg.wpnts[i].kappa_radpm for i in gb_idxs])
        max_kappa = np.max(np.abs(kappas))
        outside = 'left' if np.sum(kappas) < 0 else 'right'
        ### IY : extend s_end only within the cluster whose chosen side == outside
        for c, c_side, _ in cluster_sides:
            if c_side != outside:
                continue
            extend = (c.s_end - c.s_start) % self.max_s_updated \
                * max_kappa * (self.width_car + self.evasion_dist)
            c.s_end = c.s_end + extend
            # static clusters are single-obstacle references into considered_obs;
            # dyn_swept are deepcopies so only the cluster envelope shifts

        max_s_obs_end = max(c.s_end for c in clusters)

        # ego-state prediction forward to match path render time (design §3.2 step 0)
        ### IY : adaptive lookahead — prev cycle's solve_ms + 30ms publish/render pad.
        ### Falls back to shift_lookahead_s on first cycle when last_solve_ms==0.
        ### IY : cap dt_solve to prevent feedback loop on repeated SQP timeouts.
        ### 0.25s @ v=4m/s = 1m forward shift; more than that eats the RoC window.
        dt_solve = min(0.25,
                       max(self.shift_lookahead_s,
                           self.last_solve_ms * 1e-3 + 0.03))
        cur_s_pred = self.cur_s + self.cur_v * dt_solve
        start_av = cur_s_pred
        ## IY : online mode uses time-based fixed horizon, decoupled from obstacle.
        if self.online_mode:
            horizon_m = max(self.cur_v * self.T_horizon, self.s_min_horizon)
            end_av = cur_s_pred + horizon_m
        else:
            end_av = max_s_obs_end + self.back_to_raceline_after

        s_av = np.linspace(start_av, end_av, self.avoidance_resolution)
        delta_s = float(s_av[1] - s_av[0])

        idxs = np.array([int(np.abs(self.scaled_wpnts[:, 0] - (s % self.scaled_max_s)).argmin())
                         for s in s_av])
        corr = [self.scaled_wpnts_msg.wpnts[i] for i in idxs]
        bounds = np.array([(-w.d_right + self.spline_bound_mindist,
                            w.d_left - self.spline_bound_mindist) for w in corr])
        d_lb = bounds[:, 0]
        d_ub = bounds[:, 1]

        # obstacle RoC knots
        ## IY : fixed-slot obs arrays (length n_knots), inactive = 0.
        n_knots = len(s_av)
        obs_center = np.zeros(n_knots)
        obs_min = np.zeros(n_knots)
        ### IY : always iterate clusters built above; each cluster produces an
        ### independent clearance group with one fixed envelope center to avoid
        ### GP-curve wiggle inside a corridor.
        ### IY : knot 0,1 are reserved for smooth transition from current_d —
        ### clearance starts from knot 2 to avoid infeasibility when ego is
        ### already within the exclusion zone.
        pre_buffer = 2
        for o in clusters:
            i0 = int(np.abs(s_av - o.s_start).argmin())
            i1 = int(np.abs(s_av - o.s_end).argmin())
            if i0 >= len(s_av) - 2:
                continue
            i0 = max(i0, pre_buffer)
            if i1 < i0:
                continue
            if o.is_static or i1 == i0:
                if i1 == i0:
                    i1 = i0 + 1
                for ii in range(i0, i1 + 1):
                    obs_center[ii] = (o.d_left + o.d_right) / 2.0
                    obs_min[ii] = ((o.d_left - o.d_right) / 2.0
                                   + self.width_car + self.evasion_dist)
            else:
                fixed_center = (o.d_left + o.d_right) / 2.0
                for ii in range(i0, i1 + 1):
                    obs_center[ii] = fixed_center
                    obs_min[ii] = self.width_car + self.evasion_dist

        # curvature constraint: interpolated min-radius like sqp_avoidance_node.py:331-337
        clipped_v = max(self.cur_v, 1.0)
        first_upd_v = self.wpnts_updated[idxs[0] % self.max_idx_updated].vx_mps
        radius_speed = min(clipped_v, first_upd_v)
        min_radius = float(np.interp(radius_speed, [1, 6, 7], [0.2, 2, 4]))
        kappa_limit = 1.0 / min_radius

        # ---- warm-start ---------------------------------------------------
        if self.prev_d is not None and self.prev_s is not None:
            d_init = shift_solution(self.prev_d, self.prev_s, s_av,
                                    delta_s_shift=self.cur_v * self.dt)
            d_init = np.clip(d_init, d_lb, d_ub)
        else:
            d_init = analytic_fallback(len(s_av), initial_apex)
            d_init = np.clip(d_init, d_lb, d_ub)

        # ---- SQP solve ----------------------------------------------------
        ### IY : drop side bias when >=2 clusters (one side would fight the other)
        if n_clusters >= 2:
            eff_lambda_side = 0.0
            eff_desired_side = 'any'
        else:
            eff_lambda_side = self.lambda_side
            eff_desired_side = side if self.homotopy_lock else 'any'

        problem = SQPProblem(
            n_knots=len(s_av),
            delta_s=delta_s,
            d_init=d_init,
            current_d=self.current_d,
            bounds_lower=d_lb,
            bounds_upper=d_ub,
            ## IY : obs_indices removed — obs_center/obs_min are length n_knots.
            obs_center_d=obs_center,
            obs_min_dist=obs_min,
            desired_side=eff_desired_side,
            kappa_limit=kappa_limit,
            lambda_reg=self.lambda_reg,
            lambda_smooth=self.lambda_smooth,
            lambda_start_heading=self.lambda_start_heading,
            lambda_apex_bias=self.lambda_apex_bias,
            lambda_side=eff_lambda_side,
            lambda_jerk=self.lambda_jerk,
            ## IY : soft terminal wired to online_mode.
            lambda_term=self.lambda_term,
            soft_terminal=self.online_mode,
        )
        d_opt, info = self.solver.solve(problem)

        # diag: obs_center wiggle + d stats — confirms GP d-curve drives oscillation
        ## IY : fixed-slot arrays — stat on active slots only.
        active_mask = obs_min > 0.0
        if np.any(active_mask):
            oc_active = obs_center[active_mask]
            oc_min = float(oc_active.min())
            oc_max = float(oc_active.max())
            oc_std = float(oc_active.std())
            n_active = int(active_mask.sum())
        else:
            oc_min = oc_max = oc_std = 0.0
            n_active = 0
        d_opt_std = float(np.std(d_opt)) if info['success'] else 0.0
        ### IY : add n_clusters + d_opt bump amplitude for shape diagnosis
        d_opt_min = float(np.min(d_opt)) if info['success'] else 0.0
        d_opt_max = float(np.max(d_opt)) if info['success'] else 0.0
        diag = Float32MultiArray()
        diag.data = [oc_min, oc_max, oc_std,
                     float(n_active), float(len(considered_obs)),
                     float(self.cur_s), float(start_av), float(end_av),
                     float(np.std(d_init)), d_opt_std,
                     float(n_clusters), d_opt_min, d_opt_max,
                     float(self.current_d), float(d_lb[0]), float(d_ub[0]),
                     float(dt_solve), float(self.last_solve_ms),
                     float(self.cur_v)]
        self.diag_pub.publish(diag)

        if not info['success']:
            rospy.logwarn_throttle(1.0,
                '[OvertakingIY] SQP failed (%s, iters=%d). skipping cycle.',
                info['status'], info['iter_count'])
            self._handle_failure(dry_run)
            return False, 0.0, 0.0, AbortReason.NONE, side

        # ---- dense reinterpolate to match global waypoint grid -----------
        ### IY : CCMA(w_ma=10, w_cc=3) needs >=3 points.
        ### Use 4 as safety threshold (just above CCMA minimum).
        n_dense = max(2, int((end_av - start_av) / self.scaled_delta_s))
        if n_dense < 4:
            rospy.logwarn_throttle(1.0,
                '[OvertakingIY] RoC window too small (n_dense=%d, '
                'end_av-start_av=%.2fm). skipping cycle.',
                n_dense, end_av - start_av)
            self._handle_failure(dry_run)
            return False, 0.0, 0.0, AbortReason.NONE, side
        s_dense = np.linspace(start_av, end_av, n_dense)
        ## IY : shape-preserving cubic (PCHIP) replaces linear np.interp —
        ##      removes per-knot kinks in dense d(s).
        # d_dense = np.interp(s_dense, s_av, d_opt)
        d_dense = PchipInterpolator(s_av, d_opt, extrapolate=False)(s_dense)

        ## IY : raceline tail — extend with cubic-blended d -> 0 for local_wpnts continuity.
        if self.tail_len_m > 0.0 and self.scaled_delta_s is not None:
            s_step = float(self.scaled_delta_s)
            s_tail_start = end_av + s_step
            s_tail_end = end_av + self.tail_len_m
            n_tail = max(2, int((s_tail_end - s_tail_start) / s_step))
            s_tail = np.linspace(s_tail_start, s_tail_end, n_tail)
            ## IY : clamp blend_len < tail_len_m so anchors stay strictly
            ##      increasing (PCHIP requirement).
            blend_len = max(0.1,
                            min(float(self.tail_smooth_m),
                                max(0.1, float(self.tail_len_m) - s_step)))
            blend_end_s = s_dense[-1] + blend_len
            if blend_end_s >= s_tail_end - 1e-3:
                blend_end_s = s_tail_end - max(s_step, 0.05)
            anchor_s = np.array([s_dense[-2], s_dense[-1],
                                 blend_end_s, s_tail_end])
            anchor_d = np.array([float(d_dense[-2]), float(d_dense[-1]), 0.0, 0.0])
            d_tail = PchipInterpolator(anchor_s, anchor_d, extrapolate=False)(s_tail)
            if self.scaled_wpnts is not None and self.scaled_max_s is not None:
                for ti, ss in enumerate(s_tail):
                    wi = int(np.abs(self.scaled_wpnts[:, 0]
                                    - (ss % self.scaled_max_s)).argmin())
                    w_t = self.scaled_wpnts_msg.wpnts[wi]
                    d_lb_t = -w_t.d_right + self.spline_bound_mindist
                    d_ub_t = w_t.d_left - self.spline_bound_mindist
                    d_tail[ti] = float(np.clip(d_tail[ti], d_lb_t, d_ub_t))
            s_dense = np.concatenate([s_dense, s_tail])
            d_dense = np.concatenate([d_dense, d_tail])
        ## IY : end

        s_wrap = np.mod(s_dense, self.scaled_max_s)

        xy = self.converter.get_cartesian(s_wrap, d_dense).T
        xy_smooth = self.ccma.filter(xy)
        sd_smooth = self.converter.get_frenet(xy_smooth[:, 0], xy_smooth[:, 1])
        evasion_s = sd_smooth[0]
        evasion_d = sd_smooth[1]
        evasion_x = xy_smooth[:, 0]
        evasion_y = xy_smooth[:, 1]

        coords = np.column_stack((evasion_x, evasion_y))
        el_lengths = np.linalg.norm(np.diff(coords, axis=0), axis=1)
        el_lengths = np.where(el_lengths < 1e-4, 1e-4, el_lengths)
        psi, kappa_path = tph.calc_head_curv_num.calc_head_curv_num(
            path=coords,
            el_lengths=el_lengths,
            is_closed=False,
        )
        psi = psi + np.pi / 2

        # ---- velocity profile with IC matching ---------------------------
        try:
            v_profile = self.vp.profile(
                kappa=kappa_path,
                el_lengths=el_lengths,
                v_start=max(self.cur_v, 0.1),
                v_max=self.scaled_vmax,
            )
        except Exception as exc:   # noqa: BLE001
            rospy.logwarn_throttle(1.0,
                '[OvertakingIY] velocity profile failed: %s — fallback to raceline v',
                exc)
            v_profile = np.array([c.vx_mps for c in corr])
            v_profile = np.interp(s_dense, s_av, v_profile)

        # ax profile (for Wpnt.ax_mps2)
        ax_profile = tph.calc_ax_profile.calc_ax_profile(
            vx_profile=v_profile,
            el_lengths=el_lengths,
            eq_length_output=True)

        # ---- abort checks ------------------------------------------------
        # GP samples for performance abort: gp_v at s_dense
        gp_v_at_s = self._sample_opponent_vs(s_wrap)

        # Use first observed obstacle to test safety envelope
        obs0 = considered_obs[0]
        gp_d_at_obs, gp_dvar_at_obs = self._sample_opponent_d_with_var(obs0.s_center)

        abort_reason = self.abort.step(
            now=rospy.Time.now(),
            opp_obs_d=obs0.d_center,
            opp_obs_s=obs0.s_center,
            gp_d_at_s=gp_d_at_obs,
            gp_d_var_at_s=gp_dvar_at_obs,
            s_grid=s_dense,
            v_grid=v_profile,
            gp_v_at_s=gp_v_at_s,
        )

        # compute report t_ot / t_trail for debug
        ds = np.diff(s_dense)
        v_mid = 0.5 * (v_profile[:-1] + v_profile[1:])
        v_mid = np.maximum(v_mid, 1e-3)
        t_ot = float(np.sum(ds / v_mid))
        gp_v_mid = 0.5 * (gp_v_at_s[:-1] + gp_v_at_s[1:])
        gp_v_mid = np.maximum(gp_v_mid, 1e-3)
        t_trail = float(np.sum(ds / gp_v_mid))

        if abort_reason != AbortReason.NONE:
            rospy.loginfo_throttle(1.0,
                '[OvertakingIY] ABORT=%s t_ot=%.2f t_trail=%.2f',
                abort_reason.value, t_ot, t_trail)
            self.solver.reset_warmstart()
            self._handle_failure(dry_run, is_abort=True)
            return True, t_ot, t_trail, abort_reason, side

        ### IY : log start-point gap for diagnosing "path starts behind ego"
        s0_gap = float(np.mod(evasion_s[0] - self.cur_s, self.scaled_max_s))
        if s0_gap > self.scaled_max_s / 2.0:
            s0_gap -= self.scaled_max_s    # signed: negative = behind ego
        rospy.loginfo_throttle(1.0,
            '[OvertakingIY] path start gap: evasion_s[0]-cur_s=%.3fm '
            '(start_av-cur_s=%.3f dt_solve=%.3fs v=%.2f)',
            s0_gap, start_av - self.cur_s, dt_solve, self.cur_v)

        # ---- publish OTWpntArray ------------------------------------------
        msg = OTWpntArray(header=Header(stamp=rospy.Time.now(), frame_id='map'))
        wpnts = []
        for i in range(len(evasion_s)):
            w = Wpnt(
                id=i,
                s_m=float(evasion_s[i]),
                d_m=float(evasion_d[i]),
                x_m=float(evasion_x[i]),
                y_m=float(evasion_y[i]),
                psi_rad=float(psi[i]),
                kappa_radpm=float(kappa_path[i]),
                vx_mps=float(v_profile[i]),
                ax_mps2=float(ax_profile[i]) if i < len(ax_profile) else 0.0,
            )
            wpnts.append(w)
        msg.wpnts = wpnts
        msg.ot_side = side
        mean_d = float(np.mean(evasion_d))
        msg.ot_line = 'left' if mean_d > 0 else 'right'

        if not dry_run:
            self.evasion_pub.publish(msg)
            if considered_obs:
                self.merger_pub.publish(Float32MultiArray(
                    data=[considered_obs[-1].s_end % self.scaled_max_s,
                          evasion_s[-1] % self.scaled_max_s]))
            # remember for next cycle's warm-start (in refined-knot domain)
            self.prev_d = d_opt.copy()
            self.prev_s = s_av.copy()
            self.last_ot_side = msg.ot_line
            self.fail_streak = 0

        self._visualize(evasion_s, evasion_d, evasion_x, evasion_y, v_profile,
                        dry_run=dry_run)

        return True, t_ot, t_trail, AbortReason.NONE, side

    # ---------------------------------------------------------------------
    def _sample_opponent_vs(self, s_query: np.ndarray) -> np.ndarray:
        """Sample GP opponent longitudinal speed at s_query. Fallback: raceline vmax."""
        if (self.opponent_wpnts_sm is None or self.opponent_wpnts_sm.size == 0
                or self.opponent_wpnts_vs is None):
            return np.full_like(s_query, self.scaled_vmax or 1.0, dtype=float)
        s_max = self.opponent_wpnts_sm[-1] + 1e-6
        s_mod = np.mod(s_query, s_max)
        return np.interp(s_mod, self.opponent_wpnts_sm, self.opponent_wpnts_vs)

    def _sample_opponent_d_with_var(self, s_query: float):
        if (self.opponent_wpnts_sm is None or self.opponent_wpnts_sm.size == 0):
            return None, None
        s_max = self.opponent_wpnts_sm[-1] + 1e-6
        s_mod = s_query % s_max
        d = float(np.interp(s_mod, self.opponent_wpnts_sm, self.opponent_wpnts_d))
        dvar = float(np.interp(s_mod, self.opponent_wpnts_sm, self.opponent_wpnts_dvar))
        return d, dvar


if __name__ == '__main__':
    node = OvertakingIYNode()
    node.loop()
