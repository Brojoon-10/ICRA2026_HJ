#!/usr/bin/env python3
"""
### HJ : State-aware variant of mpc_planner_node.py.

`~state ∈ {overtake, recovery, observe}` routes the MPC solver output to a
state-machine-facing topic in addition to the always-on debug outputs. Phase
1~6 run with `_observation`-suffixed topics so the state_machine keeps
consuming the legacy spliner/recovery_spliner outputs — this node publishes
only for RViz/comparison. Phase X (gate) flips `~attach_to_statemachine`.

Role outputs (after launch remaps):
  overtake → OTWpntArray on /planner/avoidance/otwpnts_observation
  recovery → WpntArray   on /planner/recovery/wpnts_observation
  observe  → WpntArray   on ~best_trajectory_observation (debug only)

Debug outputs (all states):
  ~best_sample             (nav_msgs/Path)
  ~best_sample/markers     (MarkerArray)
  ~status                  (std_msgs/String, latched)
  ~timing_ms               (std_msgs/Float32)
  ~best_trajectory_observation (WpntArray; mirror of role output for observe)

Phases 2/3/4/5 extend this skeleton — 3D lift via Track3D, state-specific
presets, obstacle injection, 4-tier fallback, dynamic_reconfigure.
"""

import os
import sys
import time
import yaml
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA, Float32, Header, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from f110_msgs.msg import (WpntArray, Wpnt, OTWpntArray,
                           ObstacleArray, OpponentTrajectory)

# ### HJ : Phase 5 — dynamic_reconfigure (per-instance server).
from dynamic_reconfigure.server import Server
from mpc_planner.cfg import MPCCostConfig

# Import solver from sibling src/ directory
_this_dir = os.path.dirname(os.path.abspath(__file__))
_src_dir = os.path.normpath(os.path.join(_this_dir, '..', 'src'))
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)
from mpcc_solver import MPCCSolver  # noqa: E402
from mpc_raceline_lifter import MPCRacelineLifter  # noqa: E402
from geometric_fallback import build_quintic_fallback  # noqa: E402

_VALID_STATES = ('overtake', 'recovery', 'observe')


class MPCPlannerStateNode:

    def __init__(self):
        rospy.init_node('mpc_planner_state_node', anonymous=False)

        # -- Role -------------------------------------------------------------
        state = str(rospy.get_param('~state', 'observe')).lower().strip()
        if state not in _VALID_STATES:
            rospy.logwarn('[mpc][%s] invalid ~state=%r — falling back to "observe".',
                          rospy.get_name(), state)
            state = 'observe'
        self.state = state

        # -- Solver params ----------------------------------------------------
        self.freq = rospy.get_param('~planner_freq', 30)
        params = {
            'N':                 rospy.get_param('~N', 20),
            'dT':                rospy.get_param('~dT', 0.05),
            'vehicle_L':         rospy.get_param('~vehicle_L', 0.33),
            'max_speed':         rospy.get_param('~max_speed', 12.0),
            'min_speed':         rospy.get_param('~min_speed', 0.5),
            'max_steering':      rospy.get_param('~max_steering', 0.6),
            'w_contour':         rospy.get_param('~w_contour', 3.9),
            'w_lag':             rospy.get_param('~w_lag', 2.0),
            'w_velocity':        rospy.get_param('~w_velocity', 3.0),
            'v_bias_max':        rospy.get_param('~v_bias_max', 1.0),
            'w_dv':              rospy.get_param('~w_dv', 9.5),
            'w_dsteering':       rospy.get_param('~w_dsteering', 14.0),
            'boundary_inflation': rospy.get_param('~boundary_inflation', 0.1),
            'w_slack':           rospy.get_param('~w_slack', 1000.0),
            'ipopt_max_iter':    rospy.get_param('~ipopt_max_iter', 500),
            'ipopt_print_level': rospy.get_param('~ipopt_print_level', 0),
        }
        self.N = params['N']
        self.dT = float(params['dT'])  # ### HJ : Phase 2 — needed for ax from u_sol
        self.solver = MPCCSolver(params)

        # ### HJ : Phase 3 — obstacle cost params (read; honored by solver in
        # Phase 3.5). Kept on the node side so dynamic_reconfigure (Phase 5)
        # can update them without rebuilding the NLP.
        self.collision_mode = str(rospy.get_param('~collision_mode', 'none')).lower()
        self.w_obstacle     = float(rospy.get_param('~w_obstacle', 0.0))
        self.obstacle_sigma = float(rospy.get_param('~obstacle_sigma', 0.35))
        self.n_obs_max      = int(rospy.get_param('~n_obs_max', 2))
        self.r_safe         = float(rospy.get_param('~r_safe', 0.35))
        if self.collision_mode not in ('none', 'soft', 'hard'):
            rospy.logwarn('[mpc][%s] unknown collision_mode=%r — falling back to "none"',
                          rospy.get_name(), self.collision_mode)
            self.collision_mode = 'none'

        # -- Ego state --------------------------------------------------------
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.car_vx = 0.0
        self.pose_received = False

        # ### HJ : Phase 4.3 — 4-tier fallback state.
        # tier 0 성공 시 trajectory + u_sol 캐시 → tier 1 에서 s-shift 로 재발행.
        # streak > H 이면 tier 2 (Frenet quintic) 진입. tier 2 실패 시 tier 3.
        self._fail_streak = 0
        self._fail_tier_H = int(rospy.get_param('~fail_streak_H', 5))
        self._last_good_traj = None    # (N+1, 3)  [x, y, psi]
        self._last_good_u = None       # (N, 2)    [v, δ]
        self._last_good_s = None       # s at which it was anchored
        self._last_good_time = None    # rospy.Time
        self._quintic_delta_s = float(rospy.get_param('~quintic_delta_s', 8.0))
        self._last_status = None       # for recovery log

        # ### HJ : Phase 5 — instance YAML for save/reset triggers. Default
        # resolves to config/state_<state>.yaml (same file the launch loaded).
        # Launch can override with ~instance_yaml.
        default_yaml = os.path.join(
            os.path.dirname(_this_dir), 'config', 'state_%s.yaml' % self.state)
        self.instance_yaml_path = rospy.get_param('~instance_yaml', default_yaml)
        self._suppress_dynreg_cb = False
        self._dyn_srv = None  # set after solver.setup() in _global_wpnts_cb

        # -- Global waypoint cache -------------------------------------------
        self.global_cached = False
        self.g_s = None
        self.g_x = None
        self.g_y = None
        self.g_z = None        # ### HJ : z_m for 3D marker lift
        self.g_psi = None
        self.g_kappa = None    # ### HJ : raceline curvature (fallback source)
        self.g_dleft = None
        self.g_dright = None
        self.g_vx = None
        self.g_mu = None       # ### HJ : raceline pitch (fallback source)
        self.track_length = None

        self._timer = None

        # -- Publishers (debug — always) -------------------------------------
        self.pub_best_trajectory = rospy.Publisher(
            '~best_trajectory_observation', WpntArray, queue_size=1)
        self.pub_best_sample = rospy.Publisher(
            '~best_sample', Path, queue_size=1)
        self.pub_best_markers = rospy.Publisher(
            '~best_sample/markers', MarkerArray, queue_size=1)
        self.pub_status = rospy.Publisher(
            '~status', String, queue_size=1, latch=True)
        self.pub_timing = rospy.Publisher(
            '~timing_ms', Float32, queue_size=1)

        # -- Publishers (role-specific) --------------------------------------
        # Launch file remaps ~out/otwpnts → /planner/avoidance/otwpnts_observation
        # and ~out/wpnts → /planner/recovery/wpnts_observation (Phase 1~6).
        if self.state == 'overtake':
            self.pub_out = rospy.Publisher(
                '~out/otwpnts', OTWpntArray, queue_size=1)
        elif self.state == 'recovery':
            self.pub_out = rospy.Publisher(
                '~out/wpnts', WpntArray, queue_size=1)
        else:
            self.pub_out = None

        # -- Subscribers ------------------------------------------------------
        rospy.Subscriber('/global_waypoints', WpntArray, self._global_wpnts_cb, queue_size=1)
        rospy.Subscriber('/car_state/pose', PoseStamped, self._pose_cb, queue_size=1)
        rospy.Subscriber('/car_state/odom', Odometry, self._odom_cb, queue_size=1)

        # ### HJ : Phase 3.5 — SQP-compatible obstacle ingestion.
        # prediction primary → tracking fallback → neither (cost disabled).
        # 100ms staleness threshold (SQP has none — this is an improvement).
        self._obs_stale_s = 0.1
        self._obs_predict = None
        self._obs_predict_t = None
        self._obs_track = None
        self._obs_track_t = None
        self._opp_wpnts = None
        self._opp_wpnts_t = None
        rospy.Subscriber('/opponent_prediction/obstacles', ObstacleArray,
                         self._obs_predict_cb, queue_size=1)
        rospy.Subscriber('/tracking/obstacles', ObstacleArray,
                         self._obs_track_cb, queue_size=1)
        rospy.Subscriber('/opponent_trajectory', OpponentTrajectory,
                         self._opp_wpnts_cb, queue_size=1)

        self._publish_status('INIT_WAITING_GLOBAL')
        rospy.loginfo(
            '[mpc][%s] ready — state=%s N=%d dT=%.3f  waiting for /global_waypoints...',
            rospy.get_name(), self.state, self.N, params['dT'],
        )

    # ---------------------------------------------------------------- callbacks
    def _global_wpnts_cb(self, msg):
        if self.global_cached:
            return
        wpnts = msg.wpnts
        n = len(wpnts)
        if n < 10:
            rospy.logwarn('[mpc][%s] too few global waypoints: %d', rospy.get_name(), n)
            return

        self.g_s = np.array([w.s_m for w in wpnts], dtype=float)
        self.g_x = np.array([w.x_m for w in wpnts], dtype=float)
        self.g_y = np.array([w.y_m for w in wpnts], dtype=float)
        self.g_z = np.array([getattr(w, 'z_m', 0.0) for w in wpnts], dtype=float)
        self.g_psi = np.array([w.psi_rad for w in wpnts], dtype=float)
        self.g_kappa = np.array([getattr(w, 'kappa_radpm', 0.0) for w in wpnts], dtype=float)
        self.g_dleft = np.array([w.d_left for w in wpnts], dtype=float)
        self.g_dright = np.array([w.d_right for w in wpnts], dtype=float)
        self.g_vx = np.array([w.vx_mps for w in wpnts], dtype=float)
        self.g_mu = np.array([getattr(w, 'mu_rad', 0.0) for w in wpnts], dtype=float)
        self.track_length = float(self.g_s[-1])

        # ### HJ : Phase 2 — raceline-base lifter (no Track3D; see module
        # docstring for the centerline/raceline mismatch rationale).
        self.lifter = MPCRacelineLifter(
            g_s=self.g_s, g_x=self.g_x, g_y=self.g_y, g_z=self.g_z,
            g_psi=self.g_psi, g_kappa=self.g_kappa, g_mu=self.g_mu,
            g_dleft=self.g_dleft, g_dright=self.g_dright, g_vx=self.g_vx,
            track_length=self.track_length,
        )

        self.solver.setup()
        self.global_cached = True

        # ### HJ : Phase 5 — start dynreg server after solver is up so hot
        # updates have a valid NLP to touch. Server(...) constructor calls
        # _weight_cb once with .cfg defaults to prime the server — we must
        # suppress rebuild BEFORE constructing, then push our YAML-loaded
        # values back into rqt in one shot.
        self._suppress_dynreg_cb = True
        try:
            self._dyn_srv = Server(MPCCostConfig, self._weight_cb)
            self._push_params_to_dynreg()
        finally:
            self._suppress_dynreg_cb = False

        self._publish_status('INIT_OK')
        rospy.loginfo('[mpc][%s] solver ready: %d waypoints, track=%.1fm',
                      rospy.get_name(), n, self.track_length)

        if self._timer is None:
            self._timer = rospy.Timer(rospy.Duration(1.0 / self.freq), self._plan_loop)

    def _pose_cb(self, msg):
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.car_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_received = True

    def _odom_cb(self, msg):
        self.car_vx = msg.twist.twist.linear.x

    # ---------------------------------------------------------------- obstacle inputs
    def _obs_predict_cb(self, msg):
        self._obs_predict = msg
        self._obs_predict_t = rospy.Time.now()

    def _obs_track_cb(self, msg):
        self._obs_track = msg
        self._obs_track_t = rospy.Time.now()

    def _opp_wpnts_cb(self, msg):
        self._opp_wpnts = msg
        self._opp_wpnts_t = rospy.Time.now()

    def _is_fresh(self, t_stamp):
        if t_stamp is None:
            return False
        return (rospy.Time.now() - t_stamp).to_sec() <= self._obs_stale_s

    def _pick_obs_source(self):
        """Return (ObstacleArray, 'predict'|'track'|'none'). Prediction wins
        when fresh; tracking is fallback; otherwise signal cost-off."""
        if self._is_fresh(self._obs_predict_t) and \
           self._obs_predict is not None and self._obs_predict.obstacles:
            return self._obs_predict, 'predict'
        if self._is_fresh(self._obs_track_t) and \
           self._obs_track is not None and self._obs_track.obstacles:
            return self._obs_track, 'track'
        return None, 'none'

    def _extrapolate_obs_traj(self, obs, N, dT):
        """Expand one Obstacle into (N, 2) Cartesian trajectory.

        Static obstacle → repeat (x_m, y_m). Dynamic → constant-velocity on
        (s_center, d_center) using (vs, vd), then sn→xy via the lifter.
        """
        if obs.is_static or abs(obs.vs) < 1e-3:
            return np.tile([obs.x_m, obs.y_m], (N, 1))
        out = np.zeros((N, 2), dtype=np.float64)
        s0 = float(obs.s_center)
        d0 = float(obs.d_center)
        vs = float(obs.vs)
        vd = float(obs.vd)
        for k in range(N):
            s_k = (s0 + vs * k * dT) % self.track_length
            d_k = d0 + vd * k * dT
            x_k, y_k = self.lifter.sn_to_xy(s_k, d_k)
            out[k, 0] = x_k
            out[k, 1] = y_k
        return out

    def _build_obstacle_array(self, ego_s):
        """### HJ : Phase 3.5 — SQP-compatible obstacle → (n_obs_max, N, 3).

        Returns (obs_arr, status_str). obs_arr disabled (far coord + w=0)
        when no fresh source; otherwise top-`n_obs_max` obstacles picked by
        s-distance ahead of ego and expanded to the MPC horizon.
        """
        N = self.N
        dT = self.dT
        far = self.solver.FAR_XY
        obs_arr = np.zeros((self.n_obs_max, N, 3), dtype=np.float64)
        obs_arr[:, :, 0] = far
        obs_arr[:, :, 1] = far
        # w_obs = 0 already.

        if self.collision_mode == 'none' or self.w_obstacle <= 0.0:
            return obs_arr, 'disabled'

        src, tag = self._pick_obs_source()
        if tag == 'none':
            return obs_arr, 'stale'

        # Pick obstacles ahead of ego_s (shortest forward s-distance first).
        def fwd_dist(o):
            ds = (o.s_center - ego_s) % self.track_length
            return ds
        obs_list = sorted(src.obstacles, key=fwd_dist)
        # Drop anything too far away (more than half track ahead = behind).
        obs_list = [o for o in obs_list if fwd_dist(o) < 0.5 * self.track_length]

        n_used = 0
        for o in obs_list:
            if n_used >= self.n_obs_max:
                break
            traj = self._extrapolate_obs_traj(o, N, dT)
            obs_arr[n_used, :, 0] = traj[:, 0]
            obs_arr[n_used, :, 1] = traj[:, 1]
            obs_arr[n_used, :, 2] = self.w_obstacle
            n_used += 1

        if n_used == 0:
            return obs_arr, 'empty'
        return obs_arr, '%s:%d' % (tag, n_used)

    # ---------------------------------------------------------------- dynamic_reconfigure
    def _push_params_to_dynreg(self):
        """Seed rqt sliders with the rosparam-loaded values so the first
        connect doesn't snap to .cfg defaults and silently overwrite YAML."""
        if self._dyn_srv is None:
            return
        self._suppress_dynreg_cb = True
        try:
            self._dyn_srv.update_configuration({
                'w_contour':          float(self.solver.w_contour),
                'w_lag':              float(self.solver.w_lag),
                'w_velocity':         float(self.solver.w_velocity),
                'v_bias_max':         float(self.solver.v_bias_max),
                'w_dv':               float(self.solver.w_dv),
                'w_dsteering':        float(self.solver.w_dsteering),
                'w_slack':            float(self.solver.w_slack),
                'max_speed':          float(self.solver.v_max),
                'min_speed':          float(self.solver.v_min),
                'max_steering':       float(self.solver.theta_max),
                'boundary_inflation': float(self.solver.inflation),
                'w_obstacle':         float(self.w_obstacle),
                'obstacle_sigma':     float(self.obstacle_sigma),
                'r_safe':             float(self.r_safe),
                'fail_streak_H':      int(self._fail_tier_H),
                'quintic_delta_s':    float(self._quintic_delta_s),
                'ipopt_max_iter':     int(self.solver.ipopt_max_iter),
                'save_params':        False,
                'reset_params':       False,
            })
        finally:
            self._suppress_dynreg_cb = False

    def _weight_cb(self, config, level):
        # ### HJ : Phase 5 — save/reset one-shot triggers (pattern from
        # sampling_planner — mutate `config` in place on reset so the outer
        # return replays the reloaded values into rqt).
        if config.save_params:
            self._save_yaml(config)
            config.save_params = False
        if config.reset_params:
            new_cfg = self._reload_yaml()
            if new_cfg is not None:
                for k, v in new_cfg.items():
                    if hasattr(config, k):
                        setattr(config, k, v)
                rospy.loginfo('[mpc][%s][reset] reloaded YAML: %s',
                              rospy.get_name(), self.instance_yaml_path)
            config.reset_params = False

        # --- Hot cost-weight swap (no NLP rebuild) --------------------------
        w_changed = False
        for k in ('w_contour', 'w_lag', 'w_velocity', 'v_bias_max',
                  'w_dv', 'w_dsteering', 'w_slack'):
            new = float(getattr(config, k))
            if abs(getattr(self.solver, k, None) - new) > 1e-12:
                w_changed = True
        # Cost weights ARE baked into the symbolic NLP — hot swap alone would
        # not take effect. We rebuild if any weight actually changed; cheap
        # compared to the planner tick.
        rebuild_needed = False

        # --- Box bounds (truly hot) -----------------------------------------
        if (abs(self.solver.v_max - float(config.max_speed)) > 1e-12 or
                abs(self.solver.v_min - float(config.min_speed)) > 1e-12 or
                abs(self.solver.theta_max - float(config.max_steering)) > 1e-12):
            self.solver.update_box_bounds(
                v_min=float(config.min_speed),
                v_max=float(config.max_speed),
                theta_max=float(config.max_steering),
            )

        # --- Corridor inflation (picked up on next ref slice) ---------------
        self.solver.update_inflation(float(config.boundary_inflation))

        # --- Obstacle params ------------------------------------------------
        self.w_obstacle = float(config.w_obstacle)
        self.r_safe = float(config.r_safe)
        new_sigma = float(config.obstacle_sigma)
        if abs(self.solver.obstacle_sigma - new_sigma) > 1e-9:
            rebuild_needed = True
            self.obstacle_sigma = new_sigma

        # --- IPOPT iterations (rebuild) -------------------------------------
        new_iter = int(config.ipopt_max_iter)
        if new_iter != self.solver.ipopt_max_iter:
            rebuild_needed = True

        # --- Fallback tuning (pure node state, no rebuild) ------------------
        self._fail_tier_H = int(config.fail_streak_H)
        self._quintic_delta_s = float(config.quintic_delta_s)

        # ### HJ : Suppress rebuild during the boot-time push. The values
        # going in are identical to what the solver was constructed with
        # (node reads rosparam → builds solver → pushes same values into
        # rqt), so "changed" here is a float-equality false positive.
        if (w_changed or rebuild_needed) and not self._suppress_dynreg_cb:
            t0 = rospy.Time.now()
            self.solver.rebuild_nlp(
                w_contour=float(config.w_contour),
                w_lag=float(config.w_lag),
                w_velocity=float(config.w_velocity),
                v_bias_max=float(config.v_bias_max),
                w_dv=float(config.w_dv),
                w_dsteering=float(config.w_dsteering),
                w_slack=float(config.w_slack),
                ipopt_max_iter=new_iter,
                obstacle_sigma=new_sigma,
            )
            dt_ms = (rospy.Time.now() - t0).to_sec() * 1000.0
            rospy.loginfo(
                '[mpc][%s] rebuilt NLP in %.1f ms (weights/sigma/iter changed)',
                rospy.get_name(), dt_ms)

        if not self._suppress_dynreg_cb:
            rospy.loginfo_throttle(
                2.0,
                '[mpc][%s] tune w(c/l/v/dv/dδ/slack)=%.2f/%.2f/%.2f/%.1f/%.1f/%.0f  '
                'v[%.1f,%.1f] δ=%.2f inf=%.2f  obs(w=%.0f σ=%.2f r=%.2f) H=%d Δs=%.1f iter=%d',
                rospy.get_name(),
                self.solver.w_contour, self.solver.w_lag, self.solver.w_velocity,
                self.solver.w_dv, self.solver.w_dsteering, self.solver.w_slack,
                self.solver.v_min, self.solver.v_max, self.solver.theta_max,
                self.solver.inflation,
                self.w_obstacle, self.obstacle_sigma, self.r_safe,
                self._fail_tier_H, self._quintic_delta_s,
                self.solver.ipopt_max_iter,
            )
        return config

    def _save_yaml(self, config):
        """Write current dynreg state back to the instance YAML, preserving
        fields the cfg doesn't know about (e.g., N, dT, vehicle_L, n_obs_max,
        collision_mode — structural or launch-once params)."""
        if not self.instance_yaml_path:
            rospy.logwarn('[mpc][%s][save] ~instance_yaml not set — skipping',
                          rospy.get_name())
            return
        try:
            if os.path.exists(self.instance_yaml_path):
                with open(self.instance_yaml_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {}

            # Cost weights
            data['w_contour']    = float(config.w_contour)
            data['w_lag']        = float(config.w_lag)
            data['w_velocity']   = float(config.w_velocity)
            data['v_bias_max']   = float(config.v_bias_max)
            data['w_dv']         = float(config.w_dv)
            data['w_dsteering']  = float(config.w_dsteering)
            data['w_slack']      = float(config.w_slack)
            # Box bounds + inflation
            data['max_speed']    = float(config.max_speed)
            data['min_speed']    = float(config.min_speed)
            data['max_steering'] = float(config.max_steering)
            data['boundary_inflation'] = float(config.boundary_inflation)
            # Obstacle
            data['w_obstacle']     = float(config.w_obstacle)
            data['obstacle_sigma'] = float(config.obstacle_sigma)
            data['r_safe']         = float(config.r_safe)
            # Fallback
            data['fail_streak_H']  = int(config.fail_streak_H)
            data['quintic_delta_s'] = float(config.quintic_delta_s)
            # IPOPT
            data['ipopt_max_iter'] = int(config.ipopt_max_iter)

            with open(self.instance_yaml_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            rospy.loginfo('[mpc][%s][save] YAML updated: %s',
                          rospy.get_name(), self.instance_yaml_path)
        except Exception as e:
            rospy.logerr('[mpc][%s][save] failed: %s', rospy.get_name(), e)

    def _reload_yaml(self):
        """Return dict of dynreg-field values from YAML, or None on error."""
        if not self.instance_yaml_path or not os.path.exists(self.instance_yaml_path):
            rospy.logwarn('[mpc][%s][reset] ~instance_yaml missing — skip',
                          rospy.get_name())
            return None
        try:
            with open(self.instance_yaml_path, 'r') as f:
                d = yaml.safe_load(f) or {}
            return {
                'w_contour':          float(d.get('w_contour',    self.solver.w_contour)),
                'w_lag':              float(d.get('w_lag',        self.solver.w_lag)),
                'w_velocity':         float(d.get('w_velocity',   self.solver.w_velocity)),
                'v_bias_max':         float(d.get('v_bias_max',   self.solver.v_bias_max)),
                'w_dv':               float(d.get('w_dv',         self.solver.w_dv)),
                'w_dsteering':        float(d.get('w_dsteering',  self.solver.w_dsteering)),
                'w_slack':            float(d.get('w_slack',      self.solver.w_slack)),
                'max_speed':          float(d.get('max_speed',    self.solver.v_max)),
                'min_speed':          float(d.get('min_speed',    self.solver.v_min)),
                'max_steering':       float(d.get('max_steering', self.solver.theta_max)),
                'boundary_inflation': float(d.get('boundary_inflation', self.solver.inflation)),
                'w_obstacle':         float(d.get('w_obstacle',     self.w_obstacle)),
                'obstacle_sigma':     float(d.get('obstacle_sigma', self.solver.obstacle_sigma)),
                'r_safe':             float(d.get('r_safe',         self.r_safe)),
                'fail_streak_H':      int(d.get('fail_streak_H',   self._fail_tier_H)),
                'quintic_delta_s':    float(d.get('quintic_delta_s', self._quintic_delta_s)),
                'ipopt_max_iter':     int(d.get('ipopt_max_iter',  self.solver.ipopt_max_iter)),
                'save_params':        False,
                'reset_params':       False,
            }
        except Exception as e:
            rospy.logerr('[mpc][%s][reset] parse failed: %s', rospy.get_name(), e)
            return None

    # ---------------------------------------------------------------- helpers
    def _publish_status(self, s):
        self.pub_status.publish(String(data=s))

    def _nearest_idx(self, x, y):
        d2 = (self.g_x - x) ** 2 + (self.g_y - y) ** 2
        return int(np.argmin(d2))

    def _slice_local_ref(self, s_cur):
        """Build N+1 local reference from /global_waypoints around s_cur.
        Port of the original mpc_planner_node._slice_local_ref — unchanged."""
        N = self.N
        dT = self.solver.dT
        tl = self.track_length
        inflation = self.solver.inflation

        center_pts, left_pts, right_pts = [], [], []
        ref_v, ref_s, ref_dx, ref_dy = [], [], [], []
        s_offset = 0.0
        prev_s = None

        target_s = s_cur
        for k in range(N + 1):
            s_wrap = target_s % tl if tl > 0 else target_s
            idx = int(np.argmin(np.abs(self.g_s - s_wrap)))

            x = self.g_x[idx]
            y = self.g_y[idx]
            psi = self.g_psi[idx]
            dl = self.g_dleft[idx]
            dr = self.g_dright[idx]

            normal = np.array([-np.sin(psi), np.cos(psi)])
            center = np.array([x, y])
            center_pts.append(center)
            left_pts.append(center + normal * max(dl - inflation, 0.0))
            right_pts.append(center - normal * max(dr - inflation, 0.0))
            ref_v.append(float(self.g_vx[idx]))
            ref_dx.append(np.cos(psi))
            ref_dy.append(np.sin(psi))

            s_val = float(self.g_s[idx])
            if prev_s is not None and s_val + s_offset < prev_s - 1.0:
                s_offset += max(tl or 100.0, 1.0)
            s_val += s_offset
            ref_s.append(s_val)
            prev_s = s_val

            local_v = max(float(self.g_vx[idx]), 1.0)
            target_s += local_v * dT

        return {
            'center_points': np.array(center_pts),
            'left_points': np.array(left_pts),
            'right_points': np.array(right_pts),
            'ref_v': np.array(ref_v),
            'ref_s': np.array(ref_s),
            'ref_dx': np.array(ref_dx),
            'ref_dy': np.array(ref_dy),
        }

    # ---------------------------------------------------------------- main loop
    def _plan_loop(self, event):
        if not self.solver.ready or not self.pose_received:
            return

        s_cur = float(self.g_s[self._nearest_idx(self.car_x, self.car_y)])
        ref_slice = self._slice_local_ref(s_cur)
        initial_state = np.array([self.car_x, self.car_y, self.car_yaw])

        # ### HJ : Phase 3.5 — fuse obstacle inputs and hand to solver.
        obs_arr, obs_tag = self._build_obstacle_array(s_cur)

        t0 = time.time()
        speed, steering, trajectory, success = self.solver.solve(
            initial_state, ref_slice, obstacles=obs_arr)
        solve_ms = (time.time() - t0) * 1000.0
        self.pub_timing.publish(Float32(data=solve_ms))

        # ### HJ : Phase 4.3 — 4-tier fallback. Even at tier 3 we still
        # publish a sane Wpnt[] so controller has zero-gap input.
        if success and trajectory is not None:
            self._handle_tier0_success(trajectory, s_cur, solve_ms, obs_tag,
                                       speed, steering)
            return

        # NLP failed — move down the ladder.
        self._fail_streak += 1
        status = getattr(self.solver, 'last_return_status', '?')
        self.solver.reset_warm_start()

        if self._last_good_traj is not None and self._fail_streak <= self._fail_tier_H:
            self._handle_tier1_hold_last(status, solve_ms, obs_tag)
            return

        traj_fb = self._try_quintic_fallback(s_cur)
        if traj_fb is not None:
            self._handle_tier2_geometric(traj_fb, status, solve_ms, obs_tag)
            return

        self._handle_tier3_raceline(s_cur, status, solve_ms, obs_tag)

    # ---- Tier 0 ------------------------------------------------------------
    def _handle_tier0_success(self, trajectory, s_cur, solve_ms, obs_tag,
                              speed, steering):
        if self._last_status and self._last_status != 'OK':
            rospy.loginfo('[mpc][%s] recovered after tier=%s  streak=%d',
                          rospy.get_name(), self._last_status, self._fail_streak)
        self._fail_streak = 0

        self._publish_outputs(trajectory)
        self._last_good_traj = np.array(trajectory, copy=True)
        u_sol = getattr(self.solver, 'last_u_sol', None)
        self._last_good_u = np.array(u_sol, copy=True) if u_sol is not None else None
        self._last_good_s = s_cur
        self._last_good_time = rospy.Time.now()

        self._publish_status('OK obs=%s' % obs_tag)
        self._last_status = 'OK'
        rospy.loginfo_throttle(
            2.0,
            '[mpc][%s] state=%s solve=%.1fms v0=%.2f steer=%.3f obs=%s',
            rospy.get_name(), self.state, solve_ms, speed, steering, obs_tag,
        )

    # ---- Tier 1 ------------------------------------------------------------
    def _handle_tier1_hold_last(self, ipopt_status, solve_ms, obs_tag):
        """Re-publish last good trajectory. No s-shift yet (Phase 4 minimal);
        controller lookahead handles small ego progress within one tick."""
        self._publish_outputs(self._last_good_traj,
                              u_sol_override=self._last_good_u)
        self._publish_status('HOLD_LAST streak=%d obs=%s' %
                             (self._fail_streak, obs_tag))
        self._last_status = 'HOLD_LAST'
        rospy.logwarn_throttle(
            0.5,
            '[mpc fallback tier1][%s] HOLD_LAST streak=%d ipopt=%s solve=%.1fms obs=%s',
            rospy.get_name(), self._fail_streak, ipopt_status, solve_ms, obs_tag,
        )

    # ---- Tier 2 ------------------------------------------------------------
    def _try_quintic_fallback(self, s_cur):
        """Build (N+1, 3) Cartesian trajectory from Frenet quintic. Returns
        None iff the underlying projection/lifter blow up."""
        try:
            s_raceline, n_raceline = self.lifter.project_xy_to_sn(
                self.car_x, self.car_y)
            psi_track = self.lifter._interp_psi(s_raceline)
            psi_delta = float(np.arctan2(
                np.sin(self.car_yaw - psi_track),
                np.cos(self.car_yaw - psi_track)))
            traj_fb = build_quintic_fallback(
                self.lifter, s_raceline, n_raceline, psi_delta,
                delta_s=self._quintic_delta_s, n_samples=self.N + 1)
            return traj_fb
        except Exception as exc:  # pragma: no cover — guarded log only
            rospy.logerr_throttle(
                1.0, '[mpc fallback tier2][%s] quintic build raised: %s',
                rospy.get_name(), exc)
            return None

    def _handle_tier2_geometric(self, traj_fb, ipopt_status, solve_ms, obs_tag):
        # Synthetic u_sol — constant-velocity placeholder so the lifter can
        # still fill vx_mps / ax_mps2. v chosen as mean of the raceline-slice
        # local target to stay close to the velocity planner's expectation.
        v_fb = float(np.clip(self.car_vx, 0.5, max(self.solver.v_max, 0.5)))
        u_fb = np.zeros((self.N, 2), dtype=np.float64)
        u_fb[:, 0] = v_fb
        self._publish_outputs(traj_fb, u_sol_override=u_fb)
        n0 = self.lifter.project_xy_to_sn(self.car_x, self.car_y)[1]
        self._publish_status('GEOMETRIC_FALLBACK streak=%d n0=%.2f obs=%s' %
                             (self._fail_streak, n0, obs_tag))
        self._last_status = 'GEOMETRIC_FALLBACK'
        rospy.logwarn_throttle(
            0.5,
            '[mpc fallback tier2][%s] GEOMETRIC streak=%d n0=%.2f Δs=%.1f '
            'ipopt=%s solve=%.1fms obs=%s',
            rospy.get_name(), self._fail_streak, n0, self._quintic_delta_s,
            ipopt_status, solve_ms, obs_tag,
        )

    # ---- Tier 3 ------------------------------------------------------------
    def _handle_tier3_raceline(self, s_cur, ipopt_status, solve_ms, obs_tag):
        """Last-resort: publish raceline slice itself as trajectory (N+1, 3).
        Not ego-anchored — controller may jump, but output never drops."""
        N_total = self.N + 1
        ds_grid = np.linspace(0.0, max(self._quintic_delta_s, 4.0), N_total)
        traj = np.zeros((N_total, 3), dtype=np.float64)
        for i, ds in enumerate(ds_grid):
            s_i = s_cur + ds
            x, y = self.lifter.sn_to_xy(s_i, 0.0)
            psi = self.lifter._interp_psi(s_i)
            traj[i, 0] = x
            traj[i, 1] = y
            traj[i, 2] = psi

        v_fb = float(np.clip(self.car_vx, 0.5, max(self.solver.v_max, 0.5)))
        u_fb = np.zeros((self.N, 2), dtype=np.float64)
        u_fb[:, 0] = v_fb
        self._publish_outputs(traj, u_sol_override=u_fb)
        self._publish_status('RACELINE_SLICE streak=%d obs=%s' %
                             (self._fail_streak, obs_tag))
        self._last_status = 'RACELINE_SLICE'
        rospy.logerr_throttle(
            0.5,
            '[mpc fallback tier3][%s] RACELINE_SLICE streak=%d ipopt=%s '
            'solve=%.1fms obs=%s — geometric failed or no last-good',
            rospy.get_name(), self._fail_streak, ipopt_status, solve_ms, obs_tag,
        )

    def _publish_outputs(self, trajectory, u_sol_override=None):
        """trajectory: (N+1, 3) [x, y, psi]. Publish:
         - WpntArray on ~best_trajectory_observation (always)
         - Path + Markers on ~best_sample (debug, always)
         - role-specific output on ~out/... (overtake/recovery only)

        `u_sol_override` allows fallback tiers to supply a synthetic control
        sequence when the solver didn't produce one (tier 2/3) or when the
        last-good snapshot was re-published (tier 1).
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        # ### HJ : Phase 2 — vx comes from solver u_sol[k, 0], ax from its
        # forward difference. Last horizon step has no successor → ax=0.
        if u_sol_override is not None:
            u_sol = u_sol_override
        else:
            u_sol = getattr(self.solver, 'last_u_sol', None)
        N_traj = trajectory.shape[0]

        wp_arr = WpntArray()
        wp_arr.header = header
        idx_hint = None
        for i in range(N_traj):
            x = float(trajectory[i, 0])
            y = float(trajectory[i, 1])
            psi_mpc = float(trajectory[i, 2])

            if u_sol is not None and i < u_sol.shape[0]:
                vx = float(u_sol[i, 0])
                if i + 1 < u_sol.shape[0]:
                    ax = (float(u_sol[i + 1, 0]) - vx) / self.dT
                else:
                    ax = 0.0
            else:
                # Terminal state (i == N) has no control; hold last u.
                vx = float(u_sol[-1, 0]) if u_sol is not None else 0.0
                ax = 0.0

            fields = self.lifter.fill_wpnt(
                x, y, psi_mpc, vx_mpc=vx, ax_mpc=ax, idx_hint=idx_hint,
            )
            # Reuse nearest segment as hint for the next step (horizon is
            # monotone along s — saves an O(M) argmin per step).
            idx_hint = self.lifter._nearest_idx(x, y)

            w = Wpnt()
            w.id = i
            w.s_m = fields['s_m']
            w.d_m = fields['d_m']
            w.x_m = fields['x_m']
            w.y_m = fields['y_m']
            if hasattr(w, 'z_m'):
                w.z_m = fields['z_m']
            w.psi_rad = fields['psi_rad']
            w.kappa_radpm = fields['kappa_radpm']
            w.vx_mps = fields['vx_mps']
            w.ax_mps2 = fields['ax_mps2']
            if hasattr(w, 'mu_rad'):
                w.mu_rad = fields['mu_rad']
            w.d_left = fields['d_left']
            w.d_right = fields['d_right']
            wp_arr.wpnts.append(w)

        self.pub_best_trajectory.publish(wp_arr)
        self._publish_debug_path(header, trajectory)
        self._publish_debug_markers(header, trajectory)

        if self.state == 'overtake' and self.pub_out is not None:
            ot = OTWpntArray()
            ot.header = header
            ot.last_switch_time = rospy.Time.now()
            ot.side_switch = False
            # Dominant side of the chosen trajectory (mean signed d_m).
            mean_d = float(np.mean([w.d_m for w in wp_arr.wpnts])) if wp_arr.wpnts else 0.0
            ot.ot_side = 'left' if mean_d >= 0.0 else 'right'
            ot.ot_line = 'mpc'
            ot.wpnts = wp_arr.wpnts
            self.pub_out.publish(ot)
        elif self.state == 'recovery' and self.pub_out is not None:
            self.pub_out.publish(wp_arr)

    def _publish_debug_path(self, header, trajectory):
        path = Path()
        path.header = header
        for i in range(trajectory.shape[0]):
            p = PoseStamped()
            p.header = header
            p.pose.position.x = float(trajectory[i, 0])
            p.pose.position.y = float(trajectory[i, 1])
            s_i, _ = self.lifter.project_xy_to_sn(
                float(trajectory[i, 0]), float(trajectory[i, 1]))
            p.pose.position.z = self.lifter._interp(s_i, self.lifter.g_z)
            q = quaternion_from_euler(0, 0, float(trajectory[i, 2]))
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            path.poses.append(p)
        self.pub_best_sample.publish(path)

    def _publish_debug_markers(self, header, trajectory):
        arr = MarkerArray()

        # LINE_STRIP — predicted path
        line = Marker()
        line.header = header
        line.ns = 'mpc_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        # Color by state — easy RViz disambiguation.
        if self.state == 'overtake':
            line.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.9)   # orange
        elif self.state == 'recovery':
            line.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.9)   # blue
        else:
            line.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9)   # green
        line.pose.orientation.w = 1.0
        for i in range(trajectory.shape[0]):
            s_i, _ = self.lifter.project_xy_to_sn(
                float(trajectory[i, 0]), float(trajectory[i, 1]))
            pt = Point()
            pt.x = float(trajectory[i, 0])
            pt.y = float(trajectory[i, 1])
            pt.z = self.lifter._interp(s_i, self.lifter.g_z)
            line.points.append(pt)
        arr.markers.append(line)

        # SPHERE_LIST — per-step
        pts = Marker()
        pts.header = header
        pts.ns = 'mpc_steps'
        pts.id = 1
        pts.type = Marker.SPHERE_LIST
        pts.action = Marker.ADD
        pts.scale.x = pts.scale.y = pts.scale.z = 0.12
        pts.color = ColorRGBA(r=line.color.r, g=line.color.g, b=line.color.b, a=0.9)
        pts.pose.orientation.w = 1.0
        for i in range(trajectory.shape[0]):
            s_i, _ = self.lifter.project_xy_to_sn(
                float(trajectory[i, 0]), float(trajectory[i, 1]))
            pt = Point()
            pt.x = float(trajectory[i, 0])
            pt.y = float(trajectory[i, 1])
            pt.z = self.lifter._interp(s_i, self.lifter.g_z)
            pts.points.append(pt)
        arr.markers.append(pts)

        self.pub_best_markers.publish(arr)


if __name__ == '__main__':
    try:
        MPCPlannerStateNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
