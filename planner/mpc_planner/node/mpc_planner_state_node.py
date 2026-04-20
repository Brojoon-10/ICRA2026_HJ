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
  ~best_sample/markers     (MarkerArray — trajectory LINE_STRIP + spheres)
  ~debug/markers           (MarkerArray — corridor walls, obstacle blobs,
                            ref-slice centerline, tier/status text label)
  ~debug/tick_json         (std_msgs/String, JSON payload per tick — used by
                            Claude-side `rostopic echo -c` monitoring loop.
                            Contains tier, ipopt status, solve_ms, cost,
                            ego (s,n,v), margins, jitter, obstacles, weights.)
  ~debug_log/live_summary  (std_msgs/String, rolling-window stats every N ticks)
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
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Float32, Header, String
from tf.transformations import euler_from_quaternion
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
from frenet_d_solver import FrenetDSolver  # noqa: E402 — Frenet-d perturbation backend
from frenet_kin_solver import (FrenetKinMPC,  # noqa: E402 — new frenet-kin backend
                               SIDE_CLEAR, SIDE_LEFT, SIDE_RIGHT, SIDE_TRAIL)
from side_decider import SideDecider  # noqa: E402 — rule-based side selection
from mpc_raceline_lifter import MPCRacelineLifter  # noqa: E402
from geometric_fallback import build_quintic_fallback  # noqa: E402

# ### HJ : debug_log — optional structured logger (CSV + NPZ per tick).
_dbg_dir = os.path.normpath(os.path.join(_this_dir, '..', 'debug_log'))
if _dbg_dir not in sys.path:
    sys.path.insert(0, _dbg_dir)
try:
    from debug_logger import DebugLogger  # noqa: E402
except Exception as _e:  # pragma: no cover
    DebugLogger = None
    _debug_logger_import_err = _e
else:
    _debug_logger_import_err = None

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
            # ### HJ : stage-wise contour/lag ramp (1.0 = flat legacy). <1.0
            # softens near-car lateral pull to kill snap-back overshoot.
            'contour_ramp_start': rospy.get_param('~contour_ramp_start', 1.0),
            'lag_ramp_start':     rospy.get_param('~lag_ramp_start', 1.0),
        }
        self.N = params['N']
        self.dT = float(params['dT'])  # ### HJ : Phase 2 — needed for ax from u_sol

        # ### HJ : solver backend switch.
        # `frenet_kin` (default): Frenet kinematic bicycle MPC. State
        #                [n,mu,v], 4-term cost, obstacle as hard half-plane
        #                via external SideDecider, corridor box with
        #                wall_safe baked in. See src/frenet_kin_solver.py.
        # `frenet_d`  : legacy n(s)-perturbation on fixed s-grid (backup).
        # `xy`        : legacy Cartesian MPCC (backup).
        self.solver_backend = str(
            rospy.get_param('~solver_backend', 'frenet_kin')).lower()
        if self.solver_backend == 'frenet_kin':
            params['q_n']           = float(rospy.get_param('~q_n', 3.0))
            params['gamma_progress']= float(rospy.get_param('~gamma_progress', 10.0))
            params['r_a']           = float(rospy.get_param('~r_a', 0.5))
            params['r_delta']       = float(rospy.get_param('~r_delta', 5.0))
            params['r_steer_reg']   = float(rospy.get_param('~r_steer_reg', 0.1))
            params['v_min']         = float(rospy.get_param('~min_speed', 0.5))
            params['v_max']         = float(rospy.get_param('~max_speed', 8.0))
            params['a_min']         = float(rospy.get_param('~a_min', -4.0))
            params['a_max']         = float(rospy.get_param('~a_max', 3.0))
            params['delta_max']     = float(rospy.get_param('~max_steering', 0.6))
            params['mu_max']        = float(rospy.get_param('~mu_max', 0.9))
            params['inflation']     = float(rospy.get_param('~boundary_inflation', 0.05))
            params['wall_safe']     = float(rospy.get_param('~wall_safe', 0.15))
            params['gap_lat']       = float(rospy.get_param('~gap_lat', 0.25))
            params['gap_long']      = float(rospy.get_param('~gap_long', 0.8))
            params['w_slack']       = float(rospy.get_param('~w_slack', 2000.0))
            params['n_obs_max']     = int(rospy.get_param('~n_obs_max', 2))
            params['ipopt_max_iter']= int(rospy.get_param('~ipopt_max_iter', 200))
            # ### HJ : v2 redesign — soft obstacle bubble + side bias + wall cushion
            params['w_obs']         = float(rospy.get_param('~w_obs', 180.0))
            params['sigma_s_obs']   = float(rospy.get_param('~sigma_s_obs', 0.7))
            params['sigma_n_obs']   = float(rospy.get_param('~sigma_n_obs', 0.18))
            params['w_side_bias']   = float(rospy.get_param('~w_side_bias', 25.0))
            params['w_wall_buf']    = float(rospy.get_param('~w_wall_buf', 2500.0))
            params['wall_buf']      = float(rospy.get_param('~wall_buf', 0.30))
            # ### HJ : v3 — C^1 curvature (δ as state) + continuity + terminal
            params['r_dd']          = float(rospy.get_param('~r_dd', 5.0))
            params['r_dd_rate']     = float(rospy.get_param('~r_dd_rate', 1.0))
            params['w_cont']        = float(rospy.get_param('~w_cont', 20.0))
            params['q_n_term']      = float(rospy.get_param('~q_n_term', 10.0))
            params['q_v_term']      = float(rospy.get_param('~q_v_term', 0.5))
            params['delta_rate_max'] = float(rospy.get_param('~delta_rate_max', 3.0))
            # ### HJ : v3b — solver-side ego half-width (was only in decider).
            params['ego_half_width'] = float(
                rospy.get_param('~ego_half_width', 0.15))
            # ### HJ : v3c+ — HSL ma27 swap. set '~linear_solver:=mumps' to revert.
            params['linear_solver'] = str(
                rospy.get_param('~linear_solver', 'ma27'))
            self.solver = FrenetKinMPC(**params)
            self.n_obs_max = params['n_obs_max']
            self.wall_safe = params['wall_safe']
            self.w_wall = 0.0
        elif self.solver_backend == 'frenet_d':
            params['obstacle_sigma'] = float(rospy.get_param(
                '~obstacle_sigma', 0.5))
            params['n_obs_max'] = int(rospy.get_param('~n_obs_max', 2))
            params['w_wall'] = float(rospy.get_param('~w_wall', 0.0))
            params['wall_safe'] = float(rospy.get_param('~wall_safe', 0.15))
            self.solver = FrenetDSolver(params)
        elif self.solver_backend == 'xy':
            self.solver = MPCCSolver(params)
        else:
            rospy.logwarn(
                '[mpc][%s] unknown ~solver_backend=%r — using frenet_kin',
                rospy.get_name(), self.solver_backend)
            self.solver_backend = 'frenet_kin'
            self.solver = FrenetKinMPC(**params)
        rospy.loginfo('[mpc][%s] solver_backend=%s',
                      rospy.get_name(), self.solver_backend)

        # ### HJ : External side decider (rule-based, with hysteresis). Only
        # meaningful for frenet_kin backend — legacy backends ignore side.
        self.side_decider = SideDecider(
            ego_half_width=float(rospy.get_param('~ego_half_width', 0.15)),
            gap_lat=float(rospy.get_param('~gap_lat', 0.25)),
            trail_dv_thresh=float(rospy.get_param('~trail_dv_thresh', 0.5)),
            hold_ticks=int(rospy.get_param('~side_hold_ticks', 10)),
            min_pass_margin=float(rospy.get_param('~min_pass_margin', 0.10)),
            trail_entry_ticks=int(rospy.get_param('~trail_entry_ticks', 3)),
            # ### HJ : v3b — mirror solver's hard-corridor wall inset so
            # decider's can_pass matches what the solver can actually do.
            wall_safe=float(rospy.get_param('~wall_safe', 0.15)),
            inflation=float(rospy.get_param('~boundary_inflation', 0.05)),
        )
        self._last_side_int = SIDE_CLEAR
        self._last_side_str = 'clear'
        self._last_side_scores = {}
        # ### HJ : v3 — bias ramp REMOVED. Solution continuity cost + stable
        # side decision (feasibility gate) make ramp unnecessary / harmful.
        # Kept attributes as stubs so tick_json stays backward compatible.
        self._ticks_since_flip = 0
        self._last_bias_scale = 1.0
        # ### HJ : v3 — obstacle s/n EMA filter. Opponent predictor jitter
        # makes the bubble centre wobble tick-to-tick, which shakes the cost
        # landscape and induces tiny trajectory jitter. EMA smoothes it.
        self.obs_ema_alpha = float(rospy.get_param('~obs_ema_alpha', 0.30))
        self._obs_arr_ema = None
        # ### HJ : v3 — TRAIL velocity ramp. On TRAIL commit, v_target is
        # ramped from current down to v_obs*0.95 over trail_vel_ramp_ticks
        # ticks. Softens the "deceleration feel" during fallback.
        self.trail_vel_ramp_ticks = int(
            rospy.get_param('~trail_vel_ramp_ticks', 8))
        self._trail_ticks_since_enter = 0

        # ### HJ : Phase 3 — obstacle cost params (read; honored by solver in
        # Phase 3.5). Kept on the node side so dynamic_reconfigure (Phase 5)
        # can update them without rebuilding the NLP.
        self.collision_mode = str(rospy.get_param('~collision_mode', 'none')).lower()
        self.w_obstacle     = float(rospy.get_param('~w_obstacle', 0.0))
        self.obstacle_sigma = float(rospy.get_param('~obstacle_sigma', 0.35))
        self.n_obs_max      = int(rospy.get_param('~n_obs_max', 2))
        self.w_wall         = float(rospy.get_param('~w_wall', 0.0))
        self.wall_safe      = float(rospy.get_param('~wall_safe', 0.15))
        if self.collision_mode not in ('none', 'soft', 'hard'):
            rospy.logwarn('[mpc][%s] unknown collision_mode=%r — falling back to "none"',
                          rospy.get_name(), self.collision_mode)
            self.collision_mode = 'none'

        # -- Ego state --------------------------------------------------------
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_z = 0.0  # ### HJ : need z to disambiguate 3D overpass layers in _nearest_idx
        self.car_yaw = 0.0
        self.car_vx = 0.0
        self.pose_received = False

        # ### HJ : canonical 3D-aware Frenet from frenet_odom_republisher
        # (stack standard, used by state_machine/controllers). Prefer this
        # over a local 2D xy-nearest that flips between overpass layers.
        self.ego_s = None
        self.ego_n = None
        self.ego_s_idx = None
        self._frenet_t = None
        self._frenet_stale_s = 0.2

        # ### HJ : Phase 4.3 — 4-tier fallback state.
        # tier 0 성공 시 trajectory + u_sol 캐시 → tier 1 에서 s-shift 로 재발행.
        # streak > H 이면 tier 2 (Frenet quintic) 진입. tier 2 실패 시 tier 3.
        self._fail_streak = 0
        self._fail_tier_H = int(rospy.get_param('~fail_streak_H', 5))
        self._last_good_traj = None    # (N+1, 5)  [x, y, psi, s, z] (lifted)
        self._last_good_frenet_traj = None  # (N+1, 4) raw frenet [s, n, mu, v]
        self._last_good_u = None       # (N, 2)    [v, δ]
        self._last_good_s = None       # s at which it was anchored
        self._last_good_time = None    # rospy.Time
        self._quintic_delta_s = float(rospy.get_param('~quintic_delta_s', 8.0))
        self._last_status = None       # for recovery log
        # ### HJ : v3c — track fallback tier so RViz marker colors stay in
        # sync with solver health. _publish_debug_markers picks the colour
        # from (_viz_tier, _viz_status, _viz_pass) set by the owning handler
        # right before the publish path runs.
        self._viz_tier = 0
        self._viz_status = 'OK'
        self._viz_pass = 1

        # ### HJ : Phase 5 — instance YAML for save/reset triggers. Default
        # resolves to config/state_<state>.yaml (same file the launch loaded).
        # Launch can override with ~instance_yaml.
        default_yaml = os.path.join(
            os.path.dirname(_this_dir), 'config', 'state_%s.yaml' % self.state)
        self.instance_yaml_path = rospy.get_param('~instance_yaml', default_yaml)
        self._suppress_dynreg_cb = False
        self._dyn_srv = None  # set after solver.setup() in _global_wpnts_cb

        # ### HJ : debug_log — structured per-tick CSV + on-anomaly NPZ dump.
        # All knobs are ROS params, runtime-writable via rosparam set + the
        # logger reads them on every tick (see _refresh_debug_cfg).
        self._debug_logger = None
        self._dbg_params_snapshot = None
        self._dbg_cfg = None
        self._dbg_tick_counter = 0
        if DebugLogger is not None:
            default_dbg_dir = os.path.join(_dbg_dir, 'runs')
            self._dbg_cfg = {
                'enable':  bool(rospy.get_param('~debug_log_enable', False)),
                'dir':     str(rospy.get_param('~debug_log_dir', default_dbg_dir)),
                'tier_b':  str(rospy.get_param('~debug_log_tier_b', 'on_anomaly')),
                'every_n': int(rospy.get_param('~debug_log_every_n', 10)),
                'anomaly_kappa_max': float(rospy.get_param(
                    '~debug_log_anomaly_kappa_max', 3.0)),
                'summary_every':  int(rospy.get_param('~debug_log_summary_every', 10)),
                'summary_window': int(rospy.get_param('~debug_log_summary_window', 50)),
                'state':   self.state,
                'node_name': rospy.get_name(),
            }
            self._dbg_params_snapshot = dict(params)
            self._dbg_params_snapshot.update({
                'obstacle_sigma': self.obstacle_sigma,
                'n_obs_max':      self.n_obs_max,
                'w_steer_reg':    float(params.get('w_steer_reg', 1e-3)),
                'solver_backend': self.solver_backend,
                'w_wall':         float(params.get('w_wall', self.w_wall)),
                'wall_safe':      float(params.get('wall_safe', self.wall_safe)),
                'contour_ramp_start': float(params.get('contour_ramp_start', 1.0)),
            })
            if self._dbg_cfg['enable']:
                self._spawn_debug_logger(reason='launch')
        else:
            rospy.logwarn_once(
                '[mpc] DebugLogger unavailable (%s) — logging disabled',
                _debug_logger_import_err)

        # Publisher for live rolling-window summary (latched; consumed by me
        # during live runs via `rostopic echo` or simply by reading the file).
        self.pub_debug_summary = rospy.Publisher(
            '~debug_log/live_summary', String, queue_size=1, latch=True)

        # ### HJ : per-tick JSON for live Claude-side `rostopic echo -c`
        # monitoring (CLAUDE.md "디버깅은 Claude가 직접 터미널에서 실시간
        # rostopic echo로 한다"). Not latched, published every tick.
        self.pub_debug_tick = rospy.Publisher(
            '~debug/tick_json', String, queue_size=1)
        # Extended debug markers (corridor walls + obstacles + ref-slice +
        # tier/status text). Kept separate from ~best_sample/markers so RViz
        # can toggle the "trajectory only" vs "context" layers independently.
        self.pub_debug_markers = rospy.Publisher(
            '~debug/markers', MarkerArray, queue_size=1)

        # Monotonic tick counter — independent of DebugLogger availability so
        # tick_json numbering stays consistent across enable/disable cycles.
        self._tick_counter = 0
        # Previous tick trajectory xy for jitter RMS computation.
        self._prev_traj_xy = None
        self._prev_traj_ego_s = None

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
        # ### HJ : Path publisher removed — RViz was resolving ~best_sample to
        # MarkerArray from an overlapping config; keep MarkerArray as the only
        # pose-list output so no consumer gets type-confused.
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
        # ### HJ : frenet_odom_republisher — 3D-aware (z included in nearest search)
        rospy.Subscriber('/car_state/odom_frenet', Odometry,
                         self._frenet_odom_cb, queue_size=1)

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
        # ### HJ : skip dynreg server for frenet_kin backend — MPCCost.cfg
        # fields are tied to legacy frenet_d/xy solver attributes. frenet_kin
        # uses YAML-only tuning; a dedicated FrenetKinCost.cfg is TODO.
        if self.solver_backend == 'frenet_kin':
            self._dyn_srv = None
            rospy.loginfo('[mpc][%s] dynreg server skipped (solver_backend=frenet_kin; '
                          'tune via state_*.yaml + node restart)', rospy.get_name())
        else:
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
        self.car_z = msg.pose.position.z  # ### HJ : 3D overpass disambiguation
        q = msg.pose.orientation
        _, _, self.car_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_received = True

    def _odom_cb(self, msg):
        self.car_vx = msg.twist.twist.linear.x

    def _frenet_odom_cb(self, msg):
        # ### HJ : frenet_odom_republisher packs s into pose.x, n into pose.y,
        # and closest waypoint index into child_frame_id (string). See
        # f110_utils/nodes/frenet_odom_republisher/.../frenet_odom_republisher_node.cc
        self.ego_s = float(msg.pose.pose.position.x)
        self.ego_n = float(msg.pose.pose.position.y)
        try:
            self.ego_s_idx = int(msg.child_frame_id)
        except (TypeError, ValueError):
            self.ego_s_idx = None
        self._frenet_t = rospy.Time.now()

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
        """### HJ : Phase 3.5 — SQP-compatible obstacle → (n_obs_max, *, 3).

        Dispatch by solver backend.
          xy:         (n_obs_max, N,   3) Cartesian [x, y, w_obs].
          frenet_d:   (n_obs_max, N+1, 3) Frenet    [s, n, w_obs].
          frenet_kin: (n_obs_max, N+1, 3) Frenet    [s, n, w_obs]  — same shape
                      as frenet_d; FrenetKinMPC._build_corridor_bounds reads
                      obs_arr[o, :, 1] (n) and obs_arr[o, :, 2] (w gate).
        All paths share the SQP-style prediction-first / tracking-fallback /
        staleness-off fusion.
        """
        if self.solver_backend in ('frenet_d', 'frenet_kin'):
            return self._build_obstacle_array_frenet(ego_s)

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

    def _build_obstacle_array_frenet(self, ego_s):
        """### HJ : Frenet-space obstacle array for FrenetDSolver.

        Returns (n_obs_max, N+1, 3) with columns [s_o, n_o, w_obs]. Obstacle
        s is unwrapped relative to ego_s (same ±½·L domain as ref_s) so the
        flat-Frenet distance `ds = ref_s[k] - s_o[k]` is correct even near a
        lap boundary. Static/low-vs obstacles are held constant; dynamic
        obstacles propagate linearly in (s, d) without modulus — slicer keeps
        ref_s monotone so a linear s_o stays consistent.
        """
        N_plus_1 = self.N + 1
        dT = self.dT
        tl = self.track_length
        far = FrenetDSolver.FAR_SN
        obs_arr = np.zeros((self.n_obs_max, N_plus_1, 3), dtype=np.float64)
        obs_arr[:, :, 0] = far  # s
        obs_arr[:, :, 1] = far  # n (far lateral too, harmless)

        if self.collision_mode == 'none' or self.w_obstacle <= 0.0:
            return obs_arr, 'disabled'

        src, tag = self._pick_obs_source()
        if tag == 'none':
            return obs_arr, 'stale'

        def fwd_dist(o):
            return (o.s_center - ego_s) % tl

        obs_list = sorted(src.obstacles, key=fwd_dist)
        obs_list = [o for o in obs_list if fwd_dist(o) < 0.5 * tl]
        # ### HJ : dedup near-duplicate obstacles (prediction+detection race can
        # emit the same object twice within ~0.5m s and ~0.1m d). Keeping both
        # loads two half-plane constraints on the same target → solver pinches
        # the trajectory and thrashes side decisions.
        n_obs_raw = len(obs_list)
        dedup = []
        for o in obs_list:
            keep = True
            for o2 in dedup:
                if (abs(o.s_center - o2.s_center) < 0.5 and
                        abs(o.d_center - o2.d_center) < 0.1):
                    keep = False
                    break
            if keep:
                dedup.append(o)
        obs_list = dedup
        self._last_n_obs_raw = n_obs_raw

        n_used = 0
        for o in obs_list:
            if n_used >= self.n_obs_max:
                break
            s0 = float(o.s_center)
            d0 = float(o.d_center)
            # Unwrap s0 into the ego's lap domain (± tl/2 around ego_s).
            while s0 - ego_s > 0.5 * tl:
                s0 -= tl
            while s0 - ego_s < -0.5 * tl:
                s0 += tl

            if o.is_static or abs(o.vs) < 1e-3:
                obs_arr[n_used, :, 0] = s0
                obs_arr[n_used, :, 1] = d0
            else:
                vs = float(o.vs)
                vd = float(o.vd)
                for k in range(N_plus_1):
                    obs_arr[n_used, k, 0] = s0 + vs * k * dT
                    obs_arr[n_used, k, 1] = d0 + vd * k * dT
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
                'contour_ramp_start': float(self.solver.contour_ramp_start),
                'lag_ramp_start':     float(self.solver.lag_ramp_start),
                'max_speed':          float(self.solver.v_max),
                'min_speed':          float(self.solver.v_min),
                'max_steering':       float(self.solver.theta_max),
                'boundary_inflation': float(self.solver.inflation),
                'w_obstacle':         float(self.w_obstacle),
                'obstacle_sigma':     float(self.obstacle_sigma),
                'w_wall':             float(self.w_wall),
                'wall_safe':          float(self.wall_safe),
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

        # ### HJ : Snapshot current weights BEFORE mutation, to log actual
        # diffs into events.jsonl. Only when not suppressing boot-time.
        before_snap = self._current_weights_snapshot() \
            if not self._suppress_dynreg_cb else None

        # --- Hot cost-weight swap (no NLP rebuild) --------------------------
        w_changed = False
        for k in ('w_contour', 'w_lag', 'w_velocity', 'v_bias_max',
                  'w_dv', 'w_dsteering', 'w_slack',
                  'contour_ramp_start', 'lag_ramp_start'):
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
        # ### HJ : Gaussian obstacle cost + convex wall buffer soft cost.
        # Both w_wall and wall_safe are baked into the frenet_d NLP, so a
        # change requires a rebuild.
        new_w_wall = float(config.w_wall)
        new_wall_safe = float(config.wall_safe)
        if self.solver_backend == 'frenet_d':
            if abs(getattr(self.solver, 'w_wall', new_w_wall) - new_w_wall) > 1e-9:
                rebuild_needed = True
            if abs(getattr(self.solver, 'wall_safe', new_wall_safe) - new_wall_safe) > 1e-9:
                rebuild_needed = True
        self.w_wall = new_w_wall
        self.wall_safe = new_wall_safe
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
                contour_ramp_start=float(config.contour_ramp_start),
                lag_ramp_start=float(config.lag_ramp_start),
                ipopt_max_iter=new_iter,
                obstacle_sigma=new_sigma,
                w_wall=new_w_wall,
                wall_safe=new_wall_safe,
            )
            dt_ms = (rospy.Time.now() - t0).to_sec() * 1000.0
            rospy.loginfo(
                '[mpc][%s] rebuilt NLP in %.1f ms (weights/sigma/iter changed)',
                rospy.get_name(), dt_ms)

        if not self._suppress_dynreg_cb:
            rospy.loginfo_throttle(
                2.0,
                '[mpc][%s] tune w(c/l/v/dv/dδ/slack)=%.2f/%.2f/%.2f/%.1f/%.1f/%.0f  '
                'v[%.1f,%.1f] δ=%.2f inf=%.2f  obs(w=%.0f σ=%.2f) wall(w=%.0f s=%.2f) H=%d Δs=%.1f iter=%d',
                rospy.get_name(),
                self.solver.w_contour, self.solver.w_lag, self.solver.w_velocity,
                self.solver.w_dv, self.solver.w_dsteering, self.solver.w_slack,
                self.solver.v_min, self.solver.v_max, self.solver.theta_max,
                self.solver.inflation,
                self.w_obstacle, self.obstacle_sigma,
                self.w_wall, self.wall_safe,
                self._fail_tier_H, self._quintic_delta_s,
                self.solver.ipopt_max_iter,
            )

        # ### HJ : events.jsonl — per-rqt-change timeline for live analysis.
        if before_snap is not None and self._debug_logger is not None:
            after_snap = self._current_weights_snapshot()
            diff = {}
            for k, v_new in after_snap.items():
                v_old = before_snap.get(k)
                if v_old is None:
                    continue
                try:
                    if abs(float(v_new) - float(v_old)) > 1e-9:
                        diff[k] = {'old': float(v_old), 'new': float(v_new)}
                except (TypeError, ValueError):
                    if v_new != v_old:
                        diff[k] = {'old': v_old, 'new': v_new}
            if diff:
                self._debug_logger.log_event('config_change', diff)
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
            data['contour_ramp_start'] = float(config.contour_ramp_start)
            data['lag_ramp_start']     = float(config.lag_ramp_start)
            # Box bounds + inflation
            data['max_speed']    = float(config.max_speed)
            data['min_speed']    = float(config.min_speed)
            data['max_steering'] = float(config.max_steering)
            data['boundary_inflation'] = float(config.boundary_inflation)
            # Obstacle
            data['w_obstacle']     = float(config.w_obstacle)
            data['obstacle_sigma'] = float(config.obstacle_sigma)
            data['w_wall']         = float(config.w_wall)
            data['wall_safe']      = float(config.wall_safe)
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
                'contour_ramp_start': float(d.get('contour_ramp_start', self.solver.contour_ramp_start)),
                'lag_ramp_start':     float(d.get('lag_ramp_start',     self.solver.lag_ramp_start)),
                'max_speed':          float(d.get('max_speed',    self.solver.v_max)),
                'min_speed':          float(d.get('min_speed',    self.solver.v_min)),
                'max_steering':       float(d.get('max_steering', self.solver.theta_max)),
                'boundary_inflation': float(d.get('boundary_inflation', self.solver.inflation)),
                'w_obstacle':         float(d.get('w_obstacle',     self.w_obstacle)),
                'obstacle_sigma':     float(d.get('obstacle_sigma', self.solver.obstacle_sigma)),
                'w_wall':             float(d.get('w_wall',         self.w_wall)),
                'wall_safe':          float(d.get('wall_safe',      self.wall_safe)),
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

    def _get_ego_frenet(self):
        """Return (s, n) for the ego. Prefer /car_state/odom_frenet (3D-aware,
        stack-standard), fall back to local 3D xy+z argmin + lifter projection.

        Fallback is never silent — logs a throttled warning so we notice if the
        frenet republisher is down in a real run.
        """
        now = rospy.Time.now()
        fresh = (
            self.ego_s is not None
            and self._frenet_t is not None
            and (now - self._frenet_t).to_sec() < self._frenet_stale_s
        )
        if fresh:
            return float(self.ego_s), float(self.ego_n)

        rospy.logwarn_throttle(
            1.0,
            '[mpc] /car_state/odom_frenet stale/missing — falling back to '
            'local 3D nearest (overpass layers may alias).',
        )
        idx = self._nearest_idx(self.car_x, self.car_y, self.car_z)
        s_fb = float(self.g_s[idx])
        n_fb = 0.0
        try:
            _, n_fb = self.lifter.project_xy_to_sn(
                self.car_x, self.car_y, idx_hint=idx,
            )
        except Exception:
            pass
        return s_fb, n_fb

    def _nearest_idx(self, x, y, z=None):
        # ### HJ : 3D distance (xy + z) so overpass layers don't collide.
        # In 2D, xy=(3.28,-2.08) matches both s=11.7 (z=0.14) and s=53.7
        # (z=0.56) on the gazebo_wall_2 map, causing ego_s to flip between
        # floors and feeding the NLP a 90°-rotated reference.
        if z is None:
            z = getattr(self, 'car_z', 0.0)
        d2 = ((self.g_x - x) ** 2
              + (self.g_y - y) ** 2
              + (self.g_z - z) ** 2)
        return int(np.argmin(d2))

    # ---------------------------------------------------------------- side
    def _decide_side(self, obs_arr, ref_slice):
        """Rule-based side-of-passing decision (LEFT / RIGHT / TRAIL / CLEAR).
        Kept external to the NLP so the MPC enforces one-sided hard
        constraints without ambiguity. See src/side_decider.py."""
        if obs_arr is None:
            return SIDE_CLEAR, 'clear', {'reason': 'no_obs_arr'}

        rs = np.asarray(ref_slice['ref_s'])
        dL_ref = np.asarray(ref_slice['d_left_arr'])
        dR_ref = np.asarray(ref_slice['d_right_arr'])
        N_plus_1 = rs.shape[0]

        obs_list = []
        for o in range(obs_arr.shape[0]):
            w_ts = obs_arr[o, :, 2]
            if float(np.max(w_ts)) <= 0.0:
                continue
            s0 = float(obs_arr[o, 0, 0])
            n0 = float(obs_arr[o, 0, 1])
            sN = float(obs_arr[o, -1, 0])
            nN = float(obs_arr[o, -1, 1])
            v_s_obs = (sN - s0) / max(self.N * self.dT, 1e-3)
            # d_L, d_R at the obstacle's s (use nearest ref_slice index)
            k_near = int(np.argmin(np.abs(rs - s0)))
            k_near = int(np.clip(k_near, 0, N_plus_1 - 1))
            obs_list.append({
                's0': s0, 'n0': n0, 'v_s_obs': v_s_obs,
                'half_width': 0.15,      # conservative ego-like half width
                'd_L': float(dL_ref[k_near]),
                'd_R': float(dR_ref[k_near]),
                'ref_v': float(ref_slice['ref_v'][k_near]),
            })
        # sort by forward distance (smallest s0 first assuming ref_s is
        # monotone forward from ego — which _slice_local_ref guarantees)
        obs_list.sort(key=lambda d: d['s0'])

        ego_v = float(getattr(self, 'car_vx', 0.0))
        return self.side_decider.decide(ego_v, obs_list)

    # ---------------------------------------------------------------- lift
    def _lift_frenet_to_xy(self, traj_frenet, ref_slice):
        """Convert solver (s, n, mu, v) trajectory into (x, y, psi, s, z).
        Uses ref_slice center_points + tangent (ref_dx, ref_dy) so no
        xy→frenet round-trip is needed (CLAUDE.md: 3D 트랙에서 Frenet xy
        round-trip 금지). `s` is the solver's Frenet station (raceline
        arc-length) which uniquely identifies the overpass floor; `z` is
        interpolated from g_z at that s. Downstream viz/markers MUST use
        the carried s/z instead of re-projecting (x, y) — otherwise the
        2D nearest-xy lookup aliases overpass layers at crossing points.
        Returned shape: (M, 5) columns [x, y, psi, s, z]."""
        N1 = traj_frenet.shape[0]
        rc = ref_slice['center_points']          # (N+1, 2)
        rdx = ref_slice['ref_dx']                # cos(psi_ref)
        rdy = ref_slice['ref_dy']                # sin(psi_ref)
        M = int(min(N1, rc.shape[0]))
        # Left (+n) normal in xy = (-sin(psi_ref), cos(psi_ref)) = (-rdy, rdx)
        lnx = -rdy[:M]
        lny = rdx[:M]
        out = np.zeros((M, 5), dtype=np.float64)
        for k in range(M):
            s_k = float(traj_frenet[k, 0])
            n_k = float(traj_frenet[k, 1])
            mu_k = float(traj_frenet[k, 2])
            out[k, 0] = float(rc[k, 0] + n_k * lnx[k])
            out[k, 1] = float(rc[k, 1] + n_k * lny[k])
            psi_ref = float(np.arctan2(rdy[k], rdx[k]))
            out[k, 2] = psi_ref + mu_k
            out[k, 3] = s_k
            out[k, 4] = float(self.lifter._interp(s_k, self.lifter.g_z))
        return out

    def _slice_local_ref(self, s_cur):
        """Build N+1 local reference from /global_waypoints around s_cur.
        Port of the original mpc_planner_node._slice_local_ref — unchanged."""
        N = self.N
        dT = self.solver.dT
        tl = self.track_length
        inflation = self.solver.inflation

        center_pts, left_pts, right_pts = [], [], []
        d_left_arr, d_right_arr = [], []
        ref_v, ref_s, ref_dx, ref_dy, ref_psi, ref_kappa = [], [], [], [], [], []
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
            kappa = self.g_kappa[idx]

            normal = np.array([-np.sin(psi), np.cos(psi)])
            center = np.array([x, y])
            center_pts.append(center)
            left_pts.append(center + normal * max(dl - inflation, 0.0))
            right_pts.append(center - normal * max(dr - inflation, 0.0))
            d_left_arr.append(float(dl))
            d_right_arr.append(float(dr))
            ref_v.append(float(self.g_vx[idx]))
            ref_dx.append(np.cos(psi))
            ref_dy.append(np.sin(psi))
            ref_psi.append(float(psi))
            ref_kappa.append(float(kappa))

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
            'd_left_arr': np.array(d_left_arr),
            'd_right_arr': np.array(d_right_arr),
            'ref_v': np.array(ref_v),
            'ref_s': np.array(ref_s),
            'ref_dx': np.array(ref_dx),
            'ref_dy': np.array(ref_dy),
            'ref_psi': np.array(ref_psi),
            'kappa_ref': np.array(ref_kappa),
        }

    # ---------------------------------------------------------------- main loop
    def _plan_loop(self, event):
        if not self.solver.ready or not self.pose_received:
            return

        # Runtime debug_log toggles (rosparam poll — cheap, ~μs).
        self._refresh_debug_runtime()

        # ### HJ : prefer canonical 3D-aware Frenet from odom_frenet_republisher.
        # Only fall back to local 3D xy+z nearest if odom_frenet is stale/missing
        # (startup, publisher crash). The local fallback can still alias overpass
        # layers if |Δz| < xy noise — odom_frenet uses proper frenet_converter
        # with z-aware segment projection which is more robust.
        s_cur, ego_n = self._get_ego_frenet()
        ref_slice = self._slice_local_ref(s_cur)
        # ### HJ : FrenetDSolver needs current lateral offset as the fixed
        # initial constraint n_0 = n_ego. Harmless for xy backend (ignored).
        ref_slice['n_ego'] = float(ego_n)

        # ### HJ : Phase 3.5 — fuse obstacle inputs and hand to solver.
        obs_arr, obs_tag = self._build_obstacle_array(s_cur)

        # ### HJ : v3 — obstacle EMA filter. Blend new obs_arr with prev
        # tick's (same shape (n_slot, N+1, 3)). Only blend (s, n) columns;
        # keep active-weight (col 2) as a hard on/off to avoid zombie slots.
        if self.solver_backend == 'frenet_kin':
            if obs_arr is None:
                # no obstacles this tick — drop stale EMA so a future
                # re-acquisition doesn't blend against 5-sec-old data.
                self._obs_arr_ema = None
            elif (self._obs_arr_ema is not None
                    and self._obs_arr_ema.shape == obs_arr.shape):
                a = float(self.obs_ema_alpha)
                blend = self._obs_arr_ema.copy()
                # carry active gate from new (fresh activation decisions)
                blend[:, :, 2] = obs_arr[:, :, 2]
                # EMA on (s, n) only where both prev and new are active
                active_mask = (obs_arr[:, :, 2] > 0.0) \
                              & (self._obs_arr_ema[:, :, 2] > 0.0)
                for col in (0, 1):
                    blend[:, :, col] = np.where(
                        active_mask,
                        a * obs_arr[:, :, col] + (1.0 - a) * self._obs_arr_ema[:, :, col],
                        obs_arr[:, :, col])
                self._obs_arr_ema = blend
                obs_arr = blend
            else:
                self._obs_arr_ema = obs_arr.copy()

        # ### HJ : Build initial_state + side decision per backend.
        if self.solver_backend == 'frenet_kin':
            # ψ_ref at s_cur (ref_psi[0]).
            psi_ref0 = float(ref_slice['ref_psi'][0])
            mu0 = float(self.car_yaw - psi_ref0)
            # wrap to [-pi, pi]
            mu0 = (mu0 + np.pi) % (2.0 * np.pi) - np.pi
            v0 = float(getattr(self, 'car_vx', 0.5))
            initial_state = np.array([float(ego_n), mu0, v0])
            # side decision (rule-based, external, now feasibility-aware)
            side_int, side_str, side_scores = self._decide_side(obs_arr, ref_slice)
            # ### HJ : v3 — bias_scale ramp REMOVED. Continuity cost in
            # solver handles tick-to-tick smoothness directly. Fixed 1.0.
            if side_int != self._last_side_int:
                self._ticks_since_flip = 0
            else:
                self._ticks_since_flip += 1
            bias_scale = 1.0
            self._last_bias_scale = bias_scale
            # TRAIL entry tick counter (for debug only; velocity ramp lives
            # in the solver via v_max cap when side==TRAIL)
            if side_int == SIDE_TRAIL and self._last_side_int != SIDE_TRAIL:
                self._trail_ticks_since_enter = 0
            elif side_int == SIDE_TRAIL:
                self._trail_ticks_since_enter += 1
            else:
                self._trail_ticks_since_enter = 0
            self._last_side_int = side_int
            self._last_side_str = side_str
            self._last_side_scores = side_scores
        else:
            initial_state = np.array([self.car_x, self.car_y, self.car_yaw])
            side_int = SIDE_CLEAR; side_str = 'n/a'; side_scores = {}
            bias_scale = 0.0

        warm_used = int(bool(getattr(self.solver, 'warm', False)))

        t0 = time.time()
        if self.solver_backend == 'frenet_kin':
            speed, steering, trajectory, success = self.solver.solve(
                initial_state, ref_slice, obstacles=obs_arr, side=side_int,
                bias_scale=bias_scale)
        else:
            speed, steering, trajectory, success = self.solver.solve(
                initial_state, ref_slice, obstacles=obs_arr)
        solve_ms = (time.time() - t0) * 1000.0
        self.pub_timing.publish(Float32(data=solve_ms))

        u_sol = getattr(self.solver, 'last_u_sol', None)
        ipopt_status = getattr(self.solver, 'last_return_status', '-')
        iter_count = getattr(self.solver, 'last_iter_count', -1)
        slack_max = float(getattr(self.solver, 'last_slack_max', 0.0))

        # ### HJ : frenet_kin returns trajectory in (s, n, mu, v). The rest
        # of the pipeline (_publish_outputs, _publish_debug_markers) expects
        # (x, y, psi). Lift before handing off. Also stash the original
        # frenet trajectory for the tick_json payload (margins etc.) AND for
        # the downstream Wpnt filler (`fill_wpnt_from_s`, 3D-safe) — do NOT
        # reset to None on failure. Failure paths (tier1/2/3) assign their
        # own synthetic frenet trajectory below so `_publish_outputs` never
        # falls back to the 2D xy→s round-trip that aliases overpass layers.
        if (self.solver_backend == 'frenet_kin' and success
                and trajectory is not None):
            self._last_frenet_traj = np.array(trajectory, copy=True)
            trajectory = self._lift_frenet_to_xy(trajectory, ref_slice)
        else:
            # Keep the last successful frenet traj so tier1 can re-publish it
            # via fill_wpnt_from_s. Tiers 2/3 overwrite this with their own
            # synthetic (s, n) array before calling _publish_outputs.
            pass

        # ### HJ : Phase 4.3 — 4-tier fallback. Even at tier 3 we still
        # publish a sane Wpnt[] so controller has zero-gap input.
        if success and trajectory is not None:
            self._handle_tier0_success(trajectory, s_cur, solve_ms, obs_tag,
                                       speed, steering)
            self._debug_log(
                tier=0, status='OK', ipopt_status=ipopt_status,
                iter_count=iter_count, solve_ms=solve_ms, slack_max=slack_max,
                trajectory=trajectory, u_sol=u_sol, obs_arr=obs_arr,
                obs_tag=obs_tag, ref_slice=ref_slice,
                initial_state=initial_state, ego_s=s_cur, ego_n=ego_n,
                warm_used=warm_used, speed0=speed, steer0=steering,
            )
            return

        # NLP failed — move down the ladder.
        self._fail_streak += 1
        status = ipopt_status
        self.solver.reset_warm_start()

        if self._last_good_traj is not None and self._fail_streak <= self._fail_tier_H:
            self._handle_tier1_hold_last(status, solve_ms, obs_tag)
            self._debug_log(
                tier=1, status='HOLD_LAST', ipopt_status=status,
                iter_count=iter_count, solve_ms=solve_ms, slack_max=slack_max,
                trajectory=self._last_good_traj, u_sol=self._last_good_u,
                obs_arr=obs_arr, obs_tag=obs_tag, ref_slice=ref_slice,
                initial_state=initial_state, ego_s=s_cur, ego_n=ego_n,
                warm_used=warm_used,
            )
            return

        # ### HJ : v3b — tier2 gets (s_cur, ego_n) from /odom_frenet directly.
        # Previously `_try_quintic_fallback(s_cur)` re-projected (car_x, car_y)
        # via lifter.project_xy_to_sn — 2D nearest that aliases overpass
        # floors. The /odom_frenet s is z-aware and cannot alias.
        traj_fb, frenet_fb = self._try_quintic_fallback(
            s_cur, ego_n, delta_s=self._quintic_delta_s)
        if traj_fb is not None:
            self._handle_tier2_geometric(
                traj_fb, frenet_fb, ego_n, status, solve_ms, obs_tag)
            self._debug_log(
                tier=2, status='GEOMETRIC_FALLBACK', ipopt_status=status,
                iter_count=iter_count, solve_ms=solve_ms, slack_max=slack_max,
                trajectory=traj_fb, u_sol=None, obs_arr=obs_arr,
                obs_tag=obs_tag, ref_slice=ref_slice,
                initial_state=initial_state, ego_s=s_cur, ego_n=ego_n,
                warm_used=warm_used,
            )
            return

        # ### HJ : v3b — tier3 used to be a raw raceline slice (n=0 forced),
        # which jumps the controller whenever ego_n is non-zero. Per user's
        # recovery-style directive: tier3 is now an aggressive short-Δs
        # quintic (ego_n → 0 convergence). If even that blows up, we fall
        # through to the last-resort pure raceline slice.
        traj_short, frenet_short = self._try_quintic_fallback(
            s_cur, ego_n, delta_s=max(self._quintic_delta_s * 0.5, 3.0))
        if traj_short is not None:
            self._handle_tier3_convergence_quintic(
                traj_short, frenet_short, ego_n, status, solve_ms, obs_tag)
            self._debug_log(
                tier=3, status='CONVERGENCE_QUINTIC', ipopt_status=status,
                iter_count=iter_count, solve_ms=solve_ms, slack_max=slack_max,
                trajectory=traj_short, u_sol=None, obs_arr=obs_arr,
                obs_tag=obs_tag, ref_slice=ref_slice,
                initial_state=initial_state, ego_s=s_cur, ego_n=ego_n,
                warm_used=warm_used,
            )
            return

        traj_last = self._handle_tier3_raceline(
            s_cur, status, solve_ms, obs_tag)
        # ### HJ : v3b — feed the raceline-slice trajectory to debug_log so
        # the RViz marker keeps showing a trajectory line even in absolute
        # worst-case (both quintics raised).
        self._debug_log(
            tier=3, status='RACELINE_SLICE', ipopt_status=status,
            iter_count=iter_count, solve_ms=solve_ms, slack_max=slack_max,
            trajectory=traj_last, u_sol=None, obs_arr=obs_arr, obs_tag=obs_tag,
            ref_slice=ref_slice, initial_state=initial_state,
            ego_s=s_cur, ego_n=ego_n, warm_used=warm_used,
        )

    # ---------------------------------------------------------------- debug_log
    def _spawn_debug_logger(self, reason='runtime'):
        """(Re)create the DebugLogger → new run directory + fresh files."""
        if DebugLogger is None or self._dbg_cfg is None:
            return
        if self._debug_logger is not None:
            try:
                self._debug_logger.close()
            except Exception:
                pass
            self._debug_logger = None
        try:
            self._debug_logger = DebugLogger(
                dict(self._dbg_cfg, enable=True),
                self._dbg_params_snapshot,
                repo_hint_path=os.path.dirname(_dbg_dir))
            self._debug_logger.log_event('run_start', {
                'reason': reason,
                'state': self.state,
                'cfg': {k: v for k, v in self._dbg_cfg.items()
                        if k not in ('node_name',)},
            })
            rospy.loginfo('[mpc][%s] debug_log run started (%s): %s',
                          rospy.get_name(), reason,
                          getattr(self._debug_logger, '_run_dir', '?'))
        except Exception as e:
            rospy.logwarn('[mpc][%s] debug_log init failed: %s',
                          rospy.get_name(), e)
            self._debug_logger = None

    def _current_weights_snapshot(self):
        """Dict of every tunable the logger records per tick. Mirrors rqt.
        Backend-aware: frenet_kin exposes a different set of knobs than
        the legacy frenet_d / xy backends."""
        if self.solver_backend == 'frenet_kin':
            s = self.solver
            return {
                'backend':        'frenet_kin',
                'q_n':            float(s.q_n),
                'gamma_progress': float(s.gamma),
                'r_a':            float(s.r_a),
                'r_steer_reg':    float(s.r_reg),
                'w_slack':        float(s.w_slack),
                'v_min':          float(s.v_min),
                'v_max':          float(s.v_max),
                'a_min':          float(s.a_min),
                'a_max':          float(s.a_max),
                'delta_max':      float(s.delta_max),
                'delta_rate_max': float(s.delta_rate_max),
                'mu_max':         float(s.mu_max),
                'inflation':      float(s.inflation),
                'wall_safe':      float(s.wall_safe),
                'gap_lat':        float(s.gap_lat),
                'gap_long':       float(s.gap_long),
                'w_obs':          float(s.w_obs),
                'sigma_s_obs':    float(s.sigma_s_obs),
                'sigma_n_obs':    float(s.sigma_n_obs),
                'w_side_bias':    float(s.w_side_bias),
                'w_wall_buf':     float(s.w_wall_buf),
                'wall_buf':       float(s.wall_buf),
                # ### HJ : v3 — C^1 / continuity / terminal weights
                'r_dd':           float(s.r_dd),
                'r_dd_rate':      float(s.r_dd_rate),
                'w_cont':         float(s.w_cont),
                'q_n_term':       float(s.q_n_term),
                'q_v_term':       float(s.q_v_term),
                # feasibility gate + obstacle filter (node-side)
                'min_pass_margin':    float(self.side_decider.min_pass_margin),
                'trail_entry_ticks':  int(self.side_decider.trail_entry_ticks),
                'obs_ema_alpha':      float(self.obs_ema_alpha),
                'trail_vel_ramp_ticks': int(self.trail_vel_ramp_ticks),
                'ipopt_max_iter': int(s.ipopt_max_iter),
            }
        # Legacy frenet_d / xy (kept for A/B rollback).
        return {
            'backend':      self.solver_backend,
            'w_contour':    float(self.solver.w_contour),
            'w_lag':        float(self.solver.w_lag),
            'w_velocity':   float(self.solver.w_velocity),
            'v_bias_max':   float(self.solver.v_bias_max),
            'w_dv':         float(self.solver.w_dv),
            'w_dsteering':  float(self.solver.w_dsteering),
            'w_slack':      float(self.solver.w_slack),
            'contour_ramp_start': float(self.solver.contour_ramp_start),
            'lag_ramp_start':     float(self.solver.lag_ramp_start),
            'max_speed':    float(self.solver.v_max),
            'min_speed':    float(self.solver.v_min),
            'max_steering': float(self.solver.theta_max),
            'boundary_inflation': float(self.solver.inflation),
            'w_obstacle':     float(self.w_obstacle),
            'obstacle_sigma': float(self.solver.obstacle_sigma),
            'w_wall':         float(self.w_wall),
            'wall_safe':      float(self.wall_safe),
            'ipopt_max_iter': int(self.solver.ipopt_max_iter),
        }

    def _refresh_debug_runtime(self):
        """### HJ : Runtime rosparam polling — lets me toggle logging on/off
        and rotate the run directory without restarting the node. Called at
        the top of every _plan_loop tick; ~1 μs when params haven't changed."""
        if DebugLogger is None or self._dbg_cfg is None:
            return
        try:
            want_enable = bool(rospy.get_param(
                '~debug_log_enable', self._dbg_cfg['enable']))
            want_reset = bool(rospy.get_param('~debug_log_reset_run', False))
        except Exception:
            return
        currently_on = self._debug_logger is not None

        if want_reset:
            rospy.set_param('~debug_log_reset_run', False)
            self._spawn_debug_logger(reason='reset_run')
            return

        if want_enable and not currently_on:
            self._dbg_cfg['enable'] = True
            self._spawn_debug_logger(reason='rosparam_enable')
        elif (not want_enable) and currently_on:
            self._dbg_cfg['enable'] = False
            try:
                self._debug_logger.close()
            except Exception:
                pass
            self._debug_logger = None
            rospy.loginfo('[mpc][%s] debug_log paused (rosparam disable)',
                          rospy.get_name())

    def _debug_log(self, **fields):
        """### HJ : Single entry point into the per-tick logger. No-op when
        the logger is disabled or failed to init. Exceptions are swallowed —
        logging must never crash the planner.

        Even when the file logger is disabled, we still publish the live
        `~debug/tick_json` + `~debug/markers` so Claude-side `rostopic echo`
        monitoring (CLAUDE.md HJ mode) keeps working.
        """
        # Live topics first (independent of file-logging state).
        try:
            self._publish_tick_live(fields)
        except Exception as e:  # pragma: no cover
            rospy.logwarn_throttle(
                5.0, '[mpc][%s] tick publish error: %s',
                rospy.get_name(), e)

        if self._debug_logger is None:
            return
        try:
            t_ros = rospy.Time.now().to_sec()
            rec = {
                't_ros': t_ros,
                'state': self.state,
                'car_x': self.car_x, 'car_y': self.car_y,
                'car_yaw': self.car_yaw, 'car_vx': self.car_vx,
                'car_vy': float(getattr(self, 'car_vy', 0.0)),
                'fail_streak': self._fail_streak,
                'weights': self._current_weights_snapshot(),
            }
            rec.update(fields)
            self._debug_logger.log_tick(rec)
            self._dbg_tick_counter += 1
            # Publish live summary every N ticks (cheap, file is already
            # written — we just echo it to ROS as well).
            if (self._dbg_tick_counter %
                    max(self._dbg_cfg.get('summary_every', 10), 1)) == 0:
                summary = self._debug_logger.get_live_summary()
                if summary is not None:
                    import json as _json
                    self.pub_debug_summary.publish(
                        String(data=_json.dumps(summary)))
        except Exception as e:  # pragma: no cover
            rospy.logwarn_throttle(
                5.0, '[mpc][%s] debug_log error: %s',
                rospy.get_name(), e)

    # ---- Tier 0 ------------------------------------------------------------
    def _handle_tier0_success(self, trajectory, s_cur, solve_ms, obs_tag,
                              speed, steering):
        if self._last_status and self._last_status != 'OK':
            rospy.loginfo('[mpc][%s] recovered after tier=%s  streak=%d',
                          rospy.get_name(), self._last_status, self._fail_streak)
        self._fail_streak = 0
        self._viz_tier = 0
        self._viz_status = 'OK'
        self._viz_pass = int(getattr(self.solver, 'last_pass', 1) or 1)

        self._publish_outputs(trajectory)
        self._last_good_traj = np.array(trajectory, copy=True)
        # ### HJ : v3b — cache raw frenet too so HOLD_LAST can re-publish
        # via fill_wpnt_from_s (3D-safe) instead of fill_wpnt (2D xy lookup).
        if self._last_frenet_traj is not None:
            self._last_good_frenet_traj = np.array(
                self._last_frenet_traj, copy=True)
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
        self._viz_tier = 1
        self._viz_status = 'HOLD_LAST'
        # ### HJ : v3b — restore the cached raw frenet trajectory so
        # _publish_outputs takes the s-direct path (fill_wpnt_from_s) and
        # bypasses xy→s projection that would alias overpass floors.
        self._last_frenet_traj = (
            np.array(self._last_good_frenet_traj, copy=True)
            if self._last_good_frenet_traj is not None else None)
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
    def _try_quintic_fallback(self, s_cur, ego_n, delta_s=None):
        """### HJ : v3b — recovery-style "ego_n → 0" smooth return primitive.

        Uses (s_cur, ego_n) from /odom_frenet directly so the 3D overpass
        layer is preserved. Previous implementation called
        `lifter.project_xy_to_sn(car_x, car_y)` which is 2D nearest and
        aliases overpass floors at bridge entry — exactly the failure mode
        the user is debugging (solver dies at bridge, tier2 then re-projects
        to the wrong floor and sends the car onto the lower path).

        Returns (xy_traj (N+1,5) [x, y, psi, s, z], frenet (N+1,4) [s, n, 0, v])
        or (None, None) if the lifter blows up.
        """
        if delta_s is None:
            delta_s = self._quintic_delta_s
        try:
            psi_track = self.lifter._interp_psi(s_cur)
            psi_delta = float(np.arctan2(
                np.sin(self.car_yaw - psi_track),
                np.cos(self.car_yaw - psi_track)))
            xy_traj, sn_traj = build_quintic_fallback(
                self.lifter, s_cur, ego_n, psi_delta,
                delta_s=delta_s, n_samples=self.N + 1,
                return_frenet=True)
            # Augment xy_traj (N+1,3) → (N+1,5) with carried [s, z] so the
            # downstream viz/publish path uses 3D-safe z instead of the
            # marker-time xy nearest-index lookup (which would alias floors).
            N1 = xy_traj.shape[0]
            aug = np.zeros((N1, 5), dtype=np.float64)
            aug[:, :3] = xy_traj
            for i in range(N1):
                s_i = float(sn_traj[i, 0])
                aug[i, 3] = s_i
                aug[i, 4] = float(self.lifter._interp(s_i, self.lifter.g_z))
            # Frenet companion for fill_wpnt_from_s (solver-frenet-shape
            # compatible: (N+1, 4) [s, n, mu=0, v=ego_v_placeholder]).
            v_fb = float(np.clip(self.car_vx, 0.5,
                                 max(self.solver.v_max, 0.5)))
            frenet = np.zeros((N1, 4), dtype=np.float64)
            frenet[:, 0] = sn_traj[:, 0]
            frenet[:, 1] = sn_traj[:, 1]
            frenet[:, 3] = v_fb
            return aug, frenet
        except Exception as exc:  # pragma: no cover — guarded log only
            rospy.logerr_throttle(
                1.0, '[mpc fallback tier2][%s] quintic build raised: %s',
                rospy.get_name(), exc)
            return None, None

    def _handle_tier2_geometric(self, traj_fb, frenet_fb, ego_n,
                                ipopt_status, solve_ms, obs_tag):
        self._viz_tier = 2
        self._viz_status = 'GEOMETRIC_FALLBACK'
        # Synthetic u_sol — constant-velocity placeholder so the lifter can
        # still fill vx_mps / ax_mps2. v chosen as mean of the raceline-slice
        # local target to stay close to the velocity planner's expectation.
        v_fb = float(np.clip(self.car_vx, 0.5, max(self.solver.v_max, 0.5)))
        u_fb = np.zeros((self.N, 2), dtype=np.float64)
        u_fb[:, 0] = v_fb
        # ### HJ : v3b — stash synthetic frenet so _publish_outputs takes
        # the s-direct path (fill_wpnt_from_s) and does NOT re-project xy.
        self._last_frenet_traj = frenet_fb
        self._publish_outputs(traj_fb, u_sol_override=u_fb)
        self._publish_status('GEOMETRIC_FALLBACK streak=%d n0=%.2f obs=%s' %
                             (self._fail_streak, ego_n, obs_tag))
        self._last_status = 'GEOMETRIC_FALLBACK'
        rospy.logwarn_throttle(
            0.5,
            '[mpc fallback tier2][%s] GEOMETRIC streak=%d n0=%.2f Δs=%.1f '
            'ipopt=%s solve=%.1fms obs=%s',
            rospy.get_name(), self._fail_streak, ego_n, self._quintic_delta_s,
            ipopt_status, solve_ms, obs_tag,
        )

    # ---- Tier 3 (primary) -------------------------------------------------
    def _handle_tier3_convergence_quintic(self, traj_fb, frenet_fb, ego_n,
                                          ipopt_status, solve_ms, obs_tag):
        """### HJ : v3b — "recovery spliner" analogue. Ego_n → 0 over a
        shorter Δs (more aggressive than tier2). Used when tier2 quintic
        would also be feasible but we want a snappier return-to-center
        shape. Controller still sees continuous (s, n) from ego pose."""
        self._viz_tier = 3
        self._viz_status = 'CONVERGENCE_QUINTIC'
        v_fb = float(np.clip(self.car_vx, 0.5, max(self.solver.v_max, 0.5)))
        u_fb = np.zeros((self.N, 2), dtype=np.float64)
        u_fb[:, 0] = v_fb
        self._last_frenet_traj = frenet_fb
        self._publish_outputs(traj_fb, u_sol_override=u_fb)
        self._publish_status('CONVERGENCE_QUINTIC streak=%d n0=%.2f obs=%s' %
                             (self._fail_streak, ego_n, obs_tag))
        self._last_status = 'CONVERGENCE_QUINTIC'
        rospy.logerr_throttle(
            0.5,
            '[mpc fallback tier3][%s] CONVERGENCE_QUINTIC streak=%d n0=%.2f '
            'ipopt=%s solve=%.1fms obs=%s',
            rospy.get_name(), self._fail_streak, ego_n, ipopt_status,
            solve_ms, obs_tag,
        )

    # ---- Tier 3 (absolute last-resort) ------------------------------------
    def _handle_tier3_raceline(self, s_cur, ipopt_status, solve_ms, obs_tag):
        """Only reached when BOTH tier2 and tier3 quintics raised. Pure
        raceline slice (n=0, no ego-anchor). Controller may see a one-tick
        jump but output never drops."""
        self._viz_tier = 3
        self._viz_status = 'RACELINE_SLICE'
        N_total = self.N + 1
        ds_grid = np.linspace(0.0, max(self._quintic_delta_s, 4.0), N_total)
        traj = np.zeros((N_total, 5), dtype=np.float64)
        frenet = np.zeros((N_total, 4), dtype=np.float64)
        for i, ds in enumerate(ds_grid):
            s_i = s_cur + ds
            x, y = self.lifter.sn_to_xy(s_i, 0.0)
            psi = self.lifter._interp_psi(s_i)
            z = float(self.lifter._interp(s_i, self.lifter.g_z))
            traj[i, 0] = x
            traj[i, 1] = y
            traj[i, 2] = psi
            traj[i, 3] = s_i
            traj[i, 4] = z
            frenet[i, 0] = s_i
            frenet[i, 1] = 0.0

        v_fb = float(np.clip(self.car_vx, 0.5, max(self.solver.v_max, 0.5)))
        u_fb = np.zeros((self.N, 2), dtype=np.float64)
        u_fb[:, 0] = v_fb
        frenet[:, 3] = v_fb
        self._last_frenet_traj = frenet
        self._publish_outputs(traj, u_sol_override=u_fb)
        self._publish_status('RACELINE_SLICE streak=%d obs=%s' %
                             (self._fail_streak, obs_tag))
        self._last_status = 'RACELINE_SLICE'
        rospy.logerr_throttle(
            0.5,
            '[mpc fallback tier3-last][%s] RACELINE_SLICE streak=%d ipopt=%s '
            'solve=%.1fms obs=%s — BOTH quintics failed',
            rospy.get_name(), self._fail_streak, ipopt_status, solve_ms, obs_tag,
        )
        return traj

    def _publish_outputs(self, trajectory, u_sol_override=None):
        """trajectory: (N+1, 3) [x, y, psi]. Publish:
         - WpntArray on ~best_trajectory_observation (always)
         - MarkerArray on ~best_sample/markers (debug, always)
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
        # ### HJ : v3 — for frenet backends the solver holds s directly
        # (frenet state). Use fill_wpnt_from_s to skip xy→s argmin which
        # fails at overpass crossings in 3D (CLAUDE.md: xy round-trip 금지).
        fren_traj = getattr(self, '_last_frenet_traj', None)
        use_s_direct = (fren_traj is not None
                        and fren_traj.shape[0] == N_traj
                        and self.solver_backend in ('frenet_kin', 'frenet_d'))
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

            if use_s_direct:
                s_i = float(fren_traj[i, 0])
                n_i = float(fren_traj[i, 1])
                fields = self.lifter.fill_wpnt_from_s(
                    s_ref=s_i, n_ref=n_i,
                    x=x, y=y, psi_mpc=psi_mpc,
                    vx_mpc=vx, ax_mpc=ax,
                )
            else:
                fields = self.lifter.fill_wpnt(
                    x, y, psi_mpc, vx_mpc=vx, ax_mpc=ax, idx_hint=idx_hint,
                )
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

    def _publish_debug_markers(self, header, trajectory):
        arr = MarkerArray()

        # ### HJ : 3D 트랙의 overpass(교차 층)에서 같은 (x, y)에 서로 다른 z가
        # 존재 → project_xy_to_sn(2D 최단)로 s를 역산하면 아래층/위층이
        # 뒤섞임. frenet_kin 백엔드는 _lift_frenet_to_xy에서 (x, y, psi, s, z)
        # 5컬럼을 싣고 내려오므로 컬럼이 ≥5면 담긴 z를 그대로 쓴다. Fallback
        # 타이어(tier2/3)는 3컬럼이라 s를 모르므로 z=0을 사용.
        has_sz = trajectory.shape[1] >= 5

        # LINE_STRIP — predicted path
        line = Marker()
        line.header = header
        line.ns = 'mpc_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        # ### HJ : v3c — rainbow-graded solver health colour (best→worst):
        #   tier0 pass1  → red     (NLP 1-pass clean, primary mode)
        #   tier0 pass2  → orange  (NLP needed the TRAIL retry)
        #   tier1        → yellow  (HOLD_LAST — re-use last good)
        #   tier2        → green   (GEOMETRIC_FALLBACK quintic Δs≈8m)
        #   tier3 conv.  → cyan    (CONVERGENCE_QUINTIC Δs≈4m)
        #   tier3 rline  → blue    (RACELINE_SLICE — absolute last)
        tier = int(getattr(self, '_viz_tier', 0))
        status = str(getattr(self, '_viz_status', 'OK'))
        vpass = int(getattr(self, '_viz_pass', 1))
        if tier == 0 and vpass == 1:
            rgb = (1.0, 0.05, 0.05)   # red
        elif tier == 0:
            rgb = (1.0, 0.55, 0.0)    # orange
        elif tier == 1:
            rgb = (1.0, 0.95, 0.0)    # yellow
        elif tier == 2:
            rgb = (0.1, 0.9, 0.2)     # green
        elif tier == 3 and status == 'CONVERGENCE_QUINTIC':
            rgb = (0.0, 0.85, 0.95)   # cyan
        else:
            rgb = (0.1, 0.3, 1.0)     # blue (RACELINE_SLICE / unknown)
        line.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.95)
        line.pose.orientation.w = 1.0
        for i in range(trajectory.shape[0]):
            pt = Point()
            pt.x = float(trajectory[i, 0])
            pt.y = float(trajectory[i, 1])
            pt.z = float(trajectory[i, 4]) if has_sz else 0.0
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
            pt = Point()
            pt.x = float(trajectory[i, 0])
            pt.y = float(trajectory[i, 1])
            pt.z = float(trajectory[i, 4]) if has_sz else 0.0
            pts.points.append(pt)
        arr.markers.append(pts)

        self.pub_best_markers.publish(arr)

    # ---- live debug publishers --------------------------------------------
    def _publish_tick_live(self, fields):
        """### HJ : Build and publish the per-tick live debug bundle.

        Two surfaces:
          (1) ~debug/tick_json  — flat JSON for `rostopic echo -c` monitoring.
          (2) ~debug/markers    — RViz MarkerArray with corridor + obstacles
                                   + ref-slice + status text (ego-anchored).

        `fields` mirrors what `_debug_log` receives from `_plan_loop` tier
        handlers (tier, status, trajectory, ref_slice, obs_arr, obs_tag,
        ipopt_status, iter_count, solve_ms, slack_max, ego_s, ego_n, ...).
        Missing entries are handled gracefully so fallback tiers with partial
        info (tier 1/3 may have trajectory=None) still publish a useful tick.
        """
        import json as _json

        self._tick_counter += 1
        tick = int(self._tick_counter)
        t_now = rospy.Time.now().to_sec()

        tier = int(fields.get('tier', -1))
        status = str(fields.get('status', '-'))
        ipopt_status = str(fields.get('ipopt_status', '-'))
        iter_count = int(fields.get('iter_count', -1))
        solve_ms = float(fields.get('solve_ms', 0.0))
        slack_max = float(fields.get('slack_max', 0.0))
        warm_used = int(fields.get('warm_used', 0))
        obs_tag = str(fields.get('obs_tag', '-'))
        ego_s = float(fields.get('ego_s', float('nan')))
        ego_n = float(fields.get('ego_n', float('nan')))
        speed0 = fields.get('speed0', None)
        steer0 = fields.get('steer0', None)

        trajectory = fields.get('trajectory', None)
        ref_slice = fields.get('ref_slice', None)
        obs_arr = fields.get('obs_arr', None)

        # ---- trajectory stats (frenet-lateral margins, curvature RMS) ------
        traj_stats = {
            'len': 0, 'n_min': None, 'n_max': None, 'n_end': None,
            'margin_L_min': None, 'margin_R_min': None,
            'kappa_rms': None, 'kappa_max': None,
        }
        jitter_rms = None
        n_traj_signed = None
        if trajectory is not None and ref_slice is not None:
            try:
                rc = ref_slice['center_points']
                rdx = ref_slice['ref_dx']; rdy = ref_slice['ref_dy']
                dL = ref_slice['d_left_arr']; dR = ref_slice['d_right_arr']
                M = int(min(trajectory.shape[0], rc.shape[0]))
                # Left normal in xy (same convention as the inspect scripts).
                lnx = -rdy[:M]; lny = rdx[:M]
                dx_t = trajectory[:M, 0] - rc[:M, 0]
                dy_t = trajectory[:M, 1] - rc[:M, 1]
                n_signed = dx_t * lnx + dy_t * lny
                n_traj_signed = n_signed
                inflation = float(self.solver.inflation)
                marL = (dL[:M] - inflation) - n_signed
                marR = n_signed + (dR[:M] - inflation)
                # Curvature RMS (discrete κ from (x,y) via 2nd diff / ds).
                kappa_rms = None; kappa_max = None
                if M >= 3:
                    dx = np.diff(trajectory[:M, 0])
                    dy = np.diff(trajectory[:M, 1])
                    ds = np.hypot(dx, dy)
                    ds = np.where(ds < 1e-6, 1e-6, ds)
                    psi = np.arctan2(dy, dx)
                    dpsi = np.diff(np.unwrap(psi))
                    kappa = dpsi / ds[:-1]
                    kappa_rms = float(np.sqrt(np.mean(kappa ** 2)))
                    kappa_max = float(np.max(np.abs(kappa)))
                traj_stats = {
                    'len': M,
                    'n_min': float(np.min(n_signed)),
                    'n_max': float(np.max(n_signed)),
                    'n_end': float(n_signed[-1]),
                    'margin_L_min': float(np.min(marL)),
                    'margin_R_min': float(np.min(marR)),
                    'kappa_rms': kappa_rms,
                    'kappa_max': kappa_max,
                }
            except Exception:
                pass

        # Jitter: match against previous trajectory by index (horizon stays
        # the same N+1 and is re-sliced at same ego pace — good enough for
        # smoothness monitoring).
        if trajectory is not None:
            try:
                cur_xy = np.array(trajectory[:, :2], dtype=float, copy=True)
                if (self._prev_traj_xy is not None and
                        self._prev_traj_xy.shape == cur_xy.shape):
                    d = cur_xy - self._prev_traj_xy
                    jitter_rms = float(np.sqrt(np.mean(d[:, 0] ** 2
                                                     + d[:, 1] ** 2)))
                self._prev_traj_xy = cur_xy
                self._prev_traj_ego_s = ego_s
            except Exception:
                pass

        # ---- ref slice stats -----------------------------------------------
        ref_stats = {
            's_start': None, 's_end': None,
            'v_mean': None, 'corridor_width_mean': None,
        }
        if ref_slice is not None:
            try:
                rs = ref_slice['ref_s']
                rv = ref_slice['ref_v']
                dL = ref_slice['d_left_arr']; dR = ref_slice['d_right_arr']
                ref_stats = {
                    's_start': float(rs[0]),
                    's_end': float(rs[-1]),
                    'v_mean': float(np.mean(rv)),
                    'corridor_width_mean': float(np.mean(dL + dR)),
                }
            except Exception:
                pass

        # ---- obstacles: extract slot[0] (closest tick) per active slot -----
        obs_list = []
        if obs_arr is not None:
            try:
                for o in range(obs_arr.shape[0]):
                    w_ts = obs_arr[o, :, 2]
                    if float(np.max(w_ts)) <= 0.0:
                        continue
                    obs_list.append({
                        's0': float(obs_arr[o, 0, 0]),
                        'n0': float(obs_arr[o, 0, 1]),
                        'sN': float(obs_arr[o, -1, 0]),
                        'nN': float(obs_arr[o, -1, 1]),
                        'w': float(np.max(w_ts)),
                    })
            except Exception:
                pass

        # ### HJ : v3b — solver diagnostic snapshot. Populated on every pass
        # inside FrenetKinMPC.solve(); on failure the LAST pass's input is
        # retained so we can see what IPOPT was choking on.
        solver_input = getattr(self.solver, 'last_input', {}) or {}
        solver_infeas = getattr(self.solver, 'last_infeas_info', {}) or {}
        solver_pass = int(getattr(self.solver, 'last_pass', 0))
        pass_hist = getattr(self.solver, 'last_pass_history', []) or []

        # ### HJ : v3b — prediction freshness. Callbacks for /opponent_prediction
        # and /tracking/obstacles live on separate rospy subscriber threads,
        # so they keep updating even while _plan_loop wrestles with infeasible
        # NLPs. Publishing the age here lets the user verify that (e.g. while
        # the solver was dead for 0.87s, did prediction keep flowing?).
        _now = rospy.Time.now()
        def _age(t):
            if t is None:
                return None
            try:
                return float((_now - t).to_sec())
            except Exception:
                return None
        predict_age_s = _age(getattr(self, '_obs_predict_t', None))
        track_age_s = _age(getattr(self, '_obs_track_t', None))
        opp_age_s = _age(getattr(self, '_opp_wpnts_t', None))

        payload = {
            'tick': tick,
            't': t_now,
            'state': self.state,
            'tier': tier,
            'status': status,
            'ipopt_status': ipopt_status,
            'iter': iter_count,
            'solve_ms': round(solve_ms, 3),
            'slack_max': round(slack_max, 4),
            'warm_used': warm_used,
            'obs_tag': obs_tag,
            'fail_streak': int(getattr(self, '_fail_streak', 0)),
            # Solver-level diagnostics (new in v3b).
            'solver_pass': solver_pass,
            'solver_pass_hist': pass_hist,
            'solver_input': solver_input,
            'solver_infeas': solver_infeas,
            # Prediction freshness (independent-thread callbacks — these
            # should keep ticking even when the solver is stuck at infeasibility).
            'predict_age_s': (round(predict_age_s, 3)
                              if predict_age_s is not None else None),
            'track_age_s': (round(track_age_s, 3)
                            if track_age_s is not None else None),
            'opp_age_s': (round(opp_age_s, 3)
                          if opp_age_s is not None else None),
            'ego': {
                's': round(ego_s, 4) if ego_s == ego_s else None,
                'n': round(ego_n, 4) if ego_n == ego_n else None,
                'v': round(float(getattr(self, 'car_vx', 0.0)), 3),
                'psi': round(float(getattr(self, 'car_yaw', 0.0)), 4),
                'x': round(float(getattr(self, 'car_x', 0.0)), 3),
                'y': round(float(getattr(self, 'car_y', 0.0)), 3),
            },
            'u0': {
                'v': round(float(speed0), 3) if speed0 is not None else None,
                'steer': round(float(steer0), 4) if steer0 is not None else None,
            },
            'ref': {k: (round(v, 4) if v is not None else None)
                    for k, v in ref_stats.items()},
            'trajectory': {k: (round(v, 4) if isinstance(v, float) else v)
                           for k, v in traj_stats.items()},
            'jitter_rms_m': (round(jitter_rms, 4)
                             if jitter_rms is not None else None),
            'side': getattr(self, '_last_side_str', 'n/a'),
            'side_scores': {k: (round(v, 3) if isinstance(v, float) else v)
                            for k, v in (getattr(self, '_last_side_scores', {}) or {}).items()},
            'bias_scale': round(float(getattr(self, '_last_bias_scale', 0.0)), 3),
            'ticks_since_flip': int(getattr(self, '_ticks_since_flip', 0)),
            # ### HJ : v3 — TRAIL entry dwell counter (0 when not trailing)
            'trail_ticks': int(getattr(self, '_trail_ticks_since_enter', 0)),
            'n_obs_raw': int(getattr(self, '_last_n_obs_raw', 0)),
            'n_obs_used': len(obs_list),
            'obstacles': obs_list,
            'cost': {k: round(float(v), 3)
                     for k, v in (getattr(self.solver, 'last_cost_breakdown', {}) or {}).items()},
            'weights': self._current_weights_snapshot(),
        }

        try:
            self.pub_debug_tick.publish(String(data=_json.dumps(payload)))
        except Exception as e:  # pragma: no cover
            rospy.logwarn_throttle(
                5.0, '[mpc][%s] tick_json publish failed: %s',
                rospy.get_name(), e)

        # Markers
        try:
            self._publish_debug_extra_markers(
                trajectory=trajectory, ref_slice=ref_slice,
                obs_arr=obs_arr, tier=tier, status=status,
                solve_ms=solve_ms, obs_tag=obs_tag,
                n_traj_signed=n_traj_signed,
            )
        except Exception as e:  # pragma: no cover
            rospy.logwarn_throttle(
                5.0, '[mpc][%s] debug marker publish failed: %s',
                rospy.get_name(), e)

    def _publish_debug_extra_markers(self, trajectory, ref_slice, obs_arr,
                                     tier, status, solve_ms, obs_tag,
                                     n_traj_signed=None):
        """### HJ : Context markers for RViz: corridor walls (L/R), ref-slice
        centerline, obstacle blobs (per slot, propagated over horizon), and a
        tier/status TEXT_VIEW_FACING anchored at ego pose + ~0.8m up.
        All markers live in the ~debug/markers namespace with lifetime=0.2s
        so stale ones fade if the node dies."""
        if ref_slice is None and obs_arr is None and trajectory is None:
            return

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        life = rospy.Duration(0.25)

        arr = MarkerArray()

        # Always drop a DELETEALL first — keeps obstacle slot count correct
        # when a tracked obstacle disappears between ticks.
        clear = Marker()
        clear.header = header
        clear.ns = 'mpc_debug_clear'
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        # ---- corridor walls -------------------------------------------------
        if ref_slice is not None:
            try:
                lp = ref_slice['left_points']
                rp = ref_slice['right_points']
                cp = ref_slice['center_points']
                # Reuse z from lifter for the midpoint; walls sit at same z.
                zs = [self.lifter._interp(
                    ref_slice['ref_s'][i] % max(self.track_length, 1e-3),
                    self.lifter.g_z)
                      for i in range(lp.shape[0])]

                def _line(ns_, mid, pts_xy, rgba, width=0.04):
                    m = Marker()
                    m.header = header
                    m.ns = ns_
                    m.id = mid
                    m.type = Marker.LINE_STRIP
                    m.action = Marker.ADD
                    m.scale.x = width
                    m.color = ColorRGBA(*rgba)
                    m.pose.orientation.w = 1.0
                    m.lifetime = life
                    for i in range(pts_xy.shape[0]):
                        p = Point()
                        p.x = float(pts_xy[i, 0])
                        p.y = float(pts_xy[i, 1])
                        p.z = float(zs[i])
                        m.points.append(p)
                    return m
                arr.markers.append(_line(
                    'mpc_corridor_left', 10, lp, (1.0, 0.0, 0.0, 0.7), 0.03))
                arr.markers.append(_line(
                    'mpc_corridor_right', 11, rp, (1.0, 0.0, 0.0, 0.7), 0.03))
                arr.markers.append(_line(
                    'mpc_ref_center', 12, cp, (0.5, 0.5, 0.5, 0.5), 0.02))
            except Exception:
                pass

        # ---- obstacle blobs (horizon-propagated) ---------------------------
        if obs_arr is not None and ref_slice is not None:
            try:
                rc = ref_slice['center_points']
                rdx = ref_slice['ref_dx']; rdy = ref_slice['ref_dy']
                rs_ref = ref_slice['ref_s']
                M = int(min(obs_arr.shape[1], rc.shape[0]))
                lnx = -rdy[:M]; lny = rdx[:M]
                mid = 100
                for o in range(obs_arr.shape[0]):
                    w_ts = obs_arr[o, :, 2]
                    if float(np.max(w_ts)) <= 0.0:
                        continue
                    # Current (k=0) + propagated (k=1..N) positions.
                    pts = Marker()
                    pts.header = header
                    pts.ns = 'mpc_obs_slot_%d' % o
                    pts.id = mid; mid += 1
                    pts.type = Marker.SPHERE_LIST
                    pts.action = Marker.ADD
                    pts.scale.x = pts.scale.y = pts.scale.z = 0.18
                    pts.color = ColorRGBA(r=1.0, g=0.2, b=1.0, a=0.85)
                    pts.pose.orientation.w = 1.0
                    pts.lifetime = life
                    # Highlight sphere at k=0 (opaque, larger).
                    head_s = float(obs_arr[o, 0, 0])
                    head_n = float(obs_arr[o, 0, 1])
                    # Map (s,n) to xy using nearest ref index.
                    for k in range(M):
                        s_o = float(obs_arr[o, k, 0])
                        n_o = float(obs_arr[o, k, 1])
                        # nearest reference-slice index by ref_s
                        kref = int(np.argmin(np.abs(rs_ref[:M] - s_o)))
                        x = rc[kref, 0] + n_o * (-rdy[kref])
                        y = rc[kref, 1] + n_o * (rdx[kref])
                        p = Point(); p.x = x; p.y = y
                        p.z = self.lifter._interp(
                            rs_ref[kref] % max(self.track_length, 1e-3),
                            self.lifter.g_z)
                        pts.points.append(p)
                    arr.markers.append(pts)

                    # Big head ball (k=0) for at-a-glance location.
                    head = Marker()
                    head.header = header
                    head.ns = 'mpc_obs_head_%d' % o
                    head.id = mid; mid += 1
                    head.type = Marker.SPHERE
                    head.action = Marker.ADD
                    head.scale.x = head.scale.y = head.scale.z = 0.35
                    head.color = ColorRGBA(r=1.0, g=0.0, b=0.8, a=0.95)
                    kref0 = int(np.argmin(np.abs(rs_ref[:M] - head_s)))
                    head.pose.position.x = float(
                        rc[kref0, 0] + head_n * (-rdy[kref0]))
                    head.pose.position.y = float(
                        rc[kref0, 1] + head_n * (rdx[kref0]))
                    head.pose.position.z = float(self.lifter._interp(
                        rs_ref[kref0] % max(self.track_length, 1e-3),
                        self.lifter.g_z))
                    head.pose.orientation.w = 1.0
                    head.lifetime = life
                    arr.markers.append(head)
            except Exception:
                pass

        # ---- tier / status text floating above ego -------------------------
        try:
            txt = Marker()
            txt.header = header
            txt.ns = 'mpc_tier_status'
            txt.id = 200
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.scale.z = 0.25
            # Color by tier — green=0, yellow=1, orange=2, red=3.
            tier_color = {
                0: (0.1, 0.9, 0.2, 1.0),
                1: (1.0, 0.9, 0.0, 1.0),
                2: (1.0, 0.5, 0.0, 1.0),
                3: (1.0, 0.1, 0.1, 1.0),
            }.get(int(tier), (0.7, 0.7, 0.7, 1.0))
            txt.color = ColorRGBA(*tier_color)
            txt.pose.position.x = float(getattr(self, 'car_x', 0.0))
            txt.pose.position.y = float(getattr(self, 'car_y', 0.0))
            txt.pose.position.z = float(getattr(self, 'car_z', 0.0)) + 0.8
            txt.pose.orientation.w = 1.0
            txt.lifetime = life
            txt.text = ('[%s] tier=%d %s | solve=%.1fms | obs=%s'
                        % (self.state, int(tier), status, float(solve_ms),
                           obs_tag))
            arr.markers.append(txt)
        except Exception:
            pass

        if arr.markers:
            self.pub_debug_markers.publish(arr)


if __name__ == '__main__':
    try:
        _node = MPCPlannerStateNode()
        rospy.on_shutdown(lambda: _node._debug_logger and _node._debug_logger.close())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
