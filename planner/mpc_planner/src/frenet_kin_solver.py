#!/usr/bin/env python3
"""### HJ : Frenet kinematic-bicycle MPC for overtaking (3D-safe).

2026-04-21 redesign v3 — Liniger-style with C^1 curvature + solution continuity.

v3 changes vs v2c (why v2c failed):
  v2c put steering delta directly as an input and penalised only
  (Δδ)^2. That made δ piece-wise constant between knots → κ discontinuous
  at every knot AND tick-to-tick jumps were unbounded in the n-profile.
  User feedback: "직선으로 꺾이는 궤적", "번개 치듯 discrete 하게 바뀜".

  v3 moves δ into the STATE. The control is δ̇ (steering rate). This:
    (a) Makes δ a continuous function of k → κ = tan(δ)/L is C^1 → no
        kinked trajectory at knot boundaries.
    (b) Allows a "jerk-like" penalty r_dd_rate × Σ(Δδ̇)^2 → curvature
        rate is smooth.
    (c) Lets us warm-start δ from the previous solve's δ[1] (hard
        constraint on δ[0]) so the physical steer angle is carried
        across ticks → tick-to-tick action smoothness.

  Additional v3 cost terms:
    + w_cont × Σ (n_k - n_prev_shifted_k)^2   (solution continuity)
    + q_n_term × n_N^2 + q_v_term × (v_N - v_ref_N)^2   (terminal)
    Stage ramp on contour/lag REMOVED (it was making cost k-stepwise
    which drove the "벽에 붙다가 끝만 복귀" shape).

  Wall cushion weight raised (150 → 2500) so it wins against obstacle
  bubble without bound glue — obstacle is strong (180) but wall is
  stronger closer to the edge (quadratic 2500×(0.30)² = 225 per step).

State   x = [n, μ, v, δ]      (frenet lateral, heading vs tangent, speed, steer)
Control u = [a, δ̇]            (accel, steer rate)
Dynamics (discrete, dT = 0.05s, explicit-Euler of the frenet kinematic bicycle):
    n_{k+1}  = n_k  + v_k * sin(μ_k)                           * dT
    μ_{k+1}  = μ_k  + (v_k / L * tan(δ_k)
                       - κ_ref_k * v_k * cos(μ_k))             * dT
    v_{k+1}  = v_k  + a_k                                      * dT
    δ_{k+1}  = δ_k  + δ̇_k                                      * dT

Cost (all C^1+):
  stationary running:
    q_n       * Σ n_k^2                                   (contour, no ramp)
    r_reg     * Σ δ_k^2                                   (steer zero-bias)
    r_dd      * Σ δ̇_k^2                                   (steer-rate, κ̇ analogue)
    r_dd_rate * Σ (δ̇_{k+1} - δ̇_k)^2                       (steer-rate-rate, jerk-like)
    r_a       * Σ (a_{k+1} - a_k)^2                       (accel smoothness)
    -gamma    * Σ v_k*cos(μ_k)*dT                         (progress pull)
    w_obs     * Σ_o Σ_k prox_sk * exp(-(dn/σ_n)^2)        (obstacle bubble)
    w_bias    * Σ_o Σ_k prox_sk * hinge^2                 (side bias, one-sided)
    w_wall_buf* Σ_k (hinge_up^2 + hinge_dn^2)             (soft wall cushion)
    w_cont    * Σ_k (n_k - n_prev_k)^2 * cont_active      (tick-to-tick continuity)
    w_slack   * Σ_k slk_k^2                               (corridor slack)
  terminal:
    q_n_term  * n_N^2
    q_v_term  * (v_N - v_ref_N)^2

Hard constraints:
    n_k ∈ [n_lb_k - slk_k, n_ub_k + slk_k]       (wall corridor, slackable)
    slk_k ≥ 0
    v_k ∈ [v_min, v_max_k]                        (v_max_k lowers in TRAIL)
    a_k ∈ [a_min, a_max]
    δ_k ∈ [-δ_max, δ_max]
    δ̇_k ∈ [-δ̇_max, δ̇_max]
    μ_k ∈ [-μ_max, μ_max]
    δ[0] == δ_prev1   (carry prev solution's δ[1] → no knot jump at boundary)
    n[0] == n0, μ[0] == μ0, v[0] == v0

TRAIL behaviour:
    v_max_k capped by obstacle's s-speed × 0.95 → solver naturally holds
    station behind obstacle. Combined with w_cont + terminal cost pulling
    n → 0 (raceline), this produces a smooth "swing back to centerline +
    decelerate" morph over ~5 ticks.

Outputs:
    traj  = np.ndarray (N+1, 4) [s, n, μ, v]
    speed0 = float (v[0] + a[0]*dT) for the controller
    steer0 = float δ[0]
    success = bool
"""

import numpy as np
import casadi as ca


SIDE_CLEAR = 0
SIDE_LEFT = 1
SIDE_RIGHT = 2
SIDE_TRAIL = 3


class FrenetKinMPC:
    FAR = 1e3  # sentinel station for inactive obstacle slots

    def __init__(self, **params):
        # horizon
        self.N = int(params.get('N', 20))
        self.dT = float(params.get('dT', 0.05))
        self.L = float(params.get('vehicle_L', 0.33))

        # cost weights (running cost)
        self.q_n = float(params.get('q_n', 3.0))
        self.gamma = float(params.get('gamma_progress', 10.0))
        self.r_a = float(params.get('r_a', 0.5))
        self.r_reg = float(params.get('r_steer_reg', 0.1))
        # v3: δ̇ (rate) and δ̈ (rate-of-rate / jerk-like) penalties
        self.r_dd = float(params.get('r_dd', 5.0))
        self.r_dd_rate = float(params.get('r_dd_rate', 1.0))
        # v3: terminal stability cost
        self.q_n_term = float(params.get('q_n_term', 10.0))
        self.q_v_term = float(params.get('q_v_term', 0.5))
        # v3: tick-to-tick solution continuity
        self.w_cont = float(params.get('w_cont', 20.0))

        self.w_slack = float(params.get('w_slack', 5000.0))

        # obstacle bubble
        self.w_obs = float(params.get('w_obs', 180.0))
        self.sigma_s_obs = float(params.get('sigma_s_obs', 0.7))
        self.sigma_n_obs = float(params.get('sigma_n_obs', 0.18))

        # side bias
        self.w_side_bias = float(params.get('w_side_bias', 25.0))

        # wall cushion (quadratic hinge). v3 strengthens x6+
        self.w_wall_buf = float(params.get('w_wall_buf', 2500.0))
        self.wall_buf = float(params.get('wall_buf', 0.30))

        # limits
        self.v_min = float(params.get('v_min', 0.5))
        self.v_max = float(params.get('v_max', 8.0))
        self.a_min = float(params.get('a_min', -4.0))
        self.a_max = float(params.get('a_max', 3.0))
        self.delta_max = float(params.get('delta_max', 0.6))
        # v3: steer-rate limit (rad/s). Default 3.0 rad/s is conservative
        # for a 1:10 R/C servo; reasoned with Liniger's 2015 setup.
        self.delta_rate_max = float(params.get('delta_rate_max', 3.0))
        self.mu_max = float(params.get('mu_max', 0.9))

        # geometry / safety
        self.inflation = float(params.get('inflation', 0.05))
        self.wall_safe = float(params.get('wall_safe', 0.10))
        self.gap_lat = float(params.get('gap_lat', 0.25))
        self.gap_long = float(params.get('gap_long', 0.8))
        # ### HJ : v3b — vehicle half-width. Previously only the SideDecider
        # knew about ego body size; the solver built the hard corridor and
        # the wall cushion from the *centroid* alone, so "waypoint 15 cm
        # off the wall" meant body-edge = 0 cm (scraping). Now the solver
        # inflates both bounds by ego_half → centroid ≥ ego_half+wall_safe
        # +inflation from each wall → body edge ≥ wall_safe+inflation.
        self.ego_half = float(params.get('ego_half_width', 0.15))

        # ipopt
        self.ipopt_max_iter = int(params.get('ipopt_max_iter', 1000))
        self.ipopt_print_level = int(params.get('ipopt_print_level', 0))

        # runtime state
        self.n_obs_max = int(params.get('n_obs_max', 2))
        self.ready = False
        self.warm = False
        self._warm_X = None  # (N+1, 4) [n, μ, v, δ]
        self._warm_U = None  # (N, 2)   [a, δ̇]
        # prev solution for continuity cost parameter
        self._prev_n_profile = None   # (N+1,) n-profile from last successful solve
        # prev δ[1] for hard initial-δ constraint
        self._prev_de1 = 0.0
        self._have_prev = False

        # last-solve metadata
        self.last_u_sol = None
        self.last_return_status = '-'
        self.last_iter_count = -1
        self.last_slack_max = 0.0
        self.last_cost_breakdown = {}

        self._opti = None
        self._vars = None
        self._pars = None

    # ------------------------------------------------------------------ setup
    def setup(self):
        N = self.N
        dT = self.dT
        L = self.L
        n_obs = self.n_obs_max

        opti = ca.Opti()

        # decision vars
        n_ = opti.variable(N + 1)
        mu = opti.variable(N + 1)
        v_ = opti.variable(N + 1)
        de = opti.variable(N + 1)   # δ is now STATE (v3)
        a_ = opti.variable(N)
        dd = opti.variable(N)        # δ̇ is now CONTROL (v3)
        slk = opti.variable(N + 1)

        # parameters
        P_n0 = opti.parameter()
        P_mu0 = opti.parameter()
        P_v0 = opti.parameter()
        P_de0 = opti.parameter()     # v3: prev-solve δ[1] → hard initial δ
        P_de0_active = opti.parameter()  # 0 on first solve, 1 afterward
        P_kappa = opti.parameter(N + 1)
        P_vmax = opti.parameter(N + 1)
        P_nlb = opti.parameter(N + 1)
        P_nub = opti.parameter(N + 1)
        P_dL = opti.parameter(N + 1)
        P_dR = opti.parameter(N + 1)
        P_ref_s = opti.parameter(N + 1)
        P_ref_v = opti.parameter(N + 1)   # v3: used by terminal cost
        P_obs_s = opti.parameter(n_obs, N + 1)
        P_obs_n = opti.parameter(n_obs, N + 1)
        P_obs_active = opti.parameter(n_obs)
        P_bias_L = opti.parameter()
        P_bias_R = opti.parameter()
        # v3: previous-solution n-profile for tick-to-tick continuity cost
        P_n_prev = opti.parameter(N + 1)
        P_cont_active = opti.parameter()

        # ---- dynamics ----
        for k in range(N):
            opti.subject_to(n_[k + 1] == n_[k]
                            + v_[k] * ca.sin(mu[k]) * dT)
            opti.subject_to(mu[k + 1] == mu[k]
                            + (v_[k] / L * ca.tan(de[k])
                               - P_kappa[k] * v_[k] * ca.cos(mu[k])) * dT)
            opti.subject_to(v_[k + 1] == v_[k] + a_[k] * dT)
            opti.subject_to(de[k + 1] == de[k] + dd[k] * dT)

        # ---- initial conditions ----
        opti.subject_to(n_[0] == P_n0)
        opti.subject_to(mu[0] == P_mu0)
        opti.subject_to(v_[0] == P_v0)
        # v3: δ[0] soft-hard constrained to prev δ[1] (hard when active=1).
        # "soft-hard" = we multiply the residual by P_de0_active so the first
        # solve (active=0) leaves δ[0] free and later solves lock it in.
        opti.subject_to(P_de0_active * (de[0] - P_de0) == 0)

        # ---- input / state bounds ----
        opti.subject_to(opti.bounded(self.a_min, a_, self.a_max))
        opti.subject_to(opti.bounded(-self.delta_rate_max,
                                     dd, self.delta_rate_max))
        opti.subject_to(opti.bounded(-self.delta_max, de, self.delta_max))
        opti.subject_to(opti.bounded(-self.mu_max, mu, self.mu_max))
        for k in range(N + 1):
            opti.subject_to(v_[k] >= self.v_min)
            opti.subject_to(v_[k] <= P_vmax[k])
            opti.subject_to(n_[k] >= P_nlb[k] - slk[k])
            opti.subject_to(n_[k] <= P_nub[k] + slk[k])
            opti.subject_to(slk[k] >= 0.0)

        # ---- cost ----
        J_contour = 0
        for k in range(N + 1):
            J_contour = J_contour + self.q_n * n_[k] ** 2

        J_reg = 0
        for k in range(N + 1):
            J_reg = J_reg + self.r_reg * de[k] ** 2

        # v3: δ̇ (steer-rate) penalty — directly penalises curvature rate
        # since κ = tan(δ)/L and δ̇ changes κ̇. This is the C^1 guarantee
        # replacement for the v2 (Δδ)^2 trick.
        J_dd = 0
        for k in range(N):
            J_dd = J_dd + self.r_dd * dd[k] ** 2

        # v3: δ̈ (steer-rate-rate) penalty — jerk analogue. Keeps κ̇
        # continuous → no kinked knots even under disturbance.
        J_dd_rate = 0
        for k in range(N - 1):
            J_dd_rate = J_dd_rate + self.r_dd_rate * (dd[k + 1] - dd[k]) ** 2

        J_smooth_a = 0
        for k in range(N - 1):
            J_smooth_a = J_smooth_a + self.r_a * (a_[k + 1] - a_[k]) ** 2

        prog = 0
        for k in range(N):
            prog = prog + v_[k] * ca.cos(mu[k]) * dT
        J_progress = -self.gamma * prog

        # Obstacle bubble + side bias (same structure as v2; weights tuned)
        J_obs = 0
        J_bias = 0
        for o in range(n_obs):
            for k in range(N + 1):
                dx = (P_ref_s[k] - P_obs_s[o, k]) / self.sigma_s_obs
                dy = (n_[k] - P_obs_n[o, k]) / self.sigma_n_obs
                prox_sk = P_obs_active[o] * ca.exp(-(dx * dx))
                J_obs = J_obs + (self.w_obs * prox_sk
                                 * ca.exp(-(dy * dy)))
                viol_L = ca.fmax(0.0,
                                 (P_obs_n[o, k] + self.gap_lat) - n_[k])
                viol_R = ca.fmax(0.0,
                                 n_[k] - (P_obs_n[o, k] - self.gap_lat))
                J_bias = J_bias + P_bias_L * prox_sk * viol_L ** 2
                J_bias = J_bias + P_bias_R * prox_sk * viol_R ** 2

        # Wall cushion (strong quadratic hinge inside wall_buf).
        # ### HJ : v3b — cushion fires when the CAR BODY gets within
        # wall_buf of the wall, i.e. centroid within (ego_half + wall_buf).
        # Previously centroid was used directly, so a 30 cm centroid-buf
        # already meant body was 15 cm from the wall before cushion engaged.
        J_wall = 0
        buf_c = self.ego_half + self.wall_buf
        for k in range(N + 1):
            viol_up = ca.fmax(0.0, n_[k] - (P_dL[k] - buf_c))
            viol_dn = ca.fmax(0.0, -n_[k] - (P_dR[k] - buf_c))
            J_wall = J_wall + self.w_wall_buf * (viol_up ** 2 + viol_dn ** 2)

        # v3: solution continuity — pulls new n[k] toward prev solution's
        # shifted n[k+1]. w_cont * Σ (n - n_prev)^2 with per-tick gate.
        J_cont = 0
        for k in range(N + 1):
            J_cont = J_cont + self.w_cont * P_cont_active \
                            * (n_[k] - P_n_prev[k]) ** 2

        # v3: terminal cost — raceline return + speed-match near horizon end.
        J_term = (self.q_n_term * n_[N] ** 2
                  + self.q_v_term * (v_[N] - P_ref_v[N]) ** 2)

        J_slack = 0
        for k in range(N + 1):
            J_slack = J_slack + self.w_slack * slk[k] ** 2

        J = (J_contour + J_reg + J_dd + J_dd_rate + J_smooth_a
             + J_progress + J_obs + J_bias + J_wall
             + J_cont + J_term + J_slack)

        opti.minimize(J)
        opti.solver('ipopt', {
            'ipopt.max_iter':    self.ipopt_max_iter,
            'ipopt.print_level': self.ipopt_print_level,
            'print_time':        0,
            'ipopt.sb':          'yes',
        })

        self._opti = opti
        self._vars = dict(n=n_, mu=mu, v=v_, de=de,
                          a=a_, dd=dd, slk=slk)
        self._pars = dict(n0=P_n0, mu0=P_mu0, v0=P_v0,
                          de0=P_de0, de0_active=P_de0_active,
                          kappa=P_kappa, vmax=P_vmax,
                          nlb=P_nlb, nub=P_nub,
                          dL=P_dL, dR=P_dR,
                          ref_s=P_ref_s, ref_v=P_ref_v,
                          obs_s=P_obs_s, obs_n=P_obs_n,
                          obs_active=P_obs_active,
                          bias_L=P_bias_L, bias_R=P_bias_R,
                          n_prev=P_n_prev, cont_active=P_cont_active)
        self._cost_exprs = dict(
            contour=J_contour, reg=J_reg, dd=J_dd, dd_rate=J_dd_rate,
            smooth_a=J_smooth_a, progress=J_progress,
            obs=J_obs, bias=J_bias, wall=J_wall,
            cont=J_cont, term=J_term, slack=J_slack)
        self.ready = True

    # ----------------------------------------------------------------- helpers
    def reset_warm_start(self):
        self._warm_X = None
        self._warm_U = None
        self._prev_n_profile = None
        self._have_prev = False
        self._prev_de1 = 0.0
        self.warm = False

    def _build_wall_bounds(self, d_left_arr, d_right_arr):
        dL = np.asarray(d_left_arr, dtype=float)
        dR = np.asarray(d_right_arr, dtype=float)
        # ### HJ : v3b — include ego_half so CENTROID bound guarantees
        # body-edge ≥ wall_safe + inflation from the wall.
        margin = self.ego_half + self.inflation + self.wall_safe
        nub = np.maximum(dL - margin, 1e-3)
        nlb = -np.maximum(dR - margin, 1e-3)
        return nlb, nub

    def _build_vmax(self, ref_v, obs_arr, side):
        """TRAIL caps vmax so ego falls behind the lead obstacle."""
        vmax = np.minimum(np.asarray(ref_v, dtype=float), self.v_max)
        if side == SIDE_TRAIL and obs_arr is not None:
            cap = self.v_max
            for o in range(obs_arr.shape[0]):
                if float(np.max(obs_arr[o, :, 2])) <= 0.0:
                    continue
                s0 = float(obs_arr[o, 0, 0])
                sN = float(obs_arr[o, -1, 0])
                v_obs_s = max((sN - s0)
                              / max(self.N * self.dT, 1e-3), 0.0)
                cap = min(cap, max(v_obs_s * 0.95, self.v_min))
            vmax = np.minimum(vmax, cap)
        vmax = np.maximum(vmax, self.v_min + 0.1)
        return vmax

    def _build_obs_params(self, obs_arr):
        N1 = self.N + 1
        s_mat = np.full((self.n_obs_max, N1), self.FAR, dtype=float)
        n_mat = np.full((self.n_obs_max, N1), self.FAR, dtype=float)
        active = np.zeros(self.n_obs_max, dtype=float)
        if obs_arr is None:
            return s_mat, n_mat, active
        for o in range(min(obs_arr.shape[0], self.n_obs_max)):
            w_ts = obs_arr[o, :, 2]
            if float(np.max(w_ts)) <= 0.0:
                continue
            s_mat[o, :] = obs_arr[o, :, 0]
            n_mat[o, :] = obs_arr[o, :, 1]
            active[o] = 1.0
        return s_mat, n_mat, active

    def _seed_warm_start(self, n0, mu0, v0, de0, nlb, nub, vmax):
        N = self.N
        X = np.zeros((N + 1, 4))
        U = np.zeros((N, 2))
        for k in range(N + 1):
            X[k, 0] = float(np.clip(n0, nlb[k], nub[k]))
            X[k, 1] = mu0 * (1.0 - k / max(N, 1))
            X[k, 2] = float(np.clip(v0, self.v_min + 0.1, vmax[k]))
            X[k, 3] = de0
        return X, U

    # ------------------------------------------------------------------- solve
    def solve(self, initial_state, ref_slice,
              obstacles=None, side=SIDE_CLEAR, bias_scale=1.0):
        """
        initial_state: np.ndarray shape (3,) [n0, mu0, v0]
                       (δ0 carried internally from prev solve)
        ref_slice:    dict with 'kappa_ref'(N+1), 'd_left_arr'(N+1),
                      'd_right_arr'(N+1), 'ref_v'(N+1), 'ref_s'(N+1)
        obstacles:    (n_slot, N+1, 3) [s_o, n_o, w]
        side:         SIDE_{CLEAR,LEFT,RIGHT,TRAIL}
        bias_scale:   kept for API compatibility with node (v3 uses 1.0
                      unconditionally — continuity cost handles smoothness).
        """
        if not self.ready:
            raise RuntimeError('FrenetKinMPC.setup() must be called first')

        n0, mu0, v0 = (float(initial_state[0]),
                       float(initial_state[1]),
                       float(initial_state[2]))
        v0 = min(max(v0, self.v_min + 1e-3), self.v_max)

        kappa = np.asarray(ref_slice['kappa_ref'], dtype=float)
        dL = np.asarray(ref_slice['d_left_arr'], dtype=float)
        dR = np.asarray(ref_slice['d_right_arr'], dtype=float)
        rv = np.asarray(ref_slice['ref_v'], dtype=float)
        rs = np.asarray(ref_slice['ref_s'], dtype=float)

        nlb, nub = self._build_wall_bounds(dL, dR)
        vmax = self._build_vmax(rv, obstacles, side)

        n0_clamped = float(np.clip(n0, nlb[0], nub[0]))

        obs_s_mat, obs_n_mat, obs_active = self._build_obs_params(obstacles)

        # v3: bias_scale kept in signature for node API compat; solver uses 1.0.
        _ = bias_scale
        w_bias = self.w_side_bias
        if side == SIDE_LEFT:
            bL, bR = w_bias, 0.0
        elif side == SIDE_RIGHT:
            bL, bR = 0.0, w_bias
        else:
            bL, bR = 0.0, 0.0

        # v3: continuity anchor = prev solution shifted one step (receding
        # horizon). Inactive on first solve. Value is built from _prev_n_profile
        # (which is the shifted warm-start's n column).
        if self._have_prev and self._prev_n_profile is not None:
            n_prev = self._prev_n_profile.copy()
            cont_active = 1.0
        else:
            n_prev = np.zeros(self.N + 1)
            cont_active = 0.0

        # v3: δ[0] anchor = prev solve's δ[1]. Inactive on first solve.
        de0_val = float(self._prev_de1)
        de0_active_val = 1.0 if self._have_prev else 0.0

        opti = self._opti
        V = self._vars
        P = self._pars

        opti.set_value(P['n0'], n0_clamped)
        opti.set_value(P['mu0'], mu0)
        opti.set_value(P['v0'], v0)
        opti.set_value(P['de0'], de0_val)
        opti.set_value(P['de0_active'], de0_active_val)
        opti.set_value(P['kappa'], kappa)
        opti.set_value(P['vmax'], vmax)
        opti.set_value(P['nlb'], nlb)
        opti.set_value(P['nub'], nub)
        opti.set_value(P['dL'], dL)
        opti.set_value(P['dR'], dR)
        opti.set_value(P['ref_s'], rs)
        opti.set_value(P['ref_v'], rv)
        opti.set_value(P['obs_s'], obs_s_mat)
        opti.set_value(P['obs_n'], obs_n_mat)
        opti.set_value(P['obs_active'], obs_active)
        opti.set_value(P['bias_L'], bL)
        opti.set_value(P['bias_R'], bR)
        opti.set_value(P['n_prev'], n_prev)
        opti.set_value(P['cont_active'], cont_active)

        # ---- warm start ----
        if (self._warm_X is not None
                and self._warm_X.shape == (self.N + 1, 4)):
            Xw, Uw = self._warm_X, self._warm_U
        else:
            Xw, Uw = self._seed_warm_start(n0_clamped, mu0, v0,
                                           de0_val, nlb, nub, vmax)
        opti.set_initial(V['n'],  Xw[:, 0])
        opti.set_initial(V['mu'], Xw[:, 1])
        opti.set_initial(V['v'],  Xw[:, 2])
        opti.set_initial(V['de'], Xw[:, 3])
        opti.set_initial(V['a'],  Uw[:, 0])
        opti.set_initial(V['dd'], Uw[:, 1])

        success = False
        try:
            sol = opti.solve()
            n_sol = np.asarray(sol.value(V['n'])).ravel()
            mu_sol = np.asarray(sol.value(V['mu'])).ravel()
            v_sol = np.asarray(sol.value(V['v'])).ravel()
            de_sol = np.asarray(sol.value(V['de'])).ravel()
            a_sol = np.asarray(sol.value(V['a'])).ravel()
            dd_sol = np.asarray(sol.value(V['dd'])).ravel()
            slk_sol = np.asarray(sol.value(V['slk'])).ravel()
            self.last_return_status = sol.stats().get('return_status', 'OK')
            self.last_iter_count = int(sol.stats().get('iter_count', -1))
            try:
                self.last_cost_breakdown = {
                    k: float(sol.value(vv))
                    for k, vv in self._cost_exprs.items()}
            except Exception:
                self.last_cost_breakdown = {}
            success = True
        except RuntimeError:
            self.last_return_status = opti.stats().get('return_status', 'FAIL')
            self.last_iter_count = int(opti.stats().get('iter_count', -1))
            try:
                n_sol = np.asarray(opti.debug.value(V['n'])).ravel()
                mu_sol = np.asarray(opti.debug.value(V['mu'])).ravel()
                v_sol = np.asarray(opti.debug.value(V['v'])).ravel()
                de_sol = np.asarray(opti.debug.value(V['de'])).ravel()
                a_sol = np.asarray(opti.debug.value(V['a'])).ravel()
                dd_sol = np.asarray(opti.debug.value(V['dd'])).ravel()
                slk_sol = np.asarray(opti.debug.value(V['slk'])).ravel()
                self.last_cost_breakdown = {}
            except Exception:
                self.last_u_sol = None
                self.last_slack_max = 0.0
                self.last_cost_breakdown = {}
                return 0.0, 0.0, None, False

        # --- shift warm-start / continuity anchors one step ---
        N = self.N
        X_new = np.column_stack([n_sol, mu_sol, v_sol, de_sol])
        U_new = np.column_stack([a_sol, dd_sol])
        # shift and pad last row with last known value
        X_shift = np.empty_like(X_new)
        X_shift[:-1] = X_new[1:]
        X_shift[-1] = X_new[-1]
        U_shift = np.empty_like(U_new)
        U_shift[:-1] = U_new[1:]
        U_shift[-1] = U_new[-1]
        self._warm_X = X_shift
        self._warm_U = U_shift
        self._prev_n_profile = X_shift[:, 0].copy()
        # prev δ[1] becomes the next-solve's δ[0] anchor
        self._prev_de1 = float(de_sol[1]) if de_sol.shape[0] > 1 else float(de_sol[0])
        self._have_prev = True
        self.warm = True

        self.last_u_sol = np.column_stack([v_sol[:N], de_sol[:N]])
        self.last_slack_max = float(np.max(np.abs(slk_sol)))

        traj = np.column_stack([rs[:N + 1], n_sol, mu_sol, v_sol])
        # speed command uses v[0] + a[0]*dT (consistent with v2)
        speed0 = float(v_sol[0] + a_sol[0] * self.dT)
        steer0 = float(de_sol[0])
        return speed0, steer0, traj, success
