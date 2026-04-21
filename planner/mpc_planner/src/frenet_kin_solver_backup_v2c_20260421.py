#!/usr/bin/env python3
"""### HJ : Frenet kinematic-bicycle MPC for overtaking (3D-safe).

2026-04-21 redesign v2 — Liniger-style soft cost for obstacle + wall.

Why v2:
  v1 treated obstacle avoidance as a hard half-plane on n_k based on an
  external SideDecider. When side flipped, the feasible set [nlb, nub]
  flipped too, which (a) made warm-start useless and (b) produced visible
  tick-to-tick jumps in the solution. "MPC가 MPC답지 않게 뻣뻣하다" feedback.

  v2 keeps corridor bounds for WALLS ONLY. Obstacles are smooth Gaussian
  repulsion in the cost, and SideDecider still emits LEFT / RIGHT / TRAIL
  but only biases a one-sided quadratic penalty. Cost is C^1, so a side
  flip does not invalidate the previous warm-start — IPOPT re-converges
  from the shifted warm-X/U in ~5–15 iters and trajectory evolves
  continuously.

State   x = [n, mu, v]       (frenet lateral offset, heading-vs-tangent, speed)
Control u = [a, delta]
Station s = ref_slice.ref_s[k]  (fixed time-based grid from the slicer; the
                                  solver never touches xy — see CLAUDE.md
                                  "3D에서 Frenet xy round-trip 금지".)

Discrete dynamics (Liniger 2015 frenet kinematic bicycle):
    n_{k+1}  = n_k  + v_k * sin(mu_k)                          * dT
    mu_{k+1} = mu_k + (v_k / L * tan(delta_k)
                       - kappa_ref_k * v_k * cos(mu_k))        * dT
    v_{k+1}  = v_k  + a_k                                      * dT

Cost (all terms C^1 continuous → warm-start stays valid across side flips):
    q_n  * sum n_k^2                                          (contour)
  + r_a  * sum (Delta a)^2                                    (accel smoothness)
  + r_d  * sum (Delta delta)^2                                (steer smoothness)
  + r_reg* sum delta_k^2                                      (steer zero-bias)
  - gamma* sum v_k*cos(mu_k)*dT                               (progress)
  + w_obs* sum_o sum_k P_w[o]*exp(-((s_k-s_o_k)/sig_s)^2
                                   -((n_k-n_o_k)/sig_n)^2)    (obstacle bubble)
  + P_biasL * sum_k max(0, n_bias + gap_lat - n_k)^2          (LEFT bias)
  + P_biasR * sum_k max(0, n_k - (n_bias - gap_lat))^2        (RIGHT bias)
  + w_wbuf* sum_k (max(0, n_k - (d_L_k - wall_buf))^2
                 + max(0, -n_k - (d_R_k - wall_buf))^2)       (wall cushion)
  + w_slk * sum slack_k^2                                     (corridor slack)

Hard constraints:
  - corridor (walls only, with wall_safe + inflation) as slackable bounds on n_k
  - v, a, delta, mu bounded
  - TRAIL: enforced as a per-k v upper cap so ego stays behind obstacle.

Outputs (node consumes):
    traj = np.ndarray (N+1, 4) [s, n, mu, v]   -- solver frame (frenet)
    u_sol= np.ndarray (N, 2)   [a, delta]
    success = bool
"""

import numpy as np
import casadi as ca


SIDE_CLEAR = 0
SIDE_LEFT = 1
SIDE_RIGHT = 2
SIDE_TRAIL = 3


class FrenetKinMPC:
    FAR = 1e3  # sentinel station for inactive obstacle slots (bubble → 0)

    def __init__(self, **params):
        # horizon
        self.N = int(params.get('N', 20))
        self.dT = float(params.get('dT', 0.05))
        self.L = float(params.get('vehicle_L', 0.33))

        # cost weights (running cost)
        self.q_n = float(params.get('q_n', 3.0))
        self.gamma = float(params.get('gamma_progress', 10.0))
        self.r_a = float(params.get('r_a', 0.5))
        self.r_delta = float(params.get('r_delta', 1.5))
        self.r_reg = float(params.get('r_steer_reg', 0.1))
        self.w_slack = float(params.get('w_slack', 2000.0))

        # v2: soft obstacle bubble — Gaussian in (s, n), σ_n must be smaller
        # than ~half the corridor so the bubble doesn't swallow the whole n.
        self.w_obs = float(params.get('w_obs', 300.0))
        self.sigma_s_obs = float(params.get('sigma_s_obs', 0.7))
        self.sigma_n_obs = float(params.get('sigma_n_obs', 0.18))

        # v2: side bias (soft one-sided penalty, activated by SideDecider).
        # Gated by obstacle proximity — only fires within sigma_s_obs of the
        # obstacle station, so the horizon far from any obstacle is free.
        self.w_side_bias = float(params.get('w_side_bias', 25.0))

        # v2: soft wall cushion (quadratic hinge just inside the hard corridor)
        self.w_wall_buf = float(params.get('w_wall_buf', 150.0))
        self.wall_buf = float(params.get('wall_buf', 0.20))

        # limits
        self.v_min = float(params.get('v_min', 0.5))
        self.v_max = float(params.get('v_max', 8.0))
        self.a_min = float(params.get('a_min', -4.0))
        self.a_max = float(params.get('a_max', 3.0))
        self.delta_max = float(params.get('delta_max', 0.6))
        self.mu_max = float(params.get('mu_max', 0.9))

        # geometry / safety (hard corridor uses inflation + wall_safe)
        self.inflation = float(params.get('inflation', 0.05))
        self.wall_safe = float(params.get('wall_safe', 0.15))
        self.gap_lat = float(params.get('gap_lat', 0.25))
        self.gap_long = float(params.get('gap_long', 0.8))

        # ipopt
        self.ipopt_max_iter = int(params.get('ipopt_max_iter', 200))
        self.ipopt_print_level = int(params.get('ipopt_print_level', 0))

        # runtime state
        self.n_obs_max = int(params.get('n_obs_max', 2))
        self.ready = False
        self.warm = False
        self._warm_X = None  # (N+1, 3) [n, mu, v]
        self._warm_U = None  # (N,   2) [a, delta]

        # last-solve metadata (node pulls these)
        self.last_u_sol = None
        self.last_return_status = '-'
        self.last_iter_count = -1
        self.last_slack_max = 0.0
        self.last_cost_breakdown = {}

        # opti containers populated in setup()
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
        a_ = opti.variable(N)
        de = opti.variable(N)
        slk = opti.variable(N + 1)

        # parameters
        P_n0 = opti.parameter()
        P_mu0 = opti.parameter()
        P_v0 = opti.parameter()
        P_kappa = opti.parameter(N + 1)
        P_vmax = opti.parameter(N + 1)
        P_nlb = opti.parameter(N + 1)          # wall-only lower bound on n
        P_nub = opti.parameter(N + 1)          # wall-only upper bound on n
        P_dL = opti.parameter(N + 1)           # raw d_left (wall cushion)
        P_dR = opti.parameter(N + 1)           # raw d_right
        P_ref_s = opti.parameter(N + 1)        # ego's time-based station
        # Obstacle parameters: one (N+1) vector per slot, plus per-slot active.
        P_obs_s = opti.parameter(n_obs, N + 1)
        P_obs_n = opti.parameter(n_obs, N + 1)
        P_obs_active = opti.parameter(n_obs)   # 0 or 1 per slot
        # Side bias: scalar weight per direction. Bias is applied PER k and
        # PER obstacle slot, gated by the same Gaussian proximity mask as
        # the bubble — so bias only fires where the obstacle actually is.
        P_bias_L = opti.parameter()
        P_bias_R = opti.parameter()

        # dynamics
        for k in range(N):
            opti.subject_to(n_[k + 1] == n_[k] + v_[k] * ca.sin(mu[k]) * dT)
            opti.subject_to(mu[k + 1] == mu[k]
                            + (v_[k] / L * ca.tan(de[k])
                               - P_kappa[k] * v_[k] * ca.cos(mu[k])) * dT)
            opti.subject_to(v_[k + 1] == v_[k] + a_[k] * dT)

        # initial conditions
        opti.subject_to(n_[0] == P_n0)
        opti.subject_to(mu[0] == P_mu0)
        opti.subject_to(v_[0] == P_v0)

        # input bounds
        opti.subject_to(opti.bounded(self.a_min, a_, self.a_max))
        opti.subject_to(opti.bounded(-self.delta_max, de, self.delta_max))

        # state bounds
        opti.subject_to(opti.bounded(-self.mu_max, mu, self.mu_max))
        for k in range(N + 1):
            opti.subject_to(v_[k] >= self.v_min)
            opti.subject_to(v_[k] <= P_vmax[k])
            opti.subject_to(n_[k] >= P_nlb[k] - slk[k])
            opti.subject_to(n_[k] <= P_nub[k] + slk[k])
            opti.subject_to(slk[k] >= 0.0)

        # ---- cost ---------------------------------------------------------
        # We build a breakdown dict for debug — last_cost_breakdown is
        # populated in solve() from sol.value(...) on these expressions.
        J_contour = 0
        for k in range(N + 1):
            J_contour = J_contour + self.q_n * n_[k] ** 2

        J_reg = 0
        for k in range(N):
            J_reg = J_reg + self.r_reg * de[k] ** 2

        J_smooth = 0
        for k in range(N - 1):
            J_smooth = J_smooth + self.r_a * (a_[k + 1] - a_[k]) ** 2
            J_smooth = J_smooth + self.r_delta * (de[k + 1] - de[k]) ** 2

        prog = 0
        for k in range(N):
            prog = prog + v_[k] * ca.cos(mu[k]) * dT
        J_progress = -self.gamma * prog

        # Smooth obstacle bubble — Gaussian in both (s, n). prox_sk is the
        # s-direction envelope which ALSO gates the side bias below, so
        # the bias only fires where the obstacle actually is in station.
        # Separating prox_sk lets us reuse it: cost_obs = w * prox_sk *
        # exp(-dn^2) and cost_bias = w_bias * prox_sk * viol^2.
        J_obs = 0
        J_bias = 0
        for o in range(n_obs):
            for k in range(N + 1):
                dx = (P_ref_s[k] - P_obs_s[o, k]) / self.sigma_s_obs
                dy = (n_[k] - P_obs_n[o, k]) / self.sigma_n_obs
                prox_sk = P_obs_active[o] * ca.exp(-(dx * dx))
                J_obs = J_obs + (self.w_obs * prox_sk
                                 * ca.exp(-(dy * dy)))
                # Per-k one-sided penalty anchored at this slot's n_o[k].
                # LEFT bias pushes n above (n_o + gap_lat); RIGHT pushes
                # below (n_o - gap_lat). Gated by prox_sk → zero cost far
                # from the obstacle → corridor is free everywhere else.
                viol_L = ca.fmax(0.0, (P_obs_n[o, k] + self.gap_lat) - n_[k])
                viol_R = ca.fmax(0.0, n_[k] - (P_obs_n[o, k] - self.gap_lat))
                J_bias = J_bias + P_bias_L * prox_sk * viol_L ** 2
                J_bias = J_bias + P_bias_R * prox_sk * viol_R ** 2

        # Wall cushion: quadratic hinge starting wall_buf from the raw wall.
        # Keeps the solver off the wall even when the hard corridor is far.
        J_wall = 0
        for k in range(N + 1):
            viol_up = ca.fmax(0.0, n_[k] - (P_dL[k] - self.wall_buf))
            viol_dn = ca.fmax(0.0, -n_[k] - (P_dR[k] - self.wall_buf))
            J_wall = J_wall + self.w_wall_buf * (viol_up ** 2 + viol_dn ** 2)

        J_slack = 0
        for k in range(N + 1):
            J_slack = J_slack + self.w_slack * slk[k] ** 2

        J = (J_contour + J_reg + J_smooth + J_progress
             + J_obs + J_bias + J_wall + J_slack)

        opti.minimize(J)
        opti.solver('ipopt', {
            'ipopt.max_iter':    self.ipopt_max_iter,
            'ipopt.print_level': self.ipopt_print_level,
            'print_time':        0,
            'ipopt.sb':          'yes',
        })

        self._opti = opti
        self._vars = dict(n=n_, mu=mu, v=v_, a=a_, de=de, slk=slk)
        self._pars = dict(n0=P_n0, mu0=P_mu0, v0=P_v0,
                          kappa=P_kappa, vmax=P_vmax,
                          nlb=P_nlb, nub=P_nub,
                          dL=P_dL, dR=P_dR,
                          ref_s=P_ref_s,
                          obs_s=P_obs_s, obs_n=P_obs_n,
                          obs_active=P_obs_active,
                          bias_L=P_bias_L, bias_R=P_bias_R)
        self._cost_exprs = dict(
            contour=J_contour, reg=J_reg, smooth=J_smooth,
            progress=J_progress, obs=J_obs, bias=J_bias,
            wall=J_wall, slack=J_slack)
        self.ready = True

    # ----------------------------------------------------------------- helpers
    def reset_warm_start(self):
        self._warm_X = None
        self._warm_U = None
        self.warm = False

    def _build_wall_bounds(self, d_left_arr, d_right_arr):
        """Corridor bounds from WALLS ONLY. Side-independent → constraint
        set is the same across LEFT/RIGHT flips, so the NLP feasible region
        is stationary tick-to-tick and warm-start stays relevant."""
        dL = np.asarray(d_left_arr, dtype=float)
        dR = np.asarray(d_right_arr, dtype=float)
        margin = self.inflation + self.wall_safe
        nub = np.maximum(dL - margin, 1e-3)
        nlb = -np.maximum(dR - margin, 1e-3)
        return nlb, nub

    def _build_vmax(self, ref_v, obs_arr, side):
        """Per-k v upper cap. Normally = ref_v (clipped to v_max). In TRAIL
        we cap using the obstacle's s-speed so ego falls in behind."""
        vmax = np.minimum(np.asarray(ref_v, dtype=float), self.v_max)
        if side == SIDE_TRAIL and obs_arr is not None:
            cap = self.v_max
            for o in range(obs_arr.shape[0]):
                if float(np.max(obs_arr[o, :, 2])) <= 0.0:
                    continue
                s0 = float(obs_arr[o, 0, 0])
                sN = float(obs_arr[o, -1, 0])
                v_obs_s = max((sN - s0) / max(self.N * self.dT, 1e-3), 0.0)
                cap = min(cap, max(v_obs_s * 0.95, self.v_min))
            vmax = np.minimum(vmax, cap)
        vmax = np.maximum(vmax, self.v_min + 0.1)
        return vmax

    def _build_obs_params(self, obs_arr):
        """Split (n_slot, N+1, 3) [s, n, w] array into (s_mat, n_mat, active)
        for NLP parameters. FAR-padded for inactive slots so exp-bubble
        contributes zero regardless of the active multiplier (defense in
        depth)."""
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

    def _seed_warm_start(self, n0, mu0, v0, nlb, nub, vmax):
        N = self.N
        # Gentle seed: hold n at n0, mu at mu0, v clipped. Corridor is
        # wall-only so centerline bleed is unnecessary (used to be a v1
        # hack to escape infeasible half-plane seeds).
        X = np.zeros((N + 1, 3))
        U = np.zeros((N, 2))
        for k in range(N + 1):
            X[k, 0] = float(np.clip(n0, nlb[k], nub[k]))
            X[k, 1] = mu0 * (1.0 - k / max(N, 1))
            X[k, 2] = float(np.clip(v0, self.v_min + 0.1, vmax[k]))
        return X, U

    # ------------------------------------------------------------------- solve
    def solve(self, initial_state, ref_slice,
              obstacles=None, side=SIDE_CLEAR, bias_scale=1.0):
        """
        initial_state: np.ndarray shape (3,) [n0, mu0, v0]
        ref_slice: dict with 'kappa_ref'(N+1), 'd_left_arr'(N+1),
                   'd_right_arr'(N+1), 'ref_v'(N+1), 'ref_s'(N+1)
        obstacles: (n_slot, N+1, 3) [s_o, n_o, w]   w>0 active
        side: int SIDE_{CLEAR,LEFT,RIGHT,TRAIL}
        """
        if not self.ready:
            raise RuntimeError('FrenetKinMPC.setup() must be called first')

        n0, mu0, v0 = float(initial_state[0]), float(initial_state[1]), float(initial_state[2])
        v0 = min(max(v0, self.v_min + 1e-3), self.v_max)
        kappa = np.asarray(ref_slice['kappa_ref'], dtype=float)
        dL = np.asarray(ref_slice['d_left_arr'], dtype=float)
        dR = np.asarray(ref_slice['d_right_arr'], dtype=float)
        rv = np.asarray(ref_slice['ref_v'], dtype=float)
        rs = np.asarray(ref_slice['ref_s'], dtype=float)

        nlb, nub = self._build_wall_bounds(dL, dR)
        vmax = self._build_vmax(rv, obstacles, side)

        # Clamp n0 into [nlb[0], nub[0]] — initial state must be feasible.
        # With wall-only bounds this is a generous window; clamp almost
        # never fires unless ego is already past the wall.
        n0_clamped = float(np.clip(n0, nlb[0], nub[0]))

        obs_s_mat, obs_n_mat, obs_active = self._build_obs_params(obstacles)
        # side → bias weights (continuous function of the int: flipping
        # sets the other scalar to 0 and loads w_side_bias into the active
        # direction. Cost surface deforms C^1 because the one-sided hinge
        # that the bias multiplies is itself smooth).
        # bias_scale ∈ [0, 1] lets the node ramp bias weight up from 0 after
        # a side flip, so the cost landscape morphs continuously instead of
        # snapping to full weight on tick 1.
        bs = float(np.clip(bias_scale, 0.0, 1.0))
        w_bias = self.w_side_bias * bs
        if side == SIDE_LEFT:
            bL, bR = w_bias, 0.0
        elif side == SIDE_RIGHT:
            bL, bR = 0.0, w_bias
        else:   # CLEAR / TRAIL — no lateral bias
            bL, bR = 0.0, 0.0

        opti = self._opti
        V = self._vars
        P = self._pars

        opti.set_value(P['n0'], n0_clamped)
        opti.set_value(P['mu0'], mu0)
        opti.set_value(P['v0'], v0)
        opti.set_value(P['kappa'], kappa)
        opti.set_value(P['vmax'], vmax)
        opti.set_value(P['nlb'], nlb)
        opti.set_value(P['nub'], nub)
        opti.set_value(P['dL'], dL)
        opti.set_value(P['dR'], dR)
        opti.set_value(P['ref_s'], rs)
        opti.set_value(P['obs_s'], obs_s_mat)
        opti.set_value(P['obs_n'], obs_n_mat)
        opti.set_value(P['obs_active'], obs_active)
        opti.set_value(P['bias_L'], bL)
        opti.set_value(P['bias_R'], bR)

        # Warm start — unchanged from v1. Cost is C^1 so the previous
        # solution remains a valid (and good) seed across side flips.
        if self._warm_X is not None and self._warm_X.shape == (self.N + 1, 3):
            Xw, Uw = self._warm_X, self._warm_U
        else:
            Xw, Uw = self._seed_warm_start(n0_clamped, mu0, v0, nlb, nub, vmax)
        opti.set_initial(V['n'],  Xw[:, 0])
        opti.set_initial(V['mu'], Xw[:, 1])
        opti.set_initial(V['v'],  Xw[:, 2])
        opti.set_initial(V['a'],  Uw[:, 0])
        opti.set_initial(V['de'], Uw[:, 1])

        success = False
        try:
            sol = opti.solve()
            n_sol = np.asarray(sol.value(V['n'])).ravel()
            mu_sol = np.asarray(sol.value(V['mu'])).ravel()
            v_sol = np.asarray(sol.value(V['v'])).ravel()
            a_sol = np.asarray(sol.value(V['a'])).ravel()
            de_sol = np.asarray(sol.value(V['de'])).ravel()
            slk_sol = np.asarray(sol.value(V['slk'])).ravel()
            self.last_return_status = sol.stats().get('return_status', 'OK')
            self.last_iter_count = int(sol.stats().get('iter_count', -1))
            # cost breakdown for debug
            try:
                self.last_cost_breakdown = {
                    k: float(sol.value(v))
                    for k, v in self._cost_exprs.items()}
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
                a_sol = np.asarray(opti.debug.value(V['a'])).ravel()
                de_sol = np.asarray(opti.debug.value(V['de'])).ravel()
                slk_sol = np.asarray(opti.debug.value(V['slk'])).ravel()
                self.last_cost_breakdown = {}
            except Exception:
                self.last_u_sol = None
                self.last_slack_max = 0.0
                self.last_cost_breakdown = {}
                return 0.0, 0.0, None, False

        # shift warm-start one step (receding-horizon seed)
        self._warm_X = np.column_stack([n_sol, mu_sol, v_sol])
        self._warm_U = np.column_stack([a_sol, de_sol])
        if len(n_sol) > 1:
            self._warm_X[:-1] = self._warm_X[1:]
            self._warm_U[:-1] = self._warm_U[1:]
        self.warm = True

        self.last_u_sol = np.column_stack([v_sol[:self.N], de_sol])
        self.last_slack_max = float(np.max(np.abs(slk_sol)))

        traj = np.column_stack([rs[:self.N + 1],
                                n_sol, mu_sol, v_sol])
        a0 = float(a_sol[0])
        de0 = float(de_sol[0])
        speed0 = float(v_sol[0] + a0 * self.dT)
        return speed0, de0, traj, success
