#!/usr/bin/env python3
"""### HJ : Frenet kinematic-bicycle MPC for overtaking (3D-safe).

State   x = [n, mu, v]       (frenet lateral offset, heading-vs-tangent, speed)
Control u = [a, delta]
Station s = ref_slice.ref_s[k]  (fixed arc-length grid from the slicer; the
                                  solver never touches xy — see CLAUDE.md
                                  "3D에서 Frenet xy round-trip 금지".)

Discrete dynamics (Liniger 2015 frenet kinematic bicycle):
    n_{k+1}  = n_k  + v_k * sin(mu_k)                          * dT
    mu_{k+1} = mu_k + (v_k / L * tan(delta_k)
                       - kappa_ref_k * v_k * cos(mu_k))        * dT
    v_{k+1}  = v_k  + a_k                                      * dT

Cost (4 terms + progress + slack):
    q_n  * sum n_k^2                 (contour)
  + r_a  * sum (Delta a)^2           (accel smoothness)
  + r_d  * sum (Delta delta)^2       (steer smoothness)
  + r_reg* sum delta_k^2             (steer zero-bias)
  - gamma* sum v_k*cos(mu_k)*dT      (progress)
  + w_slk* sum slack_k^2             (corridor slack penalty)

Hard constraints:
  - corridor (with wall_safe + inflation) as slackable bounds on n_k
  - v, a, delta, mu bounded
  - OBSTACLE side decision (enforced from outside as half-plane on n_k
    for k in [k_obs_start, k_obs_end]):
        side=LEFT : n_k >=  n_obs_k + gap_lat  (pass obstacle on the left)
        side=RIGHT: n_k <= n_obs_k - gap_lat
    TRAIL is enforced as a per-k v upper cap so ego stays behind obstacle.

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
    FAR = 1e3

    def __init__(self, **params):
        # horizon
        self.N = int(params.get('N', 20))
        self.dT = float(params.get('dT', 0.05))
        self.L = float(params.get('vehicle_L', 0.33))

        # cost weights
        self.q_n = float(params.get('q_n', 3.0))
        self.gamma = float(params.get('gamma_progress', 10.0))
        self.r_a = float(params.get('r_a', 0.5))
        self.r_delta = float(params.get('r_delta', 5.0))
        self.r_reg = float(params.get('r_steer_reg', 0.1))
        self.w_slack = float(params.get('w_slack', 2000.0))

        # limits
        self.v_min = float(params.get('v_min', 0.5))
        self.v_max = float(params.get('v_max', 8.0))
        self.a_min = float(params.get('a_min', -4.0))
        self.a_max = float(params.get('a_max', 3.0))
        self.delta_max = float(params.get('delta_max', 0.6))
        self.mu_max = float(params.get('mu_max', 0.9))

        # geometry / safety
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

        # opti containers populated in setup()
        self._opti = None
        self._vars = None
        self._pars = None

    # ------------------------------------------------------------------ setup
    def setup(self):
        N = self.N
        dT = self.dT
        L = self.L

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
        P_vmax = opti.parameter(N + 1)      # per-k v upper cap (ref_v or trail cap)
        P_nlb = opti.parameter(N + 1)       # effective lower bound on n
        P_nub = opti.parameter(N + 1)       # effective upper bound on n

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
        # Slack must be non-negative; corridor bounds get slackened both sides.
        for k in range(N + 1):
            opti.subject_to(v_[k] >= self.v_min)
            opti.subject_to(v_[k] <= P_vmax[k])
            opti.subject_to(n_[k] >= P_nlb[k] - slk[k])
            opti.subject_to(n_[k] <= P_nub[k] + slk[k])
            opti.subject_to(slk[k] >= 0.0)

        # cost
        J = 0
        for k in range(N + 1):
            J = J + self.q_n * n_[k] ** 2
        for k in range(N):
            J = J + self.r_reg * de[k] ** 2
        for k in range(N - 1):
            J = J + self.r_a * (a_[k + 1] - a_[k]) ** 2
            J = J + self.r_delta * (de[k + 1] - de[k]) ** 2
        prog = 0
        for k in range(N):
            prog = prog + v_[k] * ca.cos(mu[k]) * dT
        J = J - self.gamma * prog
        for k in range(N + 1):
            J = J + self.w_slack * slk[k] ** 2

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
                          nlb=P_nlb, nub=P_nub)
        self.ready = True

    # ----------------------------------------------------------------- helpers
    def reset_warm_start(self):
        self._warm_X = None
        self._warm_U = None
        self.warm = False

    def _build_corridor_bounds(self, d_left_arr, d_right_arr,
                               obs_arr, side):
        """Assemble per-k (nlb, nub) including corridor (wall_safe + inflation)
        and obstacle half-plane (LEFT/RIGHT). TRAIL leaves n free — it's
        enforced via v upper cap in _build_vmax()."""
        N1 = self.N + 1
        dL = np.asarray(d_left_arr, dtype=float)
        dR = np.asarray(d_right_arr, dtype=float)
        margin = self.inflation + self.wall_safe
        nub = np.maximum(dL - margin, 1e-3)     # >0 so slack keeps feasibility
        nlb = -np.maximum(dR - margin, 1e-3)

        if side in (SIDE_LEFT, SIDE_RIGHT) and obs_arr is not None:
            # Use the closest (smallest max-w) active slot; if multiple active,
            # combine into the tightest half-plane per-k (take max/min).
            lb_obs = -np.inf * np.ones(N1)
            ub_obs = +np.inf * np.ones(N1)
            any_active = False
            for o in range(obs_arr.shape[0]):
                w_ts = obs_arr[o, :, 2]
                if float(np.max(w_ts)) <= 0.0:
                    continue
                any_active = True
                n_o = obs_arr[o, :, 1]
                if side == SIDE_LEFT:
                    # ego must pass LEFT of obstacle  => n >= n_o + gap
                    lb_obs = np.maximum(lb_obs, n_o + self.gap_lat)
                elif side == SIDE_RIGHT:
                    ub_obs = np.minimum(ub_obs, n_o - self.gap_lat)
            if any_active:
                nlb = np.maximum(nlb, lb_obs)
                nub = np.minimum(nub, ub_obs)
        # Enforce nlb <= nub even under conflicting obstacle half-planes; if
        # violated we relax to centerline (slack will expose infeasibility to
        # the node's fallback ladder).
        bad = nlb > nub
        if np.any(bad):
            mid = 0.5 * (nlb + nub)
            nlb = np.where(bad, mid, nlb)
            nub = np.where(bad, mid, nub)
        return nlb, nub

    def _build_vmax(self, ref_v, obs_arr, side, ref_s):
        """Per-k v upper cap. Normally = ref_v (clipped to v_max). In TRAIL
        we cap so ego never overtakes the obstacle's s-track by less than
        gap_long — expressed per-k by comparing predicted s_ego (from
        cumulative v*dT forward-eulered with the per-k cap) vs obstacle s.
        For a closed-form cap we conservatively use the obstacle's own
        reference-slice-relative speed as the ego's cap when in TRAIL."""
        vmax = np.minimum(np.asarray(ref_v, dtype=float), self.v_max)
        if side == SIDE_TRAIL and obs_arr is not None:
            # Pick the slowest active obstacle (most limiting) at k=0.
            cap = self.v_max
            for o in range(obs_arr.shape[0]):
                if float(np.max(obs_arr[o, :, 2])) <= 0.0:
                    continue
                s0 = float(obs_arr[o, 0, 0])
                sN = float(obs_arr[o, -1, 0])
                # Approximate obstacle s-speed from endpoints.
                v_obs_s = max((sN - s0) / max(self.N * self.dT, 1e-3), 0.0)
                cap = min(cap, max(v_obs_s * 0.95, self.v_min))
            vmax = np.minimum(vmax, cap)
        vmax = np.maximum(vmax, self.v_min + 0.1)
        return vmax

    def _seed_warm_start(self, n0, mu0, v0, nlb, nub, vmax):
        N = self.N
        # Pull n0 toward the center of the allowed corridor for the rest of
        # the horizon — prevents infeasible seeds when obstacle just appeared.
        n_target = 0.5 * (nlb + nub)
        X = np.zeros((N + 1, 3))
        U = np.zeros((N, 2))
        for k in range(N + 1):
            alpha = k / max(N, 1)
            X[k, 0] = (1.0 - alpha) * n0 + alpha * n_target[k]
            X[k, 1] = (1.0 - alpha) * mu0    # bleed mu to 0
            X[k, 2] = min(max(v0, self.v_min + 0.1), vmax[k])
        return X, U

    # ------------------------------------------------------------------- solve
    def solve(self, initial_state, ref_slice,
              obstacles=None, side=SIDE_CLEAR):
        """
        initial_state: np.ndarray shape (3,) [n0, mu0, v0]
        ref_slice: dict with 'kappa_ref'(N+1), 'd_left_arr'(N+1),
                   'd_right_arr'(N+1), 'ref_v'(N+1), 'ref_s'(N+1)
        obstacles: (n_slot, N+1, 3) [s_o, n_o, w]   w>0 active
        side: int SIDE_{CLEAR,LEFT,RIGHT,TRAIL}

        Returns
        -------
        a0     : float
        delta0 : float
        traj   : np.ndarray (N+1, 4) [s, n, mu, v]  (s copied from ref_slice)
        success: bool
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

        nlb, nub = self._build_corridor_bounds(dL, dR, obstacles, side)
        vmax = self._build_vmax(rv, obstacles, side, rs)

        # Clamp n0 into [nlb[0], nub[0]] — initial state must be feasible.
        n0_clamped = float(np.clip(n0, nlb[0], nub[0]))

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

        # Warm start
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
            success = True
        except RuntimeError:
            # IPOPT failure — try to pull best iterate.
            self.last_return_status = opti.stats().get('return_status', 'FAIL')
            self.last_iter_count = int(opti.stats().get('iter_count', -1))
            try:
                n_sol = np.asarray(opti.debug.value(V['n'])).ravel()
                mu_sol = np.asarray(opti.debug.value(V['mu'])).ravel()
                v_sol = np.asarray(opti.debug.value(V['v'])).ravel()
                a_sol = np.asarray(opti.debug.value(V['a'])).ravel()
                de_sol = np.asarray(opti.debug.value(V['de'])).ravel()
                slk_sol = np.asarray(opti.debug.value(V['slk'])).ravel()
            except Exception:
                self.last_u_sol = None
                self.last_slack_max = 0.0
                return 0.0, 0.0, None, False

        # shift warm-start one step
        self._warm_X = np.column_stack([n_sol, mu_sol, v_sol])
        self._warm_U = np.column_stack([a_sol, de_sol])
        if len(n_sol) > 1:
            self._warm_X[:-1] = self._warm_X[1:]
            self._warm_U[:-1] = self._warm_U[1:]
        self.warm = True

        self.last_u_sol = np.column_stack([v_sol[:self.N], de_sol])
        self.last_slack_max = float(np.max(np.abs(slk_sol)))

        # traj in (s, n, mu, v)
        traj = np.column_stack([rs[:self.N + 1],
                                n_sol, mu_sol, v_sol])
        a0 = float(a_sol[0])
        de0 = float(de_sol[0])
        # Node API expects (speed, steering, trajectory, success). We return
        # the first-step commanded v as the "speed" scalar, and steering.
        speed0 = float(v_sol[0] + a0 * self.dT)
        return speed0, de0, traj, success
