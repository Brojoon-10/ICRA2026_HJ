from __future__ import annotations

from dataclasses import dataclass

import casadi as ca
import numpy as np


## IY : probe HSL ma27; fall back to MUMPS if libhsl.so not loadable.
#       ma27 is 2~3x faster for small NLPs. Setup: see
#       planner/3d_gb_optimizer/fast_ggv_gen/solver/setup_hsl.sh.
def _select_linear_solver(verbose: bool = True) -> str:
    try:
        _x = ca.MX.sym('x')
        _probe = ca.nlpsol('hsl_probe', 'ipopt',
                           {'x': _x, 'f': (_x - 1.0) ** 2},
                           {'ipopt.linear_solver': 'ma27',
                            'ipopt.print_level': 0, 'print_time': 0})
        _probe(x0=0.0)
        if _probe.stats().get('success', False):
            if verbose:
                print('[sqp_casadi] linear_solver: ma27 (HSL)')
            return 'ma27'
    except Exception:
        pass
    if verbose:
        print('[sqp_casadi] linear_solver: mumps (HSL not available, fallback)')
    return 'mumps'


LINEAR_SOLVER = _select_linear_solver()
## IY : end


@dataclass
class SQPProblem:
    n_knots: int
    delta_s: float
    d_init: np.ndarray           # warm-start guess, shape (n_knots,)
    current_d: float             # ego lateral offset at start
    bounds_lower: np.ndarray     # right boundary (d_min), shape (n_knots,)
    bounds_upper: np.ndarray     # left boundary  (d_max), shape (n_knots,)
    ## IY : fixed slots — one constraint per knot, inactive uses center=0,min=0.
    obs_center_d: np.ndarray     # (n_knots,)
    obs_min_dist: np.ndarray     # (n_knots,)
    desired_side: str            # "left" | "right" | "any"
    kappa_limit: float           # 1/R_min given current speed
    lambda_reg: float            # regularization weight
    lambda_smooth: float = 100.0
    lambda_start_heading: float = 1000.0
    lambda_apex_bias: float = 10.0
    lambda_side: float = 50.0
    lambda_jerk: float = 0.0       # penalty on 3rd finite difference (curvature rate)
    ## IY : soft terminal — replaces d[-1]=0 hard eq with lambda_term*d[N]^2.
    lambda_term: float = 0.0
    soft_terminal: bool = False


class CasadiSQPSolver:
    """Re-usable IPOPT solver — warm-starts across cycles.

    Rebuilds the symbolic NLP only when structural inputs change
    (number of knots or obstacle-to-knot index map). Otherwise `solve()` just
    updates parameters and reuses last primal/dual as warm-start.
    """

    def __init__(self):
        self._solver = None
        self._n_knots = None
        ## IY : tracks build variant so soft_terminal toggle triggers rebuild.
        self._soft_terminal = False
        self._last_x = None
        self._last_lam_g = None
        self._last_lam_x = None

    # ------------------------------------------------------------------
    def _build(self, n_knots: int, soft_terminal: bool = False) -> None:
        ## IY : fixed-slot obs constraints — one per knot, no obs_map.
        d = ca.SX.sym('d', n_knots)

        p_d_init = ca.SX.sym('d_init', n_knots)
        p_cur_d = ca.SX.sym('cur_d')
        p_obs_c = ca.SX.sym('obs_c', n_knots)
        p_obs_m = ca.SX.sym('obs_m', n_knots)
        p_kappa_lim = ca.SX.sym('kappa_lim')
        p_lam_reg = ca.SX.sym('lam_reg')
        p_lam_smooth = ca.SX.sym('lam_smooth')
        p_lam_start = ca.SX.sym('lam_start')
        p_lam_apex = ca.SX.sym('lam_apex')
        p_lam_side = ca.SX.sym('lam_side')
        p_lam_jerk = ca.SX.sym('lam_jerk')
        ## IY : always-declared symbol; unused when soft_terminal=False.
        p_lam_term = ca.SX.sym('lam_term')
        p_delta_s = ca.SX.sym('ds')
        p_side = ca.SX.sym('side')   # +1 left, -1 right, 0 any

        # ---- cost ----
        dd = d[2:] - 2 * d[1:-1] + d[:-2]
        ### IY : 3rd finite difference ~ jerk — penalizes curvature sign flips
        ddd = d[3:] - 3 * d[2:-1] + 3 * d[1:-2] - d[:-3]
        J = (p_lam_smooth * ca.sumsqr(dd)
             + p_lam_jerk * ca.sumsqr(ddd)
             + p_lam_start * (d[1] - d[0]) ** 2
             + p_lam_apex * ca.sumsqr(d)
             + p_lam_reg * ca.sumsqr(d - p_d_init)
             + p_lam_side * ca.sumsqr(ca.fmax(-p_side * d, 0.0)))
        ## IY : soft terminal pull toward raceline (d[N]^2).
        if soft_terminal:
            J = J + p_lam_term * d[-1] ** 2

        # ---- constraints ----
        ### IY : drop d[-2]=0 — over-constrains tail curvature, drove infeasibility
        ## IY : soft_terminal=True drops d[-1]=0 hard eq (handled by cost above).
        if soft_terminal:
            g_list = [d[0] - p_cur_d]               # 1 equality
        else:
            g_list = [d[0] - p_cur_d, d[-1]]        # 2 equalities
        ## IY : obstacle slot per knot; inactive slots (obs_m[k]=0) trivially hold.
        for k in range(n_knots):
            g_list.append(p_obs_m[k] ** 2 - (d[k] - p_obs_c[k]) ** 2)    # <= 0
        for i in range(1, n_knots - 1):
            kappa = (d[i + 1] - 2 * d[i] + d[i - 1]) / (p_delta_s ** 2)
            g_list.append(kappa ** 2 - p_kappa_lim ** 2)                 # <= 0

        g = ca.vertcat(*g_list)

        p = ca.vertcat(p_d_init, p_cur_d, p_obs_c, p_obs_m,
                       p_kappa_lim, p_lam_reg, p_lam_smooth,
                       p_lam_start, p_lam_apex, p_lam_side,
                       p_lam_jerk, p_lam_term, p_delta_s, p_side)

        nlp = {'x': d, 'p': p, 'f': J, 'g': g}
        opts = {
            'print_time': 0,
            'ipopt.print_level': 0,
            ## IY : max_iter restored — ma27 per-iter is fast, safety cutoff wider.
            'ipopt.max_iter': 300,
            'ipopt.linear_solver': LINEAR_SOLVER,   ## IY : ma27 if HSL available
            ### IY : cost has λ_smooth * O(|d''|^2) ~ O(100) scale; 1e-3 tol
            ### was too strict → hit max_iter. Relax to 1e-2 (path shape is
            ### not precision-critical, CCMA smoothing covers residual noise).
            'ipopt.tol': 1e-2,
            'ipopt.acceptable_tol': 1e-1,
            'ipopt.acceptable_iter': 3,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.warm_start_bound_push': 1e-6,
            'ipopt.warm_start_mult_bound_push': 1e-6,
            'ipopt.warm_start_slack_bound_push': 1e-6,
            'ipopt.mu_init': 1e-4,
            'ipopt.hessian_approximation': 'limited-memory',
        }
        self._solver = ca.nlpsol('rhp_sqp', 'ipopt', nlp, opts)
        self._n_knots = n_knots
        self._soft_terminal = bool(soft_terminal)
        self._last_x = None
        self._last_lam_g = None
        self._last_lam_x = None

    # ------------------------------------------------------------------
    def solve(self, prob: SQPProblem) -> tuple[np.ndarray, dict]:
        n = prob.n_knots

        ## IY : obs_map removed — rebuild only on n_knots / soft_terminal change.
        need_rebuild = (self._solver is None
                        or self._n_knots != n
                        or self._soft_terminal != bool(prob.soft_terminal))
        if need_rebuild:
            self._build(n, soft_terminal=bool(prob.soft_terminal))

        side_flag = {'left': 1.0, 'right': -1.0}.get(prob.desired_side, 0.0)

        p_val = np.concatenate([
            prob.d_init,
            np.array([prob.current_d]),
            prob.obs_center_d,                  # length n_knots
            prob.obs_min_dist,                  # length n_knots
            np.array([prob.kappa_limit]),
            np.array([prob.lambda_reg]),
            np.array([prob.lambda_smooth]),
            np.array([prob.lambda_start_heading]),
            np.array([prob.lambda_apex_bias]),
            np.array([prob.lambda_side]),
            np.array([prob.lambda_jerk]),
            np.array([prob.lambda_term]),
            np.array([prob.delta_s]),
            np.array([side_flag]),
        ])

        # g bounds: [n_eq eq, n_knots obs ineq <=0, (n-2) turning ineq <=0]
        ## IY : n_eq=1 when soft_terminal (d[-1]=0 hard eq removed).
        n_eq = 1 if prob.soft_terminal else 2
        n_turn = n - 2
        n_g = n_eq + n + n_turn
        lbg = np.empty(n_g)
        ubg = np.empty(n_g)
        lbg[:n_eq] = -1e-2
        ubg[:n_eq] = 1e-2
        lbg[n_eq:] = -np.inf
        ubg[n_eq:] = 0.0

        x0 = self._last_x if (self._last_x is not None
                              and self._last_x.size == n) else prob.d_init

        args = dict(
            x0=x0, p=p_val,
            lbx=prob.bounds_lower, ubx=prob.bounds_upper,
            lbg=lbg, ubg=ubg,
        )
        if self._last_lam_g is not None and self._last_lam_g.size == n_g:
            args['lam_g0'] = self._last_lam_g
        if self._last_lam_x is not None and self._last_lam_x.size == n:
            args['lam_x0'] = self._last_lam_x

        sol = self._solver(**args)
        stats = self._solver.stats()
        success = bool(stats.get('success', False))
        d_opt = np.array(sol['x']).flatten()

        info = {
            'success': success,
            'status': str(stats.get('return_status', '')),
            'iter_count': int(stats.get('iter_count', 0)),
            'cost': float(sol['f']),
        }

        if success:
            self._last_x = d_opt.copy()
            self._last_lam_g = np.array(sol['lam_g']).flatten()
            self._last_lam_x = np.array(sol['lam_x']).flatten()
        else:
            self.reset_warmstart()
        return d_opt, info

    def reset_warmstart(self) -> None:
        self._last_x = None
        self._last_lam_g = None
        self._last_lam_x = None
