#!/usr/bin/env python3
### HJ : 3D speed-only optimizer — path fixed via REDUCED state NLP
#
# Mathematically equivalent to gen_global_racing_line.py, but with (n, chi)
# treated as INPUT FUNCTIONS of s (not decision variables).
#
# State vector    : [V, ax]            (was [V, n, chi, ax, ay])
# Control         : [jx]               (was [jx, jy])
# Parameters      : n_fixed(s), chi_fixed(s), dchi_ds_fixed(s)
# Dynamics        : dV/ds = ax/s_dot,  dax/ds = jx/s_dot
# Algebraic ay    : ay = V*s_dot*(dchi_ds + Omega_z)
# Constraints     : GGV diamond on (ax_tilde, ay_tilde)  — same as original
# Cost            : min lap time + jerk regularization   — same as original
#
# Workflow:
#   1. Subscribe to /global_waypoints (latched WpntArray)
#   2. Extract fixed path from message (s_m, d_m, psi_rad, vx_mps)
#   3. Compute chi(s) = psi_rad(s) - theta_centerline(s)  (from Track3D csv)
#   4. Solve reduced NLP
#   5. RE-PUBLISH to same topic /global_waypoints with new vx_mps, ax_mps2
#   6. Spin forever (latched) but only solve ONCE
#
# Usage:
#   rosrun stack_master 3d_optimized_vel_planner.py --map gazebo_wall_2 \
#       --vehicle_name rc_car_10th_fast1 --gg_vehicle_name rc_car_10th

# --- pandas/numpy version bypass ---
import sys as _sys
_sys.path[:] = [p for p in _sys.path if '/.local/' not in p]
import numpy as _np
_np.__version__ = '1.22.4'
if not hasattr(_np.random, 'BitGenerator'):
    from numpy.random.bit_generator import BitGenerator as _BG
    _np.random.BitGenerator = _BG
# --- end bypass ---

import os
import sys
import argparse
import time
import numpy as np
import pandas as pd
import yaml
import casadi as ca

import rospy
from f110_msgs.msg import WpntArray, Wpnt

# --- locate 3d_gb_optimizer modules ---
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_GB_OPT_DIR = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', 'planner', '3d_gb_optimizer'))
_SRC_DIR = os.path.join(_GB_OPT_DIR, 'global_line', 'src')
_DATA_DIR = os.path.join(_GB_OPT_DIR, 'global_line', 'data')

sys.path.insert(0, _SRC_DIR)
from track3D import Track3D
from ggManager import GGManager


def build_and_solve(track, gg, vehicle_params,
                    n_fixed, chi_fixed, v_init, ax_init,
                    w_T=1.0, w_jx=1e-2, V_min=0.0, RK4_steps=1, sol_opt=None):
    """Reduced-state NLP. All arrays must be on track.s (resampled grid)."""
    h = vehicle_params['h']
    N = track.s.size
    ds = track.ds
    V_max_gg = float(gg.V_max)

    # Periodic CasADi interpolants for fixed path
    def concat_arr(a):
        return np.concatenate((a, a[1:], a[1:]))
    s_aug = np.concatenate((track.s, track.s[-1] + track.s[1:], 2 * track.s[-1] + track.s[1:]))

    n_fn = ca.interpolant('n_fix', 'linear', [s_aug], concat_arr(n_fixed))
    chi_fn = ca.interpolant('chi_fix', 'linear', [s_aug], concat_arr(chi_fixed))

    chi_unwrapped = np.unwrap(chi_fixed)
    dchi_ds = (np.roll(chi_unwrapped, -1) - np.roll(chi_unwrapped, 1)) / (2.0 * ds)
    dchi_ds_fn = ca.interpolant('dchi_ds_fix', 'linear', [s_aug], concat_arr(dchi_ds))

    # Symbolic states: [V, ax]
    V = ca.MX.sym('V')
    ax = ca.MX.sym('ax')
    x = ca.vertcat(V, ax)
    nx_ = x.shape[0]

    jx = ca.MX.sym('jx')
    u = jx
    nu_ = 1

    s_sym = ca.MX.sym('s')

    n_s = n_fn(s_sym)
    chi_s = chi_fn(s_sym)
    dchi_ds_s = dchi_ds_fn(s_sym)
    Omega_z_s = track.Omega_z_interpolator(s_sym)

    s_dot = (V * ca.cos(chi_s)) / (1.0 - n_s * Omega_z_s)
    ay_alg = V * s_dot * (dchi_ds_s + Omega_z_s)

    # Apparent accelerations via Track3D (symbolic MX input OK)
    ax_tilde, ay_tilde, g_tilde = track.calc_apparent_accelerations(
        V=V, n=n_s, chi=chi_s, ax=ax, ay=ay_alg, s=s_sym, h=h,
        neglect_w_omega_y=True, neglect_w_omega_x=True,
        neglect_euler=True, neglect_centrifugal=True,
        neglect_w_dot=False, neglect_V_omega=False,
    )

    dV = ax / s_dot
    dax = jx / s_dot
    dx = ca.vertcat(dV, dax)

    L_t = w_T * 1.0 / s_dot
    L_reg = w_jx * (jx / s_dot) ** 2

    # RK4 (same as original)
    M = RK4_steps
    ds_rk = ds / M
    f = ca.Function('f', [x, u, s_sym], [dx, L_t, L_reg])
    X0 = ca.MX.sym('X0', nx_)
    U = ca.MX.sym('U', nu_)
    S0 = ca.MX.sym('S0')
    X = X0
    S = S0
    Q_t = 0
    Q_reg = 0
    for j in range(M):
        k1, k1_qt, k1_qr = f(X, U, S)
        k2, k2_qt, k2_qr = f(X + ds_rk/2 * k1, U, S + ds_rk/2)
        k3, k3_qt, k3_qr = f(X + ds_rk/2 * k2, U, S + ds_rk/2)
        k4, k4_qt, k4_qr = f(X + ds_rk * k3, U, S + ds_rk)
        X = X + ds_rk/6 * (k1 + 2*k2 + 2*k3 + k4)
        Q_t = Q_t + ds_rk/6 * (k1_qt + 2*k2_qt + 2*k3_qt + k4_qt)
        Q_reg = Q_reg + ds_rk/6 * (k1_qr + 2*k2_qr + 2*k3_qr + k4_qr)
        S = S + ds_rk
    F = ca.Function('F', [X0, U, S0], [X, Q_t, Q_reg], ['x0','u','s0'], ['xf','q_t','q_reg'])

    # Build NLP
    w = []
    w0 = []
    lbw = []
    ubw = []
    J_t = 0.0
    J_reg = 0.0
    g = []
    lbg = []
    ubg = []

    Xk = ca.MX.sym('X0', nx_)
    w += [Xk]
    lbw += [V_min, -np.inf]
    ubw += [V_max_gg, np.inf]
    w0 += [max(float(v_init[0]), max(V_min, 0.5)), float(ax_init[0])]

    for k in range(N):
        s_k = k * ds
        n_k = float(n_fixed[k])
        chi_k = float(chi_fixed[k])
        dchi_k = float(dchi_ds[k])
        Om_z_k = float(track.Omega_z_interpolator(s_k))

        s_dot_k = (Xk[0] * ca.cos(chi_k)) / (1.0 - n_k * Om_z_k)
        ay_k = Xk[0] * s_dot_k * (dchi_k + Om_z_k)

        axt_k, ayt_k, gt_k = track.calc_apparent_accelerations(
            V=Xk[0], n=n_k, chi=chi_k, ax=Xk[1], ay=ay_k, s=s_k, h=h,
            neglect_w_omega_y=True, neglect_w_omega_x=True,
            neglect_euler=True, neglect_centrifugal=True,
            neglect_w_dot=False, neglect_V_omega=False,
        )

        gg_exp, ax_min, ax_max, ay_max = ca.vertsplit(
            gg.acc_interpolator(ca.vertcat(Xk[0], gt_k))
        )
        g += [ay_max - ca.fabs(ayt_k)]
        lbg += [0.0]; ubg += [np.inf]

        g += [ca.fabs(ax_min) * ca.power(
            ca.fmax(1.0 - ca.power(ca.fmin(ca.fabs(ayt_k) / ay_max, 1.0), gg_exp), 1e-3),
            1.0 / gg_exp) - ca.fabs(axt_k)]
        lbg += [0.0]; ubg += [np.inf]

        g += [ax_max - axt_k]
        lbg += [0.0]; ubg += [np.inf]

        if k == N - 1:
            break

        Uk = ca.MX.sym('U_' + str(k), nu_)
        w += [Uk]
        lbw += [-np.inf] * nu_
        ubw += [np.inf] * nu_
        w0 += [0.0] * nu_

        Fk = F(x0=Xk, u=Uk, s0=s_k)
        Xk_end = Fk['xf']
        J_t = J_t + Fk['q_t']
        J_reg = J_reg + Fk['q_reg']

        Xk = ca.MX.sym('X_' + str(k+1), nx_)
        w += [Xk]
        lbw += [V_min, -np.inf]
        ubw += [V_max_gg, np.inf]
        w0 += [max(float(v_init[k+1]), max(V_min, 0.5)), float(ax_init[k+1])]

        g += [Xk_end - Xk]
        lbg += [0.0] * nx_
        ubg += [0.0] * nx_

    g += [w[0] - Xk]
    lbg += [0.0] * nx_
    ubg += [0.0] * nx_

    w_vec = ca.vertcat(*w)
    g_vec = ca.vertcat(*g)
    w0_vec = ca.vertcat(*w0)
    lbw_vec = ca.vertcat(*lbw)
    ubw_vec = ca.vertcat(*ubw)
    lbg_vec = ca.vertcat(*lbg)
    ubg_vec = ca.vertcat(*ubg)

    if sol_opt is None:
        sol_opt = {
            'ipopt.max_iter': 5000,
            'ipopt.hessian_approximation': 'limited-memory',
            'ipopt.line_search_method': 'cg-penalty',
            'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.acceptable_iter': 10,
            'ipopt.constr_viol_tol': 1e-4,
        }
    sol_opt = dict(sol_opt)
    sol_opt.setdefault('print_time', 0)
    sol_opt.setdefault('ipopt.print_level', 3)  ### HJ : temporarily verbose to debug convergence

    nlp = {'f': J_t + J_reg, 'x': w_vec, 'g': g_vec}
    rospy.loginfo(f'[velopt] NLP: {int(w_vec.shape[0])} vars, {int(g_vec.shape[0])} constraints, {N} points')

    t_build = time.time()
    solver = ca.nlpsol('solver', 'ipopt', nlp, sol_opt)
    t_build = time.time() - t_build
    rospy.loginfo(f'[velopt] solver built in {t_build:.2f}s, solving...')

    t_solve = time.time()
    sol = solver(x0=w0_vec, lbx=lbw_vec, ubx=ubw_vec, lbg=lbg_vec, ubg=ubg_vec)
    t_solve = time.time() - t_solve

    laptime = float(ca.Function('f_laptime', [w_vec], [J_t])(sol['x']))
    success = solver.stats()['success']
    rospy.loginfo(f'[velopt] IPOPT: {t_solve:.2f}s, success={success}, laptime={laptime:.4f}s')

    sol_x = np.array(sol['x']).flatten()
    stride = nx_ + nu_
    V_opt = np.zeros(N)
    ax_opt = np.zeros(N)
    for k in range(N):
        off = k * stride
        V_opt[k] = sol_x[off]
        ax_opt[k] = sol_x[off + 1]

    return V_opt, ax_opt, laptime, success


class VelOptNode:
    def __init__(self, map_name, raceline_variant, vehicle_yml_file, gg_dir_name,
                 step_size_opt=0.2, V_min=0.0, gg_margin=0.0):
        rospy.init_node('vel_opt_3d')
        self.map_name = map_name
        self.step_size_opt = step_size_opt
        self.V_min = V_min
        self.gg_margin = gg_margin

        # Resolve paths — folder structure fixed, filenames derived from args
        self.map_dir = os.path.abspath(os.path.join(_THIS_DIR, '..', 'maps', self.map_name))
        # track is always <map>_3d_smoothed.csv
        self.track_csv = os.path.join(self.map_dir, f'{self.map_name}_3d_smoothed.csv')
        # raceline: <map>_3d_<variant>_timeoptimal.csv
        self.raceline_csv = os.path.join(
            self.map_dir, f'{self.map_name}_3d_{raceline_variant}_timeoptimal.csv')
        self.vehicle_yml = os.path.join(_DATA_DIR, 'vehicle_params', vehicle_yml_file)
        self.gg_path = os.path.join(_DATA_DIR, 'gg_diagrams', gg_dir_name, 'velocity_frame')

        for p, label in [(self.track_csv, 'track csv'),
                         (self.raceline_csv, 'raceline csv'),
                         (self.vehicle_yml, 'vehicle yml'),
                         (self.gg_path, 'gg path')]:
            if not os.path.exists(p):
                raise FileNotFoundError(f'{label} not found: {p}')

        rospy.loginfo(f'[velopt] map={map_name}')
        rospy.loginfo(f'[velopt] track    : {self.track_csv}')
        rospy.loginfo(f'[velopt] raceline : {self.raceline_csv}')
        rospy.loginfo(f'[velopt] vehicle  : {self.vehicle_yml}')
        rospy.loginfo(f'[velopt] gg       : {self.gg_path}')

        with open(self.vehicle_yml) as f:
            self.vehicle_params = yaml.safe_load(f)['vehicle_params']

        # Load track + gg ONCE at startup
        self.track = Track3D(path=self.track_csv)
        ### HJ : compute step_size that exactly divides the full track length
        #       so the NLP grid covers [0, L_track) with uniform spacing and
        #       the periodic boundary V[0] == V[N-1] corresponds to a real
        #       loop closure at s=L_track ↔ s=0 (unlike the original
        #       gen_global_racing_line.py which leaves a small unoptimized gap).
        L_track = float(self.track.s[-1] + self.track.ds)
        N_target = max(10, int(round(L_track / self.step_size_opt)))
        actual_step = L_track / N_target
        rospy.loginfo(f'[velopt] L_track={L_track:.4f}m, '
                      f'desired_step={self.step_size_opt:.4f}, '
                      f'actual_step={actual_step:.6f} (N={N_target})')
        self.track.resample(actual_step)
        ### HJ : end
        self.gg = GGManager(gg_path=self.gg_path, gg_margin=self.gg_margin)

        # Load fixed path from timeoptimal csv (n_opt, chi_opt)
        rl = pd.read_csv(self.raceline_csv)
        s_rl = rl['s_opt'].to_numpy()
        n_rl = rl['n_opt'].to_numpy()
        chi_rl = rl['chi_opt'].to_numpy()
        v_rl = rl['v_opt'].to_numpy() if 'v_opt' in rl.columns else None
        ax_rl = rl['ax_opt'].to_numpy() if 'ax_opt' in rl.columns else None

        # Resample onto NLP grid (track.s)
        s_period = s_rl[-1] + (s_rl[-1] - s_rl[-2])
        s_q = self.track.s % s_period
        self.n_fixed = np.interp(s_q, s_rl, n_rl)
        self.chi_fixed = np.interp(s_q, s_rl, np.unwrap(chi_rl))
        self.v_init = np.interp(s_q, s_rl, v_rl) if v_rl is not None else np.full_like(self.track.s, 3.0)
        self.ax_init = np.interp(s_q, s_rl, ax_rl) if ax_rl is not None else np.zeros_like(self.track.s)

        rospy.loginfo(f'[velopt] Track3D + GGManager + raceline ready, grid={self.track.s.size} pts')
        rospy.loginfo(f'[velopt] fixed n  : [{self.n_fixed.min():.3f}, {self.n_fixed.max():.3f}] m')
        rospy.loginfo(f'[velopt] fixed chi: [{self.chi_fixed.min():.3f}, {self.chi_fixed.max():.3f}] rad')

        # Solve NLP ONCE at startup (no topic input needed for dynamics)
        self._solve_once()

        # Latched publisher on /global_waypoints
        self.pub = rospy.Publisher('/global_waypoints', WpntArray, queue_size=1, latch=True)

        # Subscribe to /global_waypoints only to grab the message template (path fields).
        # We overwrite vx_mps, ax_mps2 and republish to the same topic.
        self._processed = False
        self.sub = rospy.Subscriber('/global_waypoints', WpntArray, self._cb, queue_size=1)
        rospy.loginfo('[velopt] waiting for /global_waypoints template message ...')

    def _solve_once(self):
        """Run the reduced-state NLP once using csv-based fixed path."""
        rospy.loginfo('[velopt] solving NLP ...')
        self.V_opt, self.ax_opt, laptime, success = build_and_solve(
            track=self.track, gg=self.gg, vehicle_params=self.vehicle_params,
            n_fixed=self.n_fixed, chi_fixed=self.chi_fixed,
            v_init=self.v_init, ax_init=self.ax_init, V_min=self.V_min,
        )
        rospy.loginfo(f'[velopt] laptime={laptime:.4f}s  '
                      f'V range [{self.V_opt.min():.2f}, {self.V_opt.max():.2f}] m/s  '
                      f'success={success}')

    def _cb(self, msg):
        if self._processed:
            return
        self._processed = True
        self.sub.unregister()

        # Determine the message's own track length (for periodic wrap of our V_opt)
        s_msg = np.array([w.s_m for w in msg.wpnts], dtype=np.float64)
        L_msg = s_msg[-1] + (s_msg[-1] - s_msg[-2])  # total track length from wpnts spacing

        ### HJ : build PERIODIC interpolation arrays for V_opt and ax_opt.
        #       Our NLP enforced V[0] == V[N-1] at s = 0 and s = (N-1)*ds.
        #       Map s_msg → s_query in [0, (N-1)*ds] by scaling: s_q = s_msg * (N-1)*ds / L_msg
        #       Then interp against track.s (linear, no clamping artifacts at wrap).
        N_nlp = self.track.s.size
        s_nlp_max = self.track.s[-1]  # (N-1)*ds

        # append wrap point: s = s_nlp_max + ds, V = V_opt[0]  (periodic closure)
        ds_nlp = self.track.ds
        s_wrap = np.concatenate((self.track.s, [s_nlp_max + ds_nlp]))
        V_wrap = np.concatenate((self.V_opt, [self.V_opt[0]]))
        ax_wrap = np.concatenate((self.ax_opt, [self.ax_opt[0]]))

        # scale s_msg into NLP s range (so that s_msg=0 → 0, s_msg=L_msg → s_nlp_max + ds_nlp)
        scale = (s_nlp_max + ds_nlp) / L_msg
        s_query = s_msg * scale

        V_out = np.interp(s_query, s_wrap, V_wrap)
        ax_out = np.interp(s_query, s_wrap, ax_wrap)

        # Build output msg
        out = WpntArray()
        out.header = msg.header
        out.header.stamp = rospy.Time.now()
        for i, wpnt_in in enumerate(msg.wpnts):
            wo = Wpnt()
            wo.id = wpnt_in.id
            wo.s_m = wpnt_in.s_m
            wo.d_m = wpnt_in.d_m
            wo.x_m = wpnt_in.x_m
            wo.y_m = wpnt_in.y_m
            wo.z_m = wpnt_in.z_m
            wo.d_right = wpnt_in.d_right
            wo.d_left = wpnt_in.d_left
            wo.psi_rad = wpnt_in.psi_rad
            wo.kappa_radpm = wpnt_in.kappa_radpm
            wo.mu_rad = wpnt_in.mu_rad
            wo.vx_mps = float(V_out[i])
            wo.ax_mps2 = float(ax_out[i])
            out.wpnts.append(wo)

        self.pub.publish(out)
        rospy.loginfo(f'[velopt] published /global_waypoints '
                      f'(msg L={L_msg:.2f}m, NLP L={s_nlp_max + ds_nlp:.2f}m, '
                      f'V[0]={V_out[0]:.2f}, V[-1]={V_out[-1]:.2f})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True,
                    help='Map folder name under stack_master/maps/. '
                         'Track csv is auto-derived as <map>_3d_smoothed.csv')
    ap.add_argument('--raceline', required=True,
                    help='Raceline variant — filename becomes '
                         '<map>_3d_<raceline>_timeoptimal.csv  '
                         '(e.g. "rc_car_10th_fast4_out")')
    ap.add_argument('--vehicle_yml', required=True,
                    help='Vehicle params yml filename inside '
                         '3d_gb_optimizer/.../vehicle_params/ '
                         '(e.g. "params_rc_car_10th_backup.yml")')
    ap.add_argument('--gg_dir', required=True,
                    help='GG diagrams folder name inside '
                         '3d_gb_optimizer/.../gg_diagrams/ '
                         '(e.g. "rc_car_10th_fast4_out")')
    ap.add_argument('--step_size_opt', type=float, default=0.2)
    ap.add_argument('--V_min', type=float, default=0.0)
    ap.add_argument('--gg_margin', type=float, default=0.0)
    # parse_known_args so ROS remapping args don't choke
    args, _ = ap.parse_known_args()

    VelOptNode(
        map_name=args.map,
        raceline_variant=args.raceline,
        vehicle_yml_file=args.vehicle_yml,
        gg_dir_name=args.gg_dir,
        step_size_opt=args.step_size_opt,
        V_min=args.V_min,
        gg_margin=args.gg_margin,
    )
    rospy.spin()


if __name__ == '__main__':
    main()
