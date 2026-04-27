#!/usr/bin/env python3
"""### HJ : Phase 4 Tier-2 GEOMETRIC_FALLBACK.

Frenet quintic으로 (s0, n0, dn_ds0) → (s0+Δs, 0, 0) 복귀 곡선을 만드는
primitive. MPC NLP가 연속 실패할 때 "현재 차 위치에서 raceline 까지의 다리"를
1ms 이내로 깔아주는 역할. 특성:

  - boundary condition 6개 (양끝 n, n', n'') → 5차 다항식 6계수 → 항상 해.
  - 곡률 양끝에서 0 (n''=0)이라 controller lookahead가 덜 놀라며, 최대 곡률도
    (n0, Δs) 로만 결정되므로 예측 가능.
  - s축에 등간격 샘플링 후 raceline 기반 sn_to_xy 로 (x, y, z) 복원.

이 모듈은 Track3D를 쓰지 않고 MPCRacelineLifter 의 인터페이스만 요구한다
(raceline-base 일관성 유지). 샘플 개수는 MPC horizon 과 무관하게 외부에서
지정.
"""

import numpy as np


### HJ : 2026-04-27 — recovery_spliner-style fallback (BPoly + GB-tangent
###      lookahead + wall-aware corridor pull-in + smoothing).
###      Replaces the quintic-with-clip approach. Algorithm mirrors
###      legacy 3d_recovery_spliner_node.do_spline:
###        1) Pick lookahead idx by tangent alignment with ego heading
###        2) BPoly cubic Hermite spline (ego pose → lookahead pose)
###        3) Append GB suffix wpnts after the spline
###        4) Resample at uniform arc-length
###        5) For each sample, if too close to wall (within wall_pull_thr
###           of wall), pull n_sample toward GB by a fraction (0..1)
###        6) Light savgol smoothing on the resulting (x, y) so the pull
###           doesn't introduce kinks
###        7) Validate corridor; invalid → return None (caller fallback)
def build_recovery_path(lifter, s0, n0, ego_x, ego_y, ego_yaw,
                         g_psi=None, g_s=None, g_x=None, g_y=None, g_z=None,
                         g_dleft=None, g_dright=None,
                         delta_s=8.0, n_samples=21,
                         wall_safe=0.15, wall_pull_thr=0.08,
                         wall_pull_alpha=0.6, smooth_window=5,
                         spline_scale=0.8, return_frenet=False):
    """Recovery_spliner-style smooth ego→GB path with wall-aware pull-in.

    Returns (xy_traj (n_samples, 3) [x, y, psi], frenet_traj (n_samples, 2)
    [s, n]) or (None, None) if invalid (can't avoid corridor punch).
    """
    try:
        from scipy.interpolate import BPoly
    except Exception:
        return (None, None) if return_frenet else None

    # ---- Pick lookahead target by tangent alignment ----
    # Use the lifter to get a forward-s lookahead point and its tangent.
    s_target = float(s0) + float(delta_s)
    try:
        x_t, y_t = lifter.sn_to_xy(s_target, 0.0)  # GB at lookahead
        psi_t = lifter._interp_psi(s_target)
    except Exception:
        return (None, None) if return_frenet else None

    # ---- BPoly cubic Hermite (2-point with tangent endpoints) ----
    # Tangent at start: ego heading direction.
    # Tangent at end: raceline psi at lookahead.
    # spline_scale tunes the tangent magnitude (lower → smoother but flatter).
    P_start = np.array([ego_x, ego_y], dtype=np.float64)
    P_end = np.array([x_t, y_t], dtype=np.float64)
    L = float(np.linalg.norm(P_end - P_start))
    if L < 1e-3:
        return (None, None) if return_frenet else None

    T_start = float(spline_scale) * L * np.array(
        [np.cos(ego_yaw), np.sin(ego_yaw)], dtype=np.float64)
    T_end = float(spline_scale) * L * np.array(
        [np.cos(psi_t), np.sin(psi_t)], dtype=np.float64)
    points_x = np.array([[P_start[0], T_start[0]],
                         [P_end[0], T_end[0]]], dtype=np.float64)
    points_y = np.array([[P_start[1], T_start[1]],
                         [P_end[1], T_end[1]]], dtype=np.float64)
    bp_x = BPoly.from_derivatives([0.0, 1.0], points_x)
    bp_y = BPoly.from_derivatives([0.0, 1.0], points_y)

    # Sample uniformly along the parametric curve. We then resample to
    # uniform arc-length (so wall lookups work in s-domain consistently).
    K_dense = max(int(n_samples) * 4, 40)
    u = np.linspace(0.0, 1.0, K_dense)
    xy_dense = np.column_stack([bp_x(u), bp_y(u)])

    # Arc-length parameterise.
    diffs = np.diff(xy_dense, axis=0)
    seg = np.sqrt((diffs * diffs).sum(axis=1))
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(arc[-1])
    if total < 1e-3:
        return (None, None) if return_frenet else None
    arc_uni = np.linspace(0.0, total, int(n_samples))
    xy = np.column_stack([
        np.interp(arc_uni, arc, xy_dense[:, 0]),
        np.interp(arc_uni, arc, xy_dense[:, 1]),
    ])

    # Map each xy to (s_world, n) via lifter; need 3D-safe projection.
    # We approximate s by `s0 + arc_uni`; n by signed centerline-normal
    # offset of xy_sample from raceline at that s. This avoids 2D xy→s
    # ambiguity on overpass layers.
    n_arr = np.zeros(int(n_samples), dtype=np.float64)
    s_arr = np.zeros(int(n_samples), dtype=np.float64)
    for i in range(int(n_samples)):
        s_w = float(s0) + float(arc_uni[i])
        try:
            x_g, y_g = lifter.sn_to_xy(s_w, 0.0)
            psi_g = lifter._interp_psi(s_w)
        except Exception:
            return (None, None) if return_frenet else None
        nx, ny = -np.sin(psi_g), np.cos(psi_g)  # left-positive n direction
        # signed projection of (xy[i] - xy_g) onto (nx, ny)
        n_proj = float((xy[i, 0] - x_g) * nx + (xy[i, 1] - y_g) * ny)
        s_arr[i] = s_w
        n_arr[i] = n_proj

    # ---- Wall-aware pull-in ----
    # If a sample's |n| is within wall_pull_thr of the wall, pull it back
    # toward GB (n=0) by `wall_pull_alpha` of the violation. This nudges
    # the path away from the wall before sharp violations form.
    if g_dleft is None or g_dright is None:
        # Need lifter access for d_left / d_right
        g_dleft = getattr(lifter, 'g_dleft', None)
        g_dright = getattr(lifter, 'g_dright', None)
    pulled = False
    if g_dleft is not None and g_dright is not None:
        for i in range(int(n_samples)):
            s_w = s_arr[i]
            d_L = float(lifter._interp(s_w, g_dleft))
            d_R = float(lifter._interp(s_w, g_dright))
            n_lim_up = max(d_L - wall_safe, 0.05)   # safe wall position (left)
            n_lim_dn = -max(d_R - wall_safe, 0.05)  # safe wall position (right)
            n_i = n_arr[i]
            if n_i > n_lim_up - wall_pull_thr:
                # Within pull threshold of left wall — pull toward 0
                n_i_new = (1.0 - wall_pull_alpha) * n_i \
                          + wall_pull_alpha * (n_lim_up - wall_pull_thr)
                n_arr[i] = n_i_new
                pulled = True
            elif n_i < n_lim_dn + wall_pull_thr:
                n_i_new = (1.0 - wall_pull_alpha) * n_i \
                          + wall_pull_alpha * (n_lim_dn + wall_pull_thr)
                n_arr[i] = n_i_new
                pulled = True

    # ---- Smoothing (savgol on n_arr to remove pull-induced kinks) ----
    if pulled:
        try:
            from scipy.signal import savgol_filter
            win = int(smooth_window)
            if win % 2 == 0:
                win -= 1
            if win >= 3 and len(n_arr) >= win:
                # Don't smooth k=0 (ego's true position); pin via fix-up.
                n_orig0 = n_arr[0]
                n_arr = savgol_filter(n_arr, window_length=win, polyorder=2,
                                      mode='nearest')
                n_arr[0] = n_orig0
        except Exception:
            pass

    # ---- Re-lift xy from smoothed (s, n) ----
    out = np.zeros((int(n_samples), 3), dtype=np.float64)
    for i in range(int(n_samples)):
        s_w = s_arr[i]
        n_i = float(n_arr[i])
        x, y = lifter.sn_to_xy(s_w, n_i)
        psi = lifter._interp_psi(s_w)
        out[i, 0] = x
        out[i, 1] = y
        out[i, 2] = psi

    # ---- Validate against hard corridor ----
    # If after pull-in any sample STILL punches corridor (-(d_R-wall_safe)
    # to +(d_L-wall_safe)), declare invalid and let caller fall through.
    if g_dleft is not None and g_dright is not None:
        for i in range(1, int(n_samples)):
            s_w = s_arr[i]
            d_L = float(lifter._interp(s_w, g_dleft))
            d_R = float(lifter._interp(s_w, g_dright))
            n_lim_up = max(d_L - wall_safe, 0.05)
            n_lim_dn = -max(d_R - wall_safe, 0.05)
            n_i = float(n_arr[i])
            if n_i > n_lim_up + 1e-3 or n_i < n_lim_dn - 1e-3:
                return (None, None) if return_frenet else None

    sn = np.column_stack([s_arr, n_arr])
    if return_frenet:
        return out, sn
    return out
### HJ : end


def solve_quintic_coeffs(L, n0, n0_d, n0_dd, n1=0.0, n1_d=0.0, n1_dd=0.0):
    """Solve n(s) = c0..c5 over s∈[0,L] with full clamped BCs.

    Returns the 6-vector [c0, c1, c2, c3, c4, c5] with n(s) = Σ c_i s^i.
    The system is 6×6 and non-singular for L>0, so always returns a solution.
    """
    L = float(max(L, 1e-3))
    L2 = L * L
    L3 = L2 * L
    L4 = L3 * L
    L5 = L4 * L

    # n(0) = c0; n'(0) = c1; n''(0) = 2 c2; so:
    c0 = float(n0)
    c1 = float(n0_d)
    c2 = 0.5 * float(n0_dd)

    # Remaining 3 unknowns c3, c4, c5 from terminal BCs:
    #   n(L)   = c0 + c1 L + c2 L^2 + c3 L^3 + c4 L^4 + c5 L^5   = n1
    #   n'(L)  =        c1   + 2 c2 L + 3 c3 L^2 + 4 c4 L^3 + 5 c5 L^4 = n1_d
    #   n''(L) =               2 c2   + 6 c3 L   + 12 c4 L^2 + 20 c5 L^3 = n1_dd
    A = np.array([
        [L3,      L4,      L5],
        [3*L2,  4*L3,   5*L4],
        [6*L,  12*L2,  20*L3],
    ], dtype=np.float64)
    b = np.array([
        n1    - (c0 + c1 * L + c2 * L2),
        n1_d  - (c1 + 2 * c2 * L),
        n1_dd - (2 * c2),
    ], dtype=np.float64)

    c3, c4, c5 = np.linalg.solve(A, b)
    return np.array([c0, c1, c2, c3, c4, c5], dtype=np.float64)


def evaluate_poly(coeffs, s):
    """Horner-style eval of n(s) and n'(s) given 6-coeff vector."""
    c0, c1, c2, c3, c4, c5 = coeffs
    n = c0 + s * (c1 + s * (c2 + s * (c3 + s * (c4 + s * c5))))
    ndot = c1 + s * (2 * c2 + s * (3 * c3 + s * (4 * c4 + s * 5 * c5)))
    return float(n), float(ndot)


def build_quintic_fallback(lifter, s0, n0, psi_delta, delta_s=8.0, n_samples=21,
                           n0_d_hint=None, return_frenet=False):
    """### HJ : Phase 4 Tier-2 entry.

    Parameters
    ----------
    lifter : MPCRacelineLifter
        Provides raceline interpolation + sn_to_xy (raceline-base, matches
        every other consumer downstream of /global_waypoints).
    s0 : float
        Current ego s on raceline. In 3D tracks this MUST come from
        `/car_state/odom_frenet` (z-aware); NEVER from
        `lifter.project_xy_to_sn(car_x, car_y)` which is 2D-only and
        aliases overpass layers.
    n0 : float
        Current ego n offset (from the same 3D Frenet source as s0).
    psi_delta : float
        Ego heading − raceline tangent at s0. Used to set n'(0). Wrapped
        to (-π, π] by caller.
    delta_s : float
        Forward arc length over which to return to the raceline. 8 m ≈ 1.5×
        MPC horizon at 10 m/s with N=20, dT=0.05.
    n_samples : int
        Points to sample along the curve. 21 ≈ MPC horizon + 1.
    n0_d_hint : float, optional
        Override for n'(0). If None, derived from psi_delta via tan(psi_δ).
    return_frenet : bool, default False
        If True, also returns a parallel Frenet (s_world, n) array so the
        node can feed it to `fill_wpnt_from_s` and skip the 2D xy→s round
        trip when publishing Wpnt fields.

    Returns
    -------
    np.ndarray, shape (n_samples, 3) — Cartesian [x, y, psi_tangent].
    If return_frenet:  (xy_traj, np.ndarray shape (n_samples, 2) [s, n]).
    """
    # Small-angle approx ok; explicit tan keeps large psi_delta honest.
    if n0_d_hint is None:
        # Clamp to avoid extreme slopes if ego is pointing 90° off.
        psi_clamped = float(np.clip(psi_delta, -1.0, 1.0))  # ~57°
        n0_d = float(np.tan(psi_clamped))
    else:
        n0_d = float(n0_d_hint)

    coeffs = solve_quintic_coeffs(delta_s, n0, n0_d, 0.0, 0.0, 0.0, 0.0)

    out = np.zeros((n_samples, 3), dtype=np.float64)
    sn = np.zeros((n_samples, 2), dtype=np.float64)
    s_grid = np.linspace(0.0, float(delta_s), n_samples)
    for i, ds in enumerate(s_grid):
        n_i, _ = evaluate_poly(coeffs, ds)
        s_world = s0 + ds
        x, y = lifter.sn_to_xy(s_world, n_i)
        psi = lifter._interp_psi(s_world)
        out[i, 0] = x
        out[i, 1] = y
        out[i, 2] = psi
        sn[i, 0] = s_world
        sn[i, 1] = n_i
    if return_frenet:
        return out, sn
    return out
