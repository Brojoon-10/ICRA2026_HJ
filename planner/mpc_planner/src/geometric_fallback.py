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
                           n0_d_hint=None):
    """### HJ : Phase 4 Tier-2 entry.

    Parameters
    ----------
    lifter : MPCRacelineLifter
        Provides raceline interpolation + sn_to_xy (raceline-base, matches
        every other consumer downstream of /global_waypoints).
    s0 : float
        Current ego s on raceline (from `lifter.project_xy_to_sn`).
    n0 : float
        Current ego n offset (same projection).
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

    Returns
    -------
    np.ndarray, shape (n_samples, 3)
        Columns = [x, y, psi_tangent]. psi is track tangent at that s, so
        the caller can blend with a desired heading if needed (MPC lifter
        uses a similar convention).
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
    s_grid = np.linspace(0.0, float(delta_s), n_samples)
    for i, ds in enumerate(s_grid):
        n_i, _ = evaluate_poly(coeffs, ds)
        s_world = s0 + ds
        x, y = lifter.sn_to_xy(s_world, n_i)
        # Raceline tangent at s for Wpnt psi. Ignores the small rotation
        # contributed by ndot — acceptable for Tier-2 fallback quality.
        psi = lifter._interp_psi(s_world)
        out[i, 0] = x
        out[i, 1] = y
        out[i, 2] = psi
    return out
