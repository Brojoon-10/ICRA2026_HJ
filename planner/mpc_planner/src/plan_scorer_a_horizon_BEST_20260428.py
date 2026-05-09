"""Plan Scorer — Stage 2 Phase 2-2 (대기 중, 30min 검증 후 적용 결정).

Heuristic scoring. NLP 풀지 않고 빠르게 plan 별 적합도 평가.

cost_components (사용자 정정 반영):
  - race_progress_gain: 추월 후 ahead 가치. LEFT/RIGHT_PASS 큼, TRAIL 작음 (= v_obs).
  - collision_risk: corridor d_free 부족 시 위험. 모든 plan 평가.
  - maneuver_quality: kappa, lateral jerk 추정. plan 의 target_n_profile 부드러움.
  - long_term_value: TRAIL 의 strategic value (다음 추월 기회 평가, 사용자 정정).
  - commit_sticky: 현재 plan 유지 우선 (hysteresis).

best_plan = argmin sum(α·components).

Phase 2-1 은 SideDecider 결과 그대로 사용. Phase 2-2 는 plan_scorer 가 SideDecider
결과에 추가 평가 (또는 override). 본 모듈은 Phase 2-2 적용 시 활용.
"""
from __future__ import annotations
import math


def score_plan(plan, ego_n, side_scores, obs_in_horizon,
               current_plan_name=None, sticky_bonus=0.30):
    """Plan 의 cost score 계산. 낮을수록 좋음.

    Args:
      plan: PLAN_LEFT_PASS / RIGHT_PASS / TRAIL / RACELINE
      ego_n: ego 의 lateral 위치
      side_scores: SideDecider 의 score dict (d_free_L, d_free_R, dv 등)
      obs_in_horizon: bool
      current_plan_name: 현재 진행 중 plan name (sticky bonus 용)
      sticky_bonus: 0~1. 현재 plan 에 대한 cost 감소 비율.

    Returns:
      (score, breakdown_dict)
    """
    name = plan['name']
    d_free_L = float(side_scores.get('d_free_L', 0.0))
    d_free_R = float(side_scores.get('d_free_R', 0.0))
    dv = float(side_scores.get('dv', 0.0))
    v_obs = float(side_scores.get('v_obs', 0.0))

    # Race progress gain (negative cost = good)
    if name in ('left_pass', 'right_pass'):
        progress_gain = 5.0  # 강한 +
    elif name == 'trail':
        progress_gain = max(0.5, v_obs * 0.3)  # opp 속도에 비례 (작은 +)
    elif name == 'raceline':
        progress_gain = 8.0 if not obs_in_horizon else 0.0  # obstacle 없을 때 best
    else:
        progress_gain = 0.0

    # Collision risk (positive cost = bad). Sweet-spot tuned: lateral 거리 부족
    # 시 risk 강화하여 LEFT/RIGHT_PASS 가 안전하지 않으면 TRAIL 이 우선되도록.
    if name == 'left_pass':
        # d_free<0.15 매우 위험, 0.30 일반, 0.45+ 안전
        if d_free_L < 0.15:
            risk = 50.0   # 매우 큰 risk (TRAIL 보다 큼)
        elif d_free_L < 0.30:
            risk = max(0.0, 0.3 - d_free_L) * 30.0
        else:
            risk = 0.0
    elif name == 'right_pass':
        if d_free_R < 0.15:
            risk = 50.0
        elif d_free_R < 0.30:
            risk = max(0.0, 0.3 - d_free_R) * 30.0
        else:
            risk = 0.0
    elif name == 'trail':
        risk = 0.5   # 차 뒤 안전, 작은 risk
    elif name == 'raceline':
        risk = 0.0 if not obs_in_horizon else 30.0
    else:
        risk = 5.0

    # Maneuver quality (positive cost). LEFT/RIGHT_PASS 시 ego_n 과 target_n 차이 큼 → 큰 lateral move.
    if name == 'left_pass':
        lateral_move = abs(ego_n - 0.4)  # rough target_n=+0.4
        maneuver = lateral_move * 0.5
    elif name == 'right_pass':
        lateral_move = abs(ego_n - (-0.4))
        maneuver = lateral_move * 0.5
    else:
        maneuver = 0.1

    # Long-term value — TRAIL 의 "빈틈 노림" 가치. 사용자 정정 정확 반영:
    #   TRAIL = "차 뒤 + 일정 거리/속도, 빈틈 노림" — closure 부족 + 회피 risky 시 best.
    #   dv 작고 risk 큼 → TRAIL 가치 큼.
    #   dv 크고 risk 작음 → LEFT/RIGHT_PASS 가치 큼 (직접 추월).
    if name == 'trail':
        # dv 작거나 LEFT/RIGHT 모두 risk 있으면 trail 우호적.
        # d_free_min < 0.2 → trail 가치 +
        d_free_min = min(d_free_L, d_free_R) if (d_free_L != 0 or d_free_R != 0) else 0.5
        long_term_value = max(0.0, 1.5 - dv) + max(0.0, 0.3 - d_free_min) * 5.0
    elif name in ('left_pass', 'right_pass'):
        # dv 크면 추월 가치 큼 (negative cost = good).
        long_term_value = -max(0.0, dv - 0.5) * 0.7
    else:
        long_term_value = 0.0

    # Total (낮을수록 좋음)
    score = -progress_gain + risk + maneuver + long_term_value

    # Sticky bonus (현재 plan 유지 우선)
    if current_plan_name == name:
        score *= (1.0 - sticky_bonus)

    # ego_n 기반 commit bonus — best 발견 (3.0).
    if abs(ego_n) > 0.1:
        if ego_n > 0.1 and name == 'left_pass':
            score -= 3.0 * min(1.0, (ego_n - 0.1) / 0.3)
        elif ego_n < -0.1 and name == 'right_pass':
            score -= 3.0 * min(1.0, (-ego_n - 0.1) / 0.3)

    breakdown = {
        'progress_gain': round(progress_gain, 2),
        'risk':          round(risk, 2),
        'maneuver':      round(maneuver, 2),
        'long_term':     round(long_term_value, 2),
        'sticky':        (current_plan_name == name),
        'total':         round(score, 2),
    }
    return score, breakdown


def horizon_aware_d_free(obs_arr, ref_slice, ego_half=0.15, gap_lat=0.30, k_pass=None):
    """Horizon-aware d_free_L/R (사용자 명시 (1) — 진정한 근본).

    snapshot d_free 가 ego ≈ obs 시 ambiguous (large swing). horizon 의 min
    d_free 사용 → 안정. plan_scorer 의 input 자체를 horizon-aware 로.

    Args:
      obs_arr: (n_obs_max, N+1, 3) [s, n, w_active]
      ref_slice: dict with d_left_arr, d_right_arr (length N+1)

    Returns:
      (min_d_free_L, min_d_free_R) — horizon 안 worst-case (safety conservative)
    """
    import numpy as np
    if obs_arr is None or obs_arr.size == 0:
        return 1.0, 1.0  # obstacle 없음 = 충분히 free
    K = obs_arr.shape[1]
    if k_pass is None:
        k_pass = K
    d_lefts = ref_slice.get('d_left_arr', None)
    d_rights = ref_slice.get('d_right_arr', None)
    if d_lefts is None or d_rights is None:
        return 1.0, 1.0
    min_L = float('inf')
    min_R = float('inf')
    any_active = False
    for o in range(obs_arr.shape[0]):
        if obs_arr[o, 0, 2] <= 0: continue
        any_active = True
        for k in range(min(k_pass, K)):
            obs_n = float(obs_arr[o, k, 1])
            d_L = float(d_lefts[k]) if k < len(d_lefts) else d_lefts[-1]
            d_R = float(d_rights[k]) if k < len(d_rights) else d_rights[-1]
            # LEFT corridor 의 free space = (d_L - ego_half) - (obs_n + obs_half + gap)
            obs_half = 0.20  # default
            d_free_L = (d_L - ego_half) - (obs_n + obs_half + gap_lat)
            d_free_R = (-(-d_R) - ego_half) - (-obs_n + obs_half + gap_lat)  # 대칭
            d_free_R = (d_R - ego_half) - (-obs_n + obs_half + gap_lat)
            if d_free_L < min_L: min_L = d_free_L
            if d_free_R < min_R: min_R = d_free_R
    if not any_active:
        return 1.0, 1.0
    return min_L, min_R


def pick_plan_scored(candidates, ego_n, side_scores, obs_in_horizon,
                     current_plan_name=None,
                     score_history=None,
                     score_gap_threshold=0.15):
    """후보 plan 들 중 최저 score 선택. 근원 진동 차단:
       - Score EMA (지난 tick 50% blend) — d_free 변동 흡수
       - Score gap threshold 15% — current 보다 best 가 충분히 좋아야 변경

    Args:
      score_history: dict 또는 None. {plan_name: prev_score} EMA 누적.
      score_gap_threshold: best 가 current 보다 (1-thr) 비율 이상 좋아야 change.

    Returns:
      (best_plan, all_scores_dict, new_score_history)
    """
    all_scores = {}
    raw_scores = {}
    for plan in candidates:
        s, bd = score_plan(plan, ego_n, side_scores, obs_in_horizon,
                           current_plan_name=current_plan_name)
        raw_scores[plan['name']] = s
        all_scores[plan['name']] = bd

    # Score EMA — 진동 흡수
    if score_history is None:
        score_history = {}
    new_history = {}
    smoothed = {}
    for name, s in raw_scores.items():
        prev = score_history.get(name, s)
        smoothed[name] = 0.5 * prev + 0.5 * s
        new_history[name] = smoothed[name]
        all_scores[name]['smoothed'] = round(smoothed[name], 2)

    # Best plan = lowest smoothed score
    best_name = min(smoothed, key=smoothed.get)
    best_plan = next(p for p in candidates if p['name'] == best_name)

    # Score gap threshold: current plan 이 후보에 있으면 그것이 충분히 나쁘지
    # 않으면 유지. (best 의 score 가 current 의 (1-thr) 배 이상 좋아야 change)
    if current_plan_name is not None and current_plan_name in smoothed:
        cur_score = smoothed[current_plan_name]
        best_score = smoothed[best_name]
        # score 양수면 작을수록 좋음. cur 0 또는 음수면 abs 비교 필요
        if cur_score > 0:
            if best_score > cur_score * (1.0 - score_gap_threshold):
                # gap 부족 → current 유지
                cur_plan = next((p for p in candidates if p['name'] == current_plan_name), None)
                if cur_plan is not None:
                    best_plan = cur_plan
        else:
            # cur_score <= 0: best 가 cur 보다 더 음수 (예: -1.5 vs -1.0) 이려면 충분히 작아야
            if best_score > cur_score - abs(cur_score) * score_gap_threshold:
                cur_plan = next((p for p in candidates if p['name'] == current_plan_name), None)
                if cur_plan is not None:
                    best_plan = cur_plan

    return best_plan, all_scores, new_history
