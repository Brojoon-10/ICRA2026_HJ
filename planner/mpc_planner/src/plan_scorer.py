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

    # Collision risk: smooth quadratic ramp (NO cliff). Tracking noise on d_free
    # → continuous score change, not a jump. Coefficient tuned so d_free=0.00
    # yields risk=50 (matches old cliff magnitude).
    #   d_free >= 0.30 → 0
    #   d_free = 0.15 → ~12.5
    #   d_free = 0.00 → ~50
    if name == 'left_pass':
        deficit_L = max(0.0, 0.30 - d_free_L)
        risk = deficit_L * deficit_L * 555.6
    elif name == 'right_pass':
        deficit_R = max(0.0, 0.30 - d_free_R)
        risk = deficit_R * deficit_R * 555.6
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

    # Sticky bonus: always make current plan MORE attractive (lower score).
    # Bug fix 2026-04-28: previous `score *= (1 - sticky_bonus)` was incorrect
    # for negative scores — it pulled them toward zero (worse). Plan totals are
    # typically negative due to progress_gain dominance, so the old form
    # actively penalised the current plan and triggered single-tick noise to
    # cascade through EMA into rapid flips.
    if current_plan_name == name:
        if score >= 0.0:
            score *= (1.0 - sticky_bonus)
        else:
            score -= abs(score) * sticky_bonus

    # ego_n 기반 commit bonus — already-committed pass plan more attractive.
    # (TRAIL penalty variant tried and removed: in 5x60s aggregate trials it
    # did not consistently reduce collisions, so the simpler form stays.)
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


def horizon_aware_d_free(obs_arr, ref_slice, ego_half=0.15,
                         gap_lat_base=0.30, gap_lat_growth=0.0,
                         dT=0.05, vd_var=0.0, obs_n_std=0.0,
                         k_pass=None):
    """Horizon-aware d_free_L/R: min over k=0..k_pass of corridor margin.

    Adaptive gap_lat (per user feedback "토픽 상태 기반 유동적"). The margin at
    k=0 stays at gap_lat_base (trust current measurement, do not inflate
    constantly). Beyond k=0 the margin grows with predicted opponent variance:

        gap_lat(k) = gap_lat_base + growth_eff(state) · (k · dT)

    where growth_eff is derived from runtime topic state — prediction variance
    (vd_var from /opponent_prediction/obstacles) and recent obs.n stddev. This
    makes the planner conservative *only* when the data says it should be.

    Args:
      gap_lat_base: lateral margin at k=0 (current measurement — no inflation).
      gap_lat_growth: optional fixed minimum growth rate (m / sec_horizon).
                     Set to 0 to be fully adaptive; small positive value gives
                     a floor for safety when variance reporting is unreliable.
      dT: horizon step duration (s).
      vd_var: predicted lateral velocity variance from predictor (m²/s²).
              gap added by k = sqrt(vd_var) · k·dT (1-σ growth).
      obs_n_std: recent obs.n rolling stddev (m). Acts as a lower bound on
                 growth_eff in case predictor reports near-zero variance.
    """
    import numpy as np
    if obs_arr is None or obs_arr.size == 0:
        return 1.0, 1.0
    K = obs_arr.shape[1]
    if k_pass is None:
        k_pass = K
    d_lefts = ref_slice.get('d_left_arr', None)
    d_rights = ref_slice.get('d_right_arr', None)
    if d_lefts is None or d_rights is None:
        return 1.0, 1.0
    # Adaptive growth rate (m / sec) from topic state. Uses 1-σ of predicted
    # lateral motion: σ_vd · t = √vd_var · k·dT. A rolling obs.n stddev floor
    # protects against predictors that under-report variance. Total per-second
    # ramp: max(√vd_var, obs_n_std/T_h, fixed_floor_per_sec).
    T_h = max(K - 1, 1) * dT
    sigma_vd = math.sqrt(max(vd_var, 0.0))
    growth_per_sec = max(sigma_vd,
                         obs_n_std / T_h if T_h > 0 else 0.0,
                         gap_lat_growth)
    # Cap to avoid pathological inflation when sensor goes haywire.
    growth_per_sec = min(growth_per_sec, 0.50)

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
            obs_half = 0.20
            gap_lat_k = gap_lat_base + growth_per_sec * (k * dT)
            d_free_L = (d_L - ego_half) - (obs_n + obs_half + gap_lat_k)
            d_free_R = (d_R - ego_half) - (-obs_n + obs_half + gap_lat_k)
            if d_free_L < min_L: min_L = d_free_L
            if d_free_R < min_R: min_R = d_free_R
    if not any_active:
        return 1.0, 1.0
    return min_L, min_R


def filter_feasible_plans(candidates, ego_n, commit_threshold=0.15):
    """Drop plans that ego cannot physically realize from current lateral commit.

    Root-cause for LEFT/RIGHT plan flips: the scorer compares both passes even when
    ego is already lateral-committed. From ego_n=+0.30 to RIGHT_PASS target_n=-0.40
    is a 0.70m cross within one horizon — not realizable, so a flip is just
    score-noise, not a strategic switch.

    This is NOT hysteresis (no time-locking, no state). It removes only the
    physically infeasible candidates; LEFT↔TRAIL, RACELINE↔LEFT, etc. stay open.

    Args:
      ego_n: ego's lateral position (positive = LEFT side).
      commit_threshold: |ego_n| above this → opposite-side pass dropped.
    """
    if ego_n > commit_threshold:
        return [p for p in candidates if p.get('name') != 'right_pass']
    if ego_n < -commit_threshold:
        return [p for p in candidates if p.get('name') != 'left_pass']
    return candidates


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
