"""Plan Library — Stage 2 Phase 2-1.

각 plan 은 solver weights overlay 를 정의. _apply_mode_weights 결과
위에 덮어써서 (selective override) plan-specific 동작을 구현.

5종 plan:
  - LEFT_PASS / RIGHT_PASS — 능동적 추월. side_bias 강화.
  - TRAIL — 차 뒤 + 일정 거리/속도 + 빈틈 노림 (사용자 정정).
            side_bias=0, q_n_term/q_n_ramp 약화 (ego_n 유지).
  - RACELINE — obstacle 없을 때 GB 복귀.
  - BETWEEN — 정+동 사이 통과 (sim 에 정적 없으므로 보류, side_int 매핑 X).

pick_plan(side_int, obs_in_horizon) → plan dict
"""
from __future__ import annotations

# side_int 상수 (side_decider 와 동일)
SIDE_CLEAR = 0
SIDE_LEFT = 1
SIDE_RIGHT = 2
SIDE_TRAIL = 3


# Plan 정의: name, side_bias_dir (LEFT 양수 / RIGHT 음수), weight overlay dict
# overlay dict 의 key 는 solver.set_weight() 또는 solver 의 yaml-mapped weight 명.
# None 이면 yaml/_apply_mode_weights 의 baseline 그대로.
PLAN_LEFT_PASS = {
    'name': 'left_pass',
    'side_int_match': SIDE_LEFT,
    'weights': {
        'w_side_bias': 60.0,    # Phase 2-2: side bias 약화 (target_n profile 주 가이드)
        'w_obs':       50.0,    # (d) N=40 stage 비례 + target_n 일부 줄임
        'sigma_n_obs': 0.60,
        'q_n_term':    10.0,
        'q_n':         1.5,     # (d) N=40 stage 비례
        'q_n_ramp':    7.0,
        'w_cont':      300.0,
        'gamma_progress': 13.0,
        'q_n_target':  8.0,     # Phase 2-2 — plan-aware target_n cost
    },
}

PLAN_RIGHT_PASS = {
    'name': 'right_pass',
    'side_int_match': SIDE_RIGHT,
    'weights': {
        'w_side_bias': 60.0,
        'w_obs':       50.0,
        'sigma_n_obs': 0.60,
        'q_n_term':    10.0,
        'q_n':         1.5,
        'q_n_ramp':    7.0,
        'w_cont':      300.0,
        'gamma_progress': 13.0,
        'q_n_target':  8.0,
    },
}

# TRAIL plan — 사용자 정정 정확 반영:
#   "차 뒤에서 잘 따라가다 빈틈 노림"
#   side_bias = 0 (lateral 정렬 X)
#   q_n_term/q_n_ramp 약화 (ego_n 유지, raceline 끌어당김 X)
#   w_obs 약화 (obstacle bubble 이 ego_n 을 obstacle 반대로 푸시하는 효과 줄임)
#   w_cont 강화 (tick-to-tick continuity 강하게 → ego_n 자연 유지)
PLAN_TRAIL = {
    'name': 'trail',
    'side_int_match': SIDE_TRAIL,
    'weights': {
        'w_side_bias': 0.0,
        'w_obs':       25.0,    # (d) N=40 stage 비례
        'sigma_n_obs': 0.40,
        'q_n_term':    1.5,     # (d) N=40 stage 비례
        'q_n':         0.5,     # (d) N=40 stage 비례
        'q_n_ramp':    0.0,
        'w_cont':      250.0,
        'gamma_progress': 8.0,
        'q_n_target':  15.0,    # Phase 2-2 — TRAIL 시 ego_n 유지 강력
    },
}

PLAN_RACELINE = {
    'name': 'raceline',
    'side_int_match': SIDE_CLEAR,
    'weights': {
        'w_side_bias': 0.0,
        'w_obs':       0.0,
        'sigma_n_obs': 0.45,
        'q_n_term':    15.0,
        'q_n':         2.5,     # (d) N=40 stage 비례
        'q_n_ramp':    4.0,     # (d) N=40 stage 비례
        'w_cont':      300.0,
        'gamma_progress': 16.0,
        'q_n_target':  5.0,     # Phase 2-2 — target_n=0 (raceline)
    },
}


def pick_plan(side_int, obs_in_horizon):
    """Side decision + obstacle 유무로 plan 선택.

    Args:
      side_int: side_decider 의 결정 (SIDE_LEFT/RIGHT/TRAIL/CLEAR)
      obs_in_horizon: bool — obstacle 이 horizon 안에 있는지

    Returns:
      plan dict (name, side_int_match, weights)
    """
    if side_int == SIDE_LEFT:
        return PLAN_LEFT_PASS
    if side_int == SIDE_RIGHT:
        return PLAN_RIGHT_PASS
    if side_int == SIDE_TRAIL:
        return PLAN_TRAIL
    # SIDE_CLEAR
    if obs_in_horizon:
        # CLEAR + obstacle in horizon = decider 가 결정 안 함 / mode 전환 중.
        # 현 ego_n 유지 = TRAIL plan 으로 처리 (안전 우선).
        return PLAN_TRAIL
    return PLAN_RACELINE


def make_target_n_profile(plan, ego_n, obs_n, N):
    """Plan 의 target_n profile (length N+1) 계산.

    Args:
      plan: PLAN_LEFT_PASS / RIGHT_PASS / TRAIL / RACELINE
      ego_n: ego 의 현재 lateral 위치
      obs_n: closest obstacle 의 lateral 위치 (없으면 None)
      N: horizon stages

    Returns:
      list of N+1 floats. plan 별:
        LEFT_PASS:  k=0 ego_n → k=N/2 (obs_n + 0.5m) → k=N raceline (0)
                    -> 점진적 회피 후 raceline 복귀
        RIGHT_PASS: 대칭 (obs_n - 0.5m)
        TRAIL:      모든 k = ego_n (lateral 유지, 사용자 정정)
        RACELINE:   모든 k = 0 (raceline)
    """
    name = plan['name']
    if name == 'trail':
        # 사용자 정정: "차 뒤 + 일정 거리/속도, lateral 정렬 X"
        return [ego_n] * (N + 1)
    if name == 'raceline':
        return [0.0] * (N + 1)
    # LEFT_PASS / RIGHT_PASS — 점진적 회피 + raceline 복귀
    if obs_n is None:
        # obstacle 정보 없으면 raceline 으로 fallback
        return [ego_n + (0.0 - ego_n) * (k / float(N)) for k in range(N + 1)]
    if name == 'left_pass':
        # obstacle 의 LEFT (n>obs_n+0.5m) 으로 우회 후 raceline 복귀
        avoid_n = obs_n + 0.5
    elif name == 'right_pass':
        avoid_n = obs_n - 0.5
    else:
        avoid_n = 0.0
    profile = []
    for k in range(N + 1):
        # k=0 ego_n, k=N/2 avoid_n (peak), k=N raceline (0)
        half = N // 2
        if k <= half:
            t = k / float(max(half, 1))
            target = ego_n + (avoid_n - ego_n) * t
        else:
            t = (k - half) / float(max(N - half, 1))
            target = avoid_n + (0.0 - avoid_n) * t
        profile.append(target)
    return profile


def plan_ot_line(plan):
    """Plan name 을 OTWpntArray.ot_line 으로 변환.

    SM 측 호환을 위해 'avoid' / 'recover' 도 매핑 가능하나, 본 Phase 2-1
    에서는 plan name 그대로 발행 (5종 ot_line 확장은 Phase 2-3).
    """
    return plan['name']
