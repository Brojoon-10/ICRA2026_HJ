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
        'q_n_target':  15.0,    # 8→15: bag analysis showed ego rarely reached
                                #         avoid_n target (PASS collisions during
                                #         alongside moments). Stronger target
                                #         tracking pulls ego to avoid_n earlier.
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
        'q_n_target':  15.0,    # 8→15 (matching LEFT_PASS)
    },
}

# TRAIL plan — 2026-05-02 (사용자 본질 지적 반영):
#   "MPC 의 본업 = 시간 함수 trajectory 풀이. TRAIL 은 obs 를 인지하면서 따라가기.
#    lateral repulsion (w_obs Gaussian bubble) 은 trail 의도 정반대 — ego 가
#    obs 옆으로 빠지려 함 → corridor 위반 → infeasibility. TRAIL 의 obs 인지는
#    종방향 ref_v cap (painter / Phase 4) 으로 처리해야 함. NLP 안에서는 lateral
#    cost off + corridor wall constraint + progress + smoothness 만으로 풀이 →
#    항상 feasible 보장. left/right 어느 쪽 trailing 이든 NLP 가 자유 풀이."
PLAN_TRAIL = {
    'name': 'trail',
    'side_int_match': SIDE_TRAIL,
    'weights': {
        'w_side_bias': 0.0,
        'w_obs':       0.0,     # 2026-05-02: lateral repulsion 폐기.
        'sigma_n_obs': 0.40,
        ### HJ : 2026-05-02 (분석 19-04-06) — trajectory shape 강제.
        ###      이전 q_n=0.5 / q_n_term=1.5 가 너무 작아 큰 곡률 + smoothness +
        ###      progress maximization 이 trajectory 를 corridor 밖 -2.5m 로 push.
        ###      14건 infeas 모두 trajectory n_max>2 또는 n_min<-2 로 corridor
        ###      위반. q_n / q_n_term 강화 → trajectory raceline 근처 강제.
        'q_n_term':    8.0,     # 1.5 → 8.0
        'q_n':         5.0,     # 0.5 → 5.0
        'q_n_ramp':    0.0,
        'w_cont':      250.0,
        'gamma_progress': 4.0,  # 8.0 → 4.0 (progress 욕심 줄임 — 곡률 코너 안전)
        'q_n_target':  0.0,
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

    (b) fine-tune: avoid_n margin 0.5→0.7m (overlap 줄임), peak 위치 N/2→0.6N
    (later peak — obstacle 통과 후 빨리 raceline 복귀, jitter 줄임).

    plan 별:
      LEFT_PASS:  k=0 ego_n → k=0.6N (obs_n + 0.7m) → k=N raceline (0)
      RIGHT_PASS: 대칭 (obs_n - 0.7m)
      TRAIL:      모든 k = ego_n
      RACELINE:   모든 k = 0
    """
    name = plan['name']
    if name == 'trail':
        # Lateral push tried (long_trail_lat 2026-04-28): TRAIL collisions
        # dropped from 6→3 but PASS-plan collisions rose 3→7 because the
        # extra lateral motion triggered more PASS transitions while the
        # solver was still mid-tracking. Net 12→13 — kept the simple form.
        return [ego_n] * (N + 1)
    if name == 'raceline':
        return [0.0] * (N + 1)
    if obs_n is None:
        return [ego_n + (0.0 - ego_n) * (k / float(N)) for k in range(N + 1)]
    if name == 'left_pass':
        avoid_n = obs_n + 0.55   # best (lateral min 216mm 유지)
    elif name == 'right_pass':
        avoid_n = obs_n - 0.55
    else:
        avoid_n = 0.0
    profile = []
    peak_idx = int(0.6 * N)  # (b) N/2 → 0.6N (obstacle 옆 통과 후 빨리 복귀)
    for k in range(N + 1):
        if k <= peak_idx:
            t = k / float(max(peak_idx, 1))
            target = ego_n + (avoid_n - ego_n) * t
        else:
            t = (k - peak_idx) / float(max(N - peak_idx, 1))
            target = avoid_n + (0.0 - avoid_n) * t
        profile.append(target)
    return profile


def plan_ot_line(plan):
    """Plan name 을 OTWpntArray.ot_line 으로 변환.

    SM 측 호환을 위해 'avoid' / 'recover' 도 매핑 가능하나, 본 Phase 2-1
    에서는 plan name 그대로 발행 (5종 ot_line 확장은 Phase 2-3).
    """
    return plan['name']
