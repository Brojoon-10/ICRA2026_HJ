---
name: Strategic Overtake Vision
description: 모드는 명시(TRAILING/OVERTAKE), 전이는 전략(intent gradient + 누적 활용). 마감 무관 본질 설계.
type: design
date: 2026-05-01
supersedes: 없음 (dynamic_overtake_master_plan_20260429.md 의 보완 / 상위 비전)
---

# Strategic Overtake Vision

> **한 줄 비전**: TRAILING / OVERTAKING 모드는 **그대로 명시 분리**. 그 사이의 전이를 **strategic intent gradient + 누적 observation** 으로 결정해서, "trailing 풀리고 냅다 가속" 의 불안정함 없이 자연스러운 PASS 가 emerging behavior 로 나오게 한다.

---

## 다음 세션 첫 5분 복귀 가이드

- 이 문서는 ICRA 마감 무관 **본질 설계**. 마감 안 구현 가능한 부분은 §10 Phase 로드맵의 단기/중기 표시 참조.
- 단기 (마감 안): Phase 1~6 (안전 baseline, `dynamic_overtake_master_plan_20260429.md` 참조).
- 중기 (마감 후 또는 여유 시): Phase 7~11 (이 문서의 strategic layer).
- **모드는 분리, 전이는 의도** 가 본 문서의 핵심 thesis.

---

## 1. 사용자 directive (불변, 6개)

`dynamic_overtake_master_plan_20260429.md` §0 와 동일. 요약:
1. **충돌 0** (절대값, 통계 아님)
2. **MPC 경로 항상 publish**
3. **동적 우선**
4. **Strategic plan** (reactive 토글 금지)
5. **Prediction 정확 활용**
6. **안전 buffer 명시**

본 비전은 위 directive 의 **상위 추상화**. 특히 #4 (strategic) 와 #5 (prediction) 의 의미를 보다 야심차게 해석한 결과.

---

## 2. 현재 구조의 본질 결함

### 2.1 Discrete mode switching 의 한계

현재 SM 의 mode 전이:
```
TRAILING:  trailing PID, ego.v ≈ obs.vs        (거리 유지)
OVERTAKE:  trailing 해제 → ref_v 풀린 raceline (가속)
TRAILING ↔ OVERTAKE 전이: 1-tick reactive 결정
```

이 구조의 두 가지 결함:

**(a) PASS 시도의 가속 점프**
- TRAILING 동안 ego.v = obs.vs ≈ 5 m/s.
- OVERTAKE 진입 순간 ref_v = raceline 7 m/s → 1-tick 안에 +2 m/s 지령.
- Controller PID 가 throttle full → 차량 거동 불안정 (drift, wall hit).
- 사용자 표현: "trailing 풀리고 냅다 가속을 해야하는데 그럼 너무 불안정하잖아".

**(b) 의사결정의 1-tick 반응**
- "지금 옆이 비었나?" 라는 즉석 판단으로 PASS 결정.
- 의지적·전략적 사전 계획 없음. 운 좋으면 성공, 운 나쁘면 충돌.
- 사용자 표현: "그냥 미친듯이 항상 OT를 하려고 하다가 차나 벽이랑 충돌".

### 2.2 그러나 모드 분리 자체는 옳음

모드를 합치자는 게 아님. TRAILING / OVERTAKE 분리는 다음 이유로 옳음:
- **안전성**: OVERTAKE 의 lateral commitment 는 별개의 corridor / obstacle bubble 처리 필요. trailing 거리 유지와 동일한 controller 로 처리하면 어느 한쪽이 깨짐.
- **명확성**: 디버깅 / 로깅 / RViz 시각화 모두 모드 구분이 있어야 추적 가능.
- **컨트롤 분리**: trailing PID 와 raceline tracking 의 gain 이 다름. mode 별 별도 튜닝.

### 2.3 따라서 진짜 문제는 "전이의 의사결정"

> 모드 자체는 그대로 두고, 모드 사이의 전이가 **strategic + 사전 계획** 으로 결정되어야 한다.

이게 본 비전의 thesis.

---

## 3. 핵심 설계 원칙

### 3.1 "모드는 명시, 의도는 연속"

- **Mode (discrete)**: `{ GB_TRACK, TRAILING, OVERTAKE, RECOVERY }` — 그대로.
- **Intent (continuous)**: `e ∈ [0, 1]` — 매 tick 의지의 강도.
  - `e=0`: PASS 의지 없음 (그저 따라감).
  - `e=0.3`: 거리 좁힘 시작.
  - `e=0.7`: line 옆 prepositioning.
  - `e=1.0`: 진입 commit.
- **Mode 전이 규칙**: `e` 가 임계값 통과 + safety guard 충족 시.
- **Hysteresis**: 한 번 e≥0.8 → e≥0.5 유지 (commit 해제 지연).

### 3.2 TRAILING 안에 사전 단계 도입

TRAILING 모드를 두 sub-state 로 분할 — **외부에서는 여전히 TRAILING**, 컨트롤 흐름은 동일, 다만 ref_v / d_ref 만 다름:

```
TRAILING_PASSIVE   ego.v = obs.vs + 0.2.  d_ref = raceline.
                   조용히 따라감. PASS 의지 없음. (e=0)

TRAILING_INTENT    ego.v = obs.vs + ramp(0.5..1.5).
                   d_ref = strategic side preference.
                   거리 미리 좁힘 + line preposition.
                   PASS 의 사전 단계. (e=0.3..0.7)

OVERTAKE           (현 OVERTAKE 모드 그대로, 짧은 commit window)
                   ref_v = raceline_v ramp from obs.vs+1.5.
                   이미 catch-up + prepositioned 상태로 진입. (e=1.0)
```

이 구조의 효과:
- **TRAILING_INTENT 가 PASS 의 launch pad 역할**. OVERTAKE 진입 직전에 이미 catch-up + lateral preposition 완료.
- **OVERTAKE 자체는 1~2초의 짧은 commit window**. 그 안에서 명시적 PASS curve.
- **가속이 단계적 ramp**. 점프 없음 → 차량 거동 안정.
- **PASS 결정이 사전 계획**. e 가 점진적으로 올라가면서 strategic planner 가 commit window 를 준비.

### 3.3 누적 observation 활용

상대 행동을 단일-tick GP prediction 만으로 예측하는 건 한계. 이미 우리 스택에 있는 `OpponentTrajectory` 누적을 strategic decision 에 결합:

- **단기 (수초)**: GP prediction (현재 사용 중) — N=20 timestep mean + variance.
- **중기 (수 lap)**: sector-level 통계 — 상대가 어느 sector 에서 어느 line / 어느 속도 잡는지.
- **장기 (race 전체)**: driver style 추정 — aggressive / smooth / inconsistent.

이 세 시간 스케일을 결합하면, "상대가 다음 sector 에서 우회전을 크게 할 거라는 걸 미리 안다" 같은 사용자 시나리오가 strategic planner 의 입력으로 들어옴.

### 3.4 MPC 의 본업 = 시간 함수 trajectory 풀이 (2026-05-02 추가)

사용자 본질 지적:
> "MPC 가 도대체 왜 spline 처럼 작동하는거야? time horizon 을 다루는게 mpc만의 특권인데."

MPC 의 본업:
- N+1 timestep 의 trajectory 를 매 tick 풀이.
- 매 timestep ego(t) 와 obs(t) 가 **각각 시간 함수**.
- 충돌 회피는 "ego(t) 와 obs(t) 가 동시에 같은 (s, n) 에 있는가" 만 체크.
- TRAIL 이 자연스럽게 표현됨: ego(t) 가 obs(t) - safe_gap 따라가면 됨, lateral 자유.

기존 코드의 함정:
- `J_obs = w_obs * exp(-((n-n_obs)/σ_n)² - ((s-s_obs)/σ_s)²)` 같은 spatial Gaussian repulsion.
- 매 timestep ego 가 obs 옆 빠지려 함 → **시간 정보 활용 안 함**, 단지 spatial push.
- 결과 = spline 같은 lateral 곡선 (사용자 표현 정확).

다듬은 방향:
- **TRAIL**: obs cost 폐기 + 종방향 ref_v cap → ego 가 자연스럽게 obs 뒤 따라감. lateral 자유. left/right 어느 쪽 trailing 도 NLP 자유 풀이.
- **PASS**: directional bubble (한쪽 hard) + side 라벨 LOCK + 종방향 obs cost 약간.
- **충돌 회피**: 속도 cap 으로 ego 가 obs 못 따라잡는 자연 메커니즘 + collision constraint (Phase 4 영역).

이게 MPC 의 본업과 일치. 학계 racing 논문 (TUM, KIT IAC, MIT-Pitt-RW) 의 표준 패턴.

---

## 4. 아키텍처

### 4.0 모듈 구성 원칙 (2026-05-01 갱신)

**Strategy 는 별도 노드 X. mpc_planner 패키지 안 모듈 파일** 로 두고 **import 형태로 호출**.

```
planner/mpc_planner/
├── node/
│   └── mpc_planner_state_node.py     # MPC 노드 (strategy import 사용)
├── src/
│   ├── frenet_kin_solver.py          # 기존 NLP solver
│   ├── side_decider.py               # 기존 단순 side 결정 (점차 strategy 가 대체)
│   ├── strategy_planner.py           # 신규 — Layer 1+2+3 묶은 facade
│   ├── opp_profile.py                # Layer 1 module
│   ├── encounter_forecaster.py       # Layer 2 module
│   └── ...
└── config/
    └── strategy.yaml                 # strategy 튜닝 파라미터
```

**MPC 노드 안 사용 패턴**:
```python
from mpc_planner.strategy_planner import StrategyPlanner

class MPCNode:
    def __init__(self):
        self.strategy = StrategyPlanner(...)
        self.strategy_debug_pub = rospy.Publisher(
            '/strategy/debug/plan_json', String, queue_size=1)
    def control_loop(self):
        plan = self.strategy.decide(ego_state, obs_pred, opp_traj)
        self.strategy.publish_debug(self.strategy_debug_pub)
        # MPC NLP 가 plan 받아 풀이
```

**왜 노드 분리 X**:
- Strategy 와 MPC 가 같이 죽거나 같이 살음 (분리 운용 가치 없음).
- per-tick 호출이라 ROS pub/sub latency 0 으로 절약 가치 있음.
- 분리 디버깅은 별도 디버그 토픽 (`/strategy/debug/plan_json`) 으로 충분 — bag 분석 / RViz 가능.
- 다른 백엔드 (sampling, SQP, IY) 가 나중에 필요하면 그때 갖다 쓰기 (코드 재사용 또는 별도 패키지로 분리).

**Plan 자료 구조 (`StrategyPlan` dataclass — 메시지 아님, 같은 process 안 dict)**:
```python
@dataclass
class StrategyPlan:
    plan_type:    str    # LEFT_PASS / RIGHT_PASS / TRAIL / WAIT / CLEAR
    intent:       float  # 0..1 continuous engagement
    commit_phase: str    # DETECT / APPROACH / WAIT / COMMIT / PASS / RECOVER / CLEAR
    commit_dwell: int    # ticks since current commit_phase entry
    target_obs_id: int

    # commit 의 두 시간 개념 (사용자 시나리오 대응):
    commit_decision_lookahead: float  # commit 결정 시 고려한 시간 (s) — 5.0
    commit_hold_until:         float  # 이 시점까지 side 라벨 LOCK (rospy.Time)

    # MPC 가 직접 사용할 ref (시점 별, phase 진행에 따라 매 tick 갱신):
    ref_v_cap:    float
    d_ref_offset: float
    target_gap_s: float
    side_sign:    int    # +1 LEFT / -1 RIGHT / 0 NONE — directional bubble

    encounter_window: tuple  # (t_start, t_end, side, pass_margin, ttc, confidence)
    safety_margin: float
    reason:        str
    opp_stationarity: float  # 0..1 (1=rule-based, 0=reactive)
```

**Strategy 결정 못할 때 default**: `StrategyPlan(plan_type='TRAIL', intent=0, ref_v_cap=raceline_v, d_ref_offset=0)` — MPC 가 단독으로도 안전 baseline 작동.

**디버그 토픽**:
- `/strategy/debug/plan_json` — 매 tick StrategyPlan + intermediate state JSON
- `/strategy/debug/encounter_windows` — Layer 2 forecaster 출력 (RViz marker)
- `/strategy/debug/opp_profile` — Layer 1 누적 통계 (RViz, lap 갱신 시)

### 4.1 Layer 다이어그램

```
┌──────────────────────────────────────────────────────────────────────┐
│ Layer 0  Tracking + GP Prediction  (현재 있음, 그대로)               │
│   /tracking/obstacles            : 현재 (s, d, vs, vd)               │
│   /opponent_prediction/obstacles : N timestep 단기 예측              │
└──────────────────────────────────────────────────────────────────────┘
                               │
┌──────────────────────────────────────────────────────────────────────┐
│ Layer 1  Opponent Profile Accumulator (horizon-based, sector 의존 X) │
│   타겟: rule-based planner 팀 (lap 마다 재현 가능한 line / 속도).     │
│         reactive 팀은 stationarity_score 로 자동 감지 후 prior 가중   │
│         치 자동 낮춤 (Layer 2 confidence 에 반영).                    │
│                                                                        │
│   기존 인프라 활용:                                                    │
│     /opponent_trajectory       (lap 누적 trajectory, 이미 publish 됨) │
│     GP prior (3d_opp_prediction.py 안 학습된 mean d(s))                │
│                                                                        │
│   산출 (s 의 함수, sector 라벨 없음):                                  │
│     opp_d_profile(s)      = GP prior mean d 함수                      │
│     opp_d_var(s)          = GP prior variance                         │
│     opp_v_profile(s)      = OpponentTrajectory.vs 함수                │
│     opp_brake_points      = [s where dvs/ds < -threshold]             │
│     stationarity_score    = lap 간 opp_d_profile std 적분 (낮을수록  │
│                              prior 신뢰 ↑, 높으면 reactive 팀 추정)  │
│                                                                        │
│   note: ot_sectors.yaml 의 "추월 금지" sector 는 hard rule 로 활용.    │
│         그 외 strategy 결정에는 sector 라벨 의존성 없음.               │
└──────────────────────────────────────────────────────────────────────┘
                               │
┌──────────────────────────────────────────────────────────────────────┐
│ Layer 2  Encounter Forecaster (horizon-based, sector 라벨 X)         │
│   왜: MPC horizon 1~3초 안에 안 들어오는 long-horizon 결정 필요.      │
│       Layer 3 commit 결정의 lookahead 정확도가 PASS 안전성을 결정.    │
│                                                                        │
│   알고리즘 (5초 lookahead, dt=0.1s rollout):                          │
│     for t in [0, 5.0, 0.1]:                                           │
│       ego_s_t   = ego.s + ego.v * t   (raceline 가정)                 │
│       if t <= obs_pred_short.horizon:    (단기 GP 영역, ≈1~2s)        │
│         obs_s_t, obs_d_t = GP.at(t);  conf = 1.0                      │
│       else:                              (lap-prior blend)            │
│         obs_s_t   = ego.obs_s + opp_v_profile(obs_s) * t              │
│         obs_d_t   = opp_d_profile(obs_s_t)   (s 함수 평가)            │
│         conf      = 1.0 - opp_profile.stationarity_score              │
│       if |ego_s_t - obs_s_t| < proximity_threshold:                   │
│         for side in [LEFT, RIGHT]:                                    │
│           margin = corridor_w(ego_s_t) - obs_w - safety               │
│                  - lateral_separation(ego, obs_d_t, side)             │
│           if margin > 0:                                              │
│             windows.append({t, side, margin, ttc, conf, obs_d_t})     │
│     return merge_adjacent(windows)                                    │
│                                                                        │
│   출력: encounter_window list = [(t_start, t_end, side, pass_margin, │
│           ttc, confidence, obs_d_seq), …] (Layer 3 가 best 선택)     │
│   디버그: /strategy/debug/encounter_windows (RViz 마커)                │
│                                                                        │
│   핵심: 출력에 obs_d_seq (commit window 안 obs 의 lateral 시퀀스)     │
│         포함. Layer 3 가 commit 시 사용자 시나리오 (obs in→out swap)  │
│         처리 가능 — "이 commit window 안 obs lateral 변화" 까지 봄.   │
└──────────────────────────────────────────────────────────────────────┘
                               │
┌──────────────────────────────────────────────────────────────────────┐
│ Layer 3  Strategic Planner                                            │
│   책임 1 (TRAIL 단계 유리한 구도): target_d_offset, target_gap,       │
│           closure_rate. "쟤가 다음 코너 outer 가니까 미리 inner 쪽    │
│           에서 trail" 같은 preposition.                               │
│   책임 2 (PASS commit 결정): Layer 2 window list 에서 best 선택.      │
│           commit 결정 시 5s lookahead 안 obs lateral 시퀀스 전체 검사. │
│                                                                        │
│   side commit 정책 (Design X — strategy 가 macro side 확정):           │
│     - Strategy 가 LEFT/RIGHT 확실히 정함. MPC 는 micro d(t) 만 풀이.   │
│     - Lookahead 5s 안 모든 timestep 에서 안전 margin 충족하는 side 만 │
│       commit. 미래 obs swap 도 사전 검사 → 사용자 시나리오 처리.      │
│                                                                        │
│   commit 의 두 시간 (사용자 직관 반영):                                │
│     T_lookahead = 5s     (commit 결정 시 검사할 미래 시간)            │
│     T_hold      = encounter_window 길이 + 1s    (side LOCK 유지)      │
│                                                                        │
│   commit abort 트리거 (T_hold 안에서도 abort):                        │
│     (a) 현재 timestep pass margin < min_safety                        │
│     (b) 새 GP prediction 이 commit 시점 prediction 과 lateral 0.3m+   │
│         차이 (예측 빗나감)                                            │
│     (c) 새 obstacle horizon 진입                                      │
│     (d) ego 가 d_ref 추적 못 함 (실측 d 와 d_ref 차이 > 0.3m, 0.5s+)  │
│                                                                        │
│   intent score (continuous, 0..1):                                    │
│     score = 0.4*sigmoid(margin/safety - 1)                            │
│           + 0.3*sigmoid(ttc - min_ttc)                                │
│           + 0.2*sigmoid(closure_rate)                                 │
│           + 0.1*confidence                                            │
│     if corridor_efficiency(s) is high: score *= 1.2                   │
│       (corridor_efficiency = corridor 폭 / |kappa(s)| 가중. sector    │
│        라벨 없이 곡률 / 폭 만으로 계산. horizon-based)                │
│     intent = 0.7*prev_intent + 0.3*score   (hysteresis blend)         │
│                                                                        │
│   commit_phase FSM (7-state, 7-state transition + abort):              │
│     DETECT    obs in horizon. plan=TRAIL, intent=0.0                  │
│        ↓ closure rate > 0, ego 와 같은 trajectory horizon 안           │
│     APPROACH  plan=TRAIL, intent=0.2, ref_v_cap=obs.vs+0.5            │
│        ↓ Layer 2 window 검출, side preference 결정                    │
│     WAIT      plan=TRAIL, intent=0.4, d_ref_offset=±0.2 preposition   │
│        ↓ window 임박 (t_start<1s) + safety guard + lookahead 5s 통과  │
│     COMMIT    plan=L/R_PASS, intent=0.7, side LOCK, ref_v_cap up      │
│        ↓ ego 옆 도달 (s_gap < 0.5m)                                   │
│     PASS      intent=1.0, ref_v_cap=raceline, dwell K_pass=10 tick    │
│        ↓ 추월 완료 (s_gap > 0.5m, 부호 반전)                          │
│     RECOVER   intent=0.5, d_ref_offset ramp back to 0                 │
│        ↓ ego_n < 0.1                                                  │
│     CLEAR     plan=CLEAR, intent=0.0                                  │
│   ※ abort: commit 후 abort 트리거 (a~d) 발동 시 → RECOVER phase 직행.  │
│                                                                        │
│   intent threshold (선택적 추상화 — 빼도 무방):                       │
│     0.0~0.3 PASSIVE  (조용히 따라감)                                   │
│     0.3~0.7 INTENT   (검토 + preposition)                             │
│     0.7~1.0 COMMIT/PASS                                               │
│   threshold 자체는 튜닝 값. mode FSM entry condition 으로도 대체 가능. │
└──────────────────────────────────────────────────────────────────────┘
                               │
┌──────────────────────────────────────────────────────────────────────┐
│ Layer 4  Plan-aware MPC                                               │
│   입력: StrategyPlan (import 호출 결과) + obs horizon (Layer 0)        │
│                                                                        │
│   plan 별 NLP cost / constraint 분기 (2026-05-02 사용자 비전 반영):   │
│   ─────────────────────────────────────────────────────────────────  │
│     PASS (LEFT/RIGHT)                                                  │
│       w_obs > 0   (Gaussian bubble — ego 가 obs side 반대로 lateral) │
│       w_side_bias > 0 (선택 side 쪽으로 ego 끌어당김)                  │
│       directional bubble (반대 side 침범 시 hard, 선택 side soft)    │
│       d_ref = preposition (commit_phase 별)                          │
│                                                                        │
│     TRAIL                                                              │
│       w_obs = 0     ★ lateral repulsion 폐기 (obs 옆 안 빠짐)        │
│       w_side_bias = 0  (lateral free)                                │
│       q_n_target = 0   (ego_n 강제 X — 자유 lateral)                  │
│       ref_v_cap = obs.vs - margin   ★ 종방향만 obs 인지              │
│       → NLP 가 corridor + progress + smoothness + ref_v tracking     │
│         만으로 풀이 → 항상 feasible. ego 속도가 obs.vs 보다 낮으니    │
│         자연스럽게 obs 못 따라잡음 = 충돌 회피.                       │
│       NOTE: TRAIL 은 baseline 만. 전략적 행동 (lateral preposition,  │
│             dynamic ref_v) 은 Phase 7~10 에서 도입.                  │
│                                                                        │
│     RACELINE / NO_OBS                                                  │
│       w_obs = 0, q_n_target = 0 (raceline tracking)                  │
│   ─────────────────────────────────────────────────────────────────  │
│                                                                        │
│   macro side + micro d(t) 분리 (PASS 만 해당):                         │
│     - Strategy 가 macro side (LEFT/RIGHT) 라벨 LOCK 으로 전달.         │
│     - MPC 는 micro d(k) trajectory 를 NLP 가 obs(k) 시간 함수 받아     │
│       풀이. obs 가 timestep 마다 lateral 변하면 ego d(k) 도 시점별     │
│       다르게 풀림 — 사용자 시나리오 in→out 중 lateral 미세 조정 가능.  │
│                                                                        │
│   변경 vs 현재 MPC:                                                    │
│     ref_v   현재: GB raceline vx_lookup(s) 가 NLP v[k] 통째 덮어씀    │
│             신규: NLP v[k] 보존 (P-3a fix), plan.ref_v_cap 만 적용    │
│             ※ ref_v 점프 안전망: 1-tick max delta 0.5 m/s 만 강제.     │
│                급정지 / 급가속 의도는 그대로 표현 가능.                │
│                                                                        │
│     d_ref   현재: 항상 raceline d=0                                    │
│             신규: plan.d_ref_offset (phase 진행 따라 매 tick 갱신)     │
│             ※ 명시적 sigmoid 시정수 강제 X. NLP a_max constraint 로    │
│                자체 ramp. 단계적 변화는 phase 진행으로 emerging.       │
│                                                                        │
│     bubble  현재: 양쪽 동일 Gaussian penalty                           │
│             신규: plan.side_sign 받아 한쪽만 hard, 한쪽 soft hinge    │
│                  J_obs(k) = w_obs * sigmoid((d_signed(k) - m)/scale)  │
│                  d_signed(k) = (n_k - n_obs_k(t)) * side_sign         │
│                  ※ obs(k) 가 시간 함수라 매 timestep 별 다름.           │
│                                                                        │
│     a_max   현재: 고정 a_acc / a_dec                                   │
│             신규: plan-aware (TRAIL 시 brake 강도 ↑, PASS 시 acc ↑)   │
│                                                                        │
│     weights 현재: 고정                                                 │
│             신규: commit_phase 별 weight set (RECOVER 시 q_n_term ↑) │
│                                                                        │
│   continuity guard 도 plan-aware (TRAIL 중에는 ego_v 강제 clip 완화). │
│                                                                        │
│   controller (NODE D) 의 TRAILING PID 제거 (P-3b fix) 가 함께 들어가   │
│   야 NLP v[k] 가 끝까지 살아남음.                                      │
└──────────────────────────────────────────────────────────────────────┘
                               │
┌──────────────────────────────────────────────────────────────────────┐
│ Layer 5  Safe Publish  (현재 있음)                                    │
│   ds_min 0.02 + sparsify + kappa_max guard (Phase 0 완료)             │
└──────────────────────────────────────────────────────────────────────┘
```

각 Layer 의 contract:
- Layer 1 은 lap 갱신 시 publish (수 초 단위). Layer 2 가 cache.
- Layer 2 는 Layer 1 갱신 시 + Layer 0 매 tick 시 갱신.
- Layer 3 은 매 tick. Layer 2 출력 받아 intent / side / mode 결정.
- Layer 4 의 ref_v / d_ref / bubble 은 Layer 3 출력의 단순 함수. Layer 4 자체에는 의사결정 없음.

### 4.2 Intent threshold + sub-state mapping

intent 는 연속 [0,1] 이지만 sub-state 전이는 discrete. threshold + hysteresis:

| intent 범위 | sub-state | painter ref_v_cap | d_ref_offset | 의미 |
|-------------|-----------|-------------------|--------------|------|
| 0.0 ≤ e < 0.3 | TRAILING_PASSIVE | obs.vs + 0.2 | 0 | 의지 없음 |
| 0.3 ≤ e < 0.7 | TRAILING_INTENT | obs.vs + ramp(0.5..1.5) | ramp 0..±0.2 | 검토 + preposition |
| 0.7 ≤ e ≤ 1.0 | OVERTAKE | ramp obs.vs+1.5 → raceline_v | ramp ±0.2..±0.4 | commit |

**Hysteresis** (oscillation 방지):
- Up-threshold: 0.7 (TRAILING_INTENT → OVERTAKE 진입)
- Down-threshold: 0.5 (OVERTAKE → TRAILING_INTENT 빠져나감, +0.2 hysteresis)
- Dwell: 한 번 OVERTAKE 진입 후 최소 K_pass=10 tick 유지
- 같은 원리로 PASSIVE↔INTENT 도 0.3/0.2 hysteresis

threshold 0.3, 0.7 은 **튜닝 값** — 의미적 근거:
- 0.3: "반신반의지만 catch-up 시작은 가치 있음"
- 0.7: "PASS 가 안전 + 효율적이라는 확신"
- 사이 0.4 폭 = 검토 / preposition 단계 (TRAILING_INTENT)

### 4.2.1 Commit feasibility 검증 — 단계적

**사용자 원칙**: "벽과 상대 차 사이에 껴서 좁혀질 공간은 애초에 들어가면 안 돼. 그걸 위해 상대 경로를 쓰는거잖아."

**즉 commit 결정 시 5s lookahead 안 ego trajectory 가 안전하게 존재 가능한가** 를 미리 검증. Layer 2 forecaster 의 obs trajectory 시퀀스가 이 검증의 입력.

세 단계로 진화:

#### MVP (Phase 7~9) — Single-side commit, same-side corridor 폭 검사

```
∀t ∈ [0, T_lookahead]:
   side=LEFT 의 경우:
     wall_left(obs.s_t) + ego_w/2 + safety  ≤  obs_d(t) − ego_w/2 − safety_lat
   side=RIGHT 의 경우:
     obs_d(t) + ego_w/2 + safety_lat  ≤  wall_right(obs.s_t) − ego_w/2 − safety
```
한 timestep 이라도 fail 이면 그 side commit 안 함. lap-stable rule-based 상대 (대부분 케이스) 에 충분.

#### Phase 10 — Feasibility check (옵션 A): 시간별 lateral 도달 범위

사용자 시나리오: t=1.0 obs LEFT 벽, t=1.5 obs RIGHT 벽 → 사이 비집고 PASS 가능.
이걸 single-side 검사로 표현 못 함. **forward reachable lateral set** 으로 검사:

```python
feasible_range = (ego.d - max_lat_step, ego.d + max_lat_step)   # t=0 도달 범위
for t, obs_t in obs_traj_seq:
    wall_l = corridor.left(obs_t.s) + safety
    wall_r = corridor.right(obs_t.s) - safety
    obs_left_band  = (wall_l,  obs_t.d - safety_lat)
    obs_right_band = (obs_t.d + safety_lat, wall_r)

    feasible_range = expand(feasible_range, max_lat_step_per_dt)  # lateral acc 한계
    intersection = (feasible_range ∩ obs_left_band) ∪
                   (feasible_range ∩ obs_right_band)
    if intersection.empty:
        return commit_INFEASIBLE
    feasible_range = intersection
return commit_FEASIBLE
```

이 검사가 통과하면 ego 가 매 timestep 어느 lateral 영역으로 빠질 수 있다는 보장. 시간 따라 LEFT→RIGHT 변동 OK.

#### Phase 11+ — 시간별 d 시퀀스 commit

Strategy 가 single side 라벨 대신 **거친 d 시퀀스** 출력:
`[(t=0..0.5, d=+0.2), (t=0.5..1.0, d=+0.3), (t=1.0..1.5, d=0), (t=1.5..2.0, d=-0.3), …]`

MPC 가 이 시퀀스를 ref 로 받아 micro NLP 풀이. lap-stable 가정 깨지는 코너 swap 케이스 대응.

#### 검증 단계 요약 — Phase 별 활용 결정

| Phase | 검사 종류 | 사용 case |
|-------|----------|---------|
| 7~9 | Single-side same-side corridor | 대부분 (lap-stable rule-based) |
| 10 | Forward reachable feasibility | obs 가 시간 따라 lateral 변동 |
| 11+ | 시간별 d 시퀀스 | 코너 swap 비집고 PASS |

### 4.3 Commit 변경 정책 (Design X)

| commit_phase | 변경 가능 여부 | 변경 종류 | 처리 |
|--------------|--------------|----------|------|
| DETECT / APPROACH / WAIT | 자유 | LEFT ↔ RIGHT swap (hysteresis +20%) | d_ref 점진 변경, ramp 으로 새 side 로 |
| COMMIT | LOCK | side swap 금지 | abort 만 가능 |
| PASS | LOCK | 불가 | abort 어려움 (이미 obs 옆), 비상 brake 만 |
| RECOVER | — | 새 commit 안 함 | raceline 회귀 |
| ABORT cool-down | LOCK | 같은 obstacle 에 대한 새 commit 금지 (2s) | 진동 방지 |

**Abort 트리거** (4 종류):
1. 현재 timestep pass margin < min_safety
2. 새 GP prediction 이 commit 시점 prediction 과 lateral 0.3m 이상 차이 (예측 빗나감)
3. 새 obstacle horizon 진입
4. ego 가 d_ref 추적 못 함 (실측 d 와 d_ref 차이 > 0.3m, 0.5s 지속)

**TTC 정의 (정지 / 동적 obstacle 통합)**:
- `closure_rate = ego.vs - obs.vs` 기반.
- 정지 obs (obs.vs ≈ 0) → ttc = s_gap / ego.vs.
- 동적 obs 같은 속도 / 더 빠름 → ttc = ∞ (안 따라잡음).
- 동적 obs 더 느림 → ttc = s_gap / closure_rate.
- 미래 closure 검사: prediction horizon 안 obs(t).vs 시퀀스 활용 — obs 가 brake 시작 케이스 (현재 closure 작아도 미래 큼).
- safety threshold 차별화: 정지 obs lateral_sep < 0.2m, 동적 obs < 0.3m + prediction confidence 낮으면 더 보수적.

**Abort 처리 5 단계**:
1. 즉시 `commit_phase = ABORT_RECOVER` (별도 phase).
2. D_ref ramp: 현재 d → 0 회귀. NLP a_max 안에서 자체 풀이.
3. Ref_v 조정: PASS 중이었으면 obs 와 충돌 위험 평가 → 필요 시 emergency brake (ref_v cap = obs.vs - margin).
4. Side LOCK 해제 — side_sign = 0. Bubble 양쪽 hard 로 복원.
5. Cool-down 2s — 같은 obstacle 새 commit 금지.

**Side swap (WAIT phase 안에서)**:
- 새 best side score 가 현재 side score 보다 +20% 이상 우월할 때만.
- swap 시 d_ref preposition 이 점진 변경 — 직접 점프 금지, 1-tick max delta 0.1m.

### 4.4 디버그 마커 (단순)

**핵심 원칙**: 새 마커 형태 만들지 않음. **이미 publish 되는 상대 예측 경로의 색깔만 horizon (시간) heatmap 으로**.

#### M1. 상대 예측 경로 — 시간 색깔 heatmap (필수)

- 대상: `3d_opp_prediction.py` 가 publish 하는 obs prediction 의 RViz marker.
- 변경: marker 의 color 필드만 시간에 따라 적용. **형태 / 위치 / 개수 변경 X**.
- **색깔 매핑: 0.5초 단위 discrete binning** — RViz 에서 색 띠가 명확히 구분.
  - bin 0 (0.0~0.5s) — 빨강
  - bin 1 (0.5~1.0s) — 주황
  - bin 2 (1.0~1.5s) — 노랑
  - bin 3 (1.5~2.0s) — 연두 / 초록
  - bin 4 (2.0~2.5s) — 청록 (단기 GP horizon 종료 부근)
  - bin 5 (2.5~3.0s) — 청
  - bin 6 (3.0~3.5s) — 파랑
  - bin 7 (3.5~4.0s) — 남청
  - bin 8 (4.0~4.5s) — 보라
  - bin 9 (4.5~5.0s) — 자주
- 단기 GP 영역과 lap-prior 영역 경계 (보통 t≈2s) 에서 hue 변화 크게 → 시각 구분 강조.
- 구현 위치: prediction 노드 안 marker publish 부분에 color = `time_to_color_binned(t)` 한 줄 변경.
- 효과: RViz 에서 path 색 띠만 봐도 "이 구간은 1.5~2초 후 obs" 즉시 인식.

#### M2~ 부가 마커 (선택, Phase 진행 시 필요하면 추가)

마커는 최소화. 시각화 욕심 부리지 않음. 필요할 때 그 phase 에서 추가.

- **M2 strategy state 텍스트**: ego 위 떠있는 텍스트 (intent / commit_phase / side). 1줄.
- **M3 abort event**: abort 발동 시 잠깐 깜빡이는 텍스트. 1줄.

이 외 (encounter window fill / lookahead check sphere 등) 은 **Phase 진행 시 부족함 느끼면** 추가. 미리 만들지 않음.

**색깔 매핑 함수 (0.5s 단위 binning)**:
```python
def time_to_color_binned(t, bin_size=0.5, t_max=5.0):
    """0.5s 단위로 색 띠 형성. 같은 bin 안 timestep 들은 동일 색."""
    import colorsys
    bin_idx = int(min(t / bin_size, t_max / bin_size - 1))
    n_bins = int(t_max / bin_size)
    hue = bin_idx / n_bins  # 0~1, 빨강→자주
    rgb = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
    return ColorRGBA(rgb[0], rgb[1], rgb[2], 1.0)
```
또는 viridis colormap 으로 대체 가능 (matplotlib.cm.viridis(bin_idx / (n_bins-1))).

**구현 위치**:
- M1: prediction 노드 (`3d_opp_prediction.py`) 안 marker publish 부분. 가장 자연스러움 — 어차피 prediction 노드가 timestep 정보 갖고 있음.
- M2/M3: `strategy_planner.py` 안 `DebugMarkerPublisher` (간단 클래스).

---

## 5. 시나리오 케이스 스터디 — 사용자 비전의 시간순 전개

> 시나리오: 상대차가 전방 sector 6 (코너 R) 에서 outer line (d=+0.4) 잡는 패턴이 lap 통계상 명확. ego 는 직선 sector 5 끝부터 PASS window 가 있음을 인지.

```
t=0..3s   sector 4 (직선)                                          
          mode=TRAILING  intent=0.0 (PASSIVE)                      
          ref_v = obs.vs + 0.2 = 5.2 m/s                           
          d_ref = raceline                                          
          → 조용히 따라감                                            

t=3..4s   sector 5 진입 (직선 마지막 부분)                          
          Layer 2 forecast: sector 6 진입 시 obs 가 outer (d=+0.4) 잡음.
                            sector 5 끝까지 0.7s 안에 PASS window 열림.
                            pass margin 0.45m, ttc safe.            
          Layer 3: intent ramp 0 → 0.5 (INTENT 진입)                
                   side = LEFT (상대 outer 의 반대)                  
          Layer 4: ref_v ramp 5.2 → 6.0 (catch-up)                  
                   d_ref ramp 0.0 → -0.2 (left preposition)         
          → 미리 거리 좁히고 왼쪽 line 으로 이동 시작                

t=4..4.5s sector 5 끝, sector 6 진입 직전                          
          Layer 3: intent → 0.9 (commit imminent)                  
                   mode = OVERTAKE 전이 결정                         
          Layer 4: ref_v ramp 6.0 → 6.8                            
                   d_ref ramp -0.2 → -0.5 (side commit)            
                   directional bubble: 오른쪽 hard, 왼쪽 soft       
          → OVERTAKE 진입 시점에 이미 catch-up + lateral 완료 상태  

t=4.5..5.5s sector 6 (코너 R)                                       
          mode=OVERTAKE  intent=1.0 (COMMIT)                       
          ref_v = raceline_v 7.0 m/s                               
          상대는 outer (d=+0.4), ego 는 inner (d=-0.5) → 안전 통과  
          → 가속 / lateral 모두 단계적이라 차량 안정                 

t=5.5..6.5s sector 7 (코너 출구)                                    
          상대 detach (gap > threshold)                            
          Layer 3: intent ramp 1.0 → 0.0                          
          mode = GB_TRACK 또는 RECOVERY                             
          → raceline 복귀                                           
```

핵심 관찰:
- **OVERTAKE 모드는 1초만 활성**. 전후는 모두 TRAILING (PASSIVE 또는 INTENT) 또는 GB_TRACK.
- **가속이 5.2 → 6.0 → 6.8 → 7.0 으로 단계적**. 1-tick 점프 없음.
- **lateral 이동도 0.0 → -0.2 → -0.5 단계적**. PASS 진입 시 이미 -0.2 위치라 OVERTAKE 가 0.3m 만 더 이동하면 됨.
- **결정이 사전 계획**. t=3s 부터 PASS 를 준비하고 t=4.5s 에 commit. 1-tick 반응 아님.

이게 사용자가 묘사한 **"우측 뒤에서 매우 잘 따라가다가 매우 효과적으로 가속 타이밍과 경로를 잘 생성해서 자연스럽게 추월"** 의 정확한 시간 전개.

---

## 6. 학계 용어 정렬

본 설계의 학술적 위치:

| 컴포넌트 | 학계 분야 / 용어 |
|---------|-----------------|
| Layer 1 | online opponent modeling (Bayesian prior from past laps) |
| Layer 2 | model-based encounter forecasting / interaction prediction |
| Layer 3 | hierarchical receding-horizon planner with engagement gradient |
| Layer 4 | continuous-reference MPC with directional constraints |
| 모드 명시 + intent 연속 | hierarchical mode switching with continuous intent |
| 누적 + 단기 결합 | Bayesian filter (prior=lap 통계, posterior=current GP) |

비교 사례:
- DARPA Urban Challenge 후속 / Toyota 의 hierarchical planner
- KIT IAC team 의 receding-horizon overtake (Indy)
- TUMFTM 의 raceline + sector-aware velocity planner
- F1Tenth 학회 논문 중 multi-modal opponent prediction (Bayesian belief)

미반영 (장기 future work):
- Multi-modal opponent behavior tree (60% brake / 40% line hold)
- Game-theoretic 다단계 plan (상대 reaction 반영)
- 슬립스트림 / aero drag 모델

---

## 7. 우리 스택의 인프라 현황

이미 있는 것:

| 컴포넌트 | 위치 | 상태 |
|---------|------|------|
| GP 단기 prediction | `prediction/gp_traj_predictor/src/3d_opp_prediction.py` | 사용 중. N=20 timestep mean + variance |
| Lap-level opponent trajectory | `prediction/gp_traj_predictor/src/3d_opponent_trajectory.py` | publish 중 (`/opponent_trajectory`). MPC 가 아직 안 씀 |
| 메시지 정의 | `f110_msgs/OppWpnt`, `ProjOppTraj`, `OpponentTrajectory` | 이미 정의됨 |
| 다른 planner 의 사용 사례 | `spliner_planner`, `lane_change_planner` | 이미 구독 중. 참조 가능 |
| Raceline + sector | `stack_master/maps/<map>/`, `ot_sectors.yaml` | 사용 중 |
| MPC NLP + corridor + obstacle bubble | `planner/mpc_planner/` | 사용 중 |
| Continuity guard | `planner/mpc_planner/node/mpc_planner_state_node.py` | 사용 중 |
| Painter (`_post_process_speed`) | 동상 | hook 가능 |

신규로 만들어야 하는 것:

| 컴포넌트 | 예상 줄 수 | Layer |
|---------|-----------|-------|
| Sector-level statistics aggregator | ~200 | Layer 1 |
| Encounter forecaster | ~250 | Layer 2 |
| Strategic planner (intent / side / mode) | ~300 | Layer 3 |
| ref_v blend + d_ref preposition + directional bubble | ~150 | Layer 4 |
| TRAILING_PASSIVE / TRAILING_INTENT sub-state hook | ~50 | Layer 3 / SM |

총 **~950 줄**. 5~7 세션 작업.

---

## 8. 안전 보장 (directive #1 의 구조적 보장)

**모드는 명시, 전이는 의도** 구조에서도 충돌 0 보장은 그대로 유지:

- **Layer 3 의 commit decision 은 항상 safety guard 통과 후만**:
  - pass_margin > min_lateral_safety
  - ttc > min_ttc_safety
  - obs.vd_var rolling stddev < uncertainty_max
  - 어느 하나라도 False 면 intent 가 0.7 에서 멈춤. COMMIT 안 감.
- **OVERTAKE 진입 후에도 매 tick safety guard 재검사**. 깨지면 intent → 0, OVERTAKE → TRAILING_PASSIVE 후퇴.
- **Layer 4 의 directional bubble 은 항상 양쪽 hard 가 default**. soft hinge 는 commit 시에만.
- **Layer 5 publish gate** (kappa_max, ds_min, finite) 그대로.

즉 strategic 결정이 잘못돼도 **safety 하한선** 은 깨지지 않음. directive #1 위반 가능성 0.

---

## 9. 사용자 비전과 ICRA 마감의 정렬

ICRA 마감 (가까움) 안에 도달 가능한 범위:

| 단계 | 내용 | 사용자 시나리오 달성도 |
|------|------|---------------------|
| Phase 1~6 (마스터 플랜) | 안전 baseline. 충돌 0. reactive PASS 까지 | ~40% |
| Phase 7 (단기 strategic) | TRAILING_PASSIVE / INTENT 분리, 단순 intent | ~60% |
| Phase 8 (sector 활용) | sector-level lap 통계 활용 | ~70% |
| Phase 9 (encounter forecaster) | 누적 + 단기 결합 forecasting | ~80% |
| Phase 10 (continuous ref_v + ramp) | 단계적 가속, 점프 없음 | ~85% |
| Phase 11 (튜닝 / 검증) | 100/50 시나리오 통과 | ~85% (현재 비전 한계) |
| 미래 | multi-modal, 게임이론, aero | ~95% (학회 논문 영역) |

ICRA 마감 안에 가능한 건 **Phase 1~6 + 그 위 일부 (7, 10 정도)**. 즉 **사용자 시나리오 60~70% 도달** 이 현실적 목표.
마감 후 / 후속 작업으로 Phase 8, 9 추가하면 80~85%. multi-modal 까지는 별도 연구 영역.

---

## 10. Phase 로드맵

### 단기 (마스터 플랜, 안전 baseline) — `dynamic_overtake_master_plan_20260429.md` 참조

| Phase | 이름 | 목표 |
|-------|------|------|
| 0 | Publish + SM 정리 | 완료 (2026-05-01 commit b8580b4, a982af7) |
| 1 | Layer A — ObstacleHorizon | GP timestep-by-timestep 사용, vs/vd 외삽 폐기 |
| 2 | Layer B — EncounterAnalyzer (단순) | pass margin / ttc 계산 |
| 3 | Layer C — StrategicPlanner (단순) | rule-based plan + commit_phase FSM |
| 4 | Layer D — PlanAwareMPC | directional bubble + plan-aware ref_v / a_max |
| 5 | Safety buffer + ggv polar (옵션) | 추가 안전 가속 한계 |
| 6 | 통합 검증 | 10분 bag × 5 회. 충돌 0 |

### 중기 (이 문서 비전) — Phase 7~11

| Phase | 이름 | 핵심 |
|-------|------|------|
| 7 | TRAILING sub-state + Continuous intent | TRAILING_PASSIVE / INTENT 분할. e ∈ [0,1] scalar. hysteresis. 단순 distance / closure-rate 기반. strategy_planner.py 모듈 import 형태로 시작 |
| 8 | Layer 1 — Opponent Profile (horizon-based) | `/opponent_trajectory` + GP prior 활용. opp_d_profile(s) / opp_v_profile(s) / stationarity_score. sector 라벨 의존 X |
| 9 | Layer 2 — Encounter Forecaster | 5s lookahead rollout. Bayesian (단기 GP + lap prior) blend. encounter window list (obs_d_seq 포함) |
| 10 | Layer 4 확장 + Layer 3 commit 정밀화 | side LOCK macro + obs(t) 기반 micro d(k). commit lookahead 5s + hold (window+1s) + 4 abort 트리거. 명시적 sigmoid ramp 강제 X — 큰 점프 안전망 (max delta 0.5 m/s) 만 |
| 11 | 검증 / 튜닝 | 사용자 시나리오 (in→out swap 포함) 재현. 충돌 0. PASS 성공률 |

### 장기 (마감 후 future work) — 좁게 재정의

| Phase | 이름 | 비고 |
|-------|------|------|
| 12 | Multi-modal opponent (reactive) | 상대가 ego plan 에 reactive 하게 line/속도 변경 (60% block / 40% no-react). rule-based 가정 깨질 때 |
| 13 | Game-theoretic plan tree | Nash equilibrium / depth-3 plan tree, 상대를 능동 agent 로 취급 |
| 14 | Aero / 슬립스트림 | F1-급 드래프팅 / 가속 모델. kinematic bicycle 한계 |
| 15 | Auto-segmented sector (디버깅 보조) | 곡률 기반 자동 segment 분할 + 라벨. RViz 시각화 / 인간 직관 향상. 우리 결정상 horizon-based 가 default 라 보조 도구로만 |

> **중요 분류 정정 (이전 답에서 수정)**:
> - **상대 trajectory 가 시간에 따라 line 변하는 시나리오 (in→out swap 등)** = **Phase 7~11 영역**.
>   상대가 rule-based 면 lap 누적 + 5s lookahead 만으로 처리. 위 시나리오 케이스 스터디 §5 가 정확히 이 영역.
> - **Phase 12+ 는 상대가 ego 의 plan 에 reactive 하게 행동** 하는 케이스만 (즉 상대가 "능동 agent").
> - 우리 사용자 시나리오 (planner 기반 상대 / lap-stable trajectory) 는 Phase 7~11 안에서 다 해결 가능.

### Phase 10 의 의미 — 다듬은 정의

Phase 10 은 **side LOCK 정밀화 + 큰 점프 안전망**. **명시적 sigmoid 시정수 강제 X**.

**왜 강제 X**:
- MPC NLP 의 a_max constraint 가 자체 ramp 풀이.
- Sigmoid 시정수 강제하면 사용자 우려 ("부드럽게 하다가 죽도 밥도 안 됨") 발생 가능.
- 급정지 / 급가속 의도는 그대로 표현 가능해야 함 (사용자 명시).

**Phase 10 이 실제 하는 것**:
1. **Side LOCK 정밀화** — Layer 3 commit 의 5s lookahead + hold + abort 4 트리거 구현.
2. **큰 점프 안전망** — ref_v 의 1-tick max delta 0.5 m/s, d_ref 의 1-tick max delta 0.1m. 그 이상 변화 시 NLP 가 단계적으로 풀이.
3. **D_ref(t) phase 진행 곡선** — WAIT 진입 시 d_ref=0 → ±0.2, COMMIT 진입 시 → ±0.4. 명시적 시정수 없이 phase 진행 자체가 단계적.
4. **NLP solve 안정성** — side LOCK 으로 cost convexity 유지.

**Phase 4 와 Phase 10 의 구분**:
- Phase 4 = NLP `v[k]` / `d(k)` 보존 + plan parameter 단순 적용 (구조 fix).
- Phase 10 = commit 결정의 long-horizon 안전성 + 큰 점프 안전망 (정밀화).

**급정지 / 급가속**: 의도된 ref_v 점프 (예: emergency brake) 는 max delta 안전망 우회 — `plan.intent_brake_emergency=True` 같은 flag 가 set 되면 점프 통과.

---

## 11. 검증 기준

각 Phase 통과 기준:

| Phase | 통과 기준 |
|-------|----------|
| 7 | TRAILING 안에서 ego.v 가 obs.vs + ramp 으로 변화. PASS 시 1-tick 점프 없음. 충돌 0 유지 |
| 8 | sector-level 통계가 lap 1 후부터 publish. RViz 에서 sector 별 d_mean / v_profile 시각화 가능 |
| 9 | Layer 2 의 encounter window list 가 매 tick 갱신. window 별 score / safety guard 결과 publish |
| 10 | 시나리오 §5 의 시간 전개가 실제 bag 에서 재현. 가속 / lateral 단계적 ramp 시각 검증 |
| 11 | 100 lap × 50 encounter 시나리오 (또는 30분 bag × 5 회) 충돌 0. PASS 성공률 70% 이상 |

---

## 12. 단일 source of truth 와의 관계

본 문서는 **상위 비전 / 본질 설계**.
`dynamic_overtake_master_plan_20260429.md` 는 **단기 안전 baseline 의 단일 source of truth** (Phase 0~6).

두 문서의 차이:
- Master plan: ICRA 마감 안 구현, 안전 baseline, reactive PASS 까지.
- This vision: 마감 무관 본질, strategic + 누적, 사용자 비전 emerging behavior.

진행 순서: **단기 (Master plan Phase 1~6) 먼저, 그 위에 중기 (이 문서 Phase 7~11) 얹기**. 단기 baseline 이 안 만들어진 상태에서 중기 layer 만 쌓으면 안전 보장이 깨짐.

각 phase 진입 시 이 문서의 §3 (설계 원칙), §4 (아키텍처), §5 (시나리오) 가 **계속 reference**. 코드의 임시 patch 와 별개로 본 비전의 layer / contract 가 우선.
