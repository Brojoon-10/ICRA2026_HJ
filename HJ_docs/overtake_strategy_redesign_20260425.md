# Overtake Strategy Redesign — Predictive · Commit-aware · Retry-capable

작성일: 2026-04-25
**최종 업데이트: 2026-04-27 22:42** — 부분 구현 후 회고 + 다음 세션 가이드.

상태: **부분 구현 (SideDecider v4 + 솔버 layer 부분 변경). 검증 미완. Strategic FSM 미구현.**

다음 세션 첫 10분 복귀 가이드:
1. 본 문서 §0 + §A (현재 상태) + §B (남은 작업) 만 읽으면 즉시 진입 가능.
2. **현재 코드는 SideDecider v4 + per-obstacle σ 솔버까지 들어간 상태**. Strategic FSM 은 아직 안 만듦.
3. 다음 세션의 첫 결정: (a) 현 SideDecider v4 위에 Strategic FSM 얹기, (b) 다 갈아엎고 깨끗이 Strategic FSM 부터, 둘 중 선택.
4. 백업: `side_decider.py`, `mpc_planner_state_node.py`, `frenet_kin_solver.py` 의 `_backup_20260427.py` 권장.

---

## §A. 2026-04-27 세션의 현재 코드 상태 (rollback 후)

### A.1 적용된 변경 (build 된 상태, 검증 일부)

**SideDecider (`planner/mpc_planner/src/side_decider.py` v4)**:
- ✅ `decide(ego_v, obs_list, ego_n, ego_s)` — ego state 받음
- ✅ multi-obstacle aggregation: `relevant` 필터 (`immediate_decision_range=5m`, `sequential_pair_window=2m`)
- ✅ AND across relevant obstacles: 모든 relevant obs 가 통과 가능해야 그 side 가능
- ✅ `_per_obstacle_d_free`: 각 obstacle 에 대해 d_free_L/R 계산
- ✅ `_ego_reach`: ego 의 lateral budget = `v · sin(mu_max=0.9) · ds/v` 로 도달 가능성 체크
- ✅ `_swing_safe`: prev_side 와 raw 가 swap 일 때 forward obstacle 거리 < `swing_safe_distance=2m` 이면 TRAIL 강제
- ✅ side_scores 에 `n_obs_seen`, `n_obs_relevant`, `lat_budget`, `can_pass_L_corr/R_corr`, `can_reach_L/R` 등 노출

**solver (`planner/mpc_planner/src/frenet_kin_solver.py`)**:
- ✅ `_apply_side_commit`: side 결정을 hard corridor 로 박음. range gate `ds_window=3m` (가까운 obs 에서만 발동)
- ✅ slack ε=0.01m for k≥1: numerical edge 흡수 (sub-mm noise)
- ✅ per-obstacle σ_n inflation: TRAIL 시 dynamic ×10, static ×1 — 정적 lateral push 살리고 동적 trailing 가능
- ✅ `set_obs_static(arr)` setter: node 가 static/dynamic flag 푸시

**node (`mpc_planner_state_node.py`)**:
- ✅ `_obs_static_arr` sidecar: `_build_obstacle_array_frenet` 에서 채움 + 솔버에 push
- ✅ obs_arr rear_window: `signed_dist` 로 후방 3m 까지 keep (modulo wrap 버그 차단)
- ✅ `_decide_side`: SideDecider 에 ego_n/ego_s 전달
- ✅ side_decider d_L/d_R lookup: lifter._interp at obstacle's exact s (wrap-aware) → 정적 obs d_L 흔들림 제거
- ✅ Cache hijack gate: obstacle in obs_arr 시 cache skip (failure ladder + always-on hijack 둘 다)
- ✅ Tier 2 forbidden when obstacle in horizon: HOLD_LAST 무한 연장 (단, ego 가 stuck 되면 위험)

**config (`planner/mpc_planner/config/state_overtake.yaml`)**:
- ✅ `min_pass_margin`: 0.10 → 0.15

### A.2 Roll back 된 변경 (시도했다가 되돌림)

- ❌ slack cap 0.01 → 0.05 (사용자 거부 — slack 으로 풀려는 거 어거지)
- ❌ TRAIL vmax cap 에서 STATIC 제외 (사용자가 stuck 얘기 그만하라고 함)
- ❌ swing_safe 의 mid-OT lateral commit guard (`|ego_n| > 0.15` AND prev_side 방향)
- ❌ ot_line='trail' 태그 (SM 에 솔버 pass 정보 전달)

### A.3 검증된 것 / 안 된 것

| 항목 | 검증 상태 |
|---|---|
| side_decider d_L/d_R lookup wrap | ✅ bag 검증 완료 (08-59 bag) |
| _apply_side_commit (range gate) | ✅ bag 검증 완료 |
| obs_arr rear_window | ✅ bag 검증 완료 |
| multi-obstacle AND + relevant filter | ⚠️ 미검증 (bag 1 은 obstacle 1개라 단일 obs 케이스만 봄) |
| ego reach feasibility | ⚠️ 미검증 |
| swing_safe (forward distance only) | ⚠️ 미검증 |
| per-obstacle σ_n inflation | ⚠️ 미검증 |
| static/dynamic sidecar 전체 chain | ⚠️ 미검증 |
| cache hijack gate | ✅ bag 검증 완료 |

---

## §B. 남은 작업 (다음 세션)

### B.1 즉시 해결해야 할 본질적 문제

이번 세션에서 발견한 진짜 root cause 중 미해결:

**[문제 1] Pass 1 ↔ Pass 2 fallback 진동 (깜빡임의 정체)**

bag 1 (`2026-04-27-13-12-51`) 의 OT 진입 구간 t=0.25~2.04 에서 매 tick:
- pass 1 (LEFT side commit) success/fail 변동
- fail 시 pass 2 (TRAIL 강제) 가 succeeds
- 결과: 출력 trajectory 가 OT 와 trailing 사이 매 tick 진동
- 시각적으로 깜빡임, ego 가속/감속 반복

원인 추정: side_commit 의 range gate (3m binary on/off) 가 obstacle range boundary 에서 진동 → nlb/nub clamp 가 매 tick 변함 → IPOPT 수렴성 진동.

후보 fix (미구현):
- (A) range gate 를 binary 가 아닌 smooth ramp (거리에 따라 clamp 강도 점진)
- (B) pass 2 fallback 이 SIDE_TRAIL 강제 대신 같은 side commit 으로 더 관대한 조건 (lower min_pass_margin) 으로 재시도
- (C) Strategic FSM (commit-and-hold) — 이게 본 문서 원래 설계

**[문제 2] State machine 에 솔버의 실제 모드 미전달**

publish 되는 OTWpntArray 의 ot_line:
- 'avoid' (mpc_mode=WITH_OBS), 'recover' (NO_OBS) — 단순 FSM mode 만
- pass 2 TRAIL fallback 시도 'avoid' 로 publish → SM 은 솔버가 OT 시도인지 trailing fallback 인지 구분 불가
- SideDecider 의 결정 (left/right/trail/clear) 도 SM 에 안 들어감

**[문제 3] Lap-wrap glitch (obstacle EMA on lap boundary)**

ego_s 85→0 wrap 시 EMA 가 wrap 전후 obs s 평균 → nonsense (61, 69 같은 값) → mode 강제 flip → cache hijack.
fix 1줄: ego s 점프 감지 시 `_obs_arr_ema = None` reset.

### B.2 Strategic FSM (본 문서 §2 의 원래 설계, 미구현)

`StrategicOvertakeFSM` 미구현 상태. SideDecider v4 가 부분적으로 그 역할을 하지만:
- ✅ multi-obstacle / ego-aware: 부분 reach feasibility 만
- ❌ commit-and-hold (mid-OT lock)
- ❌ ABORT / REENGAGE / TRAIL_PREP 같은 명시적 retry FSM
- ❌ Predictive feasibility (k=0..k_pass over horizon, multi-step)
- ❌ TRAIL with prep (lateral repositioning)
- ❌ Map-aware tie-break

다음 세션의 큰 결정 — 아래 둘 중 선택:

**옵션 A**: 현 SideDecider v4 위에 Strategic FSM 얹기.
- SideDecider v4 는 inner helper 로 활용 (`eval_side_score`), Strategic FSM 이 commit/abort/retry 관리
- 작업량: 본 문서 §3 의 Day 1~4 그대로
- 장점: 점진적, 기존 검증된 부분 살림
- 단점: 두 layer 가 부분 중복 (multi-obstacle 처리 등)

**옵션 B**: 현 코드 다 roll back 하고 Strategic FSM 으로 재시작.
- side_decider 비우고 strategic_overtake_fsm.py 로 새로 시작
- 작업량: 4일치 통합 진행
- 장점: 깨끗
- 단점: 검증된 fix (lookup wrap, rear_window 등) 다시 들여야 함

### B.3 다음 세션 진입 전 결정 사항

- 옵션 A vs B
- mid-OT side flip 차단 (직전 세션에서 시도했다가 rollback) — 다시 도입할지
- TRAIL with static obstacle 처리 — 사용자 의도 정확히 받아쓸 것
- IPOPT 의 corridor_lower / corridor_upper sub-mm 위반에 대해 slack 외 다른 처리 (acceptable_constr_viol_tol 같은 IPOPT 자체 옵션 검토)

### B.4 별개 / 후순위

- **bag 1 의 1-obstacle stuck 패턴** (ego v=0 후 영원히 멈춤): 사용자가 stuck 얘기는 우선순위 낮춰달라고 했지만 실제로 OT 못 끝내는 원인. 다음 세션에서 별도 다뤄야.
- **mu_abs at v=0**: 정지 상태 yaw error 큰 케이스. controller / state estimator 영역 가능성.
- **opponent prediction 의 vd 신뢰도 게이팅**: 현재 vs/vd 단순 사용, variance 미활용.
- **horizon 길이 검토**: 현재 N=20, dT=0.05 → 1초. opponent + 정적 동시 plan 어려움. N=30~40 검토 (JIT 재컴파일 필요).
- **OT + GB 복귀 단일 plan**: 현재 cache 별개. terminal cost 강화로 통합 가능성.

### B.5 사용자가 명시한 룰 (다음 세션도 유지)

1. **장애물이 horizon 내에 있을 경우 절대 recovery / GB 경로 publish 금지** — Tier 2 차단 룰 (현재 적용됨)
2. **MPC 가 매 tick valid 한 trajectory 출력** — pass 2 의 TRAIL fallback 도 깨끗한 trailing 이어야
3. **slack 으로 어거지 풀이 금지** — slack 은 numerical guard (1cm) 만, 그 이상 늘리지 말 것
4. **여러 장애물 고려한 결정** — multi-obstacle aggregation (적용됨)
5. **정적 obstacle 은 절대 회피, 동적 obstacle 은 trailing 가능** — per-obstacle σ_n 으로 구현됨
6. **Switching 시 trailing 끼워서 안정** — swing_safe (forward distance only) 만 적용됨, mid-OT lateral guard 는 rollback

---

## §C. 이번 세션의 회고 (2026-04-27)

진행 흐름:
1. SideDecider 의 d_L/d_R lookup 흔들림 → wrap-aware lifter._interp 로 fix (검증 완료)
2. side_commit 을 hard corridor 로 박는 _apply_side_commit 추가 + range gate 3m
3. obs_arr 의 rear_window 추가 (modulo wrap 버그 차단)
4. min_pass_margin 0.10 → 0.15
5. SideDecider v4 — multi-obstacle / ego-aware / swing_safe
6. per-obstacle σ_n inflation (정적/동적 구분)
7. 사용자가 직전 bag 들 (13-12-51, 13-09-22) 분석 요청
8. 분석에서 pass 1↔2 fallback 진동, mid-OT side flip, lap-wrap glitch 발견
9. fix 시도 중 사용자가 "어거지" 라며 제동 → 추가 변경 (slack cap, mid-OT guard, ot_line trail tag, TRAIL static cap 제외) 모두 rollback

회고:
- 다층 시스템 (SideDecider / 솔버 pass / FSM mode / fallback ladder) 의 ripple effect 추론에서 약함
- IPOPT infeasibility 의 진짜 원인 분석에 깊이 부족 (raw infeas 데이터 vs 실제 IPOPT 내부 동작)
- 사용자 지적 받기 전까지 fix 의 근거가 약함 (추측 → 패치 → 사용자 지적 → roll back 의 사이클)

다음 세션의 자세:
- 코드 변경 전 root cause 의 직접적 증거 (raw IPOPT log, 단계별 trace) 확인
- "이 fix 가 X 를 해결한다" 의 X 가 사용자가 실제 본 증상과 정확히 일치하는지 확인
- 큰 재설계 (Strategic FSM) 가 필요할 수도 있음 — 작은 fix 누적이 한계 있음

---

## 0. Executive summary

현재 MPC 추월 파이프라인은 다음 4가지 고장 패턴을 보임:

1. **순간 corridor-width 기반 side flip** — `SideDecider` 가 매 tick t=0 snapshot 의 `(s0, n0, d_L, d_R)` 만 보고 LEFT/RIGHT 결정. 한쪽 corridor 가 `min_pass_margin=0.10` 아래로 떨어지면 hysteresis (10 ticks ≈ 0.33s) 가 풀리는 즉시 반대편으로 flip. **추월 commit 개념 없음**.
2. **추월 도중 L↔R flip** — 옆을 나란히 달리며 (|ds| < 1.5 m, dv ≈ +0.16 m/s) corridor 가 좌우로 번갈아 닫히면 ego 가 좌→우→좌 방향으로 핸들을 돌려가며 5초간 추월 실패. 곡률 spike 와 v_max_solver_min 클립으로 속도 손실.
3. **TRAIL 의 lateral 중립성** — TRAIL 은 vmax 캡만 적용. 실패한 쪽 (예: LEFT) 에서 retreat 한 뒤 RIGHT 추월을 노릴 lateral repositioning 이 없음. 단순 raceline 수렴.
4. **Abort 의미 부재** — closure rate (dv) 가 작거나 lateral 진행이 막혀도 시스템은 같은 side 추월을 계속 시도. 사용자가 원한 "안될 거 같으면 뒤로 빠지고 반대쪽 다시 노려" 가 구현 안 됨.

**Bag 증거 (2026-04-24-16-46-40, 23.4s, 696 ticks)**:
- side flip 9회 중 2회 가 mid-parallel L↔R (tick 77787 R→L, 77831 L→R).
- "stuck-parallel" run: 4.95초간 ego_v 가 2.0~5.0 m/s 사이에서 진동, opponent_v ≈ 1.84 m/s, dv 평균 0.16 m/s, kappa_max ≥ 2.2 rad/m 가 25 tick 연속, v_max_solver_min = 2.29 (corridor 가 곡률 캡).
- mpc_mode 는 WITH_OBS 에 5초 내내 갇혀있음 — TRAIL 한 번 안 거치고 LEFT 후 RIGHT 후 LEFT 로 진행.
- solver 자체는 700/700 성공, p99 = 33.66ms — **solver 가 아니라 decision layer 의 문제**.

**redesign 목표**:
- 추월을 **state-ful, commit-aware** 로 재설계. side 결정은 짧은 hysteresis 가 아니라 명시적 FSM 의 commit-and-hold 로 lock.
- **Predictive feasibility** — 한쪽 side 가 horizon 끝까지 (또는 commit 길이만큼) feasible 인지를 본다. opponent prediction 은 이미 solver 까지 도달 중 (`obs_arr[o, k, :]`); decider 가 같은 데이터를 쓰지 않는 것이 누락.
- **Strategic abort** — closure rate stall + lateral progress stall 시 명시적으로 ABORT, **opposite-side prep** 모드로 진입 후 재시도.
- **하루 단위 단계화** (CLAUDE.md 원칙):
  - **Day 1**: Side-Commit FSM + abort/retry 골격 — 본 bag 의 mid-parallel flip 을 잡아냄.
  - **Day 2**: Predictive feasibility window — flip 발생 전에 미리 가본다.
  - **Day 3**: TRAIL-with-prep — 진짜 "뒤로 빠진 뒤 반대쪽 도전" 구현.
  - **Day 4**: Map-aware tie-break + 튜닝 + 회귀.

---

## 1. 진단 — 무엇이 어디서 깨지는가

### 1.1 SideDecider 의 정보 단절 (코드 트레이스)

`planner/mpc_planner/src/side_decider.py:73-128` `decide(ego_v, obs_list)`:

| 입력 | 값 | 누락된 정보 |
|---|---|---|
| `ego_v` | 스칼라 | ego 의 향후 (s,n) 계획 (이미 `prev_traj` 가 있음) |
| `n0`, `w_o` | 장애물 t=0 위치 | k=1..N 단계의 `obs_arr[o, k, :]` (solver 는 이미 사용) |
| `dL`, `dR` | s0 한 점의 wall margin | s0 ± Δ 구간의 평균/최소 margin |
| `v_s_obs`, `ref_v` | 스칼라 | opponent 의 lateral 속도 `vd` (predictor 에 있음) |

→ **decider 가 t=0 정적 단면을 보고, solver 는 동적 horizon 을 본다**. 두 layer 가 다른 정보로 동작.

`planner/mpc_planner/node/mpc_planner_state_node.py:1470-1507` `_decide_side`:
- `obs_arr` (n_obs_max × (N+1) × 3) 가 들어옴.
- 단계 k=0 과 k=N 만 추출해서 `v_s_obs = (sN-s0) / (N·dT)` 산출.
- `obs_list` 빌드 후 `s0` 기준 정렬, decider 는 **첫 번째 (가장 가까운) 만** 평가 (line 78 `o = obs_list[0]`).
- 다중 장애물 시나리오 시 두 번째 장애물이 corridor 추가 압박해도 결정에 못 들어감.

### 1.2 Hysteresis 의 한계 (`side_decider.py:131-156`)

- LEFT↔RIGHT flip: `hold_ticks=10` (≈ 0.33 s @ 30 Hz). 10 tick 동안 같은 raw 결정이 나오면 풀림.
- TRAIL 진입: `trail_entry_ticks=3` (≈ 0.1 s).
- **commit 개념 없음**: ego_n 이 이미 한쪽으로 가있는 상태 (예: ego_n = -0.45, 즉 RIGHT 으로 절반 commit) 에서도 raw 가 LEFT 로 10 tick 떨어지면 LEFT 으로 flip. ego 의 현재 lateral state 와 무관한 결정.
- **bag 의 tick 77831 사례**: ego_n 이 이미 -0.19 (오른쪽으로 진행 중), 그러나 hysteresis 풀리고 LEFT 로 flip → solver 의 side_bias 가 +n 방향으로 swing. 추월 도중에 핸들이 반대편으로 갔다.

### 1.3 TRAIL 의 lateral 중립성 (`frenet_kin_solver.py:573-608, 714-719`)

- `side=SIDE_TRAIL` → `bL=0, bR=0` (one-sided side bias 0). **n 방향 명령 부재**. 
- vmax 만 ramp-down 으로 cap (`a_dec_ramp=3.0 m/s²` 으로 v0→ v_obs·0.95).
- 결과: TRAIL 동안 ego 는 단순히 raceline 수렴 (q_n_term=10 + q_n_ramp 가 끄는 곳). 다음 추월 시 lateral 시작점이 재사용되지 않음 → 다음 sample 에서 또 같은 side 로 commit 할 수도 (decider 가 prev_side 만 기억함).

### 1.4 MPC 2-state FSM 의 표현력 부족 (`mpc_planner_state_node.py:1542-1605`)

`_update_mpc_mode` 의 두 상태:
- `WITH_OBS`: 장애물 < `obs_enter_dist_m=5` m 에서 진입.
- `NO_OBS`: 장애물 > `obs_exit_dist_m=10` m 또는 stale 에서 진입.

→ **장애물이 시야 안에 있는 동안은 WITH_OBS 한 상태**. 이 안에서 추월 commit / abort / retry 의 sub-state 가 없음. Side 결정과 mode 결정이 직교. side 가 매 tick 바뀌어도 mode 는 그대로.

추월의 의미적 단계 (SCAN → COMMIT → PASS → ABORT → RETRY) 가 weight profile 1개 (`WITH_OBS_baseline`) 에 모두 압축됨. `q_n_ramp=8`, `w_obs=260`, `w_side_bias=55` 가 SCAN 과 PASS 와 ABORT 모두에서 동일하게 적용 → "commit 도중에는 raceline pull 약하게, abort 후엔 강하게" 같은 단계별 weight tuning 자체가 안 됨.

### 1.5 Predictor 데이터 흐름의 불균형

| 데이터 소비처 | t=0 snapshot | k=1..N prediction |
|---|---|---|
| `_decide_side` (decider 입력) | ✅ | ❌ (sN 1점만 v_s 추정에 사용) |
| solver `_build_obs_params` | ✅ | ✅ (obstacle bubble cost 의 k 별 ds, dn) |
| solver `_build_vmax` | ✅ (ref_v) | ❌ (TRAIL 시 v_obs 단일값으로 cap) |

→ predictor 의 `vd` (lateral velocity), variance 는 **side 결정에 안 들어감**. 사용자가 원하는 "예측 기반 전략" 의 데이터는 이미 publish 되고 있는데 decider 가 안 본다.

### 1.6 사소하지만 잡아둘 것

- `min_obs_ds` lap-wrap 처리는 [mpc_planner_state_node.py:1531-1532](planner/mpc_planner/node/mpc_planner_state_node.py#L1531-L1532) 에 있음 → bag 의 tick 77661 `min_obs_ds=627.99` 는 코드 상 wrap 적용된 결과 ‼️ ds 계산 자체가 아니라 `obs_arr` 단계에서 stale slot 이 활성화되어 들어왔을 가능성 (한 tick 만). 우선순위 낮음.
- `n_obs_max=2` — bag 의 `n_obs_used` 분포 확인 필요 (보통 1, 가끔 2). 다중 장애물 시나리오에서 더 늘릴 가치 있는지는 별도 측정 후 결정.

---

## 2. 설계 — 4-Phase Strategic Overtake Pipeline

### 2.1 핵심 구조

```
                                     [tick t]
                                        |
                  +---------------------+---------------------+
                  |                                            |
        _build_obstacle_array_frenet                obs_arr[o, 0..N, :]
        (이미 prediction × tracking merge)         + raceline n* + curvature
                  |                                            |
                  v                                            v
           ┌────────────────────────────────────┐
           │  StrategicOvertakeFSM (NEW)         │  ← 본 문서 §2.2
           │   states:                            │
           │     SCAN → COMMIT_L | COMMIT_R       │
           │       → PASSING                      │
           │       → ABORT_TRAIL_PREP_L|R         │
           │       → REENGAGE                     │
           │       → CLEAR                        │
           │   transitions: feasibility window,    │
           │   closure-rate, lateral-progress      │
           └─────────────────┬───────────────────┘
                             |
              (commit_side, target_n_prep, mode_tag)
                             |
           ┌─────────────────v────────────────┐
           │  Solver weights & TRAIL targets   │
           │  (frenet_kin_solver.update_weights│
           │   + new TRAIL n_target field)     │
           └─────────────────┬────────────────┘
                             |
                             v
                       NLP solve
```

기존 `SideDecider` 는 **`StrategicOvertakeFSM` 안에 inner module 로 흡수**. 외부 인터페이스 (`side_int, side_str, side_scores`) 는 호환 유지 — 노드 publish 부 (`tick_json.side_*`, RViz) 는 그대로 작동.

### 2.2 새 FSM — `StrategicOvertakeFSM`

**상태:**

| state | 의미 | side bias 출력 | vmax cap | n_target |
|---|---|---|---|---|
| `SCAN` | 추월할만한 장애물 평가 중 (commit 전) | 0 (없음) | none | 0 (raceline) |
| `COMMIT_LEFT` / `COMMIT_RIGHT` | 한쪽 side 로 lateral 진행 중 (장애물 미접근) | full (해당 방향) | none | side-specific n_pass |
| `PASSING` | 장애물 옆 평행 (\|ds\| < ds_passing) | full (commit 방향) | none | side-specific n_pass |
| `ABORT_TRAIL_PREP_L` / `ABORT_TRAIL_PREP_R` | abort 후 반대쪽 prep 으로 lateral repositioning | prep 방향 (반대) | v_obs · 0.95 | n_prep (반대쪽 0.4·d_avail) |
| `REENGAGE` | TRAIL prep 완료, 반대쪽 commit 직전 (transition buffer) | prep 방향 | none | n_pass (반대쪽) |
| `CLEAR` | 장애물 통과 완료 (`ego_s > obs_s + clear_dist`) | 0 | none | 0 |

**전환 규칙:**

```python
# 1) Scan → Commit
SCAN -> COMMIT_X: 
    if predictive_feasibility(side=X, window=k_pass) and side_score_X >= side_score_other + margin:
        commit to X
    elif both sides infeasible: -> SCAN (TRAIL is implicit while no commit yet — vmax via solver TRAIL pass2 fallback)

# 2) Commit → Passing
COMMIT_X -> PASSING:
    if |ds_obs| < ds_passing_enter (e.g. 1.8 m)

# 3) Passing → Clear (success)
PASSING -> CLEAR:
    if ego_s - obs_s > clear_dist (e.g. ego length + 0.5 m)

# 4) Passing → Abort (retry)
PASSING -> ABORT_TRAIL_PREP_OTHER(X):
    if any of:
        (a) closure_stall_ticks > N_abort_closure  (dv < dv_min for sustained ticks)
        (b) lateral_progress_stall_ticks > N_abort_lat
            (|ego_n - n_target_X| not decreasing in last K ticks)
        (c) corridor_X collapsed: d_free_X < min_pass_margin for K ticks
            BUT |ds_obs| < ds_passing_exit (i.e., still parallel)

# 5) Commit → Abort (early abort, before reaching parallel)
COMMIT_X -> ABORT_TRAIL_PREP_OTHER(X):
    if predictive_feasibility(side=X, window=k_pass) goes False for K_pred_abort ticks

# 6) Abort prep → Reengage
ABORT_TRAIL_PREP_OTHER(X) -> REENGAGE:
    if |ego_n - n_prep_OTHER| < eps_prep AND |ds_obs| > ds_reengage_min  (we're behind enough)

# 7) Reengage → Commit (other side)
REENGAGE -> COMMIT_OTHER(X):
    if predictive_feasibility(side=OTHER) AND closure_dv > dv_reengage_min
    (else stay in REENGAGE for retry_hold_ticks, then back to SCAN if still not feasible)

# 8) Any → Clear (장애물 사라짐)
* -> CLEAR:
    if obs not in horizon OR obs went stale OR obs_ds > clear_far_dist
```

**신호 정의:**

- `predictive_feasibility(side=X, window=k_pass)`:
  - For k in 0..k_pass: 평가 점 `(s_obs[k], n_obs[k])` 에서 effective wall (이미 적용된 wall_safe + inflation 후) 과 ego_half + gap_lat 까지 빼낸 free margin `d_free_X[k]`.
  - 모든 k 에서 `d_free_X[k] > min_pass_margin` 이면 feasible. 한 tick 이라도 떨어지면 False.
  - **선택지**: hard-AND (하나라도 떨어지면 False) → 보수적, abort 빠름. Soft (평균/최소 마진 비교) → 공격적, abort 느림. 우선 hard-AND 로 시작.
  - `k_pass` 는 ego 가 obstacle 을 통과하는 데 필요한 stage 수 추정: `k_pass = ceil(ds_obs / (ego_v · dT))`, max `N`. dv 가 작을수록 k_pass 가 N 에 가까워져서 더 보수적.

- `closure_stall_ticks`: 매 tick `dv = ref_v - v_s_obs` 을 EMA 후 `dv_ema < dv_min` (예: 0.6 m/s) 인 tick 수. dv_min 은 `trail_dv_thresh` 와 의미적으로 같은 임계값. **단, `_decide_side` 의 현재 `dv < trail_dv_thresh` 분기보다 우선순위 높이는 게 핵심**. 현재는 `only_X_fits` 분기에 가려져 dv 작아도 PASSING 진입함 (bag tick 77820 등).
- `lateral_progress_stall_ticks`: ego_n 의 commit 방향 진행률. `(ego_n[t] - ego_n[t-K]) · sign(target_n) <= eps_progress` 시 +1. 즉 target 쪽으로 못 가고 있으면 카운트.
- `n_pass`: commit side 의 통과 예상 lateral. 단순화: `n_pass_LEFT = n_o + w_o + ego_half + gap_lat + min_pass_margin`. (raceline n* 무관, 일단 obstacle 회피 우선.) Phase D 에서 raceline-aware 로 변경.
- `n_prep`: opposite side 의 lateral prep 위치. `n_prep_RIGHT = - (eff_dR · k_prep_frac)` (예: 0.4 · eff_dR, 즉 raceline 에서 오른쪽 trackbound 방향으로 40%). TRAIL 도중 ego 가 이 위치에 미리 가있어서 다음 commit 이 부드러움.

**상태 변수 (FSM 내부):**

```python
class StrategicOvertakeFSM:
    state: str  # 'SCAN' / 'COMMIT_L' / 'COMMIT_R' / 'PASSING' / 'ABORT_TRAIL_PREP_L' / 'ABORT_TRAIL_PREP_R' / 'REENGAGE' / 'CLEAR'
    committed_side: int  # SIDE_LEFT / SIDE_RIGHT / SIDE_CLEAR
    prep_side: int       # 다음 시도 side (abort 시 설정)
    closure_stall_ticks: int
    lat_progress_stall_ticks: int
    feasibility_fail_streak_L: int
    feasibility_fail_streak_R: int
    enter_state_tick: int
    abort_count_for_obstacle: int  # 같은 장애물에 대해 abort 한 횟수 (cap 으로 race-condition 방지)
    last_obs_id: int     # 다른 장애물로 넘어가면 reset
    # diagnostics
    last_reason: str
    last_scores: dict
```

**publish 인터페이스 (호환):**

`decide(ego_v, obs_list, ref_slice, ego_n, prev_traj_n_target)` → 반환값:
- `side_int`: SIDE_LEFT / SIDE_RIGHT / SIDE_TRAIL / SIDE_CLEAR (TRAIL = ABORT_TRAIL_PREP_* + REENGAGE 가 매핑됨)
- `side_str`: 동일 명명
- `scores`: 기존 필드 + `state`, `committed_side`, `prep_side`, `predictive_feas_L`, `predictive_feas_R`, `closure_stall_ticks`, `lat_progress_stall_ticks`, `abort_count`

이러면 `tick_json` 구독, RViz 색깔 코드, 노드의 `_apply_mode_weights` 호출부 변경 없이 FSM 만 교체.

### 2.3 Solver 측 변경 (TRAIL n_target)

`frenet_kin_solver.py:714-719` 의 `bL/bR` 로직 옆에 **TRAIL n_target** 추가:

기존:
```python
if side == SIDE_TRAIL or side == SIDE_CLEAR:
    bL = 0.0; bR = 0.0
```

변경:
```python
# TRAIL with intent: pull ego_n toward n_target_trail (opposite side prep).
# n_target_trail = 0 이면 기존 동작 (raceline 수렴) 과 동일.
# n_target_trail != 0 이면 그쪽으로 weak side bias + soft terminal n_target.
if side == SIDE_TRAIL:
    if n_target_trail > 0:
        bL = w_side_bias_trail; bR = 0.0       # LEFT 쪽으로 prep
    elif n_target_trail < 0:
        bL = 0.0;               bR = w_side_bias_trail  # RIGHT 쪽으로 prep
    else:
        bL = 0.0; bR = 0.0      # neutral trail
    # terminal n target offset (instead of fixed 0)
    q_n_term_offset = n_target_trail
```

`q_n_term` 텀은 `(n_N - q_n_term_offset)^2` 형태로 변경. Solver constructor 수정 (한 번만).

이 변경의 인터페이스:
- 새 yaml 파라미터: `w_side_bias_trail` (예: 30, 보통 `w_side_bias=55` 의 절반), `q_n_term_trail` (기존 q_n_term 재사용 가능).
- node → solver 인자: `n_target_trail` (FSM 의 `n_prep` 직접 전달).

### 2.4 노드 측 변경 (Phase X FSM 과의 결합)

`_update_mpc_mode` 의 2-state FSM (`WITH_OBS`/`NO_OBS`) 는 **유지**. weight profile 토글 역할이 명확함 (`no_obs_profile` 으로 GB 수렴).

**추가**: `StrategicOvertakeFSM.state` 가 변할 때마다 node 가 추가로 weight overlay 를 적용:

| FSM state | overlay | comment |
|---|---|---|
| `SCAN` | none (WITH_OBS baseline) | observe 단계, raceline 따라감 |
| `COMMIT_L/R` | `w_side_bias` ↑ (예: 55→80), `gamma_progress` ↑ (10→14) | commit 강화, sprint |
| `PASSING` | `q_n_ramp` ↓ (8→4), `w_cont` ↑ (200→300) | parallel 동안 lateral 안정 |
| `ABORT_TRAIL_PREP_*` | `w_side_bias_trail` 켜짐, vmax cap, `q_n_term` 으로 prep n target | TRAIL with intent |
| `REENGAGE` | `w_side_bias` 새 방향, vmax 해제 | re-commit 직전 |
| `CLEAR` | NO_OBS profile | 기존 동일 |

이 overlay 는 `_apply_mode_weights` (Phase 1.5 에서 도입된 hot-swap) 인프라를 그대로 사용. 새 weight set 추가 + α 보간만.

### 2.5 Predictive feasibility 의 데이터 소스

decider 가 받는 입력 확장:

```python
def decide(self, ego_v, ego_n, obs_predictions, ref_slice, prev_traj):
    """
    obs_predictions: list of dicts, each {
        'id': int,
        's_traj': np.ndarray (N+1,),    # k=0..N predicted s
        'n_traj': np.ndarray (N+1,),    # k=0..N predicted n  ← NEW
        'half_width': float,
        'v_s': float,
        'predict_age_s': float,
    }
    ref_slice: existing (ref_s, d_left_arr, d_right_arr, ref_v)
    prev_traj: previous solve trajectory n[k] (for lateral progress check)
    """
```

데이터 소스:
- `obs_arr[o, k, 0]` → `s_traj[k]`
- `obs_arr[o, k, 1]` → `n_traj[k]`  
  (이미 `_build_obstacle_array_frenet` 의 vd 전파 결과로 채워져 있음. 단, vd 가 0 일 때 (정적 장애물 / vd 추정 부재) 모든 k 에 같은 값 → soft fail 됨.  vd 추정 신뢰도 게이팅 필요 — `predict_age_s`, prediction variance 등.)

`predictive_feasibility(side, k_pass)`:
```python
def predictive_feasibility(self, side, obs, ref_slice, k_pass):
    s_o = obs['s_traj']; n_o = obs['n_traj']; w_o = obs['half_width']
    rs  = ref_slice['ref_s']; dL = ref_slice['d_left_arr']; dR = ref_slice['d_right_arr']
    fail = 0
    for k in range(min(k_pass, len(s_o))):
        # walls at obs_s[k]
        idx = np.argmin(np.abs(rs - s_o[k]))
        eff_L = dL[idx] - self.wall_safe - self.inflation
        eff_R = dR[idx] - self.wall_safe - self.inflation
        if side == SIDE_LEFT:
            d_free = eff_L - (n_o[k] + w_o) - (self.ego_half + self.gap_lat)
        else:
            d_free = (n_o[k] - w_o) - (-eff_R) - (self.ego_half + self.gap_lat)
        if d_free < self.min_pass_margin:
            fail += 1
    return fail == 0  # hard-AND
```

soft 버전: `min_d_free_over_k_pass` 반환, threshold 비교는 호출자가.

### 2.6 사소한 보강

- `n_obs_max` 측정: bag 분석 시 `n_obs_used` 분포 (현재 81% > 0). 두 번째 장애물이 자주 active 가 되는지 확인 후 `n_obs_max=3` 검토.
- `tick_json` 확장:
  - `fsm.state`, `fsm.committed_side`, `fsm.prep_side`,
  - `fsm.closure_stall_ticks`, `fsm.lat_progress_stall_ticks`,
  - `fsm.predictive_feas_L`, `fsm.predictive_feas_R`,
  - `fsm.abort_count`, `fsm.last_reason`.
  bag 재현 시 mid-overtake flip 추적용.
- RViz: state 별 색깔 (SCAN=cyan, COMMIT_L=blue, COMMIT_R=red, PASSING=yellow, ABORT_*=purple, REENGAGE=orange, CLEAR=green) — visual debug 빠르게.

---

## 3. Day-by-day plan

CLAUDE.md "한 기능 / 한 재설계는 하루 안에" 원칙. 각 Day 끝에 **MVP 가 돌아가는 상태** 도달.

### Day 1 — Side-Commit FSM + Abort/Retry 골격

**목표**: 본 bag 의 mid-parallel L↔R flip 을 제거. closure stall 시 TRAIL 진입.

**구현 범위**:
1. `StrategicOvertakeFSM` 클래스 작성 (`planner/mpc_planner/src/strategic_overtake_fsm.py`).
   - 8 states + transition table. `predictive_feasibility` 는 **t=0 snapshot 기반 (기존 SideDecider 로직 호출)** 로 임시 시작 — Day 2 에서 multi-step 으로 교체. 
   - commit-once-and-hold 만 보장: COMMIT_X 진입 후 `|ds_obs| < ds_commit_lock` 이면 X→OTHER flip 금지 (ABORT 경로로만 빠짐).
   - closure_stall, lat_progress_stall 카운터 + abort 트리거.
2. `_decide_side` 를 FSM 호출로 교체. 호환 인터페이스 유지 (`side_int, side_str, side_scores`).
3. 기존 `SideDecider` 는 deprecate 하지 말고 inner helper 로 살림 (`SideDecider.eval_side_score` 만 사용, hysteresis/state 부분은 dead).
4. yaml 파라미터 추가 (state_overtake.yaml):
   ```yaml
   strategic_fsm_enable: true
   ds_passing_enter: 1.8
   ds_passing_exit: 2.5
   ds_commit_lock: 2.5
   ds_reengage_min: 1.5
   ds_clear: 1.0      # ego_length + buffer
   dv_min: 0.6
   N_abort_closure: 30   # ticks (≈1 s @ 30 Hz)
   N_abort_lat: 30
   K_pred_abort: 6
   eps_progress: 0.02   # m / K-window
   eps_prep: 0.10
   abort_max_per_obstacle: 2
   ```
5. `tick_json` 확장 (fsm.* 필드).
6. RViz state-color 마커.
7. 백업: `side_decider.py` → `_backup_20260425.py`, `mpc_planner_state_node.py` → `_backup_20260425_strategic.py`.

**검증 시나리오** (모두 docker bag-replay 또는 live):
- bag `2026-04-24-16-46-40` replay 후 `tick_json` 확인 — 77787~77908 의 9 flip 이 1~2 commit + 1 abort + 1 retry 로 압축되는지.
- mpc_mode 가 한 번이라도 ABORT_TRAIL_PREP_R 을 거쳐 REENGAGE 로 가는지.
- jitter_rms p95 < 0.05 m, solve_ms p99 < 50 ms 유지.

**완료 기준**: bag replay 시 mid-parallel L↔R flip 0회. 있다면 `ds_commit_lock` / `ds_passing_*` 튜닝.

### Day 2 — Predictive feasibility window

**목표**: Day 1 의 t=0 snapshot 을 multi-step (k=0..k_pass) horizon-aware feasibility 로 강화. flip 발생 전에 미리 알아채고 ABORT 트리거.

**구현 범위**:
1. `predictive_feasibility(side, obs_pred, ref_slice, k_pass)` 함수 작성. hard-AND + soft-min 둘 다 반환.
2. `_decide_side` 의 obs 변환부 — `s_traj[k]`, `n_traj[k]`, `predict_age_s`, prediction variance 추출. `obs_arr[o, k, :]` 가 이미 (s, n, w) 라서 자명.
3. FSM 의 `COMMIT → ABORT` early-abort: `K_pred_abort` tick 연속 feasibility=False 면 ABORT.
4. `vd` 신뢰도 게이팅: prediction freshness < 0.2 s + (옵션) variance 임계 미만일 때만 multi-step 사용. 그렇지 않으면 t=0 snapshot fallback.
5. `tick_json` 에 `predictive_feas_L`, `predictive_feas_R`, `min_d_free_L_over_k`, `min_d_free_R_over_k`, `k_pass_used` 추가.

**검증**:
- bag replay: tick 77787 의 R→L flip 이 발생하기 **전** (예: 77770~77780 구간) `predictive_feas_R` 가 미리 False 로 떨어지는지.
- 정적 장애물 시나리오 (vd=0) 에서 multi-step 과 t=0 결과 일치하는지.

**완료 기준**: 본 bag 의 mid-parallel 사례에서 ABORT 가 PASSING 이전에 트리거. dummy 장애물 (정적) 시나리오 회귀 통과.

### Day 3 — TRAIL with prep (lateral repositioning)

**목표**: ABORT 시 ego 가 단순 raceline 수렴이 아니라 **반대쪽 prep 위치로 lateral move**. REENGAGE 후 다음 commit 이 깔끔.

**구현 범위**:
1. Solver `frenet_kin_solver.py` 변경:
   - `n_target_trail` 인자 추가 (default 0).
   - terminal cost: `q_n_term · (n_N - n_target_trail)^2`.
   - side bias: TRAIL + `n_target_trail != 0` 시 `w_side_bias_trail` 으로 prep 방향 약한 bias.
   - JIT 캐시 무효화 — 한 번 cold-start 받기 (10~20s).
2. yaml: `w_side_bias_trail: 30`.
3. FSM: `n_prep` 계산 — `n_prep_RIGHT = -0.4 · eff_dR_at_obs_s`, `n_prep_LEFT = +0.4 · eff_dL_at_obs_s`. ABORT_TRAIL_PREP_X 진입 시 결정, REENGAGE 까지 유지.
4. node `_apply_mode_weights` 에 TRAIL prep overlay 추가.

**검증**:
- bag replay 후 ABORT 시점부터 ego_n 이 실제로 prep 쪽으로 이동하는지 (`tick_table.csv` ego_n trace).
- REENGAGE 진입 시 |ego_n - n_pass_OTHER| 가 commit 시작점보다 작아지는지.
- Live: 같은 시나리오 재구성해서 abort → trail → opposite-side commit → pass 흐름 확인.

**완료 기준**: bag 의 stuck-parallel 5초 → 시뮬레이션 기준 ABORT (1.5s 정도) → TRAIL prep (1.5s) → 반대쪽 PASSING (1.5s) 로 분해.

### Day 4 — Map-aware tie-break + 종합 튜닝

**목표**: side score 가 비슷할 때 raceline n* / 곡률 / overtake-sector 를 고려해서 최적 side 선택.

**구현 범위**:
1. `predictive_feasibility` 에 `score = min_d_free - λ_curv · max_kappa_at_obs_s - λ_offline · |n_pass - n_star|` 추가.
2. `λ_curv`, `λ_offline` yaml 노출.
3. `OvertakingSectorScaler` (`/global_waypoints/overtaking`) 정보 활용: ot_sector 안이면 `gamma_progress` 살짝 ↑, 밖이면 보수적.
4. 종합 튜닝:
   - bag 시나리오 회귀 (사용자 보유 bag 이 더 있으면 추가).
   - 정적 wall 회피 회귀.
   - `n_obs_max=3` 으로 늘려서 두 번째 장애물 시나리오 확인 (필요시).

**검증**:
- 곡률 50% 이상 차이 나는 좌우 corridor 시나리오에서 직선 쪽 선택 빈도 ≥ 80%.
- 단순 시나리오에서 raceline n* 와 같은 부호 선택 빈도 ≥ 70%.

**완료 기준**: bag 의 stuck-parallel + 새로 만든 곡률-편향 시나리오 둘 다 통과. mpc_planner_state_machine_integration.md Phase Y 진입 (state_machine 직결) 가능 상태.

---

## 4. 위험 요소 / 트레이드오프 / 보류 사항

### 4.1 trade-offs

| 결정 | 보수적 | 공격적 | 채택안 |
|---|---|---|---|
| `predictive_feasibility` AND vs soft-min | hard-AND: 한 k 실패 시 ABORT, abort 빈번 | soft-min: 평균 마진 비교, 통과 다수 | hard-AND 시작 (Day 2) → Day 4 에서 soft 옵션 |
| `ds_commit_lock` | 작게 (0.5m): commit 후에도 close range flip 허용 | 크게 (3m): 멀리서부터 lock | 2.5m (장애물 길이 + ego 길이 + 안전) |
| `abort_max_per_obstacle` | 1: 한 번 abort 시 다음 추월 시도 X | ∞: 무한 retry | 2 (좌→TRAIL→우→TRAIL→좌 까지 허용 후 그냥 trail) |
| `n_prep_frac` | 0.2: 약하게 prep | 0.6: 멀리 prep | 0.4 (raceline 에서 살짝 떨어진 위치, REENGAGE 시 commit 부담 적음) |

### 4.2 위험

- **JIT cold-start 추가 비용**: solver constructor 변경 → 첫 tick ~15s 컴파일. 매 launch 시 발생. 받아들임 (`ipopt_jit:true` 정책 유지).
- **TRAIL n_target 변경이 v_max ramp 와 결합**: prep 방향 lateral move 도 vmax 캡 안에서 일어나야 함. 곡률 + lateral velocity 결합 시 v_max_solver_min 이 더 떨어질 수 있음. Day 3 검증 필수.
- **State 폭발**: 8개 + transition 표가 복잡해보이지만 의미적으로 "scan / commit-then-pass / abort-and-retry" 셋. transition 단위 unit test 작성 권장 (`test_strategic_fsm.py`).

### 4.3 보류 사항

- **Multi-obstacle commit**: 두 번째 장애물이 commit side 코앞에 또 있으면? (예: 좌측 commit 했더니 다음 코너에 또 좌측에 정적 obs.) Day 4 이후. 임시: `n_obs_max=2` 안에서 두 번째 obstacle 도 같은 feasibility 평가, 두 obs 모두 같은 side feasible 일 때만 commit 유지.
- **Opponent prediction variance 활용**: 신뢰도 낮은 prediction (예: 큰 σ) 일 때 t=0 snapshot fallback. variance topic 에 어떤 형식으로 publish 되는지는 prediction 노드 코드 확인 필요 (`prediction/gp_traj_predictor/src/3d_opp_prediction.py`).
- **State machine 측 직결**: 본 redesign 은 MPC 내부 결정만 다룸. 외부 state_machine 의 `OVERTAKING ↔ TRAILING` 플래핑 (bag 에서 5s 동안 14회) 은 별개 문제. 본 FSM 이 안정화되면 그 출력 (`fsm.committed_side`, `fsm.state`) 을 state machine 이 구독해서 자체 hysteresis 강화 가능. mpc_planner_state_machine_integration.md Phase Y 에서 다룸.

---

## 5. 검증 도구 (이미 갖춘 것 + 추가)

### 5.1 이미 있는 것

- `tick_json` debug topic — Day 1~4 모두 활용. 새 필드 추가만.
- `rosbag info / echo` — 본 분석에 활용한 흐름 그대로.
- bag `2026-04-24-16-46-40.bag` — Day 1 회귀 기준.

### 5.2 추가할 것

- **Unit test**: `planner/mpc_planner/test/test_strategic_fsm.py` — 인공 obs trace 로 transition table 검증.
- **Bag-replay analyzer**: `HJ_docs/scripts/replay_strategic_fsm.py` — bag 의 t별 (s, n, ego, obs) 입력을 FSM 에 injecting → state trace 출력 + visualization. 한 번 만들면 Day 2~4 회귀 자동화.
- **시나리오 카탈로그**: `HJ_docs/overtake_scenario_catalog.md` (신규) — bag 시나리오 + 새 합성 시나리오 (정적 wall, 양쪽 곡률, 좁은 corridor 등) 정리. Day 4 에서 작성.

---

## 6. 변경되는 파일 (계획)

| 파일 | 변경 | 단계 |
|---|---|---|
| `planner/mpc_planner/src/strategic_overtake_fsm.py` (신규) | FSM 클래스 | Day 1 |
| `planner/mpc_planner/src/side_decider.py` | inner helper 로 축소 (또는 deprecate 커멘트) | Day 1 |
| `planner/mpc_planner/node/mpc_planner_state_node.py` | `_decide_side` → FSM, `_apply_mode_weights` overlay 추가, `tick_json` 확장 | Day 1 ~ Day 3 |
| `planner/mpc_planner/src/frenet_kin_solver.py` | `n_target_trail` 인자, terminal cost 형태 변경 | Day 3 |
| `planner/mpc_planner/config/state_overtake.yaml` | 새 파라미터 (Day 1 5 + Day 2 2 + Day 3 1 + Day 4 2) | Day 1 ~ Day 4 |
| `planner/mpc_planner/config/state_recovery.yaml` | (Day 4) RECOVERY 모드용 동일 파라미터 동기화 | Day 4 |
| `planner/mpc_planner/test/test_strategic_fsm.py` (신규) | unit test | Day 1 |
| `HJ_docs/scripts/replay_strategic_fsm.py` (신규) | offline analyzer | Day 2 |
| `HJ_docs/overtake_scenario_catalog.md` (신규) | 시나리오 카탈로그 | Day 4 |
| `TODO_HJ.md` | Phase 5 / Phase 6 항목 추가 | Day 1 시작 |

backup: `*_backup_20260425.{py,yaml}` 규칙 (CLAUDE.md).

---

## 7. 다음 세션 첫 액션 (Day 1 진입)

1. 본 문서 읽기 (§0, §3 Day 1).
2. `git status` 로 현재 working tree 상태 점검 (CLAUDE.md git 원칙).
3. `mpc_planner_state_node.py`, `side_decider.py`, `frenet_kin_solver.py`, `state_overtake.yaml` 백업.
4. `strategic_overtake_fsm.py` 작성 (§2.2 의 transition table 그대로 코드화).
5. `_decide_side` 교체 후 docker 빌드.
6. bag replay (offline) → `tick_json` 확인 → 본 문서 §3 Day 1 검증 기준.
7. 같은 세션 안에 ABORT/RETRY 가 한 번이라도 도는 상태까지 도달.

---
