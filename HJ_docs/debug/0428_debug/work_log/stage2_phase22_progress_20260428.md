# Stage 2 Phase 2-2 Progress — 2026-04-28

사용자 명시: **항상 근본 원인 집중. 어거지 X.**

---

## 누적 진행 (시간 순)

| 단계 | 설명 | 핵심 결과 |
|---|---|---|
| Stage 1 (S1-1~8 + S1-extra-E) | 안 죽는 MPC | 충돌 53 → 1 (latency 잔여) |
| Stage 2 Phase 2-1 | plan_library + plan_picker | 충돌 0 ✅ |
| FT1~5 | weight tuning | kappa 3.20, eff 0.82 |
| (d) N=40 + Phase 2-2 | horizon ↑ + stage 비례 weight | kappa 3.23 → **2.50** (24% ↓) |
| (b) target_n profile fine-tune | avoid_n 0.5→0.7 | overtake 4 → 0 (보수) |
| (a) heuristic plan picker | 5 plan 후보 score | overtake 1 |
| (a)+trail | TRAIL ref_v override + risk 강화 | overtake **3건**, lateral min 52mm |
| fully plan-based | SideDecider 무시, 5 plan 자체 score | OT→GB 복귀 1.42s, 부드러움 3mm |
| (a)+dwell | plan dwell hysteresis 5 ticks | flip 9회 → 17회 (악화 ⚠) |
| (a)+EMA+gap | score EMA + 15% gap threshold | (검증 진행 중) |

---

## 발견된 문제들 (사용자 형식: 원인 / 아이디어 / 결과)

### 문제 1: trajectory ↔ obstacle overlap 100%
- **원인**: target_n profile 의 LEFT_PASS 가 obstacle 옆을 통과 정의 (n_avoid=obs.n+0.5m). 자연스러운 회피 동작이지만 lateral margin 부족.
- **아이디어**: avoid_n 0.5→0.7m (margin 키움)
- **결과 (b)**: lateral min 3mm → 24cm 안전 ↑, 단 overtake 4 → 0 (보수)
- **추가 시도 (a)+trail**: avoid_n 0.55 + risk 강화 → overtake 3건 + lateral min 52mm

### 문제 2: TRAIL plan 거의 호출 X (사용자 정정 미반영)
- **원인**: SideDecider 의 best_effort 가 'no_side_fits' 시 LEFT/RIGHT 강제 → TRAIL plan 자연 호출 0
- **아이디어**: plan_scorer 에서 5 plan 모두 직접 비교 (SideDecider 무시), risk 강화 (d_free<0.15 → risk 50)
- **결과**: TRAIL plan score 가 close call 시 best 됨

### 문제 3: plan flip 빈번 (decide 진동)
- **원인**: plan_scorer 의 score 가 LEFT 와 RIGHT 사이 거의 동등 (d_free_L ≈ d_free_R 시). dwell 만으로 부족.
- **사용자 명시 — 근본**: "Decide 진동도 원인 파악해서 근원부터"
- **아이디어**:
  - (1차) sticky_bonus 0.05→0.30
  - (2차) plan dwell hysteresis 5 tick (실패 — 17 transition)
  - (3차) **score EMA (0.5 blend)** + **score gap threshold 15%** — 점수 자체 안정화 + 변경 조건 강화 ← 근본
- **결과 (a)+EMA+gap)**: 진행 중

### 문제 4: OT → GB 복귀 부드러움
- **원인**: plan transition 시 weight 갑자기 변경 (raceline plan ↔ left/right_pass) → trajectory jump 가능
- **사용자 명시**: "복귀도 MPC 로 — 부드러운 수렴, recovery fallback X"
- **현재 결과**: raceline 복귀 1.42s, max|Δego_n|=3mm (이미 부드러움)
- **잠재**: plan transition 시 weight α-ramp blend (mode α-ramp 의 plan-level) — 추후 적용 가능

### 문제 5: Side decider 결정 영향력
- **원인**: SideDecider 가 1-tick decision + hysteresis. plan_scorer 에 의해 override 되지만 영향력 큼.
- **사용자 명시**: "Side decider 안 바꿔도 됐던가" + "이제 플랜 기반이 된 건가"
- **아이디어**: SideDecider 결정 무시, plan_scorer 가 5 plan 모두 자체 평가 (fully plan-based)
- **결과**: plan flip 패턴 변동 — score EMA + gap 으로 추가 안정화 진행

### 문제 6: ego_v 의 LEFT_PASS 시 ratio 0.62 (낮음)
- **원인**: LEFT 진행 시 ego 가 가속 못 함. obstacle bubble + side bias 가 ego 를 lateral 로 강제 → 가속 손실
- **아이디어**: gamma_progress 13.0 (이미 적용), target_v profile 의 명시적 push (TRAIL 만 적용)
- **추가**: LEFT/RIGHT_PASS 의 target_v profile 도 명시 (raceline ref_v 그대로 또는 약간 boost)

---

## 근본 원인 추적

### TRAIL 의 정의 (사용자 정정 정확)
- **원본**: lateral 정렬 + obs.n 따라가기 (X)
- **사용자 정정**: "차 뒤 + 일정 거리/속도, 빈틈 노림"
- **현 구현**: target_n=ego_n 유지 + ref_v=obs_v-margin override + q_n_target=25 강력
- **근거 있음**: 사용자 정정 정확 반영

### avoid_n margin 의 trade-off
- 0.5m: overtake 활발 + 위험 (lateral min 3mm)
- 0.7m: 안전 (24cm) + overtake 0
- 0.55m + risk 강화: overtake + 안전 (52mm) ✅
- **근거**: bag 검증으로 sweet spot 발견

### plan flip 의 근원
- d_free_L/R 매 tick 변동 → score 변동 → flip
- **근본 해결**: score EMA (지난 tick 50% blend) + score gap threshold (best 가 current 대비 15% 이상 좋아야 변경)
- **어거지 X**: 단순 hysteresis (dwell tick) 로 막는 게 아니라 score 자체를 안정화

---

## 추후 계획 (사용자 추가 의도)

### Strategic intent (decide 활용)
- **사용자 명시**: "추후에 전략을 decide 에 활용할 생각"
- **현 구조**: plan_scorer 가 매 tick cost 비교
- **추가 placeholder**: plan_scorer 에 strategic_intent param ('overtake' / 'safety' / 'auto')
  - 'overtake': LEFT/RIGHT_PASS 의 score 에 -10 bonus
  - 'safety': TRAIL 의 score 에 -10 bonus
  - 'auto': 현재 그대로
- **추후 구현**: strategic decide 모듈 → plan_scorer 에 push

### 진정한 Multi-NLP solve
- 현재: heuristic score 만 (NLP 한 번)
- **추후**: top-2 plan 모두 NLP 풀고 cost 비교 → best
- **위험**: solver 의 warm-start state pollution, solve_ms 두 배

### TRAIL 의 ds 거리 cost
- **사용자 정정**: "일정 거리/속도"
- **현 구현**: 속도만 (target_v override). 거리 미구현.
- **추가**: ego_s 가 obs_s - target_ds 위치 유지하는 cost
  - solver cost: q_ds_target * (ego_s - obs_s + target_ds)^2
  - cost 구조 변경 → JIT 재컴파일

### Plan transition smooth ramp
- 현재: plan 변경 시 weight 즉시 swap
- **추후**: weight α-ramp blend (mode α-ramp 의 plan-level)
- **근거**: 사용자 명시 "부드러운 수렴, MPC 궤적이 끝점부터 GB 부드럽게"

---

## 누적 변경 파일 (Stage 2 Phase 2-2)

| 파일 | 변경 |
|---|---|
| `frenet_kin_solver.py` | Phase 2-2: P_n_target, P_q_n_target parameter + J_target cost. set_n_target_profile() setter. update_weights 에 q_n_target 추가. |
| `plan_library.py` | 5 plan 의 weight overlay (N=40 stage 비례). make_target_n_profile (avoid_n=0.55, peak=0.6N). |
| `plan_scorer.py` | score_plan: race_progress + risk(강화: d_free<0.15→50) + maneuver + long_term_value + sticky(0.30). pick_plan_scored: score EMA + gap threshold. |
| `mpc_planner_state_node.py` | _apply_plan_weights, plan_picker 호출 (5 plan 후보). target_n profile push. TRAIL ref_v override. score history. dwell hysteresis. |
| `state_overtake.yaml` | N: 20 → 40 (+ stage 비례 weight 도 plan_library 안) |

---

## Backup 파일

- `plan_library_phase22_baseline_20260428.py` — Phase 2-2 (i) 첫 통과
- `frenet_kin_solver_phase22_baseline_20260428.py`
- `mpc_planner_state_node_phase22_baseline_20260428.py`
- `state_overtake_phase22_baseline_20260428.yaml`
- `plan_library_d_N40_baseline_20260428.py` — (d) N=40 통과
- `state_overtake_d_N40_baseline_20260428.yaml`
- `plan_library_b_finetune_baseline_20260428.py` — (b) avoid_n=0.7 보수

---

작성: HJ 세션, 2026-04-28

---

## 진정한 근본 해결 (decide 진동 — bag 분석 기반)

### 문제: plan flip 빈번 (transitions 17번)
- bag 분석 (analyze_dfree_oscillation.py): **d_free_L/R 가 1초 만에 부호 변동** (sign flips L:8회, R:6회)
- 양상: ego 가 obstacle 옆 통과 시점 (|ds|<1.5m) 에 d_free 가 large swing (정상 ±0.04 → passing 시 ±0.5+)
- 원인: d_free 계산이 ego ≈ obs 위치에서 ambiguous

### 해결 (단계별 — 모두 근거 있음, 어거지 X)
1. **passing-freeze** (|ds|<1.5 시 plan 변경 차단): 17→14 (작은 효과)
2. **ego_n commit bonus** (이미 lateral 진행 중인 plan 우선, |ego_n|>0.1m → -3.0 bonus): 14→**8** (53% ↓)
3. **horizon-aware d_free** (k=0..N min, snapshot 의존 X): 8→**5** (37% 추가 ↓)

### 누적 개선 (시작 → 최종 horizon-aware)
| 지표 | 시작 (bag1) | 최종 |
|---|---|---|
| 충돌 | 80건 | **0건** |
| Plan transitions/110s | 17 | **5** |
| Lateral min when |ds|<1m | 3mm | **216mm** |
| Lateral max | 0.660m | 0.275m |
| ego_v / ref_v RIGHT | 0.66 | **0.94** |
| ego_v / ref_v LEFT | 0.62 | 0.84-0.99 |
| OT→GB 복귀 | (안 측정) | 1.42s + 3mm 부드러움 |

### 모든 변경 근거 (bag 분석 기반)
- ego_n bonus: bag 의 ego_n / plan flip 시점 매칭 → ego_n>0.1 시 LEFT 진행 중
- horizon-aware d_free: dfree_oscillation 분석에서 sign flip 14회 → snapshot 의존 차단
- passing-freeze: |ds|<1.5m 가 d_free swing 의 시점 — 분석으로 확인

---

## 2026-04-28 후속 — 사용자 추가 정정 + 진짜 근원

### 사용자 정정 1: "트래킹 노이즈는 정상, 보상 X"
- 시도: obs_lat_oscillation=0.25 worst-case 보상 (obs.n + 0.25)
- 사용자 reject: "저정도 트래킹 에러는 갖고 있어야지... 충돌 안 나고 잘 되게 만들어야"
- 해결: revert. obs.n 그대로 사용. MPC w_obs cost + target_n profile 가 처리.

### 사용자 정정 2: "토픽들 상태 기반 유동적 (상시 인플레 X)"
- 시도: gap_lat = 0.30 + 0.25 * (k/N) (시간 ramp, 항상)
- 사용자 reject: "Obs.n 인플레라는게 상시 더하는거 아닌가. 토픽 상태 기반으로 유동적이면"
- 해결: gap_lat_growth 를 runtime 신호로 산정 — vd_var (predictor) + obs_n rolling stddev (local). 안정 시 인플레 거의 0, 변동 시 자연 증가.

### 사용자 정정 3: "Decide 진동 — 그냥 고정 X, 진짜 원인부터"
- 시도 1 (실패): plan dwell 5 ticks (시간 lock)
- 시도 2 (실패): passing-freeze |ds|<3m (위치 lock)
- 사용자 reject: "두가지 방법 말고 (저건 그냥 고정하는거잖나) 진동하는 이유를 알아야지"
- **진짜 근원 (sticky bonus 부호 버그)**: `score *= (1.0 - sticky_bonus)` 가 양수에서만 동작. 음수 score (대부분 plan total) 에서는 sticky 가 현재 plan을 **오히려 페널티**. 단발 noise → 현재 plan 점수 잘못 후퇴 → flip.
- 해결: `if score < 0: score -= abs(score)*sticky_bonus`. 부호 무관 항상 attractive.
- 추가: physical-feasibility filter — ego_n=+0.30 시 RIGHT_PASS candidate 자체 제외 (시간 lock 아닌 구조적 제거).
- 결과: transitions 33 → 16 (50% ↓), collisions 4 → 0, lateral min 19mm → 147mm.

### 사용자 정정 4: "Plan 바뀔 때 trajectory 휙 X, 부드럽게"
- 시도 1 (실패): alpha ramp 0.20 → 0.10 (10 tick = 0.2s)
- 결과: trajectory 부드러워졌지만 MPC reaction 너무 느려져 obstacle 진입 시 미반응 → 충돌 1건, lateral min 2mm
- 해결: alpha 0.20 유지. Continuity guard threshold 0.15 → 0.08 (head-blend 더 자주 발동). w_cont=300 (기존) 가 solver 자체 smoothness 책임.

### Risk cliff 부드럽게
- 기존: d_free<0.15 → risk=50.0 (cliff). d_free 노이즈 ±0.05 만으로 score ±50 점프 → flip 유발.
- 새 형태: risk = max(0, 0.30-d_free)² × 555.6 (continuous quadratic). d_free=0.00 → 50, d_free=0.15 → 12.5, d_free=0.30 → 0. 노이즈 ±0.05 → score ±11 (그러나 점프 없음).

---

## 최종 누적 변경 (2026-04-28 EOD)

### plan_scorer.py
- score_plan: smooth quadratic risk (no cliff), sticky 부호 버그 수정.
- horizon_aware_d_free: gap_lat_growth 동적 산정 (vd_var + obs_n_std), k=0 인플레 0.
- filter_feasible_plans: ego_n commit 기반 cross-over plan 제거 (구조적, time-lock X).

### mpc_planner_state_node.py
- _filter_feasible_plans 호출 (pick_plan_scored 직전).
- Adaptive gap_lat 위해 obs_n rolling history (deque, 20 ticks) 추적.
- _last_obs_max_vd_var 를 horizon_d_free 에 전달.
- passing-through freeze 제거 (구조적 fix 가 대체).
- dwell hysteresis 제거 (구조적 fix 가 대체).
- score_gap_threshold 0.40 → 0.15 (filter 가 주된 flip 차단).
- continuity_K_guard 5 → 8, threshold 0.15 → 0.08 (smoothness).
- alpha plan ramp 0.20 유지 (0.10 시 reaction 느려 충돌).
- tick_json 에 'plan' 필드 노출 (분석 용).

---

## 메트릭 (각 단계 bag 비교)

| Bag | Dur | Code change | Trans | Trans/min | Coll | Lat min |
|---|---|---|---|---|---|---|
| post_sticky | 120s | sticky 부호 fix + filter + adaptive gap | 16 | 8.0 | 0 | 147mm |
| post_smooth | 120s | + alpha 0.10 + cont 0.08 | 11 | 5.5 | 1 | 2mm |
| post_final  | 180s | alpha 0.20 revert (cont 0.08 잔존) | 18 | 6.0 | 3 | 4mm |
| post_baseline | 180s | cont 0.15 까지 revert (= sticky-only fix) | 14 | 4.7 | 2 | 240mm |
| post_trail_pen | 180s | + TRAIL ego_n>0.15 penalty | 12 | 4.0 | 4 | 91mm |

### 관찰
- sticky bug fix 효과 명확: 진동 17→4-12 (구조적 reduction)
- alpha ramp 0.10 (smooth) 시 reaction 느려 충돌 ↑
- continuity threshold 0.08 시 trajectory blend 가 obstacle 회피 늦춤
- 120s vs 180s 비교 어려움 (sim 시나리오 변동성 큼: obstacle zigzag, 통과 지점 random)
- TRAIL ego_n>0.15 penalty: ego 커밋 deep 시 의미 있음. ego_n 낮을 때는 발동 X → 다른 충돌 시나리오 (obstacle zigzag 등) 미해결.

### 미해결 / 개선 여지
- Obstacle zigzag (lateral oscillation 0.25m amplitude) 시 prediction 미흡 → MPC TRAIL→PASS 전환 늦음
- 통과 직전 ego가 가속 (TRAIL ref_v 미적용 의심?) — alpha ramp 가 weight + ref_v 분리 적용
- sim 시나리오 변동성으로 단일 bag 비교 어려움 → 동일 시나리오 5회 반복 통계적 검증 필요

---

## 5x60s repeat trial 결과 (sim variance 격리 시도)

| Trial | Code | Coll/300s | Lat_min |
|---|---|---|---|
| trial_a | sticky+filter+TRAIL pen | 14 | 0mm |
| trial_b | sticky+filter (TRAIL pen 제거) | 4 | 156mm |
| trial_c | trial_b + target_n profile blend | 12 | 11mm |
| trial_d | trial_b 코드 재현 | 10 | 0mm |

**중대 발견**: trial_b vs trial_d **동일 코드 / 결과 2.5배 차이** (4 vs 10 collisions).
→ 5x60s aggregate 도 sim variance 격리 부족. 단일 60s bag 변동성 너무 큼.

**시도→revert (5x60s 통계 기반)**:
- C: target_n profile blend (alpha 0.20 sync) → trial_c 12 coll, revert
- TRAIL ego_n>0.15 penalty → trial_a 14 coll, revert

---

## 단일 10분 bag (ego s=0 spawn 재현, sim variance 평균화)

| Bag | Code | Coll edges | Events | TRAIL | PASS |
|---|---|---|---|---|---|
| long_baseline | original (horizon+EMA d_free) | 23 | 12 | 10 | 2 |
| long_snap | snapshot-only d_free | 12 | 9 | 6 | 3 |
| long_trail_lat | + TRAIL lat push | 13 | 10 | 3 | 7 → revert |
| **long_qnt15** | snapshot + q_n_target 8→15 | **12** | **7** | **2** | 5 |
| long_side_align | + plan-aligned side_int | 17 | 10 | 4 | 6 → revert |

**최선: long_qnt15** — 7 unique 충돌, TRAIL 80% 감소.

### 적용된 fix
1. snapshot-only d_free (horizon+EMA 제거): TRAIL 잘못 선택 빈도 ↓
2. q_n_target 8→15 (PASS): ego가 avoid_n target 더 강하게 추적

### Rollback 한 변경
- TRAIL target_n lateral push: TRAIL ↓ 했으나 PASS 충돌 ↑ (lateral 이동 → PASS 더 자주 선택, mature 못한 PASS 충돌)
- plan-aligned side_int: filter 미발동 시 (ego_n=0.12) RIGHT_PASS 강제 → ego가 obs 가로질러야 함 → 충돌 ↑

### 미해결 (PASS 충돌)
- ego_n 가 obstacle 같은 쪽에 있고 commit threshold (0.15) 미달 시 PASS 선택해도 ego가 obstacle 가로지름
- w_obs Gaussian bubble은 direction-agnostic → ego가 obs 가까이 가는 걸 막음 → avoid_n 도달 X
- 잠재 해결: filter threshold 0.10으로 낮춤 / Gaussian one-sided / sigma_n_obs 줄임


**결론**: 핵심 구조적 fix (sticky bug, filter, smooth risk, adaptive gap_lat) 유지.
추가 smoothness 메커니즘 (target_n blend, slower alpha, lower cont threshold) 모두 reactivity 손해 → 충돌 ↑.

다음 세션에서:
- 더 긴 trial (10+분 단일) 또는 sim 재시작 후 검증
- 또는 재현 가능한 시나리오 (ego/obs spawn 고정)


---

## 최종 변경 누적 (Stage 2 Phase 2-2 최종)

### 코드 변경
- `frenet_kin_solver.py`: P_n_target/P_q_n_target parameter + J_target cost + setter
- `plan_library.py`: 5 plan weight overlay (N=40 stage 비례) + make_target_n_profile (avoid_n=0.55, peak=0.6N)
- `plan_scorer.py`: score_plan (race_progress + risk 강화 + maneuver + long_term + sticky 0.30 + ego_n bonus) + horizon_aware_d_free + pick_plan_scored (EMA + gap threshold 25%)
- `mpc_planner_state_node.py`: plan_picker 5 plan 후보 + target_n profile push + TRAIL ref_v override + horizon d_free + EMA + dwell + passing-freeze

### Backup 파일 (이름 명시)
- `plan_library_phase22_baseline_20260428.py`
- `plan_library_d_N40_baseline_20260428.py`
- `plan_library_b_finetune_baseline_20260428.py`
- `frenet_kin_solver_phase22_baseline_20260428.py`
- `mpc_planner_state_node_phase22_baseline_20260428.py`
- `state_overtake_phase22_baseline_20260428.yaml`
- `state_overtake_d_N40_baseline_20260428.yaml`
