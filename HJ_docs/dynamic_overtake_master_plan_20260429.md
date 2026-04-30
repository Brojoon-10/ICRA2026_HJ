# Dynamic Obstacle Overtake — Master Plan (2026-04-29)

> **Scope**: 동적 장애물만 우선 (정적 나중). 휙휙 토글하는 reactive 가 아닌, **prediction 정보 정확히 활용** + **multi-tick strategic intent** + **명시적 safety buffer** + **충돌 절대 X** 의 architecture.
> **상태**: 작성 중. 사용자 검토 후 단계별 구현 진입.
>
> **이 문서가 단일 source of truth**. 진행 중 결정 / 변경은 모두 여기에 반영. 코드의 임시 patch (continuity blend, sticky tweak 등) 와 별개로 본 plan 의 layer / contract 가 우선.

---

## 0. 사용자 directive (불변)

본 architecture 가 충족해야 할 6개 원칙. 충돌 시 우선순위는 위→아래.

1. **충돌 0**. **100 laps 동안 50 obstacle encounter 시 단 한 번도 충돌 X**. 추월 효율보다 우선. 안전 path 없으면 멈춤. 통계가 아니라 **structural guarantee** — Layer C R0 (BRAKE_HARD) + Layer E vx_mps cap 으로 보장.
2. **MPC 경로 항상 publish**. global tracking, overtake, recovery 어떤 state 든 매 tick 출력. publish 누락 = controller brake = 더 위험.
3. **동적 우선**. 정적 obstacle 처리는 별도 phase (이 문서 범위 밖).
4. **Strategic plan**. 매 tick reactive 토글 X. multi-tick sequence (예: close trail → opportunity → commit → pass → recover) 표현.
5. **Prediction 정확 활용**. GP / tracker 의 obstacle horizon trajectory 를 timestep-별 (s, n) 사용. 현재처럼 vs/vd 상수 propagate X.
6. **안전 buffer 명시**. prediction uncertainty (vd_var, obs.n rolling stddev) 가 lateral margin 에 반영.

### 안전 contract (directive #1 의 정량화)

100 laps × 50 encounters = **2 시간 35 분 (gazebo_wall_2 lap ≈ 21s) 동안 0 collisions**. Phase 6 의 통과 기준은 이걸 충족해야:

- 30분 bag x 5 회 = 150분 동안 충돌 0건 → 100/50 시나리오 추정 0 collisions.
- 충돌 1 건 발생 시: phase 미통과. 그 충돌 시점의 encounter_summary + plan + commit_phase + obs_horizon 풀 분석 → 어느 Layer 의 contract 위반인지 식별 → 해당 Layer fix 후 재검증.
- 통계적 metric (충돌률, lateral min) 가 아니라 **0/0 절대값** 으로 판정.

**절대 금지**:
- truth (`/tracking/obstacles_truth`) 사용 (real world 에 truth 없음)
- inter_msg_jump 류 publish reject (path 자체는 항상 통과, smoothness 는 NLP 의 w_cont)
- 임시방편 blend / smoothing 으로 근본 결함 가리기

---

## 1. 현재 stack 진단 (2026-04-28 종료 시점)

각 layer 별 결함과 실제 영향. 본 plan 이 풀어야 할 문제.

### 1.1 Prediction layer

```
GP predictor → /opponent_prediction/obstacles (N=20 timestep msgs per opponent)
   ↓
_merge_obs_sources()    [mpc_planner_state_node.py:861-955]
   ├─ tracker obstacle 받음 (canonical, KF 적용)
   ├─ GP obstacle 매치 (s, d 가까운 거 1 개) → vs, vd 추출
   ├─ obs_arr[obs, k, :] 채울 때:
   │   obs.s[k] = obs.s + vs * k * dT       ← vs 상수 propagate
   │   obs.n[k] = obs.n + vd * k * dT       ← vd 상수 propagate
   └─ GP 매치 실패 → vs=0, vd=0 정적 강제
```

**결함**:
- GP 의 N=20 timestep 별 (s_k, n_k) 정보 **0% 활용**. timestep 0 의 vs/vd 만 추출 후 직선 propagate.
- GP 가 학습한 obstacle 의 oscillation (예: 0.25m amplitude sin) / 코너링 / 곡선 motion 이 MPC 입력에 반영 X.
- GP nascent (데이터 부족) 시 정적 강제 → 동적 obstacle 을 정적으로 보고 path 만듦 → ego 따라잡으며 충돌.
- obs.vs spike (KF noise) → MPC ref_v override + plan_scorer 입력 모두 변동.

### 1.2 SideDecider + plan_picker

```
side_decider.py
   └─ d_free_L/R, dv 기반으로 SIDE ∈ {LEFT, RIGHT, TRAIL, CLEAR} 결정
       ├─ 양쪽 막혔는데 best_effort_left/right 강제 (2026-04-28 fix: SIDE_TRAIL 로 변경)
       └─ closest 1개 obstacle 만 평가 (multi-obstacle aware X)
plan_scorer.py
   └─ candidates 별 score:
       progress_gain + risk + maneuver + long_term + sticky + ego_n_bonus
       매 tick instant 평가 → reactive
```

**결함**:
- 매 tick reactive scoring → plan toggle 휙휙 (사용자 본 path 번쩍).
- "지금 trail 했다가 0.5초 뒤 commit" 같은 timing 의도 표현 X.
- closest 1개만 평가 → multi-obstacle (정적 + 동적) 시 잘못된 결정.
- score 식의 weight 들이 임의 (sticky 0.30, q_n_target=15, 등) — racing 직관과 동떨어짐.

### 1.3 MPC NLP (frenet_kin_solver)

```
kinematic bicycle dynamics:
  state [n, mu, v, delta], control [a, delta_dot]
costs:
  J = q_contour + q_progress (= -gamma·s) + q_input_rate + q_smooth
    + q_obs (Gaussian bubble symmetric on n) + q_n_target (plan-aware)
    + q_n + q_n_term + q_n_ramp + w_side_bias (plan-aware) + w_cont
constraints:
  v[k] ≤ vmax_curv[k]                 (curvature 한계)
  a[k] ∈ [a_min, a_max]
  n[k] ∈ [n_lb_corridor, n_ub_corridor] + slack  (soft corridor)
```

**결함**:
- **w_obs Gaussian bubble 이 direction-agnostic**. ego 가 obstacle 의 잘못된 side 에 있으면 cross 비용 너무 커서 같은 쪽 머무름 → obstacle 옆 통과 시 lateral 부족 → 충돌 (long_qnt15 의 5/7 충돌 패턴).
- **ggv polar 미적용**. vmax_curv 와 a_max 가 분리. 코너 진입/진출 전이 영역에서 long+lat 결합 한계 표현 X.
- **NLP infeasible 빈도 ↑** (Maximum_Iterations_Exceeded, Infeasible_Problem_Detected). corridor hard 또는 obstacle bubble 강해서. fallback 으로 떨어지면 stale path → 충돌.

### 1.4 Painter + Publish

```
_post_process_speed (painter)
   ├─ baseline = vel planner GB raceline vx (obstacle 무관!)
   ├─ curvature cap = sqrt(mu*g/|kappa|)
   ├─ ego_v continuity = clip(capped[0], ego_v ± a_max·dT)
   └─ wp.vx_mps = capped[k]   ← MPC NLP 의 v[k] 폐기
```

**결함 (architecture defect A)**:
- MPC NLP 가 풀어낸 plan-aware ref_v (TRAIL 시 obs.vs - 0.3) 가 painter 에서 덮어씀.
- trajectory.vx_mps 가 obstacle 무관 raceline 속도 → controller 가 받음 → ego 가 obstacle 따라잡으며 충돌.

```
_validate_publish_wpnts
   └─ inter_msg_jump > 0.5 → reject (2026-04-28 폐기 — publish 항상 통과)
```

### 1.5 Cache & Fallback

```
_handle_emergency_no_traj    (2026-04-28 폐기)
_handle_tier1_hold_last      (cap=3 → 무한, 2026-04-28 fix)
_sample_recovery_cache       (TTL=1s 추가, 2026-04-28 fix)
```

**잔존 결함**:
- NLP 성공 시도 cache hijack 가능 (cache 가 fallback 만 아니라 항상 사용 가능).
- quintic / raceline slice fallback 이 obstacle 무시 — 충돌 위험.

### 1.6 SM + Controller

```
SM 의 ObstacleTransition_GBMode
   └─ (TRAILING, OVERTAKE) tuple 반환 → controller 에 path = OT, state = TRAILING 동시 전달
   └─ 사용자 확인: 이 조합 OK. 단 trailing 시 거리/속도 유지 안 되는 게 진짜 문제.

controller.trailing_controller()
   └─ TRAILING state 시 trailing PID 작동 (obs.vs PID)
   └─ MPC trajectory.vx_mps 무시 (architecture defect B 확인)
   └─ 사용자: PID 끄지 말 것 (현재 유지)
```

---

## 2. 새 Architecture (5 layer + supporting)

### 전체 data flow

```
┌─────────────────────────────────────────────────────────────┐
│  [PRED]  /tracking/obstacles + /opponent_prediction/obstacles│
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────▼──────────────┐
        │  Layer A: ObstacleHorizon   │
        │  - GP timestep 별 (s,n,vs,vd) 활용
        │  - linear interp / extrapolation
        │  - rolling-median noise filter
        │  - nascent → conservative fallback (큰 variance)
        │  → obs_horizon[obs_idx, k] = (s_k, n_k, vs_k, vd_k, var_k)
        └─────────────┬──────────────┘
                      │
        ┌─────────────▼──────────────┐
        │  Layer B: EncounterAnalyzer │
        │  - ego_horizon × obs_horizon timestep-별 비교
        │  - encounter_k, encounter_t, encounter_ds, encounter_dn
        │  - obs_to_left_wall, obs_to_right_wall (at encounter)
        │  - lateral_pass_margin_left/right (with safety buffer)
        │  - corridor_kappa[encounter_k] (코너 곡률)
        │  - obs_lateral_drift (좌/우/직진 추정)
        │  → encounter_summary
        └─────────────┬──────────────┘
                      │
        ┌─────────────▼──────────────┐
        │  Layer C: StrategicPlanner   │
        │  - 룰베이스 decision tree
        │  - plan ∈ {RACELINE, TRAIL, LEFT_PASS, RIGHT_PASS, BRAKE_HARD}
        │  - plan dwell (min 0.5s 유지)
        │  - safety override: 위협 감지 시 즉시 변경 가능
        │  → plan + plan_side + commit_phase
        └─────────────┬──────────────┘
                      │
        ┌─────────────▼──────────────┐
        │  Layer D: PlanAwareMPC      │
        │  - frenet_kin_solver (개선)
        │  - obstacle bubble: directional (plan_side aware)
        │  - ref_v: plan-aware (BRAKE_HARD=0, TRAIL=obs.vs-margin)
        │  - target_n profile: plan + commit_phase 따라
        │  - ggv polar (옵션, Phase 5)
        │  → trajectory (s, n, mu, v, kappa)
        └─────────────┬──────────────┘
                      │
        ┌─────────────▼──────────────┐
        │  Layer E: SafePublish       │
        │  - 항상 publish (사용자 directive)
        │  - 우선순위:
        │    NLP success → publish
        │    NLP fail    → HOLD_LAST (무한)
        │    else        → cache (TTL 1s) → quintic → raceline
        │  - 단, plan=BRAKE_HARD 또는 NLP fail 시 vx_mps cap=0
        │  → /planner/mpc/wpnts (always)
        └─────────────────────────────┘
                      │
                      ▼
            (SM + Controller, 변경 X)
```

### 각 layer 의 contract (input / output / 의존성 / 보장)

#### Layer A: ObstacleHorizon

- **input**:
  - `/tracking/obstacles` (KF 적용 ObstacleArray, 50Hz)
  - `/opponent_prediction/obstacles` (GP, N timestep msgs per opponent)
- **output**:
  - `obs_horizon: ndarray (n_obs, K_mpc, 5)` — `[s, n, vs, vd, var]` per timestep
  - `obs_meta: List[dict]` — `id, is_static (vs<thresh), is_dynamic, gp_quality (mature/nascent)` 등
- **internal logic**:
  1. tracker obstacle 모두 (= canonical 1 entry per real opponent)
  2. tracker 별로 GP horizon msgs match (id 또는 (s,d) bucket)
  3. GP K_pred (보통 20) 와 MPC K (40) 다른 경우 linear interp:
     - k < K_pred → GP 의 (s_k, n_k) 직접
     - K_pred ≤ k → 마지막 GP step (vs, vd) 로 extrapolate
  4. 각 timestep 의 (vs_k, vd_k) 의 rolling median (5 sample) → spike 제거
  5. GP nascent (variance 이상치 또는 GP 미publish) → tracker (vs, vd) 상수 + fallback variance ↑
- **보장**:
  - 매 tick obs_horizon shape 동일 (n_obs_max=2, K_mpc=N+1)
  - 빈 slot 은 weight=0 (active flag) 로 표시 (현재 obs_arr[:,:,2] 채널 활용)
  - vs/vd 의 spike (≥ 5 m/s² jump) 는 reject + 이전 값 유지

- **구현 outline** (Python pseudocode, `_build_obstacle_horizon`):
  ```
  def _build_obstacle_horizon(self):
      """Replaces _merge_obs_sources. Build obs_horizon[obs_idx, k, :] using
      GP timestep msgs (full sequence, not just k=0). Robust to GP nascent
      / KF spike. Returns obs_arr in the existing (n_obs, K, 4+) shape so
      downstream (NLP, plan_picker) doesn't need to change shape API.
      """
      track_fresh = self._is_fresh(self._obs_track_t) and self._obs_track
      if not track_fresh:
          return np.zeros((self.n_obs_max, self.N + 1, 4), dtype=float)

      # Group GP msgs by obstacle id (or fallback (s,d) bucket)
      gp_groups = {}   # {obs_id: [(stamp, s, n, vs, vd, var), ...]}
      if self._is_fresh(self._obs_predict_t) and self._obs_predict:
          for p in self._obs_predict.obstacles:
              key = self._gp_group_key(p)   # id if available else (s_round, d_round)
              gp_groups.setdefault(key, []).append((
                  p.header.stamp.to_sec() if hasattr(p, 'header') else 0.0,
                  float(p.s_center), float(p.d_center),
                  float(p.vs), float(p.vd),
                  float(getattr(p, 'vd_var', 0.0))))
          # Sort each group by stamp (= timestep order)
          for k_ in gp_groups:
              gp_groups[k_].sort()

      # Per tracker obstacle
      obs_arr = np.zeros((self.n_obs_max, self.N + 1, 4), dtype=float)
      for slot, t in enumerate(self._obs_track.obstacles[: self.n_obs_max]):
          # Match this tracker to one GP group (closest seq[0])
          group = self._match_tracker_to_gp(t, gp_groups)
          # Rolling-median noise filter on tracker.vs (window=5)
          self._vs_rolling[slot].append(float(t.vs))
          vs_med = float(np.median(list(self._vs_rolling[slot])))
          # Spike reject: if |vs_med - prev_vs_med| > 5.0, hold prev
          if abs(vs_med - self._prev_vs_med[slot]) > 5.0:
              vs_med = self._prev_vs_med[slot]
          self._prev_vs_med[slot] = vs_med
          # Build horizon
          for k in range(self.N + 1):
              if group and k < len(group):
                  # GP timestep direct
                  _, s_k, n_k, vs_k, vd_k, var_k = group[k]
              elif group:
                  # Past GP horizon end → extrapolate from last group entry
                  _, s_last, n_last, vs_last, vd_last, var_last = group[-1]
                  dk = k - (len(group) - 1)
                  s_k = s_last + vs_last * dk * self.dT
                  n_k = n_last + vd_last * dk * self.dT
                  vs_k, vd_k, var_k = vs_last, vd_last, var_last
              else:
                  # No GP match → constant-vs/vd from tracker (nascent fallback)
                  s_k = float(t.s_center) + vs_med * k * self.dT
                  n_k = float(t.d_center) + float(t.vd) * k * self.dT
                  vs_k, vd_k = vs_med, float(t.vd)
                  var_k = max(float(getattr(t, 'vd_var', 0.0)), 0.25)  # large fallback
              obs_arr[slot, k, 0] = s_k
              obs_arr[slot, k, 1] = n_k
              obs_arr[slot, k, 2] = 1.0  # active flag
              obs_arr[slot, k, 3] = var_k   # NEW: per-timestep variance for buffer
      return obs_arr
  ```

- **Replaces** `_merge_obs_sources` ([mpc_planner_state_node.py:861-955]). The
  vs/vd-constant-propagation that loses GP's horizon learning becomes the
  fallback path only. GP timestep stream becomes the primary input.

#### Layer B: EncounterAnalyzer

- **input**:
  - `obs_horizon` (Layer A)
  - `ego_horizon`: ego 의 미래 K timestep 추정. **2 옵션**:
    - (a) MPC 이전 tick 의 trajectory (warm-start input)
    - (b) ego 의 현재 (s, n, vs) + ref_v 로 forward predict
  - `corridor_horizon`: ref_slice 의 d_left_arr, d_right_arr, kappa_arr (이미 있음)
- **output**:
  - `encounter_summary` per active obstacle:
    ```
    encounter_k             # ego ↔ obs 가장 가까운 timestep
    encounter_t             # = encounter_k * dT
    encounter_ds, encounter_dn   # frenet 상의 거리
    encounter_distance      # cartesian (보조)
    obs_n_at_encounter
    corridor_d_left_at_encounter, _d_right_
    obs_to_left_wall        # = d_left - obs.n
    obs_to_right_wall       # = d_right + obs.n
    lateral_pass_margin_left   # = obs_to_left_wall - ego_width - safety_buf
    lateral_pass_margin_right  # = obs_to_right_wall - ego_width - safety_buf
    safety_buffer           # = max(base, k*dT*sqrt(vd_var), obs.n_rolling_std)
    corridor_kappa_at_encounter
    obs_lateral_drift       # vd_k 의 추세: +/-/0
    closure_rate            # ego.vs - obs.vs (encounter 시점)
    ```
- **safety_buffer 의 정의**:
  - base = 0.30 m (현재 gap_lat 와 동일)
  - prediction uncertainty 추가: `sqrt(vd_var) * encounter_t` (1-σ 누적)
  - obs.n rolling stddev 도 추가: `obs_n_std`
  - 총: `safety_buffer = base + max(sqrt(vd_var)*encounter_t, obs_n_std, fixed_floor=0.05)`
  - cap: 0.5 m (너무 큰 buffer 는 corridor 못 통과)

#### Layer C: StrategicPlanner

- **input**: encounter_summary + ego state + obs_meta + commit_phase (이전 tick)
- **output**: plan ∈ {RACELINE, TRAIL, LEFT_PASS, RIGHT_PASS, BRAKE_HARD} + plan_side + commit_phase
- **commit_phase**: multi-tick strategic intent 표현:
  - DETECT (obstacle 감지, 평가 중)
  - APPROACH (close trail 시도, 속도 매칭)
  - WAIT (close trail 안정, opportunity 기다림)
  - COMMIT (PASS plan 결정, lateral commit 진행)
  - PASS (obstacle 옆 통과 중)
  - RECOVER (raceline 복귀)
  - CLEAR (obstacle 없음)

- **decision tree** (priority 순):

  ```
  R0 (절대 안전):
    if obs_horizon empty OR encounter_t > 3.0s:
      return RACELINE, CLEAR

    if pass_margin_left < 0 AND pass_margin_right < 0
       AND closure_rate > 0.5 AND encounter_ds < 1.5:
      # 양쪽 막힘 + 빠르게 따라잡음 + 가까이
      return BRAKE_HARD, _, EMERGENCY

    if NLP infeasible 마지막 N tick (예: ≥3) AND obstacle in horizon:
      return BRAKE_HARD, _, EMERGENCY

  R1 (close trail 진입):
    if commit_phase ∈ {DETECT, APPROACH}:
      if encounter_ds > close_trail_gap (예: 1.5m) AND closure_rate > 0:
        return TRAIL, _, APPROACH    # 속도 매칭하며 가까이
      else:
        return TRAIL, _, WAIT         # 가까이 도달, opportunity 기다림

  R2 (commit 결정):
    if commit_phase == WAIT:
      # opportunity 평가
      if pass_margin_left ≥ pass_margin_right
         AND pass_margin_left > 0.4
         AND (obs_lateral_drift = 'right' OR corridor_kappa < 0):
        # LEFT 유리 (obs 우측 이동 또는 좌커브 inside)
        return LEFT_PASS, LEFT, COMMIT
      elif pass_margin_right > 0.4
         AND (obs_lateral_drift = 'left' OR corridor_kappa > 0):
        return RIGHT_PASS, RIGHT, COMMIT
      else:
        return TRAIL, _, WAIT          # 아직 기회 없음

  R3 (pass 진행):
    if commit_phase == COMMIT:
      if encounter_ds < 0:
        # ego 가 obstacle 지났음
        return current_pass_plan, current_side, RECOVER
      else:
        return current_pass_plan, current_side, PASS    # 계속 진행

  R4 (recover):
    if commit_phase == RECOVER:
      if abs(ego_n) < 0.10:
        return RACELINE, _, CLEAR
      else:
        return RACELINE, _, RECOVER

  R5 (default safe):
    return TRAIL, _, WAIT
  ```

- **commit_phase 상태 다이어그램** (절대 금지 transition 명시):

  ```
                    +-----------------+
                    |  CLEAR (no obs) |
                    +--------+--------+
                             |
                obstacle in horizon
                             |
                    +--------v--------+
                    |    DETECT       |
                    +--------+--------+
                             |
                  encounter_t < 3s + closure>0
                             |
                    +--------v--------+
                    |   APPROACH      |   ← TRAIL plan, ego closes gap
                    +--------+--------+
                             |
                  encounter_ds < close_trail_gap
                             |
                    +--------v--------+
                    |     WAIT        |   ← TRAIL plan, opportunity 평가
                    +---+---------+---+
                        |         |
            margin OK   |         |  margin not OK
                        |         |
                    +---v---+ +---v---+
                    |COMMIT | | WAIT  |   (loop)
                    +---+---+ +-------+
                        |
                ego_n approaching avoid_n
                        |
                    +---v---+
                    | PASS  |   ← LEFT/RIGHT_PASS plan, ego beside obs
                    +---+---+
                        |
                  encounter_ds < 0
                        |
                    +---v---+
                    |RECOVER|   ← RACELINE plan, |ego_n|>0.10
                    +---+---+
                        |
                  |ego_n| < 0.10
                        |
                    +---v---+
                    | CLEAR |
                    +-------+

  EMERGENCY (R0): any state → BRAKE_HARD when:
       both pass margins < 0 + closing fast, OR
       NLP infeasible streak ≥ 3 + obs in horizon
       (즉시 transition. dwell 무시)

  금지 transition (이 다이어그램에 없음):
       COMMIT → COMMIT (다른 side) — plan_side 바꾸려면 PASS 또는 EMERGENCY 거쳐야
       PASS → CLEAR — 반드시 RECOVER 거쳐야
       APPROACH → COMMIT — 반드시 WAIT 거쳐야 (margin 평가)
  ```

- **plan dwell**:
  - 한 번 commit_phase 변경 시 최소 0.5초 (25 tick @ 50Hz) 유지
  - 예외: R0 (BRAKE_HARD) 는 즉시 가능 (안전 우선)
- **plan_side commitment**:
  - COMMIT phase 진입 후 plan_side 변경 X (LEFT 결정했으면 LEFT 유지)
  - 예외: lateral_pass_margin_LEFT < 0 → 즉시 BRAKE_HARD

#### Layer D: PlanAwareMPC

- **directional obstacle bubble — 핵심 코드 outline** (frenet_kin_solver.py NLP 안):
  ```python
  # 기존: J_obs = w_obs * sum_k exp(-(n_k - obs_n_k)^2 / 2σ^2)  (양면)
  #
  # 변경:
  P_plan_side = opti.parameter()   # 0=NEUTRAL, 1=LEFT, 2=RIGHT
  for k in range(N + 1):
      for o in range(n_obs_max):
          if active[o, k]:
              dn = n[k] - P_obs_n[o, k]
              # 양면 Gaussian (baseline)
              J_obs += w_obs * exp(-dn**2 / (2 * sigma_n**2))
              # plan_side 별 사이드 페널티 (sigmoid hinge)
              # sigmoid(x) = 1 / (1 + exp(-x))
              # LEFT_PASS: ego_n > obs_n 이 안전 (ego is to the LEFT of obs).
              #            ego_n < obs_n 이면 큰 페널티.
              if_else(P_plan_side == 1,
                  w_side * sigmoid((P_obs_n[o,k] - n[k] + 0.05) / sigma_side),
                  if_else(P_plan_side == 2,
                      w_side * sigmoid((n[k] - P_obs_n[o,k] + 0.05) / sigma_side),
                      0.0))
  ```

  - `w_side` 는 plan_library 의 `w_side_bias` (현재 60) 보다 강해야 (예: 150).
  - `sigma_side` 는 hinge sharpness (예: 0.05).
  - `+0.05` offset 은 ego_n == obs_n 경계에서 페널티 not zero — ego 가 정확히 obstacle 위에 있는 것도 막음.

- **directional bubble 의 효과** (현재 결함 vs fix):
  - 현재: `w_obs Gaussian` 양면 → ego 가 obstacle 의 wrong side 에 있으면 cross 비용 = `w_obs × max_value` (커서 cross 못 함). ego 가 같은 side 에 머물고 obstacle 옆 통과 시 lateral 부족.
  - fix 후: plan=LEFT_PASS 면 ego_n < obs_n 이 high cost (sigmoid hinge), ego_n > obs_n 은 low cost. ego 가 wrong side 에서 시작해도 cross 가 free → ego 가 의도한 side 로 이동 가능.

- **input**: ego state, plan, plan_side, commit_phase, obs_horizon, corridor
- **NLP cost 변경**:
  - **directional obstacle bubble** (가장 중요):
    ```python
    # 기존: J_obs = w_obs * sum_k exp(-(n_k - obs_n_k)^2 / 2σ^2)
    # 변경:
    if plan == LEFT_PASS:
        # obstacle 의 RIGHT 쪽만 high cost. ego 가 LEFT 로 통과 path 형성
        J_obs = w_obs * sum_k exp(-(n_k - obs_n_k)^2 / 2σ^2) * sigmoid(obs_n_k - n_k + 0.05)
    elif plan == RIGHT_PASS:
        # 대칭
        J_obs = w_obs * sum_k exp(...) * sigmoid(n_k - obs_n_k + 0.05)
    else:
        J_obs = w_obs * sum_k exp(...)   # 양면 (현재처럼)
    ```
  - **target_n profile** (현재 비슷, plan_library 활용):
    - LEFT_PASS: ego_n → obs_n + buffer (peak 0.5N) → 0
    - RIGHT_PASS: 대칭
    - TRAIL: ego_n hold
    - BRAKE_HARD: ego_n hold
    - RACELINE: 0
  - **ref_v** (NLP 안에서 결정, painter 가 무력화 안 하도록 painter 도 수정):
    - LEFT/RIGHT_PASS: vel planner GB ref_v (full speed)
    - TRAIL: max(obs.vs - 0.3, 0.5)
    - BRAKE_HARD: 0 (또는 v_min)
    - RACELINE: vel planner GB ref_v
  - **q_n_target**: plan 별 (현재 PASS=15, TRAIL=25, RACELINE=5, BRAKE_HARD=25)
- **NLP infeasible 시**:
  - HOLD_LAST 무한 (이미 fix)
  - 추가: HOLD_LAST 가 publish 하는 trajectory 의 vx_mps 도 plan_aware 로 cap (BRAKE_HARD 시 0)
- **검증**:
  - solve_ms p99 < 50ms (현재 보통 < 30ms)
  - infeasible 빈도 < 5%
  - trajectory 의 wpnts[encounter_k].n 이 plan 의도대로 lateral shift

#### Layer E: SafePublish

- **rule 1**: 항상 publish. validation 은 reject 하지 않고 vx cap 적용:
  ```
  if plan == BRAKE_HARD or NLP_failed:
      for k in horizon:
          wp[k].vx_mps = min(wp[k].vx_mps, v_brake_target)
              # v_brake_target = 0 또는 painter ego_v continuity 가 허용하는 최저
  ```
- **rule 2**: validation 은 finite/kappa/ds_min 만 (현재 그대로)
- **rule 3**: HOLD_LAST 시도 plan 의도 cap 적용 (위 rule 1)
- **fallback ladder**:
  - tier 0: NLP success → publish
  - tier 1: NLP fail + last_good_traj 있음 → HOLD_LAST publish (BRAKE_HARD 시 vx=0 cap)
  - tier 2: 위 둘 다 안 됨 + obstacle 없음 → quintic recovery
  - tier 3: 모두 실패 → raceline slice (마지막 안전망, 단 vx=0 cap if obstacle 가까움)

- **현재 코드 상태와 매핑**:
  | tier | 현재 코드 | Layer E 결정 |
  |---|---|---|
  | 0 NLP success | publish 정상 | 그대로. 단 painter 가 plan-aware vx_mps cap (BRAKE_HARD 시 vx=0) 적용 |
  | 1 HOLD_LAST | 무한 cap 적용 (2026-04-28) | 그대로 + plan-aware vx cap |
  | 2 cache | TTL=1s 적용 (2026-04-28) | 그대로. 단 NLP success 시 hijack X (Phase 1 추가) |
  | 2 quintic | obs 없을 때만 | 그대로 |
  | 3 raceline slice | 마지막 fallback | 그대로 + obstacle 옆일 때 vx=0 cap (Phase 4 추가) |
  | -1 EMERGENCY_NO_TRAJ | 폐기됨 (2026-04-28) | 다시 사용 X. 항상 publish. |

- **inter_msg_jump validation**:
  - 2026-04-28 폐기 완료. publish reject 자체 X.
  - smoothness 보장은 NLP 의 `w_cont` 와 continuity_guard 가 책임.

- **항상 publish 보장의 마지막 안전망**:
  - tier 3 raceline slice 는 last_good_traj 도 없는 첫 publish 시점에도 작동 (lifter 가 raceline waypoint 의 (s, 0) 을 lift)
  - 따라서 어떤 시점에도 `/planner/mpc/wpnts` 가 publish 됨

- **plan-aware vx_mps cap 의 정확한 처리** (`_post_process_speed` 보강):
  ```python
  # baseline = vel planner GB raceline vx (현재 그대로)
  # curvature cap 적용
  # NEW (Phase 4): plan-aware override
  if plan == BRAKE_HARD or self._fail_streak > 0:
      target_v = 0.0    # full brake
  elif plan == TRAIL and obs_vs > 0.05:
      target_v = max(obs_vs - 0.3, 0.5)
  elif plan == LEFT_PASS or plan == RIGHT_PASS:
      target_v = baseline_vx  # full speed (GB)
  elif plan == RACELINE:
      target_v = baseline_vx
  capped[k] = min(capped[k], target_v)
  # ego_v continuity ramp 는 plan_aware:
  #   BRAKE_HARD 시 a_max = 5.0 m/s² (강한 brake 허용)
  #   그 외 a_max = 3.0 (현재)
  ```
  ※ 이건 직전에 시도해서 a_max ramp 에 무력화됐던 fix 를 plan-aware a_max 로 보강한 형태. Phase 4 에서 적용.

### 추가 — Layer F: SM/Controller (변경 최소)

현재 유지:
- SM 의 ObstacleTransition_GBMode 의 (TRAILING, OVERTAKE) 조합 유지
- controller TRAILING PID 유지

단, plan_picker / encounter analyzer 가 동작 중이라면 SM 의 결정과 plan 이 자연스럽게 일관 (RACELINE plan → SM=GB_TRACK 자연 도달).

미래 작업 (이 plan 의 후속):
- BRAKE_HARD 신호 → SM/Controller 에 명시적 brake 명령 (지금은 vx_mps=0 으로 간접)
- Controller PID 폐기 + MPC vx_mps 일관 사용 (사용자 승인 후)

---

## 3. 안전 보장 (충돌 X 의 메커니즘)

본 plan 이 사용자 directive #1 ("충돌 절대 X") 를 어떻게 보장하는지 명시적으로:

### 3.1 Hard guarantees (구조적)

| 메커니즘 | 보장 |
|---|---|
| Layer C R0 (양쪽 막힘 → BRAKE_HARD) | encounter 시점 양쪽 wall 사이 통과 불가 시 ego 정지 |
| Layer C R0 (NLP infeasible 연속 → BRAKE_HARD) | NLP 신뢰 못 함 시 즉시 brake |
| Layer D directional obstacle bubble | PASS plan 시 ego 가 wrong-side 머무는 거 방지 (cross 비용 ↓ on intended side, ↑↑ on obstacle side) |
| Layer E vx_mps cap (BRAKE_HARD/NLP fail) | path 가 obstacle 옆 통과해도 속도 0 → controller 가 brake |
| Layer A obs_horizon 의 GP 정확 활용 | ego 가 obstacle 의 oscillation / 코너링 인지 → 못 만나는 path 안 만듦 |

### 3.2 Soft guarantees (확률적)

| 메커니즘 | 보장 |
|---|---|
| Layer B safety_buffer (vd_var * t + obs_n_std) | prediction uncertainty 시간 누적 → encounter 시점 더 conservative |
| Layer A rolling-median filter | obs.vs/vd noise 가 plan/MPC 결정에 spike 영향 X |
| Layer C plan dwell (0.5s) | trajectory 토글 X → controller 가 일관 path 추종 |
| Layer D ggv polar (Phase 5) | 코너 진입/진출 long+lat 결합 한계 표현 → 안전 속도 |

### 3.3 Detection (사고 직전 감지)

| 시그널 | 행동 |
|---|---|
| pass_margin_L < 0 AND pass_margin_R < 0 + closing fast | BRAKE_HARD |
| NLP infeasible streak ≥ 3 + obstacle in horizon | BRAKE_HARD |
| MPC trajectory 의 wpnts[k].n 이 obs.n 과 buffer 미만 | publish 허용 (사용자 directive) but vx_mps cap = 0 |
| obs.vs/vd spike (≥ 5 m/s² delta) | 해당 tick obs.vs 이전 값 유지, plan 재평가 보류 |

---

## 4. 구현 순서 (단계별, 각 단계 검증 후 다음)

각 phase 는 한 PR 단위로 분리. 이전 phase 통과 후 다음 phase 진입.

### Phase 1 — Foundation (2 일)

**P1.1: Layer A ObstacleHorizon 구현** (1.5일)
- `_build_obstacle_horizon()` 함수 신설 (`mpc_planner_state_node.py` 안)
- 기존 `_merge_obs_sources` 대체 (deprecation 주석 + 호출부 변경)
- GP timestep msgs 모두 활용:
  - obstacle id 또는 (s, d) bucket 으로 grouping
  - K_pred 별 (s_k, n_k, vs_k, vd_k, var_k) 추출
  - K_pred < K_mpc → linear interp + last-step extrapolate
- noise filter:
  - obs.vs / vd 의 rolling median (window=5 ticks, deque)
  - delta-vs 5 m/s² 초과 시 spike reject (이전 값 유지)
- nascent fallback:
  - GP variance 이상 OR GP 미publish OR match 실패 → tracker (vs, vd) 상수 + fallback variance = 0.5 m
- output: `obs_horizon[obs_idx, k, :]` 채움

**검증**:
- RViz marker 로 obs_horizon trajectory 시각화 (각 timestep 별 점)
- `/mpc_auto/debug/tick_json` 에 obs_horizon[0] 의 (s, n) sequence dump
- bag 분석: obs_n 의 horizon 별 sequence 가 GP 와 일치 + KF noise 가 평탄화됨

**P1.2: NLP 가 obs_horizon 활용**
- `frenet_kin_solver.py` 의 `P_obs_s, P_obs_n` parameter 가 (k, n_obs) 별로 set 되도록
- 코드는 이미 그렇게 되어 있을 것 (확인 필요). 단 _merge_obs_sources 가 vs 상수 propagate 하는 게 문제였음. obs_horizon 으로 교체하면 자동 정확.

**검증**:
- NLP 의 obs horizon trajectory 시각화 (RViz marker, 다른 색)
- obs.n 의 oscillation 이 NLP 의 obstacle bubble 위치에 반영됨

### Phase 2 — EncounterAnalyzer (1 일)

**P2.1: `_analyze_encounter()` 함수 신설**
- input: obs_horizon, ego_horizon (이전 NLP 결과 또는 forward predict), corridor_horizon
- output: encounter_summary dict
- internal:
  - timestep 별 distance 계산
  - argmin → encounter_k
  - corridor 와 obstacle 의 left/right wall 거리
  - lateral pass margin (with safety_buffer)
  - corridor curvature
  - obs lateral drift 추정 (vd 의 sign)

**검증**:
- `/mpc_auto/debug/tick_json` 에 encounter_summary 추가
- RViz marker 로 encounter point 시각화 (sphere at obs[encounter_k].xy)
- bag 의 충돌 시점 직전 encounter_summary 확인 — 의도된 정보 다 채워졌는지

### Phase 3 — StrategicPlanner (2 일)

**P3.1: 룰베이스 plan_picker 신설**
- 기존 `plan_scorer.py` 의 score 식 사용 안 함
- 새 `strategic_planner.py` 또는 `plan_scorer.py` 안에 `decide_plan(encounter_summary, ego, prev_plan, prev_phase)` 함수
- decision tree (R0~R5) 그대로 구현
- plan dwell (0.5초) 적용
- commit_phase tracking (DETECT, APPROACH, WAIT, COMMIT, PASS, RECOVER, CLEAR, EMERGENCY)

**P3.2: 호출부 변경**
- `mpc_planner_state_node.py` 의 plan_picker 호출 부분에서 새 함수로 교체
- side_decider 결과는 fallback 으로만 사용 (또는 폐기 검토)

**검증**:
- 10분 bag → plan transition log 확인
  - 의도된 sequence: DETECT → APPROACH → WAIT → COMMIT → PASS → RECOVER 가 obstacle 만남 시 한 번씩
  - 토글 빈도: < 5/min
- plan dwell 작동: commit_phase 변경 사이 ≥ 0.5초

### Phase 4 — PlanAwareMPC (2-3 일)

**P4.1: directional obstacle bubble**
- `frenet_kin_solver.py` 의 `J_obs` 계산 변경
- plan_side parameter 추가 (현재 SIDE_LEFT/RIGHT/TRAIL 그대로 사용 가능)
- sigmoid 적용:
  ```python
  if plan_side == SIDE_LEFT:
      side_weight = sigmoid(obs_n_k - n_k + 0.05)   # ego 가 obs 보다 LEFT 일 때 cost ↓
  ```
- JIT 재컴파일

**P4.2: ref_v override (NLP 안에서) + painter 의 plan-aware 보존**
- TRAIL plan: ref_v = obs.vs - 0.3 (이미 작동)
- BRAKE_HARD: ref_v = 0
- painter 의 plan-aware override 도 일관 (이전에 시도했으나 a_max ramp 에 무력화됨 — 이번엔 a_max 도 plan-aware 로):
  - BRAKE_HARD: a_max = 5.0 m/s² (강한 brake 허용)
  - 그 외: 기존 3.0

**P4.3: target_n profile 정리**
- plan_library.py 의 `make_target_n_profile` 검토
- BRAKE_HARD 추가: `[ego_n] * (N+1)` (TRAIL 과 동일, ref_v 만 0)

**검증**:
- 단순 시나리오: ego 가 obstacle 의 right 에 있고 LEFT_PASS 결정 → trajectory 가 LEFT 로 cross 가능 (현재 못함)
- 복잡: 동적 obstacle oscillation 시 trajectory 가 obstacle horizon 따라 회피
- 충돌 빈도 (10분 bag): 목표 < 1 / 10분

### Phase 5 — Safety Buffer + ggv polar (1-2 일, 선택)

**P5.1: safety_buffer adaptive (Layer B 안에 이미 포함)**
- prediction variance + obs.n rolling stddev → encounter_t 시점 lateral buffer 자동 계산
- corridor 의 d_left/d_right 에 buffer 차감해서 pass margin 산정

**P5.2: NLP 의 ggv polar (옵션)**
- frenet_kin_solver.py 에 추가 constraint:
  ```python
  ay_k = v_[k]^2 * P_kappa[k]
  opti.subject_to((a_[k]/a_max)^2 + (ay_k/ay_max)^2 ≤ 1)
  ```
- ay_max 는 friction sector 따라 동적

**검증**:
- obstacle oscillation 0.25m 환경에서 충돌 빈도 ↓
- 코너 진입 시 ay 한계 초과 X

### Phase 6 — 통합 검증 (1 일)

**P6.1: 10분 bag x 5 회**
- 각 회 ego s=0 spawn (재현성)
- 측정: 충돌 events, plan/commit_phase transition rate, lateral min, MPC publish rate, NLP infeasible rate, plan dwell hit rate

**목표 메트릭** (Phase 6 통과 기준):
| 메트릭 | 목표 | 현재 (참고) |
|---|---|---|
| 10분 충돌 events | ≤ 1 | 7 (long_qnt15) |
| MPC publish rate | ≥ 48 Hz | 24 Hz (publish gap) |
| Plan transitions / min | ≤ 5 | 8-16 |
| NLP infeasible rate | ≤ 5% | ~20% 추정 |
| Lateral min when |ds|<1m | ≥ 200mm | 80-150mm |
| ego_v / obs_vs ratio (TRAILING) | 0.95~1.05 | 1.5-2.0 (못 따라잡음) |

---

## 5. 검증 protocol

### 매 phase 후
1. `git diff` 로 변경 점검 (unintended 수정 없는지)
2. Build (`catkin build mpc_planner state_machine`)
3. ego s=0 spawn (`scripts/spawn_ego_s0.sh`)
4. obstacle_publisher + MPC 재기동
5. 10분 bag (`rosbag record -O <phase>_long --duration=600 ...`)
6. `analyze_long_bag.py` + `coll_plan_breakdown.py` + `local_wpnts_lag.py`
7. 결과 row 추가: `HJ_docs/debug/0428_debug/work_log/stage2_phase22_progress_20260428.md`
8. 목표 메트릭 미충족 시 phase 내부 fix or 직전 phase 회귀 확인

### Bag 분석 도구

이미 보유:
- `analyze_user_bag.py`: 종합 (plan, SM state, TRAILING ego_v, collisions)
- `local_wpnts_lag.py`: ego_s ↔ wpnt0_s lag
- `mpc_publish_trace.py`: publish rate + gap + status
- `coll_plan_breakdown.py`: 충돌 시점 plan / ego_n / obs_n / ds

추가 필요:
- `encounter_summary_trace.py` (Phase 2 이후): encounter point 시각화 시계열
- `commit_phase_trace.py` (Phase 3 이후): commit_phase 변화 시계열

---

## 6. 주의 사항 / 한계

- **본 plan 은 동적 obstacle 단일 또는 다수 시나리오 대상**. 정적 obstacle 처리는 별도 phase (사용자 결정 후).
- **GP predictor 자체의 정확도** 는 본 plan 의 범위 밖. GP 가 obstacle oscillation 학습 못 하면 Layer A 의 nascent fallback (큰 variance) 으로 보수적 처리. predictor 개선은 별도 작업.
- **MPC solver 의 NLP 한계**: dynamic single-track / GGV polar 같은 진짜 racing-grade 모델은 Phase 5 또는 별도 작업 (`HJ_docs/mpc_overtake_redesign_plan_20260428.md` 의 Step 10).
- **Controller PID 유지** (사용자 결정). MPC vx_mps 와 PID 출력 사이 충돌 가능. 추후 PID 폐기 결정 시 controller 변경 필요.

---

## 7. Migration plan (현재 stack → 새 architecture)

### 7.1 보존 / 폐기 / 신규

| 컴포넌트 | 결정 |
|---|---|
| `frenet_kin_solver.py` (NLP) | 보존 + 개선 (directional obstacle, plan-aware ref_v, ggv polar) |
| `_merge_obs_sources` | **폐기**. `_build_obstacle_horizon` 으로 대체 |
| `side_decider.py` | **폐기 또는 fallback**. StrategicPlanner 가 대체 |
| `plan_scorer.py` (score 식) | **폐기**. `strategic_planner.py` 가 대체 |
| `plan_library.py` (plan 정의 + target_n profile) | 보존 + BRAKE_HARD 추가 |
| `_post_process_speed` (painter) | 보존 + plan-aware override (a_max plan-aware) |
| `_validate_publish_wpnts` | 보존 (inter_msg_jump reject 폐기 완료) |
| `_handle_emergency_no_traj` | **폐기 완료** |
| Cache (`_cache_recovery_path` etc.) | 보존 + TTL (완료) + NLP success 시 hijack 차단 |
| `_check_free_xy` 의 closest_target patch | 보존 (Phase 0 fix) |
| SM (3d_mpc_state_machine_node) | 변경 X (Phase 6 까지) |
| Controller | 변경 X (사용자 결정) |

### 7.2 백업 정책

각 phase 시작 시 변경 파일 백업:
- `<file>_backup_<phase>_<date>.<ext>`
- 예: `mpc_planner_state_node_phase1_20260429.py`

기존 `_backup_*` 들은 그대로 보존 (회귀 reference).

---

## 8. 후속 (이 plan 통과 후)

본 plan 이 동적 obstacle 시나리오 통과 후:
1. **정적 obstacle 처리** — 별도 plan 단위. plan_scorer 에 multi-obstacle 평가 + 정적/동적 hybrid decision.
2. **GGV-based single-track MPC backend** (`HJ_docs/mpc_overtake_redesign_plan_20260428.md` Step 10).
3. **Strategic learning** (게임 이론 / RL / 등) — racing 효율 추가 향상 시.
4. **Controller PID 폐기 + MPC vx_mps 일관 추적** (architecture 정리).

---

## 9. 진행 트리거

본 plan 통과 (사용자 검토) 후 Phase 1 진입.

각 phase 진행 중 사용자가 직관과 다른 결정/메트릭 발견 시 즉시 멈추고 plan 갱신 후 재진입.

---

**작성**: Claude (HJ 세션, 2026-04-29)
**상태**: 작성 완료. 사용자 검토 대기.
