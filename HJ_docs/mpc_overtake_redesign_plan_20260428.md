# MPC Overtake Redesign Plan — 2026-04-28

> 상태: **draft (사용자 의논 결과 반영, 작업 우선순위 확정 후 본 문서 따라 구현)**
> 진단·논의 출처: 본 세션 (2026-04-28). 24-26일 plan_picker / scoring 수정으로 trial_qnt15 7 events / 600s 까지 축소했으나, **stack 의 architecture-level 결함 두 가지**가 본질적으로 재현 충돌의 root cause임이 확인됨.

---

## 1. 현재 stack 의 본질 결함 (사용자 검증)

### 결함 A — MPC solver 가 풀어낸 v[k] 가 publish 되지 않음

`_apply_speed_painter` ([mpc_planner_state_node.py:2393-2445](../planner/mpc_planner/node/mpc_planner_state_node.py#L2393-L2445)) 가 publish 직전에 **모든 wp.vx_mps 를 vel planner GB raceline vx + curvature cap + a_max ramp 으로 덮어씀**.

- MPC NLP 내부: TRAIL plan 시 `ref_v = obs.vs - 0.3` override → solver 가 그 cap 안에서 `v[k]` 풀이 → 진짜 trailing 속도 trajectory 안에 만들어짐.
- Publish: painter 가 그 v[k] 를 폐기. `wp.vx_mps = min(v_gb_raceline, sqrt(mu*g/|kappa|))`.
- 결과: **MPC 의 plan-aware ref_v 의도가 시스템 밖으로 안 나감**. controller 는 obstacle 모르는 vel planner 출력만 받음.

### 결함 B — Trailing 의 유일한 메커니즘이 Controller PID

[Controller.py:537-557](../controller/combined/src/Controller.py#L537-L557):
```python
if state == "TRAILING" and opponent is not None:
    speed_command = self.trailing_controller(global_speed)
else:
    speed_command = global_speed   # = trajectory.vx_mps (painter 출력)
```

- A 결함으로 인해 `global_speed` (= painter 출력) 는 obstacle 무관.
- TRAILING state 시 controller 의 trailing PID 가 **유일하게 obstacle 인식**.
- 이 PID 는 ggv / curvature 인식 X. 단순 (s gap, vs) PID.
- 또한 PID 입력의 `opponent` 가 SM 의 `_check_free_*` 부산물로만 set → path free 시 None → 충돌 시점에 trailing 꺼지는 경우 발생 (long_qnt15 분석에서 확인된 패턴).

### 정리: 두 결함 함께 작용 시

> "MPC 가 spliner 처럼 작동" — solver 의 v 의도는 NLP 안 internal 일 뿐 외부에 안 보냄. 속도는 painter (vel planner) 가 결정, trailing 은 controller PID 가 분담. **MPC trajectory 의 시간-공간 풀이 결과 전체가 publish 시점에 절반만 살아남음 (geometry 만)**.

---

## 2. 수정 계획 (우선순위 + 작업 단위)

각 단계는 **이전 단계 검증 후** 진행. 단계 사이 10분 bag 으로 충돌 / lat_min / 가속 패턴 비교.

### Step 1 — `obstacles_in_interest` 에 ttc-based encounter window 적용 ✅ (2026-04-28 적용)

**파일**: [3d_mpc_state_machine_node.py:735-761](../state_machine/src/mpc/3d_mpc_state_machine_node.py)
**변경**: gap < horizon 만 보는 게 아니라 `ttc = gap / max(ego.vs - obs.vs, 0.05) < ttc_horizon_s (default 3s)` 추가.
**효과 (예상)**: 멀리 있는 obstacle 또는 ego 가 영원히 못 따라잡는 obstacle 은 trailing trigger 안 됨. race efficiency ↑.
**위험**: 적음. rosparam `state_machine/ttc_horizon_s` 로 토글 가능.
**검증**: bag 에서 obstacles_in_interest 갯수 / TRAILING 빈도 변화 측정.

### Step 2 — `_rewrite_trailing_to_local_arc` default OFF ✅ (2026-04-28 적용)

**파일**: [3d_mpc_state_machine_node.py:357-369](../state_machine/src/mpc/3d_mpc_state_machine_node.py#L357-L369)
**변경**: rosparam `state_machine/trailing_gap_local_arc` default `True` → `False`.
**효과**: `tgt.s_center` 이 raceline frenet 그대로 controller 에 전달. local-arc 기반 rewrite 의 obs_idx ≤ ego_idx drop 부작용 제거.
**위험**: trailing PID 의 gap 계산이 raceline s 기준으로 바뀜 (legacy). 코너에서 정확도 약간 ↓ but 충돌 reduction 우선.

### Step 3 — `_check_free_xy` 가 closest_target 도 set ✅ (2026-04-28 적용)

**파일**: [3d_mpc_state_machine_node.py:1515-1547](../state_machine/src/mpc/3d_mpc_state_machine_node.py#L1515-L1547)
**변경**: free=False 일 때 closest_target / closest_gap 도 set (frenet 버전과 행동 일치).
**효과**: MPC OT path 평가에서 trailing target 누락 case 제거.

### Step 4 — `_apply_speed_painter` plan-aware override (결함 A 1차 fix)

**파일**: [mpc_planner_state_node.py:2393-2445](../planner/mpc_planner/node/mpc_planner_state_node.py#L2393-L2445)
**변경 메커니즘**:
```python
# 기존: capped[k] = min(vel_planner_gb_vx, vmax_curv)
# 변경: TRAIL plan 일 때 obs.vs cap 추가 적용
if self._last_plan == 'trail' and obs_vs is not None:
    capped[k] = min(capped[k], obs_vs - 0.3)
elif self._last_plan in ('left_pass', 'right_pass'):
    pass  # full speed (painter 의 기존 vmax_curv 만)
```
**효과**: MPC NLP 안 ref_v override 와 동일한 logic 을 painter 에 복제 → trajectory.vx_mps 가 plan 의도 반영.
**위험**: 매우 작음. 기존 painter 의 안전 마진 (curvature cap, a_max ramp) 그대로 유지.
**대안**: Step 4-alt — painter 의 baseline 을 vel planner GB vx 대신 MPC solver 의 v[k] 로. 더 깔끔하지만 NLP solver 의 v 가 noisy 할 수 있어 검증 더 필요.

### Step 5 — Controller TRAILING PID off (결함 B fix, Step 4 후 안전)

**파일**: [Controller.py:537-557](../controller/combined/src/Controller.py#L537-L557)
**변경**:
```python
# 기존
if state == "TRAILING" and opponent is not None:
    speed_command = self.trailing_controller(global_speed)
else:
    speed_command = global_speed

# 변경 (trailing PID 제거, MPC trajectory.vx_mps 충실 추적)
speed_command = global_speed
```
**전제**: Step 4 적용 후 trajectory.vx_mps 가 plan 의도 (TRAIL 시 obs.vs - 0.3) 반영. 안 적용 상태에서 단독 실행 시 **trailing 자체 사라짐 → 충돌 즉시**.
**효과**: architecture 일관. MPC ggv-aware ref_v 가 controller 까지 일관 적용 → 코너 trailing 안전.
**위험**: trajectory.vx_mps 가 stale 시 (publish 지연) controller 의 빠른 PID 반응 잃음. 그러나 NLP 50Hz, PID 도 50Hz — latency 차이 거의 없음.

### Step 6 — GP timestep-별 trajectory 활용 + interpolation/filter

**파일**:
- [mpc_planner_state_node.py:828-955](../planner/mpc_planner/node/mpc_planner_state_node.py#L828-L955) `_obs_predict_cb` + `_merge_obs_sources`
- (필요 시) 새 helper 함수

**현재 결함**: GP 가 horizon timestep 별 N=20 msgs publish 하는데, `_merge_obs_sources` 가 1개만 뽑아 vs/vd 추출 후 horizon (40 step) 에서 상수 propagate. **GP 의 oscillation 학습 0% 활용**.

**변경**:
1. `_obs_predict_cb` 가 obstacle id 별로 timestep-indexed list 보관: `{obs_id: [(s_k, n_k, vs_k, vd_k, var_k) for k in range(K_pred)]}`.
2. `_merge_obs_sources` 가 tracker obstacle 마다 GP trajectory 1개 매칭.
3. MPC `obs_arr[obs_idx, k, :]` 채울 때:
   - `k < K_pred`: GP 의 (s_k, n_k) 직접 사용
   - `k ≥ K_pred`: 마지막 GP step 의 vs/vd 로 linear extrapolation
4. **Robust handling**:
   - GP 일부 timestep 누락 → linear interpolation between known steps
   - obs.n noise → savgol filter (small window, polyorder=2) 또는 GP variance 큰 step 만 down-weight
   - GP 미publish (nascent) → tracker (vs, vd) 상수 extrapolation + 큰 fallback variance

**효과**: MPC 가 obstacle 의 oscillation 을 horizon 안에서 정확히 인식 → path 가 timestep-aware 회피 → tube buffer 효과 자동.
**작업량**: 중간 (1-2일).

### Step 7 — 상대 속도 (obs.vs) tracking-only 분석 + filtering

**원칙 (사용자 강조 2026-04-28)**: **truth 와 비교 절대 금지**. 실제 환경에선 truth 없음. tracking topic 자체의 시계열만 보고 spike / outlier 판정.

**작업**:
1. bag 에서 `/tracking/obstacles` 의 vs 시계열 plot.
2. delta vs (consecutive ticks) 분포 → outlier 임계값 결정 (예: |delta| > 5 m/s²).
3. SM 또는 MPC node 단에서 obs.vs 의 rolling median (5 tick window) — spike reject.
4. delta vs 임계 초과 시 그 tick 의 obs.vs 무시 + 이전 값 유지.
5. 또는 EMA smoothing (alpha 0.3).

**위치**: `obstacle_perception_cb` 안 또는 별도 sanitize layer.
**효과**: KF estimate 의 spike 가 MPC ref_v override 에 직접 반영되는 것 차단.

### Step 8 — Multi-modal hand-crafted hypothesis (plan_scorer 보강)

**파일**: [plan_scorer.py](../planner/mpc_planner/src/plan_scorer.py)
**변경**:
- LEFT_PASS / RIGHT_PASS plan 평가 시 **obstacle 의 가능 행동 hypothesis 들** 각각 d_free / risk 계산:
  - hypothesis 1: obstacle stays current lateral
  - hypothesis 2: obstacle drifts toward LEFT (oscillation peak)
  - hypothesis 3: obstacle drifts toward RIGHT
- weighted (or worst-case) score → plan score 결정.
- 현재 stack 의 GP 가 single-mode 이므로 multi-modal 표현 보완.

**작업량**: 중간. plan_scorer 안 함수 1-2개 추가.

### Step 9 — frenet_kin_solver 에 ggv polar constraint (옵션 B)

**현재**: `vmax_curv = sqrt(mu*g/|kappa|)` 가 lateral g 한계 1차 근사. `a_max` 는 longitudinal 한계 별도. polar 결합 X.

**추가**:
```python
# NLP 안에 (각 timestep k)
ay_k = v_[k]**2 * P_kappa[k]
opti.subject_to((a_[k]/a_max)**2 + (ay_k/ay_max)**2 <= 1)
```
**ay_max 는 sector mu 따라 동적**.

**효과**: 코너 진입/진출 전이 영역에서 longitudinal 가속/제동이 lateral 한계 잠식하지 않음.
**위험**: nonlinear constraint → IPOPT solve 시간 ↑ + infeasibility 빈도 가능 ↑. 충분한 튜닝 필요.
**작업량**: 반나절.

---

## 3. ggv-based single-track MPC 분석 (TODO — Step 10+)

### 가능 여부

**Yes**. 우리 stack 에 이미 필요한 구성요소 존재.

### 입력 자산 (이미 있음)

#### 3.1 `planner/3d_gb_optimizer/fast_ggv_gen/`
- 차량별 GGV diagram parametric NLP 생성기.
- Output: `output/<vehicle>_latest/{vehicle_frame, velocity_frame}/` 안에 npy 파일들:
  - `ax_max[v, g, alpha]` — longitudinal 최대 가속 (특정 속도 v, gravity component g, heading angle alpha)
  - `ax_min[v, g, alpha]` — 최대 제동 (= -|ax_min|)
  - `ay_max[v, g, alpha]` — lateral 최대 가속
  - `gg_exponent[v, g, alpha]` — diamond/polar 근사 exponent ρ
  - `rho[v, g, alpha]`, `alpha_list[]`, `g_list[]`, `v_list[]` — 격자
- 즉 **3D GGV map (V × g × α)** 형태로 사용자 튜닝된 차량 한계 표현.

#### 3.2 `planner/3d_gb_optimizer/global_line/`
- TUM-style global racing line 생성 (Mintime). dynamic model (single-track) + GGV 통합 OPT.
- 출력은 raceline waypoint sequence (s, x, y, kappa, vx, ...).

#### 3.3 `stack_master/scripts/fbga_velocity_planner.py`
- ROS node. global_waypoints 받아 FBGA (forward-backward gradient) 로 곡률+GGV 기반 v 재계산.
- C++ binary 호출. friction sector 별 GGV bin 사용.
- **즉 raceline 의 baseline vx 는 이미 GGV-aware**. (vel planner 가 단순 sqrt(mu·g/|kappa|) 가 아님 — full GGV polar 적용).

### 통합 가능성 평가

**가능. 다음 두 옵션**:

#### 옵션 GGV-A (큰 작업, 별도 backend)

새 파일 `planner/mpc_planner/src/ggv_singletrack_solver.py`:
- Single-track dynamics (lateral slip 표현, linear or pacejka tire)
- CasADi NLP 안에서 GGV constraint 을 ax/ay 에 직접 적용:
  ```python
  ax_max_fn = casadi.interpolant('ax_max', 'bspline',
                                 [v_grid, g_grid, alpha_grid],
                                 ax_max_npy.flatten())
  # NLP constraint
  (a_x[k] / ax_max_fn(v[k], g[k], alpha[k]))**ρ + 
  (a_y[k] / ay_max_fn(v[k], g[k], alpha[k]))**ρ ≤ 1
  ```
- node 의 `solver_backend` param 으로 swap (`frenet_kin` vs `ggv_singletrack`).
- 기존 `frenet_kin_solver.py` 그대로 유지 → 사용자 의도 (기존 파이프라인 안 망가뜨림) 부합.

**작업 규모**: **3-4일**.
- npy 로딩 + interpolant ~반나절
- single-track dynamics ~1-2일 (kinematic → dynamic 전환은 NLP 변경 큼)
- NLP setup + JIT compile ~1일
- 검증 (단순 시나리오 + 기존 obstacle 시나리오) ~1일

**위험**:
- NLP numerical conditioning. ggv polar 는 nonlinear → IPOPT solve 시간 30-50ms 증가 가능 (현재 10-20ms).
- Tire model linear 면 high-slip 영역 부정확. pacejka 면 더 정확하지만 NLP 더 nonlinear.

**장점**: 기존 코드 안 망가짐. 사용자 튜닝 GGV 직접 사용. 진짜 racing-grade MPC.

#### 옵션 GGV-B (작은 작업, 기존 backend 보강) — Step 9 와 동일

`frenet_kin_solver.py` 에 ggv polar constraint 만 추가. kinematic bicycle 유지. ay_max 는 sector mu 동적.

**작업 규모**: **반나절**.

**한계**: kinematic bicycle 은 lateral slip 표현 X → 진짜 single-track 한계 못 반영.
**효과**: 코너 진입 영역에서 long+lat 결합 한계 표현. racing 에서 큰 개선.

### 권장

**옵션 GGV-B 먼저 (Step 9), 효과 측정 후 GGV-A 결정**.

GGV-A 는 **Stage 3+ 후보**. 단기 우선순위는 위 Step 4-7 (architecture 결함 fix + GP 활용 + obs.vs filtering) 가 더 큰 임팩트 예상.

---

## 4. 큰 architecture 변경 (Stage 3+ 후보)

| 항목 | 동기 | 작업 |
|---|---|---|
| **MPC solver 의 plan-aware directional obstacle cost** | Gaussian bubble direction-agnostic → wrong-side cross 불능 | 솔버 cost 구조 변경 + JIT 재컴파일 |
| **GGV-A (single-track + GGV polar)** | 진짜 racing-grade dynamics | 새 backend file |
| **Branch / multi-modal MPC** | opponent 의 multi-modal 행동 robust 대응 | 학술 수준 |
| **Strategic plan timing (close trail → opportunity → commit)** | reactive plan_picker → multi-stage 의도 표현 | plan_scorer + state machine 확장 |

---

## 5. 검증 protocol

각 Step 후:
1. ego s=0 spawn (gazebo set_model_state)
2. obstacle_publisher + MPC 재기동
3. 10분 bag 녹화 (`scripts/spawn_ego_s0.sh` + rosbag record 600s)
4. `analyze_long_bag.py` 로 충돌 events / lat_min / transition rate 측정
5. `coll_plan_breakdown.py` 로 충돌 시점 plan 분류
6. 결과를 [HJ_docs/debug/0428_debug/work_log/stage2_phase22_progress_20260428.md](debug/0428_debug/work_log/stage2_phase22_progress_20260428.md) 에 row 추가

---

## 6. 절대 하지 말 것 (사용자 강조 2026-04-28)

1. **Tracking 노이즈 분석에 truth 사용** — `/tracking/obstacles_truth` 와 `/tracking/obstacles` 비교 금지. 실제 환경에서 truth 없음. tracking topic 자체 시계열만으로 spike 판정.
2. **MPC ref_v 무시 / 우회** — solver 의 plan-aware 의도 살리는 방향. painter / controller 가 덮는 구조 풀어야.
3. **layer 별 단편 fix 만 누적** — architecture 결함 (A, B) 위에 sticky/filter/scoring 만 fix 해 봤자 임시. 결함 자체 풀어야.
