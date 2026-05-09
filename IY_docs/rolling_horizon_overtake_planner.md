# Rolling-Horizon Overtake Planner: Design Document

> **목표**: ForzaETH `multiopponent-pspliner`의 one-shot SQP 기반 overtake 경로 생성 방식을,
> **매 주기 재최적화되는 rolling-horizon 구조**로 확장한다.
> Controller는 기존처럼 "주어진 경로+속도"를 추종하는 역할만 담당하며,
> planner가 **opponent와 ego 상태에 따라 계속 refine되는 reference trajectory**를 publish한다.

---

## 1. Motivation

### 1.1 현재 pspliner의 한계

현재 `multiopponent-pspliner`는 다음과 같이 동작한다:

1. `GB_TRACK` → opponent 감지 → `TRAILING`
2. 첫 랩 trailing 중 GP regression으로 opponent의 `GP_d(s)`, `GP_v(s)` 학습
3. Overtake trigger 시점에 **한 번** SQP로 overtake spline 생성
4. 이후 OVERTAKE 상태 동안 그 spline을 **commit**해서 tracking

**문제**: 이 구조는 **planning-and-forget** 형태이므로,
- 처음 뽑힌 경로가 suboptimal하거나 (초기 candidate가 좋지 않거나)
- Opponent가 예측과 다르게 움직여서 더 이상 그 경로가 최적이 아니거나
- 해당 경로의 속도 프로파일이 원 raceline 유지보다 느려지는 경우

→ **이미 commit되었기 때문에 경로를 손볼 수 없음**.
안전상 abort는 GP 공분산 이탈 시에만 발동하며, **"느린 경로에 갇히는"** 상황은 전혀 감지되지 않는다.

### 1.2 제안하는 구조

Overtake 경로를 **고정된 1회 계산**이 아닌 **rolling-horizon 재최적화**로 바꾼다.
Planner는 매 cycle마다:

1. 최신 opponent 상태 + 최신 ego 상태를 받아
2. 이전 solution을 warm-start로 SQP를 다시 풀어 `d(s)` refine
3. Velocity profile을 입혀 `v(s)` 생성
4. `(s, x, y, ψ, κ, v, a_x, a_y)` reference trajectory를 controller로 publish

결과적으로 overtake 상태에서도 경로가 **opponent + ego 상태에 반응하며 smooth하게 morph**한다.
이는 MPC의 철학 — *"매 step마다 다시 푸는데, 이전 solution은 좋은 초기점일 뿐"* — 과 동일하다.

---

## 2. High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Perception (기존 유지)                     │
│   LiDAR → Opponent detection, tracking, Re-ID                │
└────────────────┬────────────────────────────────────────────┘
                 │ opponent states
                 ▼
┌─────────────────────────────────────────────────────────────┐
│              GP Regression (기존 유지, per-lap)              │
│   Observed opponent data → GP_d(s), GP_v(s)                  │
└────────────────┬────────────────────────────────────────────┘
                 │ GP models
                 ▼
┌─────────────────────────────────────────────────────────────┐
│         ★ Rolling Planner (NEW, 20~50 Hz) ★                  │
│   ┌──────────────────────────────────────────────────────┐  │
│   │ 1. RoC 재계산 (GP forward-propagation)               │  │
│   │ 2. Warm-start SQP → d(s) refine                      │  │
│   │ 3. Velocity profile (GG-based FB pass) → v(s)        │  │
│   │ 4. Performance/Safety abort check                    │  │
│   │ 5. Publish reference trajectory                      │  │
│   └──────────────────────────────────────────────────────┘  │
└────────────────┬────────────────────────────────────────────┘
                 │ (s, x, y, ψ, κ, v, ax, ay)
                 ▼
┌─────────────────────────────────────────────────────────────┐
│                 Controller (기존 유지, 100 Hz+)              │
│   MAP / Pure Pursuit + PI on velocity                        │
└─────────────────────────────────────────────────────────────┘
```

**핵심 설계 원칙**:
- Planner는 **경로 + 속도가 이미 입혀진** reference trajectory를 뱉는다.
- Controller는 기존 구조 그대로 유지한다 (MPC-in-control이 아님).
- Planner 내부에서 "계속 계산"이 일어나지만, 외부 인터페이스는 기존 pspliner와 호환되게 유지한다.

---

## 3. Rolling Planner 상세 설계

### 3.1 실행 주기와 State Machine 통합

```
State: GB_TRACK
  - Rolling planner inactive (global raceline 사용)

State: TRAILING
  - Rolling planner inactive
  - GP 업데이트만 수행 (기존과 동일)

State: OVERTAKE  ← 여기서 rolling planner 활성화
  - 매 cycle (20~50 Hz) 재최적화
  - Exit 조건:
      (a) Opponent 통과 완료 (c_end 통과)        → GB_TRACK
      (b) Safety abort (GP 공분산 이탈)          → TRAILING
      (c) Performance abort (T_OT > T_trail+m)   → TRAILING  ← NEW
```

### 3.2 Planning 주기 구조

매 cycle에서 다음 순서로 실행:

```python
def rolling_planner_step():
    # ---- (0) State prediction ----
    # 다음 cycle 시작 시점의 ego 상태를 예측 (solve time 보상)
    ego_pred = predict_ego_state(ego_now, dt_solve)

    # ---- (1) Opponent & RoC update ----
    roc = compute_roc(gp_d, gp_v, ego_pred)  # [c_start, c_end]

    # ---- (2) Shape refinement (SQP) ----
    d_init = warm_start_from_previous(prev_solution, shift=ego_pred.v * dt)
    d_new = sqp_solve(
        init=d_init,
        constraints=[track_bounds, curvature_max, roc_clearance],
        cost=shape_cost + regularization(d_init)
    )

    # ---- (3) Velocity profile ----
    v_new = forward_backward_profile(
        path=d_new,
        v0=ego_pred.v,           # IC matching (중요!)
        gg_limits=gg_diagram
    )

    # ---- (4) Abort checks ----
    t_overtake = integrate_time(d_new, v_new, roc)
    t_trail = predict_trailing_time(gp_v, roc)

    if gp_covariance_exceeded(opponent_obs, gp_d, gp_v):
        return ABORT_SAFETY
    if t_overtake > t_trail + t_margin:
        return ABORT_PERFORMANCE

    # ---- (5) Publish ----
    publish_reference(d_new, v_new)
    prev_solution = d_new
```

### 3.3 Shape Refinement (SQP)

기존 pspliner SQP의 초기화 방식만 바뀜:

| 항목 | 기존 | 변경 |
|-----|-----|-----|
| 초기 guess | 해석적 spline 초기화 | **이전 cycle solution의 shifted warm-start** |
| 실행 빈도 | Overtake 시작 시 1회 | **매 cycle (20~50 Hz)** |
| Cost term | shape + clearance | shape + clearance + **regularization** |
| Constraint | track bounds, curvature | 동일 |

**Warm-start 방식**:
```
이전 cycle solution: d_{k-1}(s), s ∈ [s_0^{k-1}, s_end^{k-1}]
Ego가 Δs = v · dt 만큼 전진했다고 가정하고 shift:
    d_init^k(s) = d_{k-1}(s + Δs),  s ∈ [s_0^k, s_end^k]
```

**Regularization term** (중요):
```
J_reg = λ_reg · ∫ ||d^k(s) - d_init^k(s)||² ds
```
- 이것이 없으면 **homotopy class가 튈 수 있음** (왼쪽 추월하다가 갑자기 오른쪽이 더 싸다고 판정되어 경로 jump)
- `λ_reg`는 tuning 파라미터. 너무 크면 opponent 변화에 둔감해지고, 너무 작으면 chatter 발생.

### 3.4 Velocity Profile (Forward-Backward)

RC 차량은 다운포스 무시 가능 → GG는 속도 독립적.
TUMRT 스타일의 forward-backward pass를 그대로 활용.

```
Step 1: κ(s)에서 cornering 속도 상한
    v_max_corner(s) = sqrt(a_lat_max / |κ(s)|)

Step 2: Forward pass (가속 제약)
    v_fwd(s_{i+1})² = min(v_max_corner(s_{i+1})²,
                          v_fwd(s_i)² + 2·a_long_acc·Δs)
    IC: v_fwd(s_0) = v_ego_current   ← 반드시 매칭

Step 3: Backward pass (감속 제약)
    v_bwd(s_i)² = min(v_bwd(s_{i+1})² + 2·a_long_brake·Δs,
                      v_fwd(s_i)²)

Step 4: Combined GG 제약으로 clipping
    각 s에서 (a_lat, a_long)이 GG diamond/ellipse 안에 들도록
```

**IC matching이 특히 중요**:
`v_fwd(s_0) = v_ego_current`로 강제하지 않으면 매 cycle publish되는 trajectory의 속도 시작점이 튀어서 controller의 `a_x` 명령이 점프한다.

### 3.5 Output Interface

Controller에 넘겨주는 reference trajectory 구조:

```python
@dataclass
class OvertakeReference:
    s:     np.ndarray    # arc-length [m]
    x:     np.ndarray    # global x [m]
    y:     np.ndarray    # global y [m]
    psi:   np.ndarray    # heading [rad]
    kappa: np.ndarray    # curvature [1/m]
    v:     np.ndarray    # target speed [m/s]
    ax:    np.ndarray    # longitudinal accel [m/s²]
    ay:    np.ndarray    # lateral accel [m/s²]
    stamp: Time          # planner 실행 시점
    horizon_valid: bool  # 새 solve 성공 여부
```

---

## 4. Validity Guarantees

Controller가 안전하게 추종할 수 있도록 planner는 다음을 보장한다:

### 4.1 기하학적 Feasibility

- **Curvature 상한**: `|κ(s)| ≤ κ_max = tan(δ_max) / L`
  (δ_max: 최대 조향각, L: wheelbase)
- **연속성**: B-spline 차수 ≥ 3 → C² 연속 → yaw rate가 smooth
- **Track boundary**: `d_min(s) ≤ d(s) ≤ d_max(s)`

### 4.2 동역학적 Feasibility (GG 제약)

모든 s에서:
- `v(s)² · |κ(s)| ≤ a_lat_max`
- `|a_long(s)| ≤ a_long_max(v(s))`
- `(a_long/a_long_max)² + (a_lat/a_lat_max)² ≤ 1` (ellipse) 또는 diamond 제약

### 4.3 Cycle 간 Consistency

- **Ego 시작점 예측**: planner 시작점을 "지금 ego 위치"가 아닌 "다음 cycle 시점 predicted ego"로 설정 → controller 인계 시 discontinuity 방지
- **Warm-start + regularization**: 경로가 cycle 사이에 jump하지 않음
- **Velocity IC 매칭**: `v(s_0) = v_ego_current`

### 4.4 Opponent Avoidance

- RoC 구간 `[c_start, c_end]`에서 lateral clearance 제약
- RoC 자체는 매 cycle GP forward-propagation으로 재계산

---

## 5. Abort Logic

### 5.1 Safety Abort (기존 유지)

```
IF opponent observed position/velocity lies outside GP covariance envelope:
    return TRAILING
```

ForzaETH 원 논문 로직 그대로. Opponent가 예측과 너무 다르게 움직이면 포기.

### 5.2 Performance Abort (NEW)

```
T_overtake_est = ∫_{s_0}^{c_end} 1/v(s) ds          # 새 planner 출력에서 계산
T_trail_est    = ∫_{s_0}^{c_end} 1/GP_v(s) ds       # opponent GP에서 계산

IF T_overtake_est > T_trail_est + T_margin:
    return TRAILING
```

**의미**: "지금 이 overtake 경로가 실질적으로 더 빠른가?"를 매 cycle 검증.
Rolling horizon 구조의 velocity profile에서 **자연스럽게 얻어지는** 신호이므로 추가 계산 비용이 거의 없다.

### 5.3 Abort가 계속 발동되는 경우 처리

- 연속 N cycle 동안 동일 abort가 발동 → 해당 overtake attempt 완전 포기
- Cooldown 시간 동안 재진입 금지 (chattering 방지)

---

## 6. 구현 체크리스트

### 6.1 신규 모듈

- [ ] `rolling_planner_node.py`: 메인 ROS2 node, 20~50 Hz loop
- [ ] `warm_start.py`: 이전 solution shift + regularization term 생성
- [ ] `velocity_profiler.py`: GG-based forward-backward pass (TUMRT 포팅)
- [ ] `abort_checker.py`: safety + performance abort 로직
- [ ] `reference_publisher.py`: controller용 message publish

### 6.2 기존 모듈 수정

- [ ] SQP solver interface: warm-start 입력 받도록 확장
- [ ] State machine: OVERTAKE 상태에서 rolling loop 활성화 + performance abort 추가
- [ ] GP regression: 기존 per-lap 업데이트 유지 (변경 없음)

### 6.3 파라미터 (YAML)

```yaml
rolling_planner:
  rate_hz: 30.0
  solve_timeout_ms: 20.0

  warm_start:
    enable: true
    shift_lookahead_s: 0.05        # dt_solve [s]

  regularization:
    lambda_reg: 1.0                # 이전 solution 근접도
    homotopy_lock: true            # overtake 방향 고정 여부

  velocity_profile:
    a_lat_max: 8.0                 # [m/s²]
    a_long_acc_max: 5.0            # [m/s²]
    a_long_brake_max: 8.0          # [m/s²]
    gg_shape: "ellipse"            # "diamond" | "ellipse"

  abort:
    performance_margin_s: 0.1      # T_margin [s]
    consecutive_cycles: 3          # N cycle 연속 발동 시 포기
    cooldown_s: 2.0                # 재진입 금지 시간
```

---

## 7. 평가 계획

### 7.1 Simulation (f1tenth_gym_ros)

| Scenario | 기존 pspliner | Rolling planner (기대) |
|---------|--------------|----------------------|
| Opponent 일정 속도 | 성공 | 성공 (경로 거의 고정) |
| Opponent 랜덤 블로킹 | Abort 빈번 | 경로 adapt로 성공률 ↑ |
| Opponent 급감속 | 충돌 위험 | 경로 morph로 회피 |
| 초기 SQP가 suboptimal | 느린 lap | Refine으로 회복 |
| Overtake 불가능 상황 | 끝까지 느린 경로 추종 | Performance abort 발동 |

### 7.2 Metric

- **Overtake success rate** (pspliner 논문과 동일 정의)
- **Average overtake speed ratio** (`v_OT / v_ego_max`)
- **Path smoothness** (cycle 간 d(s) L2 diff)
- **Computation time** (cycle당 solve 시간, 99th percentile)
- **Performance abort rate** (얼마나 많은 경우에 "차라리 trailing"으로 회귀하는지)

### 7.3 Real hardware (ICRA 2026 준비)

- Intel NUC / OBC에서 30 Hz 유지 가능한지 검증
- 실차에서의 state estimation noise가 warm-start chattering을 유발하지 않는지
- Controller 인계 시 실제 steering/throttle 연속성 측정

---

## 8. Open Questions / Future Work

1. **Homotopy class 전환 허용 여부**
   - 현재 설계는 `homotopy_lock=true` 권장 (왼쪽→오른쪽 전환 금지)
   - 열어두려면 별도 behavior-level planner가 필요 (multi-hypothesis)

2. **Multi-opponent 확장**
   - M-PSpliner의 multi-RoC 구조와 결합 시, RoC 개수에 따라 solve time이 늘어남
   - Opponent 우선순위 heuristic 필요

3. **Velocity profile의 GP 활용**
   - 현재는 ego GG 기반만 사용
   - Opponent의 `GP_v(s)`를 활용해서 "opponent가 느려지는 구간"을 더 공격적으로 attack하는 velocity shaping 가능

4. **Learning-based warm-start**
   - 이전 lap의 성공적 overtake trajectory를 library화 → 유사 상황에서 더 좋은 initial guess로 활용

---

## 9. References

- Baumann et al., *Predictive Spliner: Data-Driven Overtaking in Autonomous Racing Using Opponent Trajectory Prediction*, arXiv:2410.04868 (2024)
- Baumann et al., *M-Predictive Spliner: Enabling Spatiotemporal Multi-Opponent Overtaking for Autonomous Racing*, arXiv:2506.16301 (2025)
- Baumann et al., *ForzaETH Race Stack*, Journal of Field Robotics (2024)
- TUMRT `sampling_based_3D_local_planning`, `online_3D_racing_line_planning` (velocity profile 포팅 참조)
