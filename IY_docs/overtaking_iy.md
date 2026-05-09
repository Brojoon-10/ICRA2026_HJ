# `overtaking_iy` 패키지 역할 정리

> Rolling-horizon SQP 기반 오버테이크 플래너. `sqp_planner`의 1-shot SQP와 병렬로 돌며, 매 cycle (20Hz) 이전 해를 warm-start로 재풀이한다. ROS1 Noetic.

경로: `planner/overtaking_iy/`
설계 문서(이론): [`IY_docs/rolling_horizon_overtake_planner.md`](rolling_horizon_overtake_planner.md)

---

## 1. 개요

`sqp_planner/src/sqp_avoidance_node.py`가 OT 진입 시점 1회만 SQP를 풀고 결과를 유지하는 반면, 이 패키지는 **매 cycle (50ms)마다 SQP를 다시 풀어 장애물 움직임 / ego 상태 변화 / GP 예측 업데이트를 반영**한다. CasADi + IPOPT로 NLP를 구성하고 previous primal/dual을 warm-start로 재사용해 cycle간 해가 연속적이다.

현재 standalone 모드: `/planner/rolling/otwpnts`로 publish만 하고 state_machine은 기존 `sqp_avoidance`를 소비 (dry-run). 실제 제어에 투입하려면 mux 노드 또는 remap 필요 (§7).

---

## 2. 디렉토리 구조

```
planner/overtaking_iy/
├── CMakeLists.txt          # project(overtaking_iy)
├── package.xml             # name: overtaking_iy
├── config/
│   └── overtaking_iy.yaml  # ROS param (weights, abort, warm-start)
├── launch/
│   └── overtaking_iy.launch
└── src/
    ├── overtaking_iy_node.py   # 20Hz main node (rospy)
    ├── sqp_casadi.py           # CasADi NLP builder + solver (warm-start)
    ├── warm_start.py           # 이전 해의 s-axis shift 보간
    ├── velocity_profiler.py    # vel_planner FB pass wrapper
    └── abort_checker.py        # safety / performance abort + hysteresis
```

---

## 3. 각 모듈 역할

### 3.1 `overtaking_iy_node.py` (main)

| 항목 | 내용 |
|---|---|
| ROS node name | `overtaking_iy_node` |
| Class | `OvertakingIYNode` |
| Rate | 20Hz (`~rate_hz`) |

**한 cycle의 순서**

1. 장애물 필터 — `/tracking/obstacles` + `/opponent_prediction/obstacles` 병합 후 `|d_center| < obs_traj_tresh` & `(s_start - cur_s) mod max_s < lookahead`만 유지
2. 장애물 cluster화 — dynamic 장애물은 s-gap ≤ 0.5m 기준으로 묶어 envelope으로 aggregate, static은 개별 유지
3. RoC 구성 — cluster별 s-range의 union으로 `gb_idxs` 계산 (빈 구간 제외)
4. Side 결정 — cluster **각각**에 `_more_space` 호출해 좌/우 및 apex 판단. 첫(ego-nearest) cluster의 side에만 `homotopy_lock` 적용
5. Dynamic s_end extension — cluster의 side가 curvature outside와 같은 경우에만 해당 cluster의 s_end 확장
6. Ego 예측 — `start_av = cur_s + cur_v * dt_solve`, `dt_solve = max(shift_lookahead_s, last_solve_ms*1e-3 + 0.03)`
7. Warm-start — 이전 해 `prev_d`를 `shift_solution`으로 새 s-grid에 보간. 첫 cycle은 `analytic_fallback`
8. SQP solve (`CasadiSQPSolver.solve`) — d(s) 최적화, `n_clusters ≥ 2`면 `lambda_side=0, desired_side='any'`로 양측 회피 허용
9. Dense reinterpolate → Cartesian (`FrenetConverter.get_cartesian`) → CCMA smoothing → Frenet roundtrip (2D)
10. 속도 프로파일 — `VelocityProfiler.profile`로 v(s) 생성, `v_start = v_ego` IC
11. Abort check — safety / performance 조건
12. Publish `OTWpntArray` + markers

**Subscribers**

| Topic | Msg | 용도 |
|---|---|---|
| `/tracking/obstacles` | `ObstacleArray` | 인지 (정적) |
| `/opponent_prediction/obstacles` | `ObstacleArray` | GP 기반 예측 |
| `/car_state/odom` | `Odometry` | ego (x,y,v) |
| `/car_state/odom_frenet` | `Odometry` | ego (s, yaw) |
| `/global_waypoints` | `WpntArray` | FrenetConverter 초기화 |
| `/global_waypoints_scaled` | `WpntArray` | RoC, 속도 한계 |
| `/global_waypoints_updated` | `WpntArray` | Updated raceline (fallback은 scaled) |
| `/opponent_trajectory` | `OpponentTrajectory` | GP d, d_var, v |
| `/ot_section_check` | `Bool` | OT 구간 진입 여부 |
| `/planner/avoidance/smart_static_active` | `Bool` | 기존 smart-static과 race condition 방지 |
| `/behavior_strategy` | `BehaviorStrategy` | local wpnts 참조 |

**Publishers** (모두 `/planner/rolling/*` 하위)

| Topic | Msg | 내용 |
|---|---|---|
| `/planner/rolling/otwpnts` | `OTWpntArray` | **핵심 출력 — OT 경로** |
| `/planner/rolling/markers` | `MarkerArray` | RViz 경로 시각화 (cyan=active, orange=dry-run) |
| `/planner/rolling/merger` | `Float32MultiArray` | `[s_end_obs, s_end_evasion]` (raceline merge 힌트) |
| `/planner/rolling/debug` | `Float32MultiArray` | `[solve_ms, ok, t_ot, t_trail, abort_flag]` |
| `/planner/rolling/diag` | `Float32MultiArray` | 19필드 진단 (obs_center stats, d_init/d_opt std, bounds, cluster 수, adaptive timing 등) |
| `/planner/rolling/latency` | `Float32` | `/measure` true일 때만 |

### 3.2 `sqp_casadi.py` — CasADi NLP builder

- `SQPProblem` dataclass — n_knots, delta_s, d_init, current_d, bounds, obstacle clearance params, kappa_limit, weights
- `CasadiSQPSolver` — IPOPT wrapper, NLP 구조 변화(n_knots 또는 obs_indices)가 있을 때만 `_build`로 재빌드. 그 외에는 parameter 교체 + primal/dual warm-start
- **변수**: `d ∈ R^n` (lateral deviation)
- **목적**: `λ_smooth‖d''‖² + λ_jerk‖d'''‖² + λ_start(d[1]-d[0])² + λ_apex‖d‖² + λ_reg‖d - d_init‖² + λ_side‖max(-side·d,0)‖²`
- **제약**:
  - 경계: `d_lb ≤ d ≤ d_ub` (primal bounds)
  - 등식: `d[0] ≈ current_d`, `d[-1] ≈ 0` (±1e-2 tol)
  - 장애물 클리어런스: `(d[idx] - c)² ≥ r²` per cluster (knot 2부터 시작 — knot 0,1은 smooth transition용 버퍼)
  - Turning radius: `κ² ≤ κ_limit²`

### 3.3 `warm_start.py`

- `shift_solution(prev_d, prev_s, new_s, delta_s_shift)` — `np.interp(new_s + delta_s_shift, prev_s, prev_d)` 로 이전 해를 새 grid에 reprojection
- `analytic_fallback(n_knots, apex)` — 상수 apex 초기해 (첫 cycle)

### 3.4 `velocity_profiler.py`

- `stack_master/config/<CAR>/veh_dyn_info/{ggv.csv, ax_max_machines.csv, b_ax_max_machines.csv}` 로드 (1회)
- `vel_planner.calc_vel_profile` FB pass 호출, `v_start = cur_v` IC, `v_max = scaled_vmax`
- 출력: `v(s)` array, 이후 `tph.calc_ax_profile`로 ax 계산해 `Wpnt.ax_mps2`에 담음

### 3.5 `abort_checker.py`

| Reason | 조건 |
|---|---|
| SAFETY | opponent 관측 d가 GP 평균에서 `k·σ` (기본 k=3) 벗어남 |
| PERFORMANCE | `t_ot = ∫ ds/v_ot` > `t_trail = ∫ ds/v_opp` + `T_margin` |

- Hysteresis: `consecutive_cycles` 연속 위반해야 실제 abort
- Abort 시 `cooldown_s` 동안 재진입 lockout, 빈 `OTWpntArray` publish → state_machine이 TRAILING 복귀

---

## 4. 입출력 데이터 흐름

```
/tracking/obstacles ─┐
/opponent_prediction ─┤
/car_state/odom ──────┤
/car_state/odom_frenet┤
/global_waypoints_* ──┼──→  [OvertakingIYNode]  ──→  /planner/rolling/otwpnts
/opponent_trajectory ─┤                         └──→  /planner/rolling/{markers,debug,diag,merger}
/ot_section_check ────┤
/behavior_strategy ───┘
```

상위 `sqp_planner` 및 `opponent_prediction/gp_traj_predictor`의 출력을 그대로 소비한다.

---

## 5. 주요 ROS 파라미터 (`config/overtaking_iy.yaml`)

| Param | Default | 역할 |
|---|---|---|
| `rate_hz` | 20.0 | 메인 루프 주기 |
| `racecar_version` | SRX1 | GGV CSV 경로 |
| `lookahead` | 15.0 | 고려할 장애물까지 거리 (m) |
| `width_car` | 0.30 | 차량 폭 |
| `evasion_dist` | 0.45 | 추가 회피 여유 (m) — clearance 및 dynamic extension 모두에 사용 |
| `spline_bound_mindist` | 0.10 | bound 내부 여유 |
| `avoidance_resolution` | 20 | SQP knot 수 |
| `back_to_raceline_before` | 5.0 | RoC 시작점 여유 (m) |
| `back_to_raceline_after` | 7.0 | 장애물 통과 후 raceline 복귀 여유 (m) |
| `obs_traj_tresh` | 1.5 | `|d_center|` 컷 (`≥`면 raceline 멀리 있는 장애물로 간주, 무시) |
| `warm_start.shift_lookahead_s` | 0.15 | `dt_solve` floor (초) — ego 예측용 |
| `regularization.lambda_reg` | 0.2 | `‖d - d_init‖²` 가중 (warm-start 자기강화 억제) |
| `regularization.homotopy_lock` | true | 첫 cluster의 side를 한번 정하면 유지 |
| `weights.lambda_smooth` | 1500.0 | 2nd-diff smoothness |
| `weights.lambda_start_heading` | 1000.0 | `(d[1]-d[0])²` 시작 heading 안정화 |
| `weights.lambda_apex_bias` | 0.0 | 중앙선 회귀 bias (기본 끔) |
| `weights.lambda_side` | 50.0 | side 선호 (cluster ≥ 2면 0으로 자동 전환) |
| `weights.lambda_jerk` | 30.0 | 3rd-diff (curvature rate) |
| `abort.performance_margin_s` | 1.00 | `T_margin` (debug로 완화 상태) |
| `abort.consecutive_cycles` | 3 | abort 발동 streak |
| `abort.cooldown_s` | 2.0 | 재진입 lockout |
| `abort.safety_sigma_multiplier` | 3.0 | GP 공분산 밖 허용 σ |

---

## 6. Launch

`launch/overtaking_iy.launch`:

```xml
<launch>
  <node pkg="overtaking_iy" type="overtaking_iy_node.py"
        name="planner_rolling" output="screen">
    <rosparam command="load"
              file="$(find overtaking_iy)/config/overtaking_iy.yaml"/>
  </node>
</launch>
```

`3d_headtohead.launch`와 병행 실행 (remap 없음). 혼자 띄워서는 의미 없음 — 상위 `sqp_planner`, `opponent_prediction`, `car_state`, `state_machine` 등이 먼저 떠있어야 함.

**빌드**

```bash
docker exec icra2026 bash -c "source /opt/ros/noetic/setup.bash && cd /home/unicorn/catkin_ws && catkin build overtaking_iy"
```

**실행**

```bash
roslaunch overtaking_iy overtaking_iy.launch
```

---

## 7. state_machine 통합 (현재 미연결)

state_machine [`3d_state_machine_node.py:382`](../state_machine/src/3d_state_machine_node.py#L382)이 `/planner/avoidance/otwpnts` 한 토픽만 구독. 이 패키지는 `/planner/rolling/otwpnts`로 publish하므로 현재 state_machine이 이 경로를 **선택하지 않음** (dry-run). 연결 방식 옵션:

- **A. 단순 remap** — launch에 `<remap from="/planner/rolling/otwpnts" to="/planner/avoidance/otwpnts"/>` + 기존 `3d_sqp_avoidance_node` 중지. fallback 없음.
- **B. mux 노드 (권장)** — 새 노드가 rolling/sqp 둘 다 구독해, rolling이 valid면 rolling을, 아니면 sqp를 `/planner/avoidance/otwpnts`로 forward. 안전망 유지.

Validity 판단 기준 예시:
- `len(msg.wpnts) > 0` (SQP failure 시 현재 `_publish_empty_otwpnts`로 빈 배열이 나감)
- `(now - last_rolling_valid_t) < HOLD_TIMEOUT` (stale 금지)

---

## 8. 의존성

**ROS packages**: `rospy, std_msgs, nav_msgs, visualization_msgs, f110_msgs, frenet_conversion, vel_planner`
**Python**: `numpy, casadi, ccma, trajectory_planning_helpers`

재사용 자산 (수정 없이 의존만):
- `f110_msgs/{Wpnt, WpntArray, Obstacle, ObstacleArray, OTWpntArray, OpponentTrajectory, BehaviorStrategy}`
- `frenet_converter.frenet_converter.FrenetConverter`
- `vel_planner.vel_planner.calc_vel_profile`
- `stack_master/config/<CAR>/veh_dyn_info/*.csv`

---

## 9. 기존 `sqp_planner`와의 차이 요약

| 항목 | `sqp_planner/sqp_avoidance_node.py` | `overtaking_iy_node.py` |
|---|---|---|
| Solve 타이밍 | OT 진입 시 1회 | 매 cycle (20Hz) |
| Solver | scipy SLSQP | CasADi + IPOPT (warm-start) |
| Warm-start | 없음 | 이전 primal + dual + shifted d_init |
| Regularization | 없음 | `λ_reg‖d - d_init‖²` |
| Multi-opponent | single aggregate | per-cluster envelope + 개별 side |
| Abort | 없음 | safety + performance hysteresis |
| 속도 프로파일 | raceline 그대로 | GG-FB pass (IC=v_ego) |
| Output topic | `/planner/avoidance/otwpnts` | `/planner/rolling/otwpnts` |

---

## 10. 주요 로그 태그

- `[OvertakingIY]` — 초기화, pred/obs 통계, warm/empty publish, SQP 성공/실패, abort, path start gap 진단
- 주요 진단 로그 예시:
  ```
  [OvertakingIY] path start gap: evasion_s[0]-cur_s=0.310m (start_av-cur_s=0.222 dt_solve=0.074s v=3.06)
  [OvertakingIY] SQP failed (Infeasible_Problem_Detected, iters=13). skipping cycle.
  [OvertakingIY] ABORT=performance t_ot=4.33 t_trail=1.42
  ```
