# Overtake Pipeline Redesign — Evidence-Based Plan (v3)

작성일: 2026-04-27 → 2026-04-28 다각도 검증 + 사용자 결정 반영
대상 bag: `2026-04-27-13-09-22.bag` (26.4s, 정+동 multi-obs, 충돌 80건), `2026-04-27-13-12-51.bag` (16s, 정 single-obs, 충돌 167건)

다음 세션 진입: §0 → §1 → §3 → §4 (작업 단계) 순. 10분.

---

## §0. Big Premise + 결정된 사항

### 사용자 큰 전제 (변경 금지)
> **MPC 가 전략을 짠다.** 절대 사고 안 나고, 상대 prediction / 맵 / 자차 / 상호 관계 모두 고려, 부드럽고 효율적으로 추월 + GB 복귀. 추월 안 되면 **능동적 trailing 도 1급 출력**. 슬랙 키우기 / 임계값 어거지 X. Kinematic MPC 의 장점 ("빠르고 안 죽는다") 회복. **구조적으로 robust** — ad-hoc workaround (예: 점프 detect 특수 처리) 금지.

### 검증 원칙
가설 단일 신호 결정 X. **bag 통계 + 코드 verification + 다른 토픽 + tracking truth** 의 다각도 합치 후에만 결론.

### 사용자 결정 사항 (확정)

| Q | 결정 | 의미 |
|---|---|---|
| Q1 | Stage 1 / Stage 2 분리 | Stage 1 = 안전+안정, Stage 2 = plan-aware MPC |
| Q2 | Plan picker 가 MPC 노드 안 (해석 A) + Lightweight + top-2 NLP | 솔버 1.5~2x, 마감 부담 적음 |
| Q3 | HOLD_LAST 무한 연장 폐기, 짧은 cap | 구조적으로 robust, ad-hoc 처리 X |
| Q4 | TRAIL = ego_n 유지 + 차 뒤 + 일정 거리/속도 단일 행동 | sub-mode (BRAKE/WAIT) 분리 X. 전략은 plan_picker 의 매 tick 평가에서 |
| Q5 | 현재 bag (1+2) 으로 시작 | 추가 시나리오는 진행 중 발견 시 |
| Q6 | predictor multi-ID 정상 (tracking 매칭 후 사용) | Stage 0 에서 변환 코드 트레이스 |
| Q7 | Solver tolerance = 1cm (slack cap 과 일치) | constr_viol_tol = 1e-2, IPOPT 가 정상 slack 활용 |
| Q8 | 점프 detect 같은 특수 처리 X | 구조가 자체로 처리 |

---

## §1. Bag + 코드 다각도 검증된 사실

### 1.1 폐기된 가설 (이전 v1/v2 의 오류)

| 가설 | 검증 결과 |
|---|---|
| Tier 3 raceline_slice 가 "장애물 통과 궤적" 원인 | **폐기** — 두 bag 모두 tier 3 발동 0건 |
| 외부 publisher race | **폐기** — default HJ launch MPC 단독 publisher |
| is_static 노이즈 flip | **폐기** — tracking truth 일관성 검증 |
| Mode dwell hysteresis 버그 | **폐기** — 코드 정상 |
| Solver state 가 직접 kappa=1.6M | **수정** — solver state OK, lift→painter→guard 후처리에서 spike |

### 1.2 검증된 critical 사실

**충돌 247건 발생** (bag1 80, bag2 167) — 사용자 룰 1 직접 위반.

**51-tick HOLD_LAST chain (bag1 t=521.39~525.01, 3.62s)** 의 진짜 root cause:
1. **t=519.83 충돌**: ego (s≈3.7, n=+0.06) 가 정+동 squeeze (id=1 동적 s=4 n=+0.10, id=100 정적 s=10 n=-0.16) 진입 중. side='trail' decider, 그러나 trajectory n_max=+0.55 (raceline 보다 +0.5m left 쪽) — **TRAIL 인데 lateral 펼쳐짐** (잘못된 구현)
2. **충돌 후 ego stuck**: t=521.39 부터 v≈0 으로 51 ticks 갇힘 (sim 의 collision response)
3. **NLP fail 51 ticks**: 모두 IPOPT_status=Infeasible_Problem_Detected, **worst='corridor_upper', worst_val = 0.0007~0.0025m (0.7~2.5mm)**
4. IPOPT default `constr_viol_tol = 1e-4` (0.1mm) → 우리 위배 7~25× 초과 → infeas 보고. **Slack cap 1cm 와 IPOPT tolerance 0.1mm 의 100배 차이가 직접 원인**
5. obs_in_horizon=True → HOLD_LAST 무한 연장 (C 조건) → 51 ticks stale traj
6. t=524.86 sim reset (ego_s 9.4→74.94 점프), **HOLD_LAST 의 cached traj T1 의 s 좌표 frozen** → ego 65m 떨어진 traj 추종 불가
7. t=525.03 NLP 회복, 새 traj T2 publish → **inter-msg jump 1.683m**

### 1.3 충돌 자체의 root cause (t=519.83)

bag1 t=518.5~520.83 의 60+ ticks 동안 **거의 모든 tick 에서 trajectory n range 가 obstacle n±w 와 overlap**.
- side='trail' (decider 결정) but trajectory n_max=+0.55 (LEFT 로 펼쳐짐)
- **현 TRAIL 구현 = bL=bR=0 + vmax cap** (사용자 정정 "lateral 정렬 X, ego_n 유지" 와 정확히 반대)
- decider.reason='no_side_fits' 비율 42% → SIDE_TRAIL 강제 → solver 가 단순 cost minimize 로 풀음 → trajectory 가 obstacle 위치 그대로 통과
- **즉 "TRAIL 이 lateral 중립" 자체가 충돌의 직접 원인**. 사용자 큰 전제 ("능동적 trailing 1급 출력") 의 가장 직접적 위반.

### 1.4 코드 verification 결과 (10가설 중 핵심)

- **H1** Pass2 vmax = v_obs · 0.95 — TRUE ([frenet_kin_solver.py:816-826])
- **H3** HOLD_LAST 의 s-shift 부재 + 무한 연장 — TRUE ([mpc_planner_state_node.py:2835-2837], [3172-3187])
- **H5** Hard side commit corridor inversion — TRUE ([frenet_kin_solver.py:750-792])
- **H8** Pass2 fires on RuntimeError 도 — TRUE
- **H9** ego_n 은 reach hint 만, commit tracker X — TRUE
- **H10** painter+guard 후 검증 없음 — TRUE
- **IPOPT solver options**: `constr_viol_tol`, `acceptable_constr_viol_tol`, `tol` 명시 설정 없음. IPOPT default (constr_viol_tol=1e-4) 사용. Slack cap (1cm) 과 100× 차이.

---

## §2. Causal Chain — 검증된 root cause 7개

| ID | Root cause | 코드/bag 근거 |
|---|---|---|
| **CC-1** | TRAIL 의 lateral 중립 → trajectory 가 obstacle 위치 통과 → 충돌 | bag1 t=518.5~520 의 60+ tick overlap, frenet_kin_solver.py:714-719 (bL=bR=0) |
| **CC-2** | IPOPT tolerance ↔ slack cap 100× mismatch → 정상 slack 활용 못 함 | bag worst_val 0.7~2.5mm vs default 0.1mm tolerance |
| **CC-3** | HOLD_LAST 무한 연장 (obs_in_horizon=True 시) | mpc_planner_state_node.py:2835-2837 의 OR 조건 |
| **CC-4** | publish 전 kappa/ds/finite/jump validation 부재 | painter+guard 후 검증 없음. inter-msg jump 1.7m 직접 원인 |
| **CC-5** | Hard side commit clamp multi-obs corridor inversion | frenet_kin_solver.py:750-792 |
| **CC-6** | Decider/Solver sequential 단절 (1-tick 결정 × 2 layer) | side ↔ ego_n 모순 48 ticks (bag1) |
| **CC-7** | Multi-obstacle 어휘 부족 (LEFT/RIGHT/TRAIL/CLEAR 만) | decider.reason='no_side_fits' 42% (bag1). BETWEEN/squeeze 표현 없음 |

### Root cause 우선순위

**P0 (안전, Stage 1 즉시)**:
- CC-1 (TRAIL 능동화) — 충돌 직접 원인
- CC-2 (IPOPT tolerance 1cm align) — 정상 slack 활용
- CC-3 (HOLD_LAST 무한 연장 폐기, 짧은 cap)
- CC-4 (publish-side validation chain)
- CC-5 (hard clamp 폐기, soft side bias 만)

**P1 (구조, Stage 2)**:
- CC-6 (Plan-aware MPC, sequential plan)
- CC-7 (Plan library 어휘 확장)

---

## §3. 제안 구조 — Plan-Aware MPC

### 3.1 4-Layer Architecture

```
[L0 Sensing] /tracking/obstacles + /opponent_prediction/obstacles
  ↓ (predictor multi-ID → tracking 매칭. Stage 0 에서 코드 트레이스)
[L1 Obstacle Stream] obs_arr (n_obs_max=2, id-stable)
  ↓
[L2 Plan Library + MPC]  ← MPC 가 전략 짠다 (해석 A: MPC 노드 안)
  - Plan 후보 5종 (§3.2)
  - Plan_picker: lightweight scoring → top-2 NLP solve → cost 비교 → best
  - Hard side commit clamp 폐기 (CC-5)
  - Solver tolerance 1cm align (CC-2)
  ↓
[L3 Output Smoothing + Validation]  ← CC-4 해결
  - lift → densify → painter → continuity_guard → validation
  - kappa<5, ds>0.05, finite, inter-msg jump<0.30m
  - 위배 시 다음 plan 의 traj fallback. 다 실패하면 짧은 emergency stop
  ↓
[L4 State Machine] ot_line 5종 + dwell hysteresis → OVERTAKE/TRAILING/SAFE
```

### 3.2 Plan Library — 5종 (사용자 정정 반영)

| Plan | 의도 | target_n | target_v | 발동 조건 |
|---|---|---|---|---|
| `LEFT_PASS` | 왼쪽 추월 | k=0 ego_n → k=N (obs.n + ego_half + margin) | full v_max | left corridor d_free > margin |
| `RIGHT_PASS` | 오른쪽 추월 | 대칭 | full | right 대칭 |
| `BETWEEN_PASS` | 두 obs 사이 통과 | (obs1.n + obs2.n) / 2 | full | 두 obs 의 lateral gap > 2·(ego_half + margin) |
| **`TRAIL`** | **차 뒤 + 일정 거리/속도 + 빈틈 노림** | **ego_n 유지 (lateral 정렬 X)** | v_obs - margin (smooth ramp) | 다른 plan 모두 risky/infeasible OR closure dv < threshold |
| `RACELINE` | GB 복귀 | raceline n* | full | obs_in_horizon=False |

**TRAIL 의 단일성** (사용자 정정):
- BRAKE / WAIT_OPPORTUNITY / OBSERVE 같은 sub-mode 분리 **X**
- TRAIL 자체는 단순 행동: ego_n 유지 + obs 뒤 ds 거리 + 속도 맞춤
- "전략 (브레이크 / 후퇴 / 빈틈 노림)" 은 **plan_picker 의 매 tick 다른 plan 평가**에서 자연스럽게 발생
- "무리한 추월 → 차 뒤로 빠짐 → 다음 전략" = LEFT_PASS → TRAIL → RIGHT_PASS 의 자연스러운 plan_best 흐름

### 3.3 Plan picker — Lightweight scoring + selective re-solve

매 tick:
1. **Heuristic scoring (NLP 안 풂)** — 5 plan 후보 빠른 score
   ```
   plan_cost = -α · race_progress_gain      # 추월 가치
              + β · collision_risk           # 안전
              + γ · maneuver_quality         # 부드러움 / curvature
              + δ · long_term_value          # 다음 추월 기회 (TRAIL 의 strategic value)
              + ε · commit_sticky            # 진행 중 plan 유지
   ```
2. **Top-2 plan NLP solve** (sticky 5% bonus 로 current plan 안정화)
3. **Cost 비교 → best** (NLP 의 cost 기준)
4. **Dwell hysteresis 5 tick** (plan 변경 빈도 제한)

**Cost 설계의 핵심**:
- `α` 가 충분히 커야 → "trailing 만 하면 목적 실패" 방지 (사용자 우려). race_progress 무시 안 됨.
- `δ` (long_term_value) 가 TRAIL 의 strategic 의미 살림 — "다음 corner 에서 추월 기회 있으면 TRAIL 우호적"
- 단순 risk 최소 X — 추월 가능하면 추월 우선.

### 3.4 다른 시스템과의 비교

거의 모든 실용 시스템이 2-layer 구조 (Apollo: Decision + Trajectory; Autoware: Behavior + Motion; Alex Liniger MPCC: lane 결정 외부 + MPCC 내부). 우리도 이미 2-layer (SideDecider + MPC) 인데 **decider 약함 + sequential 단절**. 본 plan = 그 2-layer 의 **더 탄탄한 버전** (해석 A: MPC 노드 안).

---

## §4. 작업 단계

### Stage 0 — 진단 보강 (30분~1시간)

- 사용자 호소 5대 증상 ↔ bag 시점 매핑 (collision_marker 시점 기준)
- Predictor multi-ID → MPC obs_arr 변환 코드 트레이스 (`mpc_planner_state_node.py:_build_obstacle_array_frenet` line 1014~1198)
- last_good_traj 의 invalidation 모든 경로 (mode flip 시 reset 여부 등)

### Stage 1 — 안전 + Numerical 안정 (1.5~2일)

**목표**: 충돌 0 / kappa<5 / pass2 < 5% / HOLD_LAST < 0.5s. P0 모두 해결. **사용자 호소 5중 4 직접 해결**.

| ID | 작업 | 변경 파일 | 검증 (bag replay) |
|---|---|---|---|
| **S1-1** | **IPOPT solver options 명시** — `constr_viol_tol = 1e-2`, `acceptable_constr_viol_tol = 1e-2`, Solved_To_Acceptable_Level 을 ok 처리 | `frenet_kin_solver.py:541-553`, `_solve_single_pass` 의 status 체크 | bag1 의 sub-mm corridor_upper fail 0건 |
| **S1-2** | **Pass2 TRAIL 의 능동화** — bL/bR=0 폐기, target_n=ego_n 유지 + smooth vmax ramp + obs ds 거리 유지 | `frenet_kin_solver.py:714-719`, `:816-826`, 새 yaml `q_n_keep_ego_trail`, `q_v_target_trail` | bag1 t=518.5~520 의 trajectory ↔ obstacle overlap 0건 |
| **S1-3** | **Hard side commit clamp 폐기** — `_apply_side_commit` 제거, soft side_bias hinge 만 (강도 boost 가능) | `frenet_kin_solver.py:750-792` 삭제, 호출부 :945 | bag1 multi-obs corridor inversion 0건. side ↔ ego_n 모순 < 5 |
| **S1-4** | **Publish-side validation chain** — lift→painter→guard 후 kappa<5, ds>0.05, finite, inter-msg jump<0.30m. 위배 시 다음 후보 plan / 짧은 emergency. | `mpc_planner_state_node.py:3599-` 새 함수 + fallback 핸들링 | bag2 의 1.6M kappa 케이스 0건 publish |
| **S1-5** | **HOLD_LAST 무한 연장 폐기** — `obs_in_horizon_strict` 로 cap 우회 폐기. 모든 케이스 fail_streak ≤ N (default 3) cap. cap 초과 시 짧은 emergency stop. **점프 detect 같은 ad-hoc 처리 없이** 구조 자체로 robust. | `mpc_planner_state_node.py:2835-2837` | bag1 t=521.39 의 51 tick HOLD_LAST → 3 tick + emergency |
| **S1-6** | **First wpnt = ego 위치 보장** — lift_frenet_to_xy 의 0번 wpnt 가 ego (x,y) 정확히 일치 | `mpc_planner_state_node.py:2432-2461` | inter-msg jump (ego 미동시) < 0.05m |
| **S1-7** | **Cost balance — 단순 weight 조정 (Stage 2 plan-aware 도입 전 가벼운 균형)** — w_obs 260→100, σ_n 0.35→0.45 | `state_overtake.yaml` | cost.obs / sum_other < 5× 이 90% ticks 만족 |
| **S1-8** | **Lap-wrap obs EMA reset** — ego_s 점프 (>50m) 감지 시 EMA reset. 단순 1줄. ad-hoc 아닌 numerical guard. | `mpc_planner_state_node.py` EMA update | 1줄 fix |

**작업 순서**: S1-5 → S1-1 → S1-3 → S1-2 → S1-4 → S1-6 → S1-7 → S1-8 (구조 robust 우선, numerical guard 다음, output validation, 마지막 cost tuning)

### Stage 2 — Plan-Aware MPC (2~3일)

**목표**: P1 (CC-6, CC-7) 해결. "MPC 가 전략 짠다" 의 실현. SM OVERTAKE 진입 가능.

| ID | 작업 | 변경 파일 |
|---|---|---|
| **S2-1** | PlanLibrary 모듈 — 5종 plan 의 cost weight overlay + target_n profile + vmax profile | 신규 `plan_library.py` |
| **S2-2** | Plan_scorer — heuristic scoring (race_progress + collision_risk + maneuver_quality + long_term_value + sticky) | 신규 `plan_scorer.py` |
| **S2-3** | Selective re-solve — top-2 plan NLP solve + cost 비교 | `mpc_planner_state_node.py` _plan_loop |
| **S2-4** | OTWpntArray.ot_line 5종 + SM dwell hysteresis | `mpc_planner_state_node.py`, `state_machine/src/mpc/3d_mpc_state_machine_node.py:441` |
| **S2-5** | SideDecider 폐기 또는 plan_scorer inner helper | `side_decider.py` |
| **S2-6** | (옵션) Predictive feasibility window k=0..k_pass | `plan_scorer.py` |

### 검증 종합

- bag1 (정+동 multi-obs): 충돌 0건, side flip < 5, pass2 < 5%, kappa < 5, HOLD_LAST 총합 < 0.5s, SM OVERTAKE 진입 발생
- bag2 (정 single-obs): 충돌 0건, pass2 < 5%, kappa < 5
- 30분 연속 sim: emergency stop < 5회

---

## §5. 변경 예정 파일

| 파일 | Stage 0 | Stage 1 | Stage 2 |
|---|---|---|---|
| `mpc_planner_state_node.py` | predictor 변환 트레이스 | S1-4, S1-5, S1-6, S1-8 | S2-3, S2-4 |
| `frenet_kin_solver.py` | — | S1-1, S1-2, S1-3 | S2-1 (plan-별 weight overlay) |
| `side_decider.py` | — | (S1-3 으로 의존 제거) | S2-5 |
| `plan_library.py` | — | — | S2-1 신규 |
| `plan_scorer.py` | — | — | S2-2 신규 |
| `state_overtake.yaml` | — | S1-7 + S1-1 thresholds | S2-1 plan-별 weight |
| `state_machine/src/mpc/3d_mpc_state_machine_node.py` | — | — | S2-4 |
| `state_machine/src/mpc/state_transitions_mpc.py` | — | — | S2-4 |

backup 규칙: `*_backup_20260427.{py,yaml}` (CLAUDE.md).

---

## §B. 운영 환경 + 복구 루틴 (2026-04-28 세션)

### 살아있는 시스템 상태 (사용자가 켜둠)
- 호스트 Gazebo + `3d_base_system.launch` (localization + frenet + sector + odom relay)
- `roslaunch stack_master 3d_mpc_headtohead.launch dynamic_avoidance_mode:=None` — controller + state_machine (`/mpc_state_machine`) + planners (sampling/sqp 비활성)
- **Obstacle**: **동적 1개만 띄워둠** (lap 따라잡는 차). 정적은 나중에. → **Stage 1 검증 = 동적 single overtake/trail 시나리오**
- **MPC 노드 (`/mpc_auto`) 는 dead** — 내가 별도 launch
- ROS env (docker → live nodes):
  ```
  -e ROS_MASTER_URI=http://192.168.70.2:11311 -e ROS_HOSTNAME=nuc3 -e ROS_IP=192.168.70.2
  ```

### MPC 노드 launch / kill
```bash
# Launch (foreground or background)
docker exec -e ROS_MASTER_URI=http://192.168.70.2:11311 -e ROS_HOSTNAME=nuc3 -e ROS_IP=192.168.70.2 icra2026 \
  bash -c 'source /opt/ros/noetic/setup.bash && source /home/nuc3/catkin_ws/devel/setup.bash && \
           roslaunch mpc_planner mpc_planner_state.launch state:=auto attach_to_statemachine:=true'

# Kill
docker exec icra2026 bash -c 'pkill -f mpc_planner_state_node'
```

### Ego 충돌/stuck 시 s=0 재스폰 (사용자 지시 2026-04-28)

s=0 의 spawn 좌표는 `stack_master/maps/gazebo_wall_2/global_waypoints.json` 의 `global_traj_wpnts_iqp[0]`:
- **x = -7.117928, y = -0.630808**
- **psi_rad = -1.71862764** → quaternion (z = -0.757395, w = 0.652957)

**model_name = "unicorn"** (사용자 PoseTeleport 코드 기준)

#### 방법 A: PoseTeleport 노드 + /initialpose 발행 (권장)
사용자 제공 `_tmp_pose_teleport.py` 노드 띄움 (z=0.7 고정):
```bash
docker exec -e ROS_MASTER_URI=http://192.168.70.2:11311 -e ROS_HOSTNAME=nuc3 -e ROS_IP=192.168.70.2 icra2026 \
  bash -c 'source /opt/ros/noetic/setup.bash && python3 /home/nuc3/catkin_ws/src/race_stack/HJ_docs/debug/0428_debug/scripts/_tmp_pose_teleport.py' &

# 텔레포트 (RViz "2D Pose Estimate" 또는 rostopic pub)
docker exec -e ROS_MASTER_URI=http://192.168.70.2:11311 -e ROS_HOSTNAME=nuc3 -e ROS_IP=192.168.70.2 icra2026 \
  bash -c 'source /opt/ros/noetic/setup.bash && \
    rostopic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
      "{header: {frame_id: map}, pose: {pose: {position: {x: -7.117928, y: -0.630808, z: 0.0}, \
        orientation: {x: 0.0, y: 0.0, z: -0.757395, w: 0.652957}}}}"'
```

#### 방법 B: 직접 set_model_state (PoseTeleport 안 띄울 때)
```bash
docker exec -e ROS_MASTER_URI=http://192.168.70.2:11311 -e ROS_HOSTNAME=nuc3 -e ROS_IP=192.168.70.2 icra2026 \
  bash -c 'source /opt/ros/noetic/setup.bash && \
           rosservice call /gazebo/set_model_state "{model_state: {model_name: \"unicorn\", \
                pose: {position: {x: -7.117928, y: -0.630808, z: 0.7}, \
                       orientation: {x: 0.0, y: 0.0, z: -0.757395, w: 0.652957}}, \
                twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, \
                reference_frame: \"world\"}}"'
```

### Bag 녹화 (라이브 분석용, 재생 X) — `HJ_docs/debug/0428_debug/bags/` 에 저장
```bash
# 시작
docker exec -e ROS_MASTER_URI=http://192.168.70.2:11311 -e ROS_HOSTNAME=nuc3 -e ROS_IP=192.168.70.2 icra2026 \
  bash -c 'cd /home/nuc3/catkin_ws/src/race_stack/HJ_docs/debug/0428_debug/bags && \
           rosbag record -O <S1-name>_$(date +%H%M%S).bag \
             /mpc_auto/debug/tick_json /planner/mpc/wpnts /car_state/odom_frenet \
             /tracking/obstacles /tracking/obstacles_truth /opponent_prediction/obstacles \
             /collision_marker /behavior_strategy &'

# 종료
docker exec icra2026 bash -c 'pkill -f rosbag'
```

### 복구 루틴 (커밋 금지 정책)

**Backup 파일만 사용** (사용자 지시 2026-04-28). `git restore` / `git stash` 등 git 명령은 사용 안 함 — working tree 의 다른 미커밋 변경 보존 + 단순한 cp 복구가 안전.

#### Stage 1 변경 예정 파일 + backup

| 파일 | Backup |
|---|---|
| `planner/mpc_planner/node/mpc_planner_state_node.py` | `mpc_planner_state_node_backup_20260428.py` |
| `planner/mpc_planner/src/frenet_kin_solver.py` | `frenet_kin_solver_backup_20260428.py` |
| `planner/mpc_planner/config/state_overtake.yaml` | `state_overtake_backup_20260428.yaml` |

#### Backup 생성 (Stage 1 시작 직전)
```bash
cd /home/nuc3/icra2026_ws/ICRA2026_HJ
cp planner/mpc_planner/node/mpc_planner_state_node.py planner/mpc_planner/node/mpc_planner_state_node_backup_20260428.py
cp planner/mpc_planner/src/frenet_kin_solver.py planner/mpc_planner/src/frenet_kin_solver_backup_20260428.py
cp planner/mpc_planner/config/state_overtake.yaml planner/mpc_planner/config/state_overtake_backup_20260428.yaml
```

#### 복구 (전체 또는 일부 파일)
```bash
cd /home/nuc3/icra2026_ws/ICRA2026_HJ
cp planner/mpc_planner/node/mpc_planner_state_node_backup_20260428.py planner/mpc_planner/node/mpc_planner_state_node.py
cp planner/mpc_planner/src/frenet_kin_solver_backup_20260428.py planner/mpc_planner/src/frenet_kin_solver.py
cp planner/mpc_planner/config/state_overtake_backup_20260428.yaml planner/mpc_planner/config/state_overtake.yaml
```

복구 후 빌드:
```bash
docker exec icra2026 bash -c 'source /opt/ros/noetic/setup.bash && source /home/nuc3/catkin_ws/devel/setup.bash && cd /home/nuc3/catkin_ws && catkin build mpc_planner'
```

### 커밋 정책
- **사용자 명시 요청 전까지 추가 커밋 금지** (사용자 지시).
- 본 세션의 모든 변경은 working tree 에만 존재. 검증 후 사용자가 직접 커밋 결정.

---

## §6. 다음 세션 진입

1. 본 문서 §0 + §1.2~1.4 + §3 + §4 만 읽기 (10분)
2. **Stage 0 진행** — predictor 변환 코드 트레이스 + last_good_traj invalidation 경로
3. **Stage 1 시작** — S1-5 (HOLD_LAST cap) → S1-1 (IPOPT tolerance) → S1-3 (hard clamp 폐기) → S1-2 (TRAIL 능동화) → ... 순
4. 각 항목 완료 시 bag replay + tick_json echo 검증, 수치 첨부 보고

---

## §A. 회고 (2026-04-27 / 28)

진행:
1. 1차 분석 (bag tick_json stat) → 가설 6개 → plan v1
2. 사용자 정정: "commit-and-hold 가 아니라 sequential plan 부재" + "is_static tracking 신뢰"
3. 2차 분석 (deep dive) — 가설 갱신
4. 사용자 정정: "여러 각도 다 접근, 진짜 문제 찾자"
5. 3차 분석 (코드 H1~H10 verification + 다른 토픽 + tracking truth) — **충돌 247건 발견**, **51-tick HOLD_LAST 의 진짜 원인 (ego stuck + IPOPT tolerance gap)**, **TRAIL 의 lateral 중립이 충돌 직접 원인** 발견
6. 사용자 정정: "MPC 가 전략 짠다" — 해석 A 확정. plan_picker 가 MPC 노드 안.
7. 사용자 정정: "trailing = 차 뒤 + 일정 거리/속도, 빈틈 노림. sub-mode 분리 X. plan_picker 에서 전략" — TRAIL 단일화
8. 사용자 정정: "구조적으로 robust. 점프 detect 같은 ad-hoc X" — HOLD_LAST 무한 연장 폐기
9. 사용자 결정: "tolerance 1cm OK" — `constr_viol_tol = 1e-2` 명시
10. plan v3 작성 (본 문서)

회고:
- 추측 → 패치 사이클 vs 다각도 검증 후 결론. 후자가 정답.
- bag stat 만으론 부족. tracking truth + 코드 verification + 다른 토픽 합치 필수.
- 사용자 정정 흡수 매번 plan 수정 — 정상. 이전 v1, v2 가 잘못된 가설 다수 (Tier3, mode dwell, predictor 발행 빈도 등) — 다각도 검증으로 정정.

---

작성: HJ 세션, 2026-04-28
