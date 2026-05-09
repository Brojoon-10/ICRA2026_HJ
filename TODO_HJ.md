# TODO — HJ (Overtaking Pipeline, 3D)

> 목적: 3D 트랙(gazebo_wall_2, sim) 위에서 **벽/장애물 무충돌 + 연속적 + feasibility** 를 만족하는 overtaking 파이프라인 제작.
> **HJ 담당 백엔드: Sampling+MPPI, MPCC**. Rolling-horizon(CasADi+IPOPT)은 IY 담당 — HJ 작업 시 `overtaking_iy.launch` 미기동.
> SQP(SLSQP)는 공용 베이스라인. state_machine 직통 연동은 후순위. 현재는 각 백엔드가 **별도 observation 토픽**으로 경로 valid함을 증명하는 단계.

---

## 0. 공통 기반 (완료 — 참조용)

- [x] Wpnt.msg에 `z_m`, `mu_rad` — 3D 웨이포인트 공통 스키마
- [x] Frenet C++ / Python **3D** 대응 (`get_frenet_3d`, `get_cartesian_3d`)
  - height filter + boundary raycast + 회전 검색 + fallback, trackbounds 자동 로드
- [x] Glob2Frenet/Frenet2Glob srv 4종 z 필드 추가, `frenet_conversion_server` 3D 지원
- [x] 3D 글로벌 라인 생성 파이프라인: Track3D + CubicSpline + mu_rad 계산 + IQP=SP
- [x] `3d_base_system.launch` (sim/real 분리, carstate_node 제거로 `/car_state/odom` 이중 발행 해결)
- [x] `3d_headtohead.launch` — controller + state_machine + planners 통합 진입점
- [x] Controller 3D (L1 xyz, future z spline, lateral error 3D frenet, 마커 z, AEB)
- [x] Gazebo static obstacle → planner 직통 연결 (`gazebo_static_obstacle_publisher.py`, Obstacle.msg `z_m` 추가)
- [x] `gazebo_wall_2` 맵으로 export/테스트 통과 (867pts 급, bounds_3d, ot_sectors 등 일체)
- [x] **trackbound marker 정확화 (2026-04-25)** — `d_left/d_right`는 centerline normal 축 위 거리인데 marker는 raceline tangent ± π/2 로 그려서 코너에서 sin(chi_opt) 만큼 어긋났음. JSON `centerline_ref` 에 `psi_center_rad` 배열 추가 (raceline wpnt 1:1, centerline x,y의 periodic CubicSpline 미분으로 계산), export/rebuild 양쪽 marker 코드가 이를 사용하도록 수정. rebuild 의 z=0 강제 버그 (centerline/IQP/SP/trackbound sphere) 도 `w["z_m"]` 로 같이 수정. 백업: `<file>_backup_20260425.<ext>`. 파일: `planner/3d_gb_optimizer/global_line/global_racing_line/export_global_waypoints.py`, `stack_master/scripts/rebuild_waypoints.sh`.

> 3D 포팅 자체의 남은 세부 버그/정리 항목은 `HJ_docs/3d_port_bug_catalog.md`, `HJ_docs/3d_dynamic_prediction_and_planner.md` 참조.

---

## 1. Overtaking 백엔드 현황 (2026-04-20 기준)

| 백엔드 | 담당 | 노드 | 출력 토픽 | 상태 |
|---|---|---|---|---|
| SQP (one-shot SLSQP) | 공용 | `planner/sqp_planner/src/3d_sqp_avoidance_node.py` | `/planner/avoidance/otwpnts` | 기본 활성, state_machine 직결 (베이스라인) |
| **Sampling + MPPI** | **HJ** | `planner/3d_sampling_based_planner/node/sampling_planner_state_node.py` | `~out/otwpnts`, `~out/wpnts` | 코드 완성, **연속성 튜닝 진행 중** |
| **MPCC → Frenet kinematic MPC** | **HJ** | `planner/mpc_planner/node/mpc_planner_state_node.py` | `/planner/avoidance/otwpnts_observation` 등 | **2026-04-20 재설계 완료 (코드+빌드), live-test 미완** |
| Rolling-horizon (CasADi+IPOPT, 20Hz) | IY | `planner/overtaking_iy/src/overtaking_iy_node.py` | `/planner/rolling/otwpnts` | HJ 작업 범위 밖 — `IY_docs/TODO_HJ.md` 참조 |

---

## 1.5 Strategic Overtake — 단계적 로드맵 (2026-05-01 갱신)

> 본질 비전: [`HJ_docs/strategic_overtake_vision_20260501.md`](HJ_docs/strategic_overtake_vision_20260501.md)
> 단기 안전 baseline: [`HJ_docs/dynamic_overtake_master_plan_20260429.md`](HJ_docs/dynamic_overtake_master_plan_20260429.md)
>
> **원칙**: 한 milestone = 1 세션 작업 단위. 한 phase 안에 milestone 2~4개. 각 milestone 은 **빌드 + 짧은 bag 검증** 까지 끝난 상태로 닫고 다음으로 넘어감. 한 세션에 phase 두 개 묶어 가는 거 금지 (어거지 없이).
>
> **Architecture 결정 (2026-05-01)**:
> - Strategy = **`planner/mpc_planner/src/strategy_planner.py` 모듈**. 별도 노드 X. MPC 노드 import 호출.
> - Strategy 와 MPC 분리 디버깅은 별도 디버그 토픽 (`/strategy/debug/plan_json` 등) 으로.
> - 다른 백엔드 필요 시 그때 갖다 쓰기 / 별도 패키지 분리.
>
> **Strategy 결정 정책**:
> - **Side commit (LEFT/RIGHT) 은 Strategy 가 macro 로 확정**. MPC 는 micro d(k) trajectory 만 풀이 (Design X).
> - Commit lookahead = 5s, hold = encounter window + 1s, abort 4 트리거 (margin / 예측 빗나감 / 새 obs / 추적 실패).
> - Sector 라벨 의존성 X (horizon-based / s-function). ot_sectors.yaml 의 추월 금지 sector 만 hard rule.
> - 명시적 sigmoid 시정수 강제 X. 큰 점프 안전망 (1-tick max delta) 만.

### 현재 스택의 본질 문제 (2026-05-01 시점)

| # | 문제 | 증상 | 해결 phase |
|---|------|------|----------|
| P-1 | GP timestep-별 정보 미활용 | `_merge_obs_sources` 가 prediction 의 N timestep 시퀀스를 버리고 vs/vd 만 추출 → constant 외삽. 코너 / 가속 변화 무시 | Phase 1 |
| P-2 | Trailing/Overtake 가 1-tick 반응 | "옆이 비었나?" 즉석 판단 → reactive 토글, PASS 안전성 보장 X. directive #4 위반 | Phase 2~3 |
| **P-3a** | **Painter 가 NLP `v[k]` 를 덮어씀 (architecture defect A)** | NLP 가 plan-aware 로 푼 속도 시퀀스 `v[k]` 가 `_post_process_speed` 에서 GB raceline `vx_lookup(s)` 로 통째 교체. plan 의도 (TRAIL/PASS 속도) 가 `trajectory.vx_mps` 까지 전달 안 됨 | Phase 4 (4.1~4.3) |
| **P-3b** | **Controller TRAILING PID 가 trajectory.vx_mps 무시 (architecture defect B)** | `Controller.py:537-557` 가 `local_wpnts_src==TRAILING` 일 때 자체 PID 로 `target_v = obs.vs - gap_err·kp` 계산. trajectory.vx_mps 미사용. wpnts_src=OVERTAKE 케이스에서는 PID 자체가 비활성 → obs 인지 사라짐 → catch-up 충돌 19/31 의 직접 원인 | Phase 4 (4.4) |
| P-4 | Trailing → Overtake 전이 시 가속 점프 | trailing PID off → ref_v 풀린 raceline 1-tick 안에 +2 m/s. 차량 거동 불안정. P-3a/b 가 풀려야 의미 있음 | Phase 7, 10 |
| P-5 | 누적 observation 미활용 | `/opponent_trajectory` 가 publish 되고 있으나 MPC 는 안 씀. sector-level lap 통계 / driver style 추정 없음 | Phase 8~9 |
| **P-6** | **TRAIL 이 baseline 만, 전략 없음** | 현재 TRAIL = "obs 뒤 따라가기 + ref_v cap". 사용자 비전 — TRAIL 은 다음 PASS window 를 위한 능동적 자세 잡기 (lateral preposition + dynamic closure rate + sector 인지). 즉 **단순 obs 추적이 아니라 strategic engagement 의 한 phase**. 본질 baseline (Phase 1 끝) 후 Phase 7~10 에서 도입. | Phase 7 (intent), Phase 9 (encounter forecaster), Phase 10 (commit 정밀화) |
| **P-7** | **NLP obs cost 가 corridor 위반 만듦 (Phase 1 분석에서 발견)** | Gaussian bubble (`J_obs = w_obs * exp(-xy_dist²/σ²)`) 가 ego 를 obs 반대 방향 lateral push. corridor 좁으면 wall 침범 → IPOPT `Infeasible_Problem_Detected` (mm 위반에도 strict fail). 사용자 비전 = TRAIL 은 lateral free + 종방향 ref_v cap (속도가 obs.vs 보다 낮으면 ego 가 따라잡지 못함 = 자연 충돌 회피). PASS 만 lateral repulsion 유지. | Phase 1 (TRAIL w_obs=0 적용 완료), Phase 4 (PASS 의 collision constraint 도입) |

> **P-3a, P-3b 가 Phase 4 의 핵심**. 두 wall 모두 무너뜨려야 NLP 가 푼 `v[k]` 가 controller 까지 살아남고, 그 위에서 비로소 vision §3 의 단계적 ramp / continuous engagement 가 emerging behavior 로 성립. 한쪽만 풀면 다른 쪽이 trajectory 속도를 지움.

### Phase 0 — Publish + SM 정리 [완료]

이전 세션 결과물. 현재 baseline 의 시작점.

- [x] `_validate_publish_wpnts` ds_min 0.05→0.02 + sparsify 후처리. publish blackout 제거 (commit b8580b4)
- [x] `_check_close_to_raceline_heading` heading 버그 fix
- [x] `NonObstacleTransition_GBMode` 의 `_check_on_spline` 게이트 제거. ego 멀면 무조건 RECOVERY (사용자 directive 반영)
- [x] `RecoveryTransition` 의 sustain-fail GB 폴백 제거. 종료 조건은 `close_tight K tick` 단일화

### Phase 1 — Layer A: ObstacleHorizon (P-1 해결)

**목표**: prediction 의 N=20 timestep 을 그대로 obs_arr 에 넣어 NLP 가 timestep 별 (s, d) 를 보고 collision avoidance 하게.

- [ ] **1.1** `_merge_obs_sources` 가 매칭된 prediction opponent 의 **N timestep 시퀀스 전체** 를 보관하도록 변경. 현재 vs/vd 만 뽑던 부분 폐기. (~80줄)
- [ ] **1.2** `_extrapolate_obs_traj` 의 vs/vd constant propagation 폐기, `obs_arr[n_used, k, 0/1]` 가 prediction sequence 를 직접 인덱싱. prediction 짧으면 tail 은 마지막 (s,d) 유지. (~60줄)
- [ ] **1.3** 검증: 30s bag (정적 + 동적 obs 1대), tick_json 에 `obs_horizon_source` 필드 추가. constant 외삽 vs sequence 결과 RViz 에서 비교. min_distance 수렴 패턴 / collision 빈도 차이 확인.
- [x] **1.4** [2026-05-02] tail (k≥M) 처리 = vs/vd extrapolate (마지막 점 hold 폐기). prediction horizon < NLP horizon 시 obs sequence 가 freeze 되던 버그 fix.
- [x] **1.5** [2026-05-02] obs_arr 빌드의 s 처리 = per-timestep ego_s 도메인 unwrap (통째 offset 폐기). lap 경계 wrap 처리.
- [x] **1.6** [2026-05-02] `_ObsWithHorizon` wrapper class 도입. ROS msg `__slots__` 우회, sidecar attribute 추가 가능.
- [x] **1.7** [2026-05-02] PLAN_TRAIL `w_obs=0`, `q_n_target=0`. obs Gaussian bubble 의 lateral push 가 corridor 위반 만들어 NLP infeasibility 유발 → TRAIL = lateral free + 종방향 ref_v cap (이미 있음). **사용자 비전 직접 반영 (P-6, P-7)**.
- [x] **1.8** [2026-05-02] 부수 fix — SM `get_recovery_wpts` None defense + GB raceline fallback. spawn 직후 `cur_recovery_wpnts.is_init=False` 케이스 처리.

**검증 통과 기준**: NLP solve_ms p99 변화 ±10% 이내, collision count 가 baseline 대비 같거나 감소, **`Infeasible_Problem_Detected` 빈도 baseline 회귀 (이전 분석 11건 → ≤2건 기대)**, TRAIL plan 시 SIDE DECISION 토글 빈도 baseline 회귀.

**검증 상태**: 코드 / 빌드 OK. 1.7 (PLAN_TRAIL) 까지 적용 후 사용자 새 bag 검증 대기.

### Phase 2 — Layer B: EncounterAnalyzer (단순) (P-2 1단계)

**목표**: 매 obstacle 에 대해 LEFT/RIGHT pass margin + ttc 계산. SM 단순 toggle 을 대체할 정량 지표.

- [ ] **2.1** `EncounterReport` 데이터 구조 + `_analyze_encounter(obs, ego, corridor)` 함수. LEFT/RIGHT pass margin = corridor 폭 − obs 폭 − safety buffer.
- [ ] **2.2** ttc / closure rate 계산 (`(obs.s - ego.s) / max(ego.vs - obs.vs, eps)`).
- [ ] **2.3** 디버그 publish: `~debug/encounter_json` (per-tick), tick_json 에도 요약 추가. RViz marker 로 pass margin / ttc 시각화.

**검증 통과 기준**: 30s bag 동안 encounter_json 매 tick publish, RViz 에서 pass margin / ttc 가 안정적 (oscillation 없음).

### Phase 3 — Layer C: StrategicPlanner (rule-based) (P-2 2단계)

**목표**: encounter report 받아 plan + commit_phase FSM 운용. 1-tick 반응 토글 제거.

- [ ] **3.1** plan = `{LEFT_PASS, RIGHT_PASS, TRAIL, WAIT}` 단순 rule (pass_margin > buffer AND ttc > min_ttc → PASS, else TRAIL).
- [ ] **3.2** commit_phase FSM (DETECT → APPROACH → WAIT → COMMIT → PASS → RECOVER → CLEAR). state 별 dwell 보장.
- [ ] **3.3** 한 번 COMMIT 진입 후 hysteresis: COMMIT → PASS 진입은 safety guard 통과 시만, 한 번 PASS 진입하면 dwell K_pass tick 보장.
- [ ] **3.4** publish: `~debug/plan_json` (plan, commit_phase, dwell, reason).

**검증 통과 기준**: plan transition 빈도 < 5/min, commit_phase 가 정상 FSM 흐름 (역행 없음).

### Phase 4 — Layer D: PlanAwareMPC (P-3a + P-3b 해결, 가장 큰 효과)

**목표**: NLP 가 푼 `v[k]` 가 painter / continuity guard / controller 어디서도 덮어씌워지지 않고 controller 의 PID 추적 ref 까지 살아남게. 즉 **MPC 속도가 시스템 끝까지 사용되도록**.

- 이 phase 가 Phase 7~10 의 strategic vision 전체의 전제. 4.1~4.4 (P-3a) 가 painter side, 4.5 (P-3b) 가 controller side. 두 wall 모두 무너져야 vision §3 (continuous intent) / §5 (단계적 ramp) 가 가능.

- [ ] **4.1 [P-3a]** `_post_process_speed` 가 NLP `v[k]` 를 **출발점으로 사용** — 현재처럼 GB raceline `vx_lookup(s)` 로 통째 교체하지 않음. painter 의 역할은 GB cap (절대 상한) + curvature cap + seam blend 만 적용. NLP 출력 보존이 default, 덮어쓰기는 명시 위반 시만.
- [ ] **4.2 [P-3a]** plan-aware ref_v cap — plan=TRAIL 일 때 NLP `v[k]` 위에 추가 cap = `obs.vs + safety_buffer` 적용. plan=PASS 일 때 cap 없음 (raceline_v 까지 허용).
- [ ] **4.3 [P-3a]** plan-aware a_max — TRAIL 시 brake 강도 허용 (a_dec 증가). PASS 시 a_acc 충분 보장. continuity guard 가 painter 결과를 ego_v±0.15 로 강제 clip 하던 부분도 plan-aware 화 (TRAIL 시 brake 허용).
- [ ] **4.4 [P-3a 검증]** painter 출력 검증 — tick_json 에 `nlp_v0`, `painter_v0`, `cap_source` 필드 추가. NLP `v[0]` 와 painter `v[0]` 가 plan=TRAIL 시 일관 (GB raceline 으로 점프 안 함).
- [ ] **4.5 [P-3b]** Controller TRAILING PID 제거 → `trajectory.vx_mps` 충실 추적. `Controller.py:537-557` trailing 분기 삭제. **반드시 4.1~4.4 통과 후 진입** — 단독 적용 시 trailing 자체 사라져 즉시 충돌. 검증 시 wpnts_src=OVERTAKE 케이스에서도 ego 가 trajectory.vx_mps 추적하는지 확인.
- [ ] **4.6** directional obstacle bubble — plan side 반대편 hard, plan side soft hinge. solver 수정 (~50줄). NLP 가 plan side 로 lateral 이동 시 cost 발산 안 하도록.
- [ ] **4.7** 통합 검증: 30분 bag 에서 catch-up 충돌 (TRAILING 인데 ego.v > obs.vs+1.0 인 충돌) **19 → 0**. 전체 collision count baseline (run01) 31 대비 **50% 이상 감소**. tick_json 의 `nlp_v0` 와 controller 가 추적한 실제 ego.v 사이 추적 오차 < 0.3 m/s.

**검증 통과 기준**: catch-up 충돌 0. NLP `v[k]` 가 controller 까지 살아남는 게 데이터로 확인 (tick_json 시계열 + ego.v 비교).

### Phase 5 — Safety buffer (P-1, P-2 보강)

**목표**: prediction uncertainty 를 명시적으로 lateral margin 에 반영. directive #6.

- [ ] **5.1** Phase 2 의 pass margin 계산에 `vd_var * horizon_time` 더해서 uncertainty inflation.
- [ ] **5.2** obs.n 의 rolling stddev (최근 N tick) 도 추가.
- [ ] **5.3** Phase 4 의 directional bubble 폭이 uncertainty 비례.

**검증 통과 기준**: high-uncertainty obstacle (rolling stddev > 0.1) 만나는 케이스에서 PASS commit 안 함, TRAIL 유지.

### Phase 6 — 통합 검증 (안전 baseline 완성)

**목표**: master plan 의 충돌 0 contract 확인.

- [ ] **6.1** 10분 bag × 5 회 (시나리오: 직선/코너/혼합).
- [ ] **6.2** `analyze_long_bag.py` + `coll_plan_breakdown.py` 결과 표 작성.
- [ ] **6.3** 100/50 시나리오 (가능 시) 또는 추정.

**검증 통과 기준**: 30분 bag × 5회 = 150분 충돌 0건. PASS 성공률 50% 이상.

---

> 여기까지가 **단기 안전 baseline (Phase 0~6)**. ICRA 마감 안 도달 목표.
> 아래는 **중기 strategic vision (Phase 7~11)**. 마감 후 / 여유 시 / strategic 기능 추가.

### Phase 7 — TRAILING sub-state + Continuous intent + strategy_planner.py 모듈 (P-4 1단계)

**목표**: `planner/mpc_planner/src/strategy_planner.py` 모듈 신설. TRAILING 안에 PASSIVE / INTENT sub-state 분할. intent scalar 도입.

- [ ] **7.1** `strategy_planner.py` 신설 — `StrategyPlanner` 클래스 + `StrategyPlan` dataclass + `decide(ego_state, obs_pred, opp_traj) → StrategyPlan` 인터페이스.
- [ ] **7.2** intent scalar `e ∈ [0,1]` 계산 — closure rate / pass margin / corridor_efficiency(s) 가중합. Phase 3 의 plan + commit_phase 위에 얹기.
- [ ] **7.3** TRAILING_PASSIVE (e<0.3) / TRAILING_INTENT (0.3≤e<0.7) sub-state 분할. 외부 mode 는 둘 다 TRAILING.
- [ ] **7.4** MPC 노드에서 `strategy.decide()` import 호출. `/strategy/debug/plan_json` publish.
- [ ] **7.5** painter ref_v 가 e 의 함수: PASSIVE → obs.vs+0.2, INTENT → obs.vs + 단계적 (1-tick max delta 0.5 m/s).
- [ ] **7.6** [디버그 마커 M1] **상대 예측 경로 색깔 heatmap (0.5s 단위 binning)** — `3d_opp_prediction.py` 의 marker color 필드를 `time_to_color_binned(t, bin_size=0.5)` 로 변경. 형태 / 위치 / 개수 변경 X. 0.5s 단위 색 띠 → RViz 에서 "이 구간은 몇 초 후" 즉시 인식. ~10줄 변경.

**검증 통과 기준**: intent 시계열 smooth, TRAIL→PASS 전이 시 ref_v step ≤ 0.5 m/s/tick. 충돌 0 유지.

### Phase 8 — Layer 1: Opponent Profile (horizon-based, P-5 1단계)

**목표**: `/opponent_trajectory` + GP prior 활용. sector 라벨 의존성 X.

- [ ] **8.1** `opp_profile.py` 모듈 — `/opponent_trajectory` 구독 hook + lap 누적 trajectory cache.
- [ ] **8.2** s 함수 산출: `opp_d_profile(s)`, `opp_v_profile(s)`, `opp_brake_points` (기존 GP prior 추출 + s 함수 wrapping).
- [ ] **8.3** `stationarity_score` 계산 — lap 간 opp_d_profile std 적분.
- [ ] **8.4** `/strategy/debug/opp_profile` publish — RViz 에서 s 위 d_pref 곡선 + stationarity 시각화.

**검증 통과 기준**: lap 1 후부터 opp_d_profile / stationarity_score 산출. rule-based 시뮬에서 stationarity < 0.2, reactive 시뮬에서 > 0.5 분리 확인.

### Phase 9 — Layer 2: Encounter Forecaster (P-5 2단계)

**목표**: 5s lookahead rollout + Bayesian (단기 GP + lap prior) blend. sector 라벨 의존성 X.

- [ ] **9.1** `encounter_forecaster.py` 모듈 — 5s lookahead rollout (dt=0.1s) 함수. ego raceline 가정 + obs trajectory 시퀀스 (단기 GP / lap prior blend).
- [ ] **9.2** Bayesian blend: t ≤ GP horizon 에서는 GP, 그 이후 prior. confidence = 1 − stationarity.
- [ ] **9.3** encounter_window list 출력 — `(t_start, t_end, side, pass_margin, ttc, confidence, obs_d_seq)`. obs_d_seq 가 commit window 안 obs lateral 시퀀스 (사용자 시나리오 in→out swap 처리용).
- [ ] **9.4** `/strategy/debug/encounter_windows` publish (선택, 부족함 느끼면 추가). 우선은 plan_json 시계열로 충분.

**검증 통과 기준**: encounter window list 가 lap 2+ 에서 prior 영향 받음. obs_d_seq 시퀀스가 RViz 에서 시각 확인 가능.

### Phase 10 — Layer 3 commit 정밀화 + Layer 4 확장 (P-4 2단계)

**목표**: side LOCK macro + obs(t) 기반 micro d(k). commit lookahead 5s + hold + abort 4 트리거. 명시적 sigmoid ramp 강제 X.

- [ ] **10.1** Layer 3 commit decision 5s lookahead — single-side same-side corridor 폭 5s 내내 충분 검사 (MVP). 사용자 원칙 "벽과 obs 사이 좁혀질 공간 절대 안 들어감" 직접 반영.
- [ ] **10.1b** Forward reachable feasibility check (옵션 A) — t=1.0 obs LEFT 벽 / t=1.5 obs RIGHT 벽 같이 obs 가 시간 따라 lateral 변동하는 케이스 대응. lateral 도달 범위 시퀀스 갱신 후 비어있지 않으면 commit OK. single-side fail 시 fallback 으로 활성화.
- [ ] **10.2** Commit hold (encounter_window 길이 + 1s) + abort 4 트리거 (margin / 예측 빗나감 / 새 obs / 추적 실패). TTC 는 closure_rate 기반 (정지 / 동적 obs 통합), 미래 closure (obs brake 시작 케이스) 도 검사.
- [ ] **10.3** Layer 4 directional bubble 이 obs(k) 시간 함수 받아 timestep 별 cost 다르게 적용. side LOCK 안에서 micro d(k) 자유 풀이.
- [ ] **10.4** ref_v 1-tick max delta 0.5 m/s 안전망. d_ref 1-tick max delta 0.1m 안전망. 의도된 점프 (emergency brake) 는 flag 로 우회.
- [ ] **10.5** D_ref(t) phase 별 정의 (WAIT 진입 시 0 → ±0.2, COMMIT 진입 시 → ±0.4). 명시적 시정수 강제 X.
- [ ] **10.6** [디버그 마커] **abort 발동 시 텍스트 마커 (M3)** — `strategy_planner.py` 안 `DebugMarkerPublisher.publish_abort(reason)`. lifetime 1s, 빨강 텍스트.

**검증 통과 기준**: 사용자 시나리오 케이스 스터디 §5 + 사용자 시나리오 (in→out swap) 둘 다 bag 에서 재현. abort 트리거 발동 시 안전 RECOVER. ref_v / d_ref step ≤ 안전망 안.

### Phase 11 — 검증 / 튜닝

- [ ] **11.1** vision 문서 §5 시나리오 (우측 뒤 trail → 코너 직전 PASS) 재현.
- [ ] **11.2** 사용자 시나리오 (상대 in→out swap, ego 가 반대 side commit 후 안전 통과) 재현.
- [ ] **11.3** 30분 bag × 5 회. commit_phase / intent / encounter window 시각 분석.
- [ ] **11.4** PASS 성공률 70% 이상, 충돌 0. abort 발동률 / 원인 분포 확인.

---

### 우선순위 정리

| 우선순위 | Phase | 이유 |
|---------|-------|------|
| **P0 (다음 세션 후보)** | Phase 1 | P-1 해결 + Phase 4 의 전제. 가장 작은 단위로 시작 가능 |
| **P0** | Phase 4 (4.1~4.4 painter side, P-3a) | NLP `v[k]` 보존. 단독으로도 효과 있음 |
| **P0** | Phase 4 (4.5 controller side, P-3b) | 4.1~4.4 통과 후. 단독 적용 금지 |
| **P0** | Phase 4 (4.6, 4.7) | directional bubble + 통합 검증. catch-up 충돌 19→0 |
| **P1** | Phase 2, 3 | strategic plan 의 뼈대. Phase 4 결과 위에 얹기 |
| **P2** | Phase 5, 6 | safety buffer + 통합 검증. baseline 완성 |
| **P3 (마감 후)** | Phase 7~11 | strategic vision. 안전 baseline 위에 얹기 |

> **Phase 4 의 milestone 순서가 매우 중요**:
> - 4.5 (controller PID 제거) 를 4.1~4.4 (painter NLP `v[k]` 보존) 보다 먼저 적용하면 → trailing 자체가 사라져 **즉시 충돌**.
> - 4.1~4.4 만 적용하고 4.5 안 하면 → NLP `v[k]` 가 painter 까지는 살아남지만 controller 가 자체 PID 로 또 무시 → vision 의 ramp 가 emerging 안 함.
> - 두 wall 모두 무너뜨려야 strategic vision 의 전제가 성립.

### 한 세션 단위 작업 추천

다음 세션에 들어갈 후보:

| 후보 | 범위 | 예상 코드량 | 효과 |
|------|------|-----------|------|
| **A** | Phase 1.1 + 1.2 + 1.3 | ~150줄 + 검증 bag | obs horizon 정확도 ↑. 다른 phase 의 전제 |
| **B** | Phase 4.1 + 4.2 + 4.3 + 4.4 | ~100줄 + tick_json 필드 + 검증 bag | painter 가 NLP `v[k]` 보존. tick_json 으로 검증 |
| **C** | Phase 4.5 + 4.6 + 4.7 | ~50줄 (controller) + ~50줄 (bubble) + 30분 bag | catch-up 충돌 19→0. **B 통과 후만 진입** |

**권장 순서**: A (Phase 1) → B (Phase 4 painter side) → C (Phase 4 controller side + 통합 검증).
한 세션에 A 와 B 묶지 않음. 각 후보 끝에 **빌드 + 검증 bag** 까지 닫고 commit, 다음 세션에 진입.

**금기 사항**:
- 한 세션에 painter 수정 (B) + controller 수정 (C) 동시 진입 (위험도 높음, 디버깅 분리 어려움).
- Phase 1 안 한 상태에서 Phase 4 진입 — NLP 가 부정확한 obs horizon 으로 풀린 `v[k]` 를 보존해도 의미 적음. 가능은 하지만 권장 X.

---

## 2. 단기 TODO — HJ 집중 백엔드 (P0)

### 2.1 Sampling + MPPI (state-aware)

> ⚠️ **2026-04-20 비상 스냅샷 (워크스페이스 유실 대비 커밋)** — 꼬불거림 진단 완료.
> 상세: `HJ_docs/sampling_planner_wobble_diagnosis.md`. 다음 세션 첫 1시간 안에 L1/L2/L3 + YAML-overwrite 수정 적용.

- [x] **tick_json 디버그 토픽 추가** — `/sampling_overtake/debug/tick_json` (std_msgs/String, JSON). ego / candidates kill 분포 / cost 분포 / best / mode / opp / mppi / timing / params 포함.
- [x] **standalone node 이름 충돌 방지** — launch default `instance_name` = `sampling_$(state)` → `/sampling_overtake`. multi.launch 영향 없음.
- [x] **check_stats** 를 upstream `calc_trajectory` 에 추가 (curvature/path/friction 각 단계 kill 수치화).
- [x] **꼬불거림 근본 원인 진단 완료** — 3중 결합 (공간/시간/초기 tangent) + YAML-overwrite 버그. 수정 계획은 wobble 진단 문서 참조.
- [ ] **L1 — 벽 경계 영향 차단**: `endpoint_chi_raceline_only: true` (YAML + cfg default). `sampling_based_planner.py:999-1004` 의 boundary-interp 경로 비활성.
- [x] **L2 — 시간축 smoothing**: `filter_alpha: 0.6` 1차 + **n_end rate constraint** (2026-04-21). `|n_end_t+1 − n_end_t| ≤ rate_cap=0.12m`, empty rate-ok pool 시 snapshot restore로 hold. 결과: p50=0.003 / p95=0.075 / max=0.120 / 0% violation (30s, 665 ticks). 구현: planner-side prev-anchored narrow linspace + node-side hard constraint + prev-traj snapshot. 파일: `sampling_based_planner.py:986-1001`, `sampling_planner_state_node.py:1760-1776, 1853-1883`.
- [x] **L2b — hard rate constraint → soft-relax 전환 (2026-04-21 늦은 오후)**: hard filter가 모든 후보를 drop → rollback 빈번 + RViz best_sample 잔상. soft penalty `w_rate_penalty*(excess/cap)²` 로 교체, `~w_rate_penalty=5.0` rosparam. 회귀: 0/463 rollback (20s), 연속성 유지. 관련: C² smootherstep blend(κ kick 제거) + `Marker.lifetime=0.2s`(잔상 제거). 상세: `HJ_docs/sampling_planner_wobble_diagnosis.md` 2026-04-21 늦은 오후 섹션. 파일: `sampling_planner_state_node.py:1867-1887`, `sampling_based_planner.py:991-1005`, `sampling_planner_state_node.py:2186-2223`.
- [ ] **🔥 n 방향 샘플 그리드 재설계 (P0 다음 세션)**: 현재 `n_samples=11` 이 너무 성겨서 raceline 근접 해상도 부족. 방향: (1) `n_samples` ↑ (21~31), (2) `n_end` 범위를 `prev_chosen_n_end` 중심 좁은 윈도우(±0.4m 수준)로 제약 — race/벽 cost + soft-rate penalty 로 중심부 유도, (3) `~n_samples`, `~n_end_window_half_m` rosparam 노출. 동시에 `calc_ms` 증가 영향 측정 (linear in n_samples). 대상 파일: `sampling_based_planner.py` 의 `generate_lateral_curves` 내부 `n_end_values` 생성부 (현재 prev-anchored narrow linspace).
- [ ] **L3 — heading feedback**: node state dict `n_dot_start = v·tan(chi_raceline_rel)·(1-Ω_z·n)` 실측 주입. `_ego_chi_vs_raceline()` 헬퍼 재사용.
- [ ] **YAML-overwrite 버그**: dynreg 서버 초기 콜백이 cfg default로 YAML 덮어씀 (live `w_pred=5000, w_race=0.1` 가 cfg default와 정확히 일치). `_weight_cb` 첫 호출 처리 또는 `update_configuration(yaml_dict)` 패턴으로 해결.
- [ ] **OVERTAKE + RECOVERY 병렬 기동** — 두 인스턴스 동시 기동 시 네임스페이스 충돌 없는지
- [ ] **cost rqt 튜닝 + save/reset 회귀** — `SamplingCost.cfg` 항목 일괄 점검 (L1~L3 fix 적용 후)
- [ ] **resample_ds_m** on/off 비교 (공간 등간격)
- [ ] **tail-blending** 동작 확인 — `_pub_local_wpnts()` 내부 cosine ramp (end 1~2m) — observation 모드에서는 blending 없이 발행하는지 vs blend 버전 비교

### 2.2 MPC 재설계 (Frenet kinematic + external side decider) — **v3c live-debug 완료 (2026-04-21)**

> ⚠️ **2026-04-20 비상 재설계** — 기존 n(s)-only `frenet_d_solver` 백엔드가 벽 hug/tick 진동/속도 추적 실패. 상세: `HJ_docs/mpc_redesign_frenet_kin_20260420.md`.
> **2026-04-21 v3c live-debug 세션** — structural infeasibility 근본 원인 발견 + 3-fix 적용 + 1997-tick 검증. 상세: `HJ_docs/mpc_frenet_kin_v3c_live_debugging_20260421.md`.

- [x] **271-tick 진단** — `margin_L_min=-0.00`(wall_safe 미적용), `jitter_rms p95=0.24m`, `u0.v=6.7 vs ego.v=3.5` 괴리 확인
- [x] **백업 규칙 적용** — solver/node/configs/launch `_backup_20260420` 접미사 보존 (롤백 가능)
- [x] **FrenetKinMPC 솔버 구현** — `src/frenet_kin_solver.py`. state `[n, mu, v, delta]`, control `[a, delta_dot]`, Liniger 이산 동역학. soft corridor + slack, Gaussian obstacle bubble + side bias, progress-maximization cost
- [x] **SideDecider 구현** — `src/side_decider.py`. LEFT/RIGHT/TRAIL/CLEAR, hold_ticks=5 hysteresis, feasibility gate `min_pass_margin=0.10`
- [x] **노드 와이어링** — `node/mpc_planner_state_node.py`에 `solver_backend=frenet_kin` 기본, `_decide_side`, `_lift_frenet_to_xy`(xy round-trip 없음)
- [x] **Live debug 토픽** — `~debug/tick_json` (std_msgs/String), `~debug/markers` (MarkerArray). Claude가 rostopic echo로 실시간 모니터
- [x] **launch 충돌 해소** — `instance_name` 기본값 `mpc_$(state)` → sampling의 `/overtake`와 분리
- [x] **Live smoke-test 1997 tick 완료** — solve_ms p99=91ms, max=125ms, jitter_rms p95=0.025m, 0 fails (tier0 pass=1 100%)
- [x] **v3c fix #1 — vmax hard bound k=0 skip** — `v_[0]==v0` equality와 `v_[0]≤vmax[0]` 충돌 (structural infeasibility) 해소
- [x] **v3c fix #2 — TRAIL vmax 감속 램프** — 1-tick discontinuity 대신 `a_dec_ramp=3.0 m/s²`로 ramp-from-v0
- [x] **v3c fix #3 — Pass 3 (obs-off) 제거** — feasibility masking 제거, 2-pass로 축소 후 노드 fallback ladder에 위임
- [x] **RViz 마커 색 — rainbow (red→blue) by tier/pass** — tier0 pass1=빨강, tier0 pass2=주황, tier1=노랑, tier2=녹색, tier3_CQ=시안, tier3_RACELINE=파랑
- [x] **HSL ma27 swap (2026-04-21)** — IPOPT `linear_solver: ma27`. 같은 시나리오 60s 비교에서 solve_ms p50 45.5→21.6ms (2.11×), p99 91.1→48.6ms (1.87×), tick rate 18.1→23.6Hz. iter 수 동일 → 순수 linear-solve 가속. libhsl.so는 `planner/3d_gb_optimizer/fast_ggv_gen/solver/setup_hsl.sh`가 제공. HSL 미설치 시 `_resolve_linear_solver`로 mumps 자동 fallback
- [x] **CasADi JIT (2026-04-21)** — `ipopt_jit: true`. obj/constraint/Jacobian/Hessian을 cold-start 시 네이티브 C로 컴파일(≈10~20s 지연, 런타임 0). ma27 위에 얹어 추가 ~2× → **MUMPS 대비 누적 p50 45.5→9.3 (4.9×), p95 73.3→19.0 (3.9×), p99 91.1→32.5 (2.8×), max 115.9→42.7 (2.7×)**. iter 수 불변, jitter_rms는 오히려 소폭 개선
- [ ] **장애물 cost body-edge 거리 기반 재정의** — 현재 Gaussian bubble은 점-점 거리 기준, ego half_width + obs half_width 미반영. 상세: `HJ_docs/mpc_frenet_kin_v3c_live_debugging_20260421.md` §5
- [ ] **obs_prediction 개수 (`n_obs_max=2`) 재검토** — `n_obs_raw=20` 관측 대비 2개 slot이 충분한지, 필터링 기준(시간/공간/TTC)과 함께 재정의. 상세: `HJ_docs/mpc_frenet_kin_v3c_live_debugging_20260421.md` §6
- [ ] **🔥 장애물 실물 1개 ↔ tick_json 2개 중복 체크** — predictor / `_build_obstacle_array` 중복 주입 여부 확인, 있으면 `n_obs_max` 튜닝보다 먼저 제거. §6.3 (1) 참조
- [ ] **state=observe/recovery 교차 검증** — 세 state 각각 한 번씩 돌려서 role output 동작 확인
- [ ] **장애물 없을 때 SIDE_CLEAR 경로** — corridor-only 케이스 smooth 유지 (obs 주입 게이팅 확인)
- [ ] **TRAIL per-k v cap 개선** — 현재 `ref_v` + 물리 deceleration ramp만 적용, 장애물 가속/감속 예측은 미포함 (GP predictor 연결은 후순위)
- [ ] **합격 시 Phase X 진입** — `_observation` suffix 제거, state_machine 직결 (사용자 승인 필수)

### 2.3 MPC overtake 재설계 — **architecture 결함 fix (2026-04-28 발견)**

> 진단·논의 출처: 본 세션 (2026-04-28). 24-26일 plan_picker / scoring 수정으로 long_qnt15 7 events / 600s 까지 축소했으나 stack architecture-level 결함 두 가지가 본질적 root cause임이 확인됨.
> **상세 수정 계획: `HJ_docs/mpc_overtake_redesign_plan_20260428.md`**

**확인된 결함**:
- A. `_apply_speed_painter` 가 MPC NLP 의 plan-aware `v[k]` 를 publish 직전에 vel planner GB raceline vx 로 **완전히 덮어씀** → MPC trailing 속도 의도가 시스템 밖으로 안 나감.
- B. Trailing 의 유일한 obstacle 인식 = controller TRAILING PID. ggv / curvature 무시. opponent가 SM 의 `_check_free_*` 부산물로만 set → path free 시 None → 충돌 시점 trailing 꺼지는 case.

**적용 완료 (2026-04-28)**:
- [x] **Step 2 — `_rewrite_trailing_to_local_arc` default OFF** — `obs_idx ≤ ego_idx` drop 부작용 제거. legacy global-s trailing gap 으로 복귀. rosparam `state_machine/trailing_gap_local_arc`.
- [x] **Step 3 — `_check_free_xy` 가 closest_target 도 set** — frenet 버전과 행동 일치. MPC OT path 평가에서 trailing target 누락 case 제거.
- [x] sticky bonus 부호 버그 fix, snapshot-only d_free, q_n_target 8→15 (Stage 2 Phase 2-2 누적, long_qnt15 best 7 events / 600s)
- [x] ~~**Step 1 — ttc-based encounter window**~~ → **2026-04-28 revert**. 사용자 bag 분석: obs.vs spike (KF noise, 0.05s 만에 1.0↔3.5 m/s) 가 closure rate 를 ttc 임계값 (3s) 근처에서 진동시켜 obstacles_in_interest 가 in/out 토글 → SM state TRAILING↔GB_TRACK 437 transitions/50.7s (≈8.6/sec). path 번쩍거림 + ego/obs 매칭 망가짐 → 충돌. ttc filter 는 obs.vs smoother (Step 7) 우선 적용 후 재시도.
- [x] ~~**Step 4 — `_post_process_speed` plan-aware override**~~ → **2026-04-28 revert**. 사용자 bag 분석: TRAIL plan 시 painter cap=`obs.vs-0.3≈1.5 m/s` 적용했으나, 직후 ego_v continuity ramp (`capped[0] = clip(capped[0], ego_v±0.15)`) 가 `capped[0]` 을 ego_v(3.8) 근처로 강제 → controller 가 받는 wpnts[0].vx_mps = 3.95 → ego brake 안 함 → obstacle 따라잡으며 충돌 (50.7s 에 6 events). 진짜 fix 는 painter 의 `a_max` 를 plan-aware (TRAIL 시 더 큰 brake 허용) 하거나 Step 4-alt (MPC v 직접 사용) 로.

**적용 대기 (우선순위 순)**:
- [ ] **Step 4 — `_apply_speed_painter` plan-aware override (결함 A 1차 fix)** — TRAIL 시 `obs.vs - 0.3` cap 추가. 기존 painter 의 안전 마진 유지하면서 MPC trailing 속도 의도가 trajectory.vx_mps 까지 살아남도록.
- [ ] **Step 5 — Controller TRAILING PID off (결함 B fix)** — Step 4 후 안전. `Controller.py:537-557` 의 trailing 분기 제거. trajectory.vx_mps 충실 추적. **Step 4 안 되면 단독 적용 시 trailing 자체 사라져 즉시 충돌**.
- [ ] **Step 6 — GP timestep-별 obs trajectory 활용 + interpolation/filter** — 현재 `_merge_obs_sources` 가 GP horizon msgs 중 timestep 0 만 사용 → vd 상수 propagate. GP 의 oscillation 학습 0% 활용. timestep-별 (s, n, vs, vd) 직접 obs_arr 에 fill + missing/noise robust handling.
- [ ] **Step 7 — 상대 속도 (obs.vs) tracking-only 분석 + filtering** — **truth 와 비교 절대 X (사용자 명령 2026-04-28)**. tracking topic 시계열만 보고 spike 판정. rolling median (5 tick) 또는 EMA + delta vs 임계 reject.
- [ ] **Step 8 — Multi-modal hand-crafted hypothesis (plan_scorer)** — LEFT/RIGHT_PASS plan 평가 시 obstacle 의 가능 행동 (stay / drift LEFT / drift RIGHT) 각각 d_free 계산 후 weighted/worst-case score. GP single-mode 보완.
- [ ] **Step 9 — frenet_kin_solver 에 ggv polar constraint 추가 (옵션 GGV-B)** — `(a_x/a_max)^2 + (a_y/ay_max)^2 ≤ 1`, ay_max 는 sector mu 따라 동적. kinematic bicycle 유지.

**Stage 3+ 후보 (큰 작업)**:
- [ ] **Step 10 — ggv-based single-track MPC backend (옵션 GGV-A)** — 새 파일 `planner/mpc_planner/src/ggv_singletrack_solver.py`. `solver_backend='ggv_singletrack'` 으로 swap. **단순 분석 결과 (2026-04-28): 자산 다 있음 — `planner/3d_gb_optimizer/fast_ggv_gen/output/<vehicle>/{vehicle,velocity}_frame/*.npy` (ax_max, ay_max, gg_exponent, alpha_list, g_list, v_list), `fbga_velocity_planner.py` (raceline baseline GGV-aware vx), `3d_gb_optimizer/global_line/` (Mintime + GGV global line)**. CasADi `interpolant('ax_max','bspline',[v,g,α], npy)` 로 wrapping → NLP 안에서 polar constraint 적용. single-track dynamics (linear 또는 pacejka tire) 통합. 작업 3-4일 (npy 로딩 + interpolant + dynamics + JIT + 검증). 위험: NLP nonlinear 증가 → solve 시간 30-50ms / infeasibility 빈도 가능.
- [ ] **Step 11 — MPC solver 의 plan-aware directional obstacle cost** — 현재 Gaussian bubble direction-agnostic → ego wrong-side 시작 시 cross 비용 무한대 → publish path 자체 lateral shift 부족. plan_side 따라 obstacle 한쪽만 push.
- [ ] **Step 12 — Strategic plan timing (close trail → opportunity → commit)** — reactive plan_picker → multi-stage 의도 표현. plan_scorer 에 opportunity score (코너 진입, brake, dv 큼 등) + commit dwell.

**검증 protocol** (각 Step 후):
1. ego s=0 spawn (`scripts/spawn_ego_s0.sh`)
2. obstacle_publisher + MPC 재기동
3. 10분 bag (`rosbag record -O <step>_long --duration=600 ...`)
4. `analyze_long_bag.py` + `coll_plan_breakdown.py`
5. 결과 row 추가: `HJ_docs/debug/0428_debug/work_log/stage2_phase22_progress_20260428.md`

**절대 금지** (사용자 강조 2026-04-28):
- Tracking 노이즈 분석에 truth 사용 금지 (`/tracking/obstacles_truth` 비교 X). 실제 환경에서 truth 없음.
- MPC ref_v 의도 우회 / 덮어쓰기 — solver 풀이 결과 살리는 방향만.
- layer 별 단편 fix 만 누적 — architecture 결함 (A, B) 풀고 가야.

### 2.4 SQP (3d_sqp_avoidance_node) — 베이스라인 유지
- [ ] **회귀 유지**: HJ 백엔드 튜닝 중 SQP가 여전히 gazebo_wall_2에서 무고장 동작하는지 정기 확인
- [ ] Track3DValidator 실패 시 `past_avoidance_d` 리셋 로직 검증 (회피 불가 → 빈 OTWpntArray 발행)

---

## 3. Sampling vs MPCC 비교 (P1)

- [ ] **공통 시나리오 세트 정의** — gazebo_wall_2에서 재현 가능한 3~5개 케이스 (블로킹 각도/속도 조합)
- [ ] **공통 메트릭**
  - 경로 곡률 RMS (Δκ), corridor margin (최소 d_left/d_right 여유), collision-free 비율
  - solve_ms 99th percentile, fallback/abort 빈도
  - 연속성: tick 간 best trajectory L2 거리 RMS
- [ ] **rosbag 수집 프로토콜**: Sampling / MPCC / SQP(기준) 동일 맵/시나리오 녹화 → offline 분석
- [ ] 결과 문서: `HJ_docs/overtake_backend_comparison.md` 신설 (Sampling + MPCC 중심, SQP는 baseline 컬럼)

---

## 4. state_machine 연동

### 4.1 MPC Phase X — Path Switching + SM 사본 기반 attach (2026-04-23, P0 진행 중)

> 상세 설계: `HJ_docs/mpc_planner_state_machine_integration.md` Phase X 섹션. 원본 SM/launch 불간섭, `3d_mpc_*` 사본으로만 작업.

**당일 완료 목표 (MVP) — 2026-04-23 구현 완료, 실측 대기:**
- [x] P0: `git status` + `mpc_planner_state_node.py` → `..._backup_20260423.py` 백업
- [x] P1: MPC 4-state FSM (IDLE / OVERTAKE / TRANSITION_OT2RC / RECOVERY) + dwell + TTC override + TRAIL handling
- [x] P1: mode별 publish 분기 (OT + TRANSITION → `/planner/avoidance/otwpnts`, RC → `/planner/recovery/wpnts`, IDLE/TRAIL skip)
- [x] P1: `tick_json` 확장 (`mpc_mode`, `dwell_count`, `alpha_ramp`, `ttc_min`, `recovery_solver_used`, painter, continuity_guard)
- [x] P1: `mpc_planner_state.launch` 인자 (`state:=auto`, `attach_to_statemachine:=true`, `recovery_solver:=quintic|nlp`)
- [x] P1.5: `_apply_mode_weights(mode, alpha)` OVERTAKE↔RECOVERY 선형 보간 (2026-04-24) — JIT 보존, `FrenetKinMPC.update_weights()` 활용, param-only switch
- [x] P1.5: warm-start seed 유지 정책 (tier 0 실패에만 reset)
- [x] P1.5: 경로 continuity guard (L2 체크 + 첫 `K_guard=5` wp xy/d_m/psi blend, s_m 보존)
- [x] P2a: RECOVERY quintic 경로 호출 (`geometric_fallback.build_quintic_fallback()` 재활용)
- [x] P2b: 기존 solver `LIVE_TUNABLE_WEIGHTS` 재활용으로 terminal `q_n_term` / `w_obs=0` 런타임 주입 (2026-04-24). `q_mu_term` 은 NLP 구조 변경 필요 → 다음 세션 (`q_n_term`만으로도 MVP 커버)
- [x] P2b: `recovery_nlp_profile:` YAML 섹션 + `_apply_mode_weights` 헬퍼 + `_plan_loop` 훅 (2026-04-24)
- [x] P2c: `~recovery_solver` ROS param + launch 인자 노출
- [x] P3: `_gb_vx_by_s` 캐시 (s_m 정렬 배열, binary search) — xy 라운드트립 금지
- [x] P3: `_post_process_speed()` (baseline + curvature cap + ego v 연속성 + seam blend)
- [x] P4: SM 사본 3파일 — `state_machine/src/mpc/{3d_mpc_state_machine_node.py, state_transitions_mpc.py, states_mpc.py}` (2026-04-24 `mpc/` 서브폴더로 이동)
- [x] P4: path_source enum (`GB`, `MPC_OT`, `MPC_RC`) + 선택 로직 + `recovery_wpnts_cb` freshness (stamp 저장 + age_ms 디버그)
- [x] P4: OVERTAKE exit → RECOVERY 경유 분기 (`state_transitions_mpc.py` OvertakingTransition, 2026-04-24) — `_check_latest_wpnts` + `_check_free_frenet` + `NOT _check_close_to_raceline` 조건에서 RECOVERY state 경유
- [x] P4: `/state_machine_mpc/debug` JSON publish
- [x] P5: trailing_targets 트리거 조건 path_source 무관하게 통일 (원본 `get_farthest_target()` 재사용)
- [x] P6: `stack_master/launch/3d_mpc_headtohead.launch` 신규 (state_machine 노드만 교체 + SQP/sampling OFF 강제)
- [x] P7: `catkin build mpc_planner state_machine` (성공, warnings only from gtest CMake policy)
- [ ] P7: 시나리오 1~4 (clear / static / dynamic-pass / dynamic-stuck) + background `rostopic echo -c` + 주기 집계 agent + 이상 감지 agent
- [ ] P7: 시나리오 5 Recovery A/B (quintic vs nlp) → 비교 표 HJ_docs 기입
- [ ] P7: 합격선 위반 시 튜닝 반복
- [x] P7: 원본 기준 commit SHA(`1242f29`) 박제 → 사본 rebase 용이성 (HJ_docs에 기록)

**합격선** (전체 표는 `HJ_docs/mpc_planner_state_machine_integration.md` Phase X 참조):
- solve_ms p95 OVERTAKE < 30ms, RECOVERY(quintic) < 3ms, RECOVERY(nlp) < 35ms
- traj jitter < 0.05m, 경로 continuity L2 (blend 후) < 0.05m
- seam 인접 vx diff < 0.3 m/s, margin_min > 0.15m
- RECOVERY 종점 |n| < 0.05m
- `/planner/avoidance/otwpnts` publisher count == 1

### 4.2 공용 state_machine 통합 (P2 — Phase X 검증 후)

> 원칙: **observation 토픽으로 valid 증명이 끝난 백엔드만** state_machine에 attach.

- [ ] 각 백엔드의 attach 스위치 (`attach_to_statemachine`, `sampling_planner_enable`, `dynamic_avoidance_mode`) 동작 표 정리
- [ ] 복수 백엔드 동시 활성 시 **single publisher 원칙** 강제 — launch-level guard 또는 런치 인자 mutex 검토
- [ ] state_machine `_pub_local_wpnts()` 쪽 tail-blending 로직이 백엔드별 출력 특성(스무딩/비등간격)에 robust한지 확인
- [ ] Phase X 사본 충분히 검증되면 원본 `3d_state_machine_node.py` 통합 검토 + `dynamic_avoidance_mode:=MPC_INTERNAL` 정식 옵션화

---

## 5. 장기 / 판단 보류

- [x] **`/local_waypoints/markers` 165Hz → 80Hz (2026-04-30)** — `_pub_local_wpnts()`가 DELETEALL marker를 별도 publish하는 패턴이라 state_machine 80Hz loop마다 publish 2번 (실측 165Hz). DELETEALL을 동일 MarkerArray의 첫 element로 합쳐 publish 1회로 정리. 파일: `state_machine/src/mpc/3d_mpc_state_machine_node.py:_pub_local_wpnts`. 동일 함수 다른 marker는 정상. 효과: cross-host packet rate 감소(특히 70.9에 rviz subscriber 다수일 때 N배 영향). 평균/spike ping과의 인과는 별개 — packet rate ↔ ping correlation 측정 r²=0.008로 직접 변동 원인은 미확정 상태.
- [x] **wifi cross-host ping 평균 ~3ms 영구 적용 (2026-05-01)** — gazebo(70.9) ↔ stack(70.2) cross-host TCPROS가 wifi airtime 점유 시 ping 평균 7~10ms / max 200~280ms spike 발생. 원인은 (a) Linux `tcp_rto_min=200ms` 기본값 (TCP loss 시 spike 폭 결정), (b) mac80211 BE 큐 saturation/큐잉, (c) AX211 BT coex jitter. 4단계 ablation으로 효과 분리: WMM Voice queue (DSCP EF, ICMP만) **-3.22ms**, RTO_min 50 (spike 200대→50대), BT off (-0.38ms, 사실상 noise). 적용 결과 평균 **2.39ms / max 70ms / >100ms 0건** (60s ping 300packets 기준). cross-host TCP까지 EF로 보내면 voice 큐 saturate → 오히려 악화 (ablation으로 검증, ping max 189ms 발생) → 의도적으로 ICMP만 EF, ROS는 BE 큐 유지. 영구 hook: `/etc/NetworkManager/dispatcher.d/99-wifi-tune` (NetworkManager가 wifi up/DHCP 마다 자동 재적용). bashrc 함수: `wifitune-on / -off / -status` (`~/.bashrc`의 `# >>> HJ wifi tune <<<` 블록). 다른 머신 호환: dispatcher script + bashrc 둘 다 default route iface/subnet/src IP 자동 감지, hardcoded 없음. 환경변수 `WIFITUNE_IFACE`, `WIFITUNE_SUBNET`로 override 가능. 70.9에 같은 변경 적용은 **무효** (효과 없음 확인됨, 70.2 측만 유지). 남은 50~100ms 단발 spike (1% 미만)는 클라이언트 측에서 못 잡는 영역 — AP↔70.9 hop, AP 펌웨어, RF 환경 본질.
- [ ] **경사면 속도 보상** — `mu_rad` 기반 속도 커맨드 보정 (controller or vel_planner)
- [ ] **Controller nearest_waypoint 3D 전환** — 현재 local 내부 검색이라 2D로 충분하나 경로 겹침 시 필요
- [ ] **시각화 마커 z 반영** — sector_server 등 z=0 하드코딩 수정 (controller 마커는 완료)
- [ ] **Perception 3D** — `abp-detection/detect.cpp`, `2.5d_detection/tracking_node.cpp` 등 `GetFrenetPoint`/`GetGlobalPoint` 시그니처 변경으로 인한 호출부 z 채우기 (detection 확정 후)
- [ ] **carstate_node.py 처분 결정** — GLIL base_odom이 vy/pitch 충분히 제공하면 제거 확정

### 판단 기록
- 3D에서 Frenet xy round-trip 금지 (오버패스 층 구분 실패). `get_cartesian_3d`/`get_frenet_3d` 경유 필수 → 백엔드 내부 obstacle 좌표 변환 시 재확인
- MPCC NLP는 2D 유지, 3D는 출력 lift 단계에서만 Track3D로 처리 (infeasible 회피)
- SQP(베이스라인)는 frenet `d(s)` 최적화, MPCC는 Cartesian kinematic bicycle — obstacle 주입 표현이 다르므로 동일 시나리오 비교 시 해석 주의
- Sampling은 candidate fan + MPPI re-weighting이라 cost tuning/필터 파라미터가 연속성에 직접 영향 → 정량 지표 없이 eyeballing 금지

---

## 6. 참고 문서

- HJ 집중 문서:
  - `HJ_docs/sampling_planner_state_machine_integration.md` — Sampling state-aware 통합 플랜
  - `HJ_docs/mpcc_planner_trial_v1.md` — MPCC 초기 포팅
  - `HJ_docs/mpc_planner_state_machine_integration.md` — MPCC state-aware 통합 플랜 (기존 Phase 계획, 재설계로 일부 대체됨)
  - **`HJ_docs/mpc_redesign_frenet_kin_20260420.md` — Frenet kinematic MPC 재설계 (기반 설계)**
  - **`HJ_docs/mpc_frenet_kin_v3c_live_debugging_20260421.md` — v3c live-debug 세션 (현행, 1997-tick validation + obs TODO)**
  - **`HJ_docs/mpc_overtake_redesign_plan_20260428.md` — MPC overtake architecture 결함 진단 + 9 Step 수정 계획 (현행)**
  - `HJ_docs/debug/0428_debug/work_log/stage2_phase22_progress_20260428.md` — Stage 2 Phase 2-2 진행 / 시도/롤백 누적 (long_qnt15 best 7 events)
  - `HJ_docs/3d_dynamic_prediction_and_planner.md` — 3D prediction + SQP 포팅 (베이스라인 참고)
- IY 작업분 (HJ는 참고만):
  - `IY_docs/rolling_horizon_overtake_planner.md`, `IY_docs/TODO_HJ.md`
- 3D 포팅 버그 카탈로그: `HJ_docs/3d_port_bug_catalog.md`
- 과거 3D 포팅 TODO 보존본: `HJ_docs/backup/TODO_HJ.md`
