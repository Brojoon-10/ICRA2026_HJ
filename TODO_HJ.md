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

### 2.3 SQP (3d_sqp_avoidance_node) — 베이스라인 유지
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
  - `HJ_docs/3d_dynamic_prediction_and_planner.md` — 3D prediction + SQP 포팅 (베이스라인 참고)
- IY 작업분 (HJ는 참고만):
  - `IY_docs/rolling_horizon_overtake_planner.md`, `IY_docs/TODO_HJ.md`
- 3D 포팅 버그 카탈로그: `HJ_docs/3d_port_bug_catalog.md`
- 과거 3D 포팅 TODO 보존본: `HJ_docs/backup/TODO_HJ.md`
