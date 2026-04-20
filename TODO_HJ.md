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
- [ ] **L2 — 시간축 smoothing**: `filter_alpha: 0.6` 1차. 부족하면 endpoint 전용 continuity cost 추가.
- [ ] **L3 — heading feedback**: node state dict `n_dot_start = v·tan(chi_raceline_rel)·(1-Ω_z·n)` 실측 주입. `_ego_chi_vs_raceline()` 헬퍼 재사용.
- [ ] **YAML-overwrite 버그**: dynreg 서버 초기 콜백이 cfg default로 YAML 덮어씀 (live `w_pred=5000, w_race=0.1` 가 cfg default와 정확히 일치). `_weight_cb` 첫 호출 처리 또는 `update_configuration(yaml_dict)` 패턴으로 해결.
- [ ] **OVERTAKE + RECOVERY 병렬 기동** — 두 인스턴스 동시 기동 시 네임스페이스 충돌 없는지
- [ ] **cost rqt 튜닝 + save/reset 회귀** — `SamplingCost.cfg` 항목 일괄 점검 (L1~L3 fix 적용 후)
- [ ] **resample_ds_m** on/off 비교 (공간 등간격)
- [ ] **tail-blending** 동작 확인 — `_pub_local_wpnts()` 내부 cosine ramp (end 1~2m) — observation 모드에서는 blending 없이 발행하는지 vs blend 버전 비교

### 2.2 MPC 재설계 (Frenet kinematic + external side decider)

> ⚠️ **2026-04-20 비상 재설계** — 기존 n(s)-only `frenet_d_solver` 백엔드가 벽 hug/tick 진동/속도 추적 실패. 상세: `HJ_docs/mpc_redesign_frenet_kin_20260420.md`.
> 코드 + Docker 빌드까지 완료, **live-test 및 튜닝 미완**. 다음 세션 첫 할 일.

- [x] **271-tick 진단** — `margin_L_min=-0.00`(wall_safe 미적용), `jitter_rms p95=0.24m`, `u0.v=6.7 vs ego.v=3.5` 괴리 확인
- [x] **백업 규칙 적용** — solver/node/configs/launch `_backup_20260420` 접미사 보존 (롤백 가능)
- [x] **FrenetKinMPC 솔버 구현** — `src/frenet_kin_solver.py`. state `[n, mu, v]`, control `[a, delta]`, Liniger 이산 동역학. hard corridor + slack, half-plane 장애물 제약, progress-maximization cost
- [x] **SideDecider 구현** — `src/side_decider.py`. LEFT/RIGHT/TRAIL/CLEAR, hold_ticks=5 hysteresis
- [x] **노드 와이어링** — `node/mpc_planner_state_node.py`에 `solver_backend=frenet_kin` 기본, `_decide_side`, `_lift_frenet_to_xy`(xy round-trip 없음)
- [x] **Live debug 토픽** — `~debug/tick_json` (std_msgs/String), `~debug/markers` (MarkerArray). Claude가 rostopic echo로 실시간 모니터
- [x] **launch 충돌 해소** — `instance_name` 기본값 `mpc_$(state)` → sampling의 `/overtake`와 분리
- [ ] **Live smoke-test** — 3D 환경 기동 후 tick_json 합격선 (margin_L_min≥0.15, jitter_rms p95≤0.05, u0.v≈ego.v) 확인. **다음 세션 최우선**
- [ ] **첫 solve infeasible 대응** — `_seed_warm_start` 초기 시드 품질 점검, 필요시 IPOPT iter 상한/warm-start 전략 튜닝
- [ ] **state=observe/recovery 교차 검증** — 세 state 각각 한 번씩 돌려서 role output 동작 확인
- [ ] **장애물 없을 때 SIDE_CLEAR 경로** — corridor-only 케이스 smooth 유지 (obs 주입 게이팅 확인)
- [ ] **TRAIL per-k v cap** — 현재 `ref_v`만 의존, 장애물 가속/감속 예측은 미포함 (GP predictor 연결은 후순위)
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

## 4. state_machine 연동 (P2 — 후순위)

> 현재 원칙: **observation 토픽으로 valid 증명이 끝난 백엔드만** state_machine에 attach.

- [ ] 각 백엔드의 attach 스위치 (`attach_to_statemachine`, `sampling_planner_enable`, `dynamic_avoidance_mode`) 동작 표 정리
- [ ] 복수 백엔드 동시 활성 시 **single publisher 원칙** 강제 — launch-level guard 또는 런치 인자 mutex 검토
- [ ] state_machine `_pub_local_wpnts()` 쪽 tail-blending 로직이 백엔드별 출력 특성(스무딩/비등간격)에 robust한지 확인

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
  - **`HJ_docs/mpc_redesign_frenet_kin_20260420.md` — Frenet kinematic MPC 재설계 (현행)**
  - `HJ_docs/3d_dynamic_prediction_and_planner.md` — 3D prediction + SQP 포팅 (베이스라인 참고)
- IY 작업분 (HJ는 참고만):
  - `IY_docs/rolling_horizon_overtake_planner.md`, `IY_docs/TODO_HJ.md`
- 3D 포팅 버그 카탈로그: `HJ_docs/3d_port_bug_catalog.md`
- 과거 3D 포팅 TODO 보존본: `HJ_docs/backup/TODO_HJ.md`
