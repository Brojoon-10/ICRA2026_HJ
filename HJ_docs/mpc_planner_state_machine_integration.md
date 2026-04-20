# MPC Planner — versatile state-conditioned planner

## Context

현재 `planner/mpc_planner/`의 kinematic MPCC는 **2D**이고 `/global_waypoints`를 reference로만 받아 `/planner/mpc/trajectory` (Path) + markers만 발행한다. 컨트롤러는 `/behavior_strategy.local_wpnts`(WpntArray)만 소비하므로 **MPC 궤적은 누구도 따라가지 않는** 상태이고, 가끔 IPOPT Infeasible도 발생한다. cost는 raceline 주행용으로 튜닝되어 있어 다른 state에 그대로 쓸 수 없다.

**유저 요구사항 (답변 반영):**
- 목표는 **MPC를 state-aware planner**로 쓰는 것. sampling planner와 **병렬 개발 트랙**으로, 동일 계약 아래 장단점 비교.
- **RECOVERY**: `recovery_spliner` **완벽 대체**. MPC 내부에서 복귀 경로 생성 (upstream 의존 없음).
- **OVERTAKE**: 기존 OT planner(spliner / sqp / static_avoidance / lane_change) **완벽 대체**. upstream OT 라인 의존 없이 MPC가 **직접** 회피 궤적 생성. reference는 raceline (`/global_waypoints`), opponent prediction/obstacle 정보는 **NLP에 필수 입력**으로 주입. sampling planner와 동일 철학.
- **START**: 추후. Future Work에만 기재.
- **GB_TRACK**: 이번 스코프 밖. controller-follow-MPC 연동은 추후. Future Work 섹션에만 기재.
- 3D 맵 기반이므로 Wpnt 출력에 **z_m, psi_rad, mu_rad, kappa_radpm** 까지 채워야 함. Track3D를 재사용.
- sampling planner 통합 설계(`HJ_docs/sampling_planner_state_machine_integration.md`)와 **동일 계약** 준수: `~state` 파라미터, dynamic_reconfigure, role별 출력 토픽, tail-blending은 state_machine 쪽에서 이미 처리됨.

---

## 의사결정 요약 (이 plan의 기조)

1. 기존 `mpc_planner_node.py`를 리팩터하지 말고, sampling과 똑같이 **신규 `mpc_planner_state_node.py`** 를 추가. 원본은 `observe` 용도로 보존.
2. MPC 코어(kinematic bicycle NLP)는 **2D 유지**. 3D는 출력 lift 단계에서만 Track3D로 채움.
3. **MPC는 "형상 우선" 플래너**다. **속도 프로파일은 부산물** (2.5d vel planner가 덮어씀). 이게 plan 전체의 철학 기반 — NLP cost에서 `w_velocity`를 최소화/제거하고, feasibility는 형상 해를 얻기 쉬운 방향으로 구성.
4. **"절대 안 죽음" 보장**: 4단 fallback — NLP OK → HOLD-LAST (s-shifted) → GEOMETRIC-FALLBACK (현재 pose에서 raceline 복귀 arc) → RACELINE-SLICE.
5. **OT obstacle은 NLP 필수 입력** (옵션 아님). reference는 raceline, `/opponent_prediction/obstacles` + `/opponent_trajectory` + `/tracking/obstacles`를 SQP 계약 그대로 fusion. 주입 형태는 **soft repulsive cost (1차)** → **hard collision constraint (2차)** 단계적.
6. **🔒 state_machine 직통 연결 회피 (observation-first).** Phase 1~6은 `_observation` suffix 토픽으로만 발행, **기존 파이프라인 무손상**. state_machine은 계속 spliner/recovery_spliner를 소비. 구현 완성도 검증 (Phase 9 메트릭) 이 합격한 뒤에만 **Phase X에서 `3d_headtohead.launch`를 건드려 실제 토픽으로 attach**.
7. Phase X 이후 통합은 sampling planner와 **동일한 런치 arg** (`planner_backend:={legacy|sampling|mpc}`) 로 공존. sampling 통합과 **충돌 금지**.

---

## Phase 0 — 백업

**Goal.** 수정 대상 파일을 모두 reversible 하게.

**대상 (수정 시작 전에 복사):**
- `planner/mpc_planner/node/mpc_planner_node.py` → `mpc_planner_node_backup_20260418.py`
- `planner/mpc_planner/src/mpcc_solver.py` → `mpcc_solver_backup_20260418.py`
- `planner/mpc_planner/config/mpcc_params.yaml` → `mpcc_params_backup_20260418.yaml`
- `planner/mpc_planner/launch/mpc_planner.launch` → `mpc_planner_backup_20260418.launch`
- `stack_master/launch/3d_headtohead.launch` (이미 sampling 쪽 백업본 있음; MPC 연동 전 재확인)

**Verify.** `git status`로 백업 파일만 추가됐는지 확인.

---

## Phase 1 — 신규 state-aware 노드 `mpc_planner_state_node.py`

**Goal.** 원본 `mpc_planner_node.py`를 베이스로 `~state ∈ {overtake, recovery, observe}` 분기와 role별 출력을 갖는 신규 노드. sampling의 `sampling_planner_state_node.py`와 동일 패턴.

**생성:**
- `planner/mpc_planner/node/mpc_planner_state_node.py`
- `planner/mpc_planner/cfg/MPCCost.cfg` (dynamic_reconfigure 정의)

**설계:**
- `~state` 파라미터로 분기. default = `observe`.
- **⚠ Phase 1~6에서는 state_machine과 직통 연결 회피**. 모든 role별 출력 토픽에 `_observation` suffix를 붙여 **기존 파이프라인과 완전 격리**된 상태로 파이프라인 완성도를 먼저 검증. 실제 state_machine 통합은 Phase X (별도 게이트) 에서 suffix 제거/remap.
- 출력 토픽 (observation 모드; Phase 1~6 기본):
  - `overtake`  → `~out/otwpnts` (OTWpntArray) → launch에서 **`/planner/avoidance/otwpnts_observation`**로 remap
  - `recovery`  → `~out/wpnts`   (WpntArray)   → **`/planner/recovery/wpnts_observation`**로 remap
  - `observe`   → **`~best_trajectory_observation`** (WpntArray) + 기존 Path/Markers 유지 (디버그)
- launch arg `~attach_to_statemachine` (default `false`) — `true`로 바꾸면 `_observation` suffix 제거된 실제 토픽으로 remap. Phase X에서만 사용.
- 공통 디버그 토픽 (모든 state에서 발행): `~best_sample` (Path), `~best_sample/markers` (MarkerArray), `~status` (String, latched), `~timing_ms` (Float32).
- 원본 `mpc_planner_node.py`는 그대로 둬서 `observe` 전용으로 계속 사용 가능.

**참고 파일:**
- [planner/3d_sampling_based_planner/node/sampling_planner_state_node.py](../../../icra2026_ws/ICRA2026_HJ/planner/3d_sampling_based_planner/node/sampling_planner_state_node.py) — role 분기, 토픽 remap 패턴
- [HJ_docs/sampling_planner_state_machine_integration.md §2.3~2.5](../../../icra2026_ws/ICRA2026_HJ/HJ_docs/sampling_planner_state_machine_integration.md)

**Verify.**
1. `roslaunch mpc_planner mpc_planner_state.launch state:=observe` → `~best_trajectory_observation` WpntArray + 기존 Path 둘 다 발행. **실제 `/planner/*` 토픽은 건드리지 않음** 확인.
2. `state:=overtake` → `/planner/avoidance/otwpnts_observation` 발행 확인 (`rostopic echo -n1`).
3. `state:=recovery` → `/planner/recovery/wpnts_observation` 발행 확인.
4. state_machine이 여전히 기존 spliner/recovery_spliner 출력을 소비하는지 확인 (`rostopic info /planner/avoidance/otwpnts` publisher가 spliner인지 체크).

---

## Phase 2 — 3D lift 후처리 (Track3D 재사용)

**Goal.** MPC 솔버 출력 `(x, y, psi)_N+1`을 받아 `Wpnt`를 채울 때 **z_m, psi_rad, kappa_radpm, mu_rad, d_left, d_right**를 Track3D 인터폴레이터로 채워 넣는다. NLP는 그대로 2D.

**생성:**
- `planner/mpc_planner/src/mpc_track3d_lifter.py` — Track3D 인스턴스를 싱글턴으로 로드, 외부 API 2개만:
  - `project_xy_to_s(x, y)` → `(s, n)` (2D 최근접 segment search; z 무시)
  - `fill_wpnt(x_mpc, y_mpc, psi_mpc, v_mpc)` → dict with `{s_m, d_m, x_m, y_m, z_m, psi_rad, kappa_radpm, mu_rad, d_left, d_right, vx_mps, ax_mps2}`

**수정:**
- `mpc_planner_state_node.py`의 publish 경로에서 각 Wpnt를 위 헬퍼로 채움.
- `vx_mps` = NLP 솔루션 `U[k,0]`; `ax_mps2` = `(U[k+1,0] - U[k,0]) / dT`.
- `### HJ : 3D lift` 주석 부착.

**설계 포인트:**
- Track3D import는 sampling 쪽이 쓰는 `sys.path.insert` 패턴 그대로 복제 (한 모듈로만 로드해서 양쪽에서 import 가능하게 `planner/common/track3d_import.py` 헬퍼 후보 — Phase 8에서 sampling과 MPC 둘 다 리팩터).
- `psi_rad`는 **track-frame 인터폴레이션 값**을 기본으로 사용. MPC 자체 `psi`와의 blend 여부는 컨트롤러 lookahead 영향 확인 후 결정 (초기값은 50/50 blend, 필요 시 파라미터화).

**참고:**
- [planner/3d_sampling_based_planner/src/track3D.py:665-679 `sn2cartesian`](../../../icra2026_ws/ICRA2026_HJ/planner/3d_sampling_based_planner/src/track3D.py)
- 기존 MPC z 캐시: `planner/mpc_planner/node/mpc_planner_node.py:113-114` (`### HJ :` 주석 있음)

**Verify.**
1. 동일 s_m에서 `/planner/mpc/best_trajectory[i].z_m` ≈ `/global_waypoints[j].z_m` (Δ < 5 cm)
2. RViz에서 MPC 궤적이 3D 맵 지면에 붙어 보임.
3. `wpnt.mu_rad`가 기울기가 큰 구간에서 0이 아닌 값을 가짐.

---

## Phase 3 — state별 reference 슬라이서 & 프리셋

**Goal.** state별로 (a) reference 소스, (b) 슬라이싱 로직, (c) cost/bound 프리셋이 달라져야 한다.

### 3.1 observe (기존)
- Reference: `/global_waypoints`
- 슬라이서: 현재 `_slice_local_ref` (target_s += g_vx · dT) 유지.
- 프리셋: 현재 `mpcc_params.yaml` 그대로.

### 3.2 overtake (기존 OT planner 완벽 대체)

**철학.** upstream OT 라인 의존 0. reference는 raceline, 회피는 MPC가 NLP 안에서 **직접** 만든다. sampling planner가 candidate fan + prediction cost로 회피선을 생성하듯, MPC는 NLP cost/constraint로 회피선을 생성.

**입력 계약 (SQP `planner/sqp_planner/src/sqp_avoidance_node.py:88-99`의 구독 계약을 그대로 채택 — apples-to-apples 비교):**
- Reference = `/global_waypoints` (raceline, 동일 슬라이서 재사용)
- **Opponent prediction (주 입력):**
  - `/opponent_prediction/obstacles` — `f110_msgs/ObstacleArray` (frenet bbox + `is_static`, `vs`, `vd`, `s_var`, `d_var`)
  - `/opponent_trajectory` — `f110_msgs/OpponentTrajectory` (array of `OppWpnt`: `s_m`, `d_m`, `proj_vs_mps`, `vd_mps`, `d_var`). SQP가 s축 인덱싱으로 상대 `d_m` 추출에 사용.
- **Tracking snapshot (fallback):**
  - `/tracking/obstacles` — `f110_msgs/ObstacleArray` (현재 frenet bbox + 속도). SQP는 staleness 체크 없이 최신 callback을 deepcopy해서 사용 — **MPC는 여기서 한발 더 나아가 timestamp freshness 체크 추가** (SQP 대비 개선점).
- **Fusion 정책 (SQP 동작 + freshness 추가):**
  1. `/opponent_prediction/obstacles`의 obstacle이 `is_static=False` → `/opponent_trajectory`의 `(s_m, d_m)`으로 MPC horizon의 각 step s를 interpolation해서 상대 위치 시계열 획득 (SQP의 `sqp_avoidance_node.py:320` 패턴 그대로).
  2. `is_static=True` 또는 prediction 없음 → `/tracking/obstacles`의 현재 snapshot에 `vs`/`vd` 상수속도 외삽으로 N 스텝 채움.
  3. 둘 다 stale (> 100ms) → obstacle 슬롯 무력화 (`w_obs=0` + 먼 좌표). 회피 없이 raceline 추종.
- **Obstacle 전처리:** SQP의 `group_objects()` (`sqp_avoidance_node.py:499`) — 중첩 obstacle들을 s/d 공간에서 bounding하여 하나로 병합 → MPC도 동일 전처리 적용해 `N_obs_max` 이내로 압축.

**SQP와 다른 점 (의도적):**
- SQP는 frenet 공간에서 SLSQP로 `d(s)` 프로파일을 풀고 hard constraint (거리·turning radius·연속성)로 회피. **MPC는 Cartesian에서 kinematic bicycle NLP**이므로 obstacle 좌표를 frenet → Cartesian 변환 후 주입한다 (Track3D의 `sn2cartesian` 재사용).
- SQP 목적함수는 `sum(d²)·10 + sum(d''²)·100`. MPC는 contour/lag 기반 cost라 철학이 다름. 비교는 **형상 품질** (Δκ RMS, corridor margin) 과 **solve_ms** 로.

**구조 설계:**

| 항목 | 기본안 | 메모 |
|---|---|---|
| `N_obs_max` (동시 주입 obstacle 수) | 2 | 1vs1 + merged group 1개 여유 |
| fusion layer 위치 | 노드 레벨. solver에는 Cartesian 좌표로 변환된 `(N_obs_max, N, 2)` 배열만 전달 | OK |
| prediction horizon vs MPC horizon | prediction 더 길면 앞 N 포인트만 사용, 짧으면 `vs`/`vd` constant-velocity 외삽 | OK |
| opponent 당 geometry | 점 + `σ` (soft) 또는 점 + `r_safe` (hard). 차량 footprint 고려한 타원은 future | OK |
| freshness threshold | 100ms (prediction/tracking 공통) | SQP 대비 개선 |

**NLP 주입 구조 — 2단계:**

| Stage | 방식 | 설명 | 장점 | 단점 |
|---|---|---|---|---|
| **Stage-1 (1차 구현)** | **Soft repulsive cost** | 각 스텝 k, 각 obstacle o에 대해 `w_obs · exp(-((x_k-ox)² + (y_k-oy)²) / (2σ²))` 을 cost에 가산 | sampling의 `prediction_weight`와 동등 철학 → 공평 비교. feasibility 거의 안 흔들림. 구현 단순 | 가까이 있어도 수치적으로 "못 지나감"은 아님 (확률적 회피) |
| **Stage-2 (2차, Stage-1 실측 후 결정)** | **Hard collision constraint** | 각 스텝에 대해 `(x_k-ox)² + (y_k-oy)² ≥ r_safe²` 부등식 + slack. 또는 타원 (차량/타깃 geometry 반영). | 확정적 회피 거리 보장 | infeasibility 위험 ↑, 수렴 느림. slack variable 추가 필요 |

**구현 (`mpcc_solver.py` 변경):**
- `N_obs_max = 4` 같은 상수 (컴파일 타임). 파라미터 벡터에 obstacle 좌표 `(2 · N · N_obs_max)` 슬롯 추가.
- 미사용 슬롯은 좌표를 먼 점(예: `(1e4, 1e4)`)으로 채우고 `w_obs=0`으로 무력화.
- Stage-2 constraint는 `~collision_mode ∈ {none, soft, hard}` 파라미터로 토글. Stage-1만 들어간 상태에서는 `soft`만 on.

**reference 슬라이싱.** observe/recovery와 동일 (`_slice_local_ref`). raceline을 그대로 사용.

**프리셋 `config/state_overtake.yaml`:**
```yaml
w_contour: 2.0       # ↓ (raceline 고집 약화 — 회피 허용)
w_lag: 0.8
w_velocity: 1.5
w_dv: 9.5
w_dsteering: 20.0    # ↑ (차선 변경 smooth)
w_slack: 2000.0
max_speed: 10.0
min_speed: 1.0
max_steering: 0.6
boundary_inflation: 0.05
ipopt_max_iter: 1000
collision_mode: soft   # Stage-1
w_obstacle: 500.0
obstacle_sigma: 0.35
n_obs_max: 4
r_safe: 0.35           # Stage-2에서만 활성
```

### 3.3 recovery (MPC가 recovery_spliner 대체) — **방식 C 확정**

**철학 (Q-G: C 확정).**
- **평상시 (Tier 0):** MPC가 NLP로 복귀선 직접 생성. reference=raceline, 초기상태=현재 pose, `w_contour`/`w_lag`가 차를 raceline 쪽으로 당김. MPC 플래너로서의 성능 측정.
- **연속 실패 시 (Tier 2 자동):** Phase 4의 GEOMETRIC_FALLBACK (Frenet quintic 복귀선) 이 자동 개입. recovery state가 특히 자주 올라갈 후보 — 차가 이미 raceline에서 멀리 있기 때문에 NLP가 초기엔 빡셀 수 있음.

**슬라이서:** observe/gb_track 슬라이서 그대로 사용 (`_slice_local_ref`). raceline 그대로.

**프리셋 `config/state_recovery.yaml`:**
```yaml
w_contour: 1.5       # ↓ (차가 raceline에서 벗어나 있으므로 overshoot 허용)
w_lag: 0.8
w_velocity: 0.5      # ↓↓ (속도 강요 X; 복귀가 우선)
w_dv: 5.0
w_dsteering: 10.0
w_slack: 500.0       # ↓ (boundary soft; 트랙 끝까지 활용)
max_speed: 4.0
min_speed: 0.0       # ★ 정지 허용 (min_speed=0 필수)
max_steering: 0.6
boundary_inflation: 0.02   # 거의 전체 코리도 사용
ipopt_max_iter: 1500
```

**주의:** `min_speed=0`이면 `v/L · tan(delta)`가 degenerate → Phase 4 R4에서 regularizer로 방어.

### 3.4 gb_track / start (이번 스코프 밖 — Future Work)

기록만.
- **GB_TRACK**: MPC 출력을 state_machine의 local_wpnts 소스로 꽂으려면 `cur_gb_wpnts`의 closed-loop invariants (전체 트랙 길이, 균등 s 간격) 때문에 별도 설계 필요. Phase 7로 연기.
- **START**: 현 `planner/spliner/start_spline_node_v2` 대체 대상. MPC로 하려면 정지 상태 → raceline 진입 궤적을 low-speed kinematic으로 풀어야 함. recovery와 유사한 구조 재사용 가능 (min_speed=0 처리, `w_velocity` 낮게). Phase 7의 하위로 연기.

**생성 파일:**
- `planner/mpc_planner/config/state_observe.yaml`
- `planner/mpc_planner/config/state_overtake.yaml`
- `planner/mpc_planner/config/state_recovery.yaml`

**Verify.**
1. bag 재생: OT 시나리오에서 `state:=overtake` + `ot_reference_source:=static_avoidance` → MPC 궤적이 spliner 라인을 따라감.
2. 차를 레이스라인 바깥 1~2m에 두고 `state:=recovery` → MPC 궤적이 부드럽게 레이스라인 위로 합류.
3. 각 프리셋 로드 시 최종 weight를 `rospy.loginfo`로 덤프.

---

## Phase 3.5 — OVERTAKE Stage-2: hard collision constraint (Stage-1 실측 후)

**Goal.** soft repulsive만으로 회피 마진이 불충분하면 `~collision_mode:=hard`로 hard constraint 도입.

**수정:**
- `mpcc_solver.py::_build_nlp` — soft cost 옆에 `(x_k - ox)² + (y_k - oy)² - r_safe² + s_col_k ≥ 0`, `s_col_k ≥ 0`, cost에 `w_col_slack · s_col_k²` 추가.
- obstacle 슬롯 미사용 시는 constraint 자동 만족 (먼 좌표).
- `~collision_mode ∈ {none, soft, hard}`. `hard`는 soft도 같이 켜져 있어야 함 (regularizer 역할).

**Verify.**
1. Stage-1로만 주행했을 때 실제 회피 마진 CDF 기록. `r_safe` 설정값보다 자주 아래로 떨어지면 Stage-2 도입 이유 확립.
2. Stage-2 on → 같은 bag에서 infeasible 발생률 측정. HOLD_LAST/FALLBACK 비율이 5% 이상이면 `r_safe` 낮추거나 Stage-1로 복귀.

---

## Phase 4 — "합리적 궤적, 절대 안 죽음" 보장

**Goal.** MPC가 **형상 우선** (속도 부산물) 플래머로서 **항상 합리적 Wpnt[]를 발행**. 발행 끊김 0, 기괴한 궤적 0.

### 4.1 MPC를 "형상 플래너"로 재조정

NLP cost에서 속도 관련 term을 거의 제거해 해 공간을 형상 관점으로 단순화.

| 변경 | 이유 |
|---|---|
| `w_velocity: 3.0 → 0.1` (전 state 공통 base) | 속도는 2.5d vel planner가 덮어씀. `w_velocity=0`도 가능하나 수치적으로 v가 너무 자유로워지면 dynamics가 꼬임 → 아주 작은 anchor만 유지 |
| `v_bias_max: 1.0 → 3.0` | 속도 편차 허용폭 넓게. velocity cost가 dominate하지 않게 정규화 완화 |
| `w_contour, w_lag` 유지 | 형상 (lateral contouring + 진행 기여) 이 메인 cost 축 |
| `w_dv, w_dsteering` 유지 또는 ↑ | smoothness가 형상 품질에 직결 |

**솔버 내부 변경:**
- `min_speed=0` 허용 (recovery, observe, overtake 공통 OK). kinematic degeneracy는 아래 R4로 방어.
- `v_bias_max` 가 큰 값이 되도록 cfg 재조정, 혹은 `w_velocity=0`일 때 cost term 완전 skip 분기.

### 4.2 근본 infeasibility 원인 대응

| ID | 의심 원인 | 대응 |
|---|---|---|
| R1 | `Maximum_Iterations_Exceeded` | `ipopt_max_iter`: observe 500 / overtake 1000 / recovery 1500 |
| R2 | 첫 solve (warm reset 후) yaw wrap 무시 [mpcc_solver.py:223-232](../../../icra2026_ws/ICRA2026_HJ/planner/mpc_planner/src/mpcc_solver.py) | `last_solved_yaw` 별도 캐시, fresh warm-start도 unwrap |
| R3 | `min_speed=0.5` vs `ref_v<0.5` 충돌 | `min_speed=0` 전면 허용 (형상 우선이면 속도 하한이 제약이 아님) |
| R4 | `v→0`에서 `v·tan(δ)/L` degenerate | `0.001·δ²` tiny regularizer 상시 + kinematic dynamics에 `v → v + ε` (ε=0.05) 수치 안정화. `δ`의 효과가 v=0에서도 유한하게 남도록 — 주의: physical meaning 감수, 문서화 |
| R5 | lap wrap ref_s 불연속 | 회귀 테스트: wrap 경계 unit test |
| R6 | slack upper bound `1e3` 너무 커 | `slack_hi = 2.0m` cap + `w_slack=1000` 유지. 너무 커지면 오히려 cost gradient 발산 |
| R7 | 초기 상태가 코리도 바깥 (예: recovery 시작 시) | solve 전 사전 guard: 초기 pose가 모든 ref step의 코리도 polygon 밖이면 `INITIAL_OUT_OF_CORRIDOR` 상태로 **로그만 남기고 solve는 진행** — slack이 큰 값으로 흡수. 진단용. |
| R8 | obstacle cost/constraint 때문에 NLP hessian 조건수 악화 | soft 먼저, hard는 Stage-2에서만. `w_obstacle` 과도하면 수렴 느림 — 실측으로 튜닝 |

### 4.3 4단 fallback (절대 안 죽음) + warn 로그

```
tier 0  NLP success
         └→ publish solution, status=OK, save last_good

tier 1  NLP fail (streak 1..H)
         └→ publish last_good with s-shift for ego movement
            (현재 차량 이동분만큼 앞으로 shift, 끝은 raceline으로 연장)
            status=HOLD_LAST
            rospy.logwarn_throttle(0.5,
              "[MPC fallback tier1] HOLD_LAST streak=%d ipopt=%s")

tier 2  streak > H  또는 last_good 없음
         └→ publish GEOMETRIC_FALLBACK:
            Frenet quintic 으로 (s0,n0,ψ0-ψ_raceline) → (s0+Δs, 0, 0)
            곡선 1개 계산 (1ms). 21점 샘플링 후 Track3D로 3D lift.
            그 뒤는 raceline을 그대로 이음.
            항상 feasible (선형시스템이라 무조건 해).
            status=GEOMETRIC_FALLBACK
            rospy.logwarn_throttle(0.5,
              "[MPC fallback tier2] GEOMETRIC streak=%d n0=%.2f Δs=%.1f")

tier 3  GEOMETRIC_FALLBACK 계산 자체 실패 (현재 pose가 맵 밖 등)
         └→ raceline 슬라이스 그대로 발행
            status=RACELINE_SLICE
            rospy.logerr_throttle(0.5,
              "[MPC fallback tier3] RACELINE_SLICE — geometric failed, "
              "pose_out_of_map=%s")
```

`H = 5` 기본 (~166ms @ 30Hz).

**로그 설계 규칙:**
- tier 진입 시 1회 warn (또는 tier 3은 err). throttle 0.5s 로 스팸 억제.
- tier 전환(예: tier 1 → tier 2) 시 원인 필드 포함: IPOPT status, fail streak, 현재 frenet 이탈량, solve time.
- tier 0으로 복귀 시 `rospy.loginfo("[MPC fallback] recovered after %.1fs")` 발생.
- `~status` latched 토픽은 즉각 반영. rosout 로그는 보조 진단용.

**Tier 2 구현 위치:** `planner/mpc_planner/src/geometric_fallback.py` 신규 파일. Frenet quintic 계수 계산 + s 샘플링 + Track3D lift 통합.

**Tier 2 GEOMETRIC_FALLBACK 설계:**
- 현재 pose → raceline nearest 점까지 2D 곡선. 구성 후보:
  - **Clothoid / Dubins** (curvature-limited) — 품질 좋음, 계산 20ms
  - **Quintic polynomial** in frenet (`d_0 → d_1=0`, `d'=0`, `d''=0` at boundaries) — 계산 1ms, curvature 부드러움 충분
- 권장: **frenet quintic**. 계산 단순, curvature 자연 제한, Track3D로 쉽게 3D lift.
- 이 fallback은 `recovery`의 내부 동작과 원리가 동일 → 재사용 가능. 실제로 recovery state는 이 geometric primitive를 **정식 출력**으로 써도 되는가 여부는 Phase 3.3 재방문 대상.

### 4.4 warm-start 정책

- `success=True & slack_max > 0.5` : degenerate success, reset.
- state 전환 (dyn_reconfigure, reference 소스 변경) 감지 시 강제 reset.
- 연속 성공 3회 후 자유 warm-start.
- streak ≥ H 에서 fallback 발행했다면 다음 tick 전에 `reset_warm_start()`.

**수정 파일:**
- `planner/mpc_planner/src/mpcc_solver.py` (4.1 cost 재조정, R1, R2, R4, R6)
- `planner/mpc_planner/node/mpc_planner_state_node.py` (4.3 fallback, 4.4 정책)
- `planner/mpc_planner/src/geometric_fallback.py` (신규 — frenet quintic → Wpnt[])

**Verify.**
1. 합성 corridor-collapse 주입 5사이클 → HOLD_LAST 5회 → GEOMETRIC_FALLBACK 유지, 토픽 단절 0.
2. 차량을 트랙 밖 1m에 두고 recovery 기동 → GEOMETRIC_FALLBACK 즉시 발행, MPC 복귀 후 OK 전환.
3. 같은 프리셋/같은 bag에서 `w_velocity=3.0` (이전) vs `w_velocity=0.1` (신규) 궤적 비교 — 형상 차이, solve time 차이.
4. 기존 실패 bag (s≈50.6m) 재생: `ok=False` 비율 + `FALLBACK` 비율 측정.

---

## Phase 5 — dynamic_reconfigure (MPCCost.cfg)

**Goal.** 런타임에 weight 튜닝 + save/reset YAML (sampling과 동일).

**생성:**
- `planner/mpc_planner/cfg/MPCCost.cfg`
  - `w_contour, w_lag, w_velocity, v_bias_max, w_dv, w_dsteering, w_slack, boundary_inflation, max_speed, min_speed, max_steering, ipopt_max_iter, obstacle_mode(enum), w_obstacle, obstacle_sigma, save_params, reset_params`

**수정:**
- `CMakeLists.txt`, `package.xml` — `dynamic_reconfigure` depend 추가.
- `mpc_planner_state_node.py` — `Server(MPCCostConfig, self._weight_cb)` 패턴.
- `mpcc_solver.py` — `update_box_bounds(v_min, v_max, theta_max)` 추가 (NLP 재빌드 없이 lbx/ubx 갱신). `ipopt_max_iter` 변경 시는 `nlpsol` 재생성.

**참고 패턴:** [planner/3d_sampling_based_planner/node/sampling_planner_state_node.py `_weight_cb`, `_save_yaml`, `_reload_yaml`](../../../icra2026_ws/ICRA2026_HJ/planner/3d_sampling_based_planner/node/sampling_planner_state_node.py)

**Verify.**
1. rqt_reconfigure → w_contour 슬라이드 → 다음 solve에서 즉시 반영.
2. save_params 트리거 → state-specific YAML 갱신 → 재기동 후 값 유지.
3. reset_params → YAML 초기값 복귀.

---

## Phase 6 — launch 파일 (observation 모드 전용)

**Goal.** 단일/멀티 인스턴스 기동. **`3d_headtohead.launch`는 수정하지 않음** — state_machine과 완전 격리된 observation 모드로만 검증.

**생성:**
- `planner/mpc_planner/launch/mpc_planner_state.launch` — 단일 인스턴스 템플릿. arg `attach_to_statemachine:=false` (default). false면 `_observation` suffix 붙은 토픽으로 remap.
- `planner/mpc_planner/launch/mpc_planner_multi.launch` — `mpc_planner_overtake` + `mpc_planner_recovery` 두 인스턴스 include. 두 인스턴스 모두 observation 모드.
- `planner/mpc_planner/launch/mpc_planner_observation.launch` — `3d_headtohead.launch`와 **병행 실행** 가능한 관측 전용 런치. 기존 base_system/headtohead가 돌아가는 상태에서 MPC 2인스턴스를 "옆에서" 띄워 출력만 관찰.

**수정 없음.** `stack_master/launch/3d_headtohead.launch`는 **손대지 않음**. sampling 쪽 `sampling_planner_enable` 통합도 이 Phase에서 건드리지 않음.

**Verify.**
1. `roslaunch stack_master 3d_headtohead.launch` (기존 그대로) → 기존 동작 100% 유지.
2. 별도 터미널에서 `roslaunch mpc_planner mpc_planner_observation.launch` → MPC 2인스턴스 추가 기동.
3. `/planner/avoidance/otwpnts_observation`, `/planner/recovery/wpnts_observation` 발행 확인.
4. `/planner/avoidance/otwpnts` (실제) publisher가 여전히 spliner인지 체크 → MPC가 끼어들지 않았는지 확인.
5. RViz에서 MPC 관측 궤적과 실제 궤적을 동시 시각화해 형상 차이 눈으로 비교.

---

## Phase X — state_machine 직통 연결 (observation 검증 통과 후 게이트)

**Goal.** Phase 1~6 observation 모드에서 **형상 품질, fallback 안정성, solve_ms** 등이 합격점을 받은 뒤에만 진입. 이 Phase는 "구현 완성도 검증 게이트".

**진입 조건 (gate):**
1. observation 모드로 bag 재생 중 MPC가 30Hz로 끊김 없이 `_observation` 토픽 발행. Tier 3 (RACELINE_SLICE) 비율 < 1%.
2. OT 시나리오에서 `_observation` 궤적의 회피 마진이 기존 spliner 대비 동등 이상.
3. Recovery 시나리오에서 raceline 복귀 smoothness (Δκ RMS) 합격.
4. solve_ms p95 < 33ms (30Hz 실시간 경계).

**작업:**
- `mpc_planner_state.launch`의 `attach_to_statemachine` arg를 `true`로 토글 → `_observation` suffix 제거된 실제 토픽으로 remap.
- `stack_master/launch/3d_headtohead.launch` 수정:
  - arg `planner_backend` (default `legacy`) 추가.
  - `planner_backend=mpc` 일 때 `mpc_planner_multi.launch` include (attach=true), 기존 `spliner`/`recovery_spliner`는 unless.
  - `planner_backend=sampling`은 sampling 쪽 통합에 위임 (충돌 금지).
  - `### HJ :` 주석 래핑.

**Verify.**
1. `roslaunch stack_master 3d_headtohead.launch planner_backend:=legacy` → 기존 100% 유지.
2. `planner_backend:=mpc` → MPC가 spliner 대신 `/planner/avoidance/otwpnts` 발행, controller가 이를 local_wpnts로 소비.
3. 재기동만으로 legacy ↔ mpc 스왑 가능.

**롤백.** Phase X에서 문제 발견 시 arg 하나만 `legacy`로 돌리면 즉시 원복.

---

## Phase 7 — GB_TRACK / START MPC 통합 (Future Work)

스코프 밖. 기록만:
- **GB_TRACK**:
  - state_machine에 `gb_source:={raceline|mpc}` rosparam 도입.
  - `GlobalTracking` state가 `mpc`일 때 `/planner/mpc/wpnts`를 `cur_gb_wpnts` 대신 소스로 사용.
  - 전제: MPC 출력 길이/간격이 controller 기대와 호환되는지 확인 (lookahead 관점).
  - 위험: `cur_gb_wpnts` 전역 참조가 state_machine 곳곳에 퍼져 있음. `_get_gb_source()` 헬퍼 도입해 한 곳에서만 분기.
- **START**:
  - 현재 `planner/spliner/start_spline_node_v2` 대체.
  - `~state:=start` 추가. recovery와 동일하게 raceline reference + 초기 상태 (정지 pose). 단, `v_bias_max` ↓, `w_velocity` ↑ 초기 몇 스텝만 속도 ramp-up을 강제하는 별도 cost term 필요할 수 있음 (sprint 프로파일).
  - 출력 토픽: `/planner/start_wpnts` (OTWpntArray; 기존 start_planner와 동일).

---

## Phase 8 — 공통 리팩터: Track3D import 헬퍼

**Goal.** sampling과 MPC가 같은 Track3D 인스턴스를 공유할 길 열기.

**생성:**
- `planner/common/track3d_import.py` — `sys.path` 조작과 Track3D import를 한곳에서.
- 두 노드 모두 여기에서 import.

**이유:** Track3D import path가 sampling 패키지 내부를 가리키고 있어 두 노드가 같은 hack을 중복. 유지보수성 향상. 이것만 따로 Phase로 둔 이유는 두 패키지 모두 수정이라 리스크 격리.

**Verify.** 두 노드 모두 기동 OK, 로드된 Track3D 인스턴스 동일성 확인은 불필요 (별도 인스턴스여도 무방, 코드 중복만 제거가 목적).

---

## Phase 9 — 비교 protocol & 스크립트 (future, 선택)

**Goal.** 동일 bag을 backend 스왑만 해서 세 번 재생하고 메트릭 CSV로 저장.

**생성:**
- `planner/mpc_planner/scripts/compare_backends.py` — 관측 토픽들을 구독, 메트릭 계산, CSV 저장.
- `planner/mpc_planner/launch/compare_run.launch`

**메트릭 후보:**
- solve_ms (p50/p95)
- success rate
- trajectory smoothness (Δκ RMS)
- lateral oscillation RMS (동일 s_m 기준 tick-to-tick)
- corridor margin (min, mean)
- obstacle clearance (perception 붙었을 때)
- lap time (odom 적분)

---

## Risk & Feasibility (Top 3)

1. **Obstacle 주입으로 NLP 조건수 악화.** soft repulsive cost가 exp-Gaussian이라 hessian 조건수가 obstacle 근접 시 나빠짐. `w_obstacle` 너무 크면 수렴 느려짐. → Stage-1부터 중간값 (`w_obstacle=500`, `σ=0.35m`) 으로 시작, solve time p95 모니터링 → 튜닝.
2. **`min_speed=0` + `v→0` kinematic degeneracy.** tiny `δ²` regularizer + `v+ε` 수치 안정화로 방어. 실측 후 불안정하면 recovery에만 `min_speed=0.2` 강제 복귀 (정지 포기).
3. **"절대 안 죽음" 4단 fallback의 복잡도.** 4층이면 버그 가능성 ↑. → 단위 테스트로 각 tier 독립 커버: Tier 1 (fail 1회 → hold), Tier 2 (fail streak → geometric), Tier 3 (geometric 실패 → raceline slice). 실패 주입 테스트 bag 준비.

---

## 결정 사항 요약 (모든 오픈 이슈 해결)

- ~~**Q-A.**~~ **결정됨**: **Stage-1 (soft repulsive cost) → Stage-2 (hard constraint) 단계적 도입**. Stage-1만으로 "합리적 회피 + 절대 안 죽음" 충족 후 CDF 측정으로 Stage-2 필요성 판단.
- ~~**Q-B.**~~ **결정됨** (SQP `planner/sqp_planner/src/sqp_avoidance_node.py:88-99` 확인): `/opponent_prediction/obstacles` (ObstacleArray) + `/opponent_trajectory` (OpponentTrajectory) + `/tracking/obstacles` (ObstacleArray). fusion = prediction 주 + tracking fallback + 100ms staleness (SQP 대비 개선점).
- ~~**Q-C.**~~ **결정됨**: **`N_obs_max = 2`** (1vs1 + grouping 여유 1). NLP 파라미터 차원 최소화.
- ~~**Q-D.**~~ **결정됨**: **원본 보존 + 신규 `mpc_planner_state_node.py` 추가**. 원본은 observe 용으로 그대로 유지, 롤백 용이.
- ~~**Q-E.**~~ **결정됨**: **`w_velocity = 0.1`** (tiny anchor). velocity는 dynamics/smoothness 주도, degeneracy 리스크 감소.
- ~~**Q-F.**~~ **결정됨**: GEOMETRIC_FALLBACK (Frenet quintic) 채택 + 각 tier 진입 시 warn 로그 동반.
- ~~**Q-G.**~~ **결정됨**: Recovery 구조 C — 평시 MPC 직접, Tier 2로 자동 quintic fallback.

---

## 구현 순서 (체크리스트)

**Observation 모드 (state_machine 격리):**
- [ ] Phase 0: 백업 파일 복사
- [ ] Phase 1: `mpc_planner_state_node.py` + `MPCCost.cfg` + state launch 템플릿 (**`_observation` 토픽**)
- [ ] Phase 2: `mpc_track3d_lifter.py` + Wpnt 3D 필드 채움
- [ ] Phase 3: state별 reference 슬라이서 + 프리셋 YAML 3개
- [ ] Phase 3.5: OT obstacle soft repulsive cost 주입 (Stage-1)
- [ ] Phase 4: feasibility 하드닝 + 4단 fallback
- [ ] Phase 5: dynamic_reconfigure 서버 + save/reset
- [ ] Phase 6: observation launch 구성 (**`3d_headtohead.launch`는 손대지 않음**)

**검증 게이트:**
- [ ] Phase 9 (권장): observation 모드로 메트릭 CSV 수집해 합격선 확인

**state_machine 연결 (게이트 통과 후):**
- [ ] Phase X: `attach_to_statemachine:=true` + `3d_headtohead.launch`에 `planner_backend` arg

**옵션/미래:**
- [ ] (옵션) Phase 3.5 Stage-2: OT hard collision constraint
- [ ] (옵션) Phase 8: Track3D import 공통 헬퍼
- [ ] (미래) Phase 7: GB_TRACK / START MPC 통합

---

## Critical 파일 목록

**생성:**
- `planner/mpc_planner/node/mpc_planner_state_node.py`
- `planner/mpc_planner/src/mpc_track3d_lifter.py`
- `planner/mpc_planner/src/geometric_fallback.py`
- `planner/mpc_planner/cfg/MPCCost.cfg`
- `planner/mpc_planner/config/state_{observe,overtake,recovery}.yaml`
- `planner/mpc_planner/launch/mpc_planner_state.launch` (observation 모드 기본)
- `planner/mpc_planner/launch/mpc_planner_multi.launch` (observation 모드 기본)
- `planner/mpc_planner/launch/mpc_planner_observation.launch` (병행 실행용)

**수정:**
- `planner/mpc_planner/src/mpcc_solver.py` (Phase 3.5, 4, 5)
- `planner/mpc_planner/CMakeLists.txt`, `package.xml` (dynamic_reconfigure)
- `stack_master/launch/3d_headtohead.launch` — **Phase X (게이트 후)에서만** 수정

**보존:**
- `planner/mpc_planner/node/mpc_planner_node.py` (observe 원본)
- `planner/mpc_planner/src/mpcc_solver_original.py`
- `planner/mpc_planner/config/mpcc_params.yaml` + `_original.yaml`

---

## Glossary (개념 풀어쓰기)

### Frenet 좌표계
트랙 중심선/레이스라인을 기준축으로 한 2D 좌표. `s` = 트랙 따라 진행 거리 (arc-length), `n` (또는 `d`) = 중심선 기준 수직 방향 이탈. 예: "차가 중심선에서 왼쪽 1m 벗어나 있다" = `n=+1.0`.

### Frenet Quintic 복귀 곡선 (Phase 4 Tier 2 GEOMETRIC_FALLBACK의 정체)

**정의:** `n(s) = c0 + c1·s + c2·s² + c3·s³ + c4·s⁴ + c5·s⁵` — 5차 다항식.

**왜 5차?** 출발점 `(s=s0)` 에서 `(n0, n0', n0'')` 3개, 도착점 `(s=s0+Δs)` 에서 `(0, 0, 0)` 3개 → 총 6개 boundary condition → 계수 6개 필요. 6×6 선형시스템 solve → ~1ms.

**그래서 어떻게 쓰는가:**
1. 현재 차의 frenet `(s0, n0)` 계산 (이미 `_cart_to_cl_frenet_exact` 가용).
2. 목표: `Δs` 앞 (예: 5~10m) 에서 `n=0` (raceline 위).
3. `n(s)` 계산 → s축에 21점 샘플링 → `(s, n)` 배열 → Track3D.sn2cartesian → `(x,y,z)` → Wpnt[] 만들어 발행.

**특성:**
- **항상 feasible** (선형시스템이라 무조건 해 나옴).
- **곡률 유한** (boundary `n''=0` 조건 덕에 양끝 매끄러움).
- **차 현재 위치에서 시작**하므로 컨트롤러가 받았을 때 위치 jump 없음.
- Frenet에서 정의되므로 3D 트랙에 그대로 올라가고 z는 자연 추종.

**왜 "합리적"인가:** MPC가 실패했다고 raceline을 그대로 던져버리면, 차가 raceline에서 1m 떨어져 있을 때 발행 궤적의 첫 점이 1m 떨어진 곳이라 controller 입장에서 "어디서 시작하라는 거지?" 상태. Quintic 복귀 곡선은 **"현재 위치에서 raceline까지의 다리"**를 즉석에서 놓아줌.

### state별 GEOMETRIC_FALLBACK 동작

| state | 평상시 (Tier 0) | 연속 실패 시 (Tier 2) |
|---|---|---|
| observe | MPC 해 | Quintic 복귀선. obstacle 없는 모드라 안전 |
| overtake | MPC 해 (obstacle 회피 포함) | Quintic 복귀선 (obstacle 무시!) — 위험. MPC가 수 tick 연속 실패한 극단 상황의 차선책. state_machine이 TRAILING/FTGONLY로 빠져나갈 시간 벌어주는 용도 |
| recovery | MPC 해 (복귀선 직접) OR Quintic 해 (primitive) | 어느 쪽이 "평상시"인지가 Q-G |

### Recovery 구조 3안 (Q-G 선택지)

**A. MPC 직접 생성.** 평상시에도 NLP가 복귀선 계산. MPC 테스트 목적에 충실. solve ~15ms, 가끔 infeasible.

**B. Quintic primitive가 주, MPC는 smoothing.** 평상시에도 Quintic으로 뼈대 뽑고 MPC는 steering rate 등 제약 맞춰 refine. 절대 안 죽음 강력. MPC "스스로 생각" 측정은 약해짐.

**C. 평시 A, fallback이 필요한 순간 B (Tier 2로 자동).** plan 기본 제안. recovery의 평상시는 MPC 직접(A), solve 실패 N회 누적 시 quintic이 자동 개입(Tier 2).

---

## 검증 end-to-end 시나리오

```bash
# 빌드
docker exec icra2026 bash -c \
  "source /opt/ros/noetic/setup.bash && \
   source /home/unicorn/catkin_ws/devel/setup.bash && \
   cd /home/unicorn/catkin_ws && catkin build mpc_planner"

# T1: localization
roslaunch glim_ros glil_cpu.launch

# T2: base system (global_waypoints 제공)
roslaunch stack_master 3d_base_system.launch

# T3: 기존 state_machine + spliner/recovery_spliner 기동 (그대로)
roslaunch stack_master 3d_headtohead.launch

# === OBSERVATION 모드 (Phase 1~6) ===
# T4-a: MPC observe 단독 (원본 회귀 + _observation 토픽)
roslaunch mpc_planner mpc_planner_state.launch state:=observe

# T4-b: MPC overtake 단독 (raceline ref + obstacle 주입)
roslaunch mpc_planner mpc_planner_state.launch state:=overtake

# T4-c: MPC recovery 단독
roslaunch mpc_planner mpc_planner_state.launch state:=recovery

# T4-d: MPC 2인스턴스 병행 관측 (T3와 동시 실행)
roslaunch mpc_planner mpc_planner_observation.launch

# === state_machine 연결 (Phase X, 게이트 통과 후만) ===
# T5: backend 통합 — 기존 spliner 대체
roslaunch stack_master 3d_headtohead.launch planner_backend:=mpc

# RViz:
#   observation 모드 → /planner/avoidance/otwpnts_observation,
#                      /planner/recovery/wpnts_observation,
#                      /planner/mpc/best_trajectory_observation
#   attached 모드    → /planner/avoidance/otwpnts, /planner/recovery/wpnts
# rqt_reconfigure: /mpc_planner_overtake, /mpc_planner_recovery
```

각 단계에서 `~status` latched 토픽을 `rostopic echo -n1`로 확인하면 현재 상태(OK/HOLD_LAST/GEOMETRIC_FALLBACK/RACELINE_SLICE) 즉시 가시화.

**관측 포인트:**
- observation 모드 동안 `/planner/avoidance/otwpnts` publisher가 여전히 `spliner_node`인지 확인 (`rostopic info`) — MPC가 끼어들지 않았다는 증거.
- `_observation` 토픽과 실제 토픽을 RViz에서 서로 다른 색으로 중첩 시각화해 형상 차이 비교.
- Phase 9 메트릭 스크립트로 CSV 로깅, Phase X 진입 전 합격선 체크.
