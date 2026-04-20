# MPC Planner 재설계 — Frenet Kinematic MPC + External Side Decider

- **작성일**: 2026-04-20
- **상태**: 코드 작성 완료 + Docker 빌드 성공 (`catkin build mpc_planner` OK)
  - **Live-test 미완**: 실제 노드 기동해서 tick_json으로 검증은 아직. 다음 세션 첫 할 일.
- **백업 규칙**: `_backup_20260420` 접미사로 원본 보존, 롤백 가능
- **비상 문서화 사유**: 워크스페이스 유실 가능성 → 세션간 맥락 상속을 위해 작성

---

## 다음 세션 첫 10분 복귀 가이드

1. **이 문서 + [TODO_HJ.md](../TODO_HJ.md) + `CLAUDE.md` 읽기** (3분)
2. **컨테이너에서 빌드 최신화** (1분)
   ```bash
   docker exec icra2026 bash -c "source /opt/ros/noetic/setup.bash && source /home/unicorn/catkin_ws/devel/setup.bash && cd /home/unicorn/catkin_ws && catkin build mpc_planner"
   ```
3. **기동 (호스트 Gazebo가 이미 떠 있다고 가정)** (2분)
   ```bash
   # 터미널 A: 3D base
   roslaunch stack_master 3d_base_system.launch map:=gazebo_wall_2 sim:=true
   # 터미널 B: controller + state_machine (overtake 백엔드 OFF)
   roslaunch stack_master 3d_headtohead.launch dynamic_avoidance_mode:=NONE
   # 터미널 C: 새 MPC (frenet_kin 기본)
   roslaunch mpc_planner mpc_planner_state.launch state:=overtake
   ```
4. **Claude가 background Bash에서 tick_json 모니터** (4분)
   ```bash
   docker exec icra2026 bash -c "source /opt/ros/noetic/setup.bash && source /home/unicorn/catkin_ws/devel/setup.bash && rostopic echo /mpc_overtake/debug/tick_json > /tmp/mpc_tick.yaml"
   ```
   15~30초 수집 후 파이썬으로 파싱, 합격선 확인:
   - `margin_L_min ≥ 0.15m` (wall_safe 0.15 + inflation 0.05 → 실제 > 0.20 기대)
   - `jitter_rms p95 ≤ 0.05m`  (이전 이슈 0.242 → 10배 개선 목표)
   - `side` 필드가 `left|right|trail|clear` 중 안정적 (5-tick hysteresis)
   - `u0.v ≈ ego.v` (±0.5m/s). 이전은 6.7 vs 3.5로 심각 괴리
   - `tier=0` 유지, `ipopt_status` ≈ Solve_Succeeded 대부분

---

## 왜 재설계했나 (271-tick 진단 요약)

이전 백엔드(`frenet_d_solver`, n(s)-only 교란) 라이브 샘플 271 tick 분석 결과:

| metric | value | 기준치 | 판정 |
|---|---|---|---|
| `margin_L min` | **-0.000** | ≥0.15 | **벽 hugging, wall_safe 미적용** |
| `jitter_rms p50 / p95` | 0.116 / 0.242 m | ≤0.05 | **심각한 tick간 진동** |
| `kappa_max max` | 2.27 | <2.0 | 경계 infeasible |
| `u0.v p50` vs `ego.v p50` | 6.7 vs 3.5 | 차이<0.5 | **컨트롤러 따라오지 못함** |
| `tier=0` | 100% | — | 솔버는 안정, 출력 품질이 나쁨 |

**진단**: n(s) 교란만 하는 구조는 (1) 벽 제약을 cost로 넣어 wall_safe 마진을 보장 못 함, (2) 진짜 차량 동역학 없이 smoothness를 cost로 억누르니 tick마다 점프, (3) side 결정이 Gaussian cost 내부에 숨어 있어 isotropic → 좌/우 오실레이션.

---

## 재설계 — 핵심 선택들

### 1. State/Dynamics (Liniger 2015 Frenet kinematic bicycle)
- 상태: `x = [n, mu, v]`  — frenet 횡오프셋 / heading vs tangent / 속도
- 제어: `u = [a, delta]`
- 이산 동역학 (dT 고정):
  ```
  n_{k+1}  = n_k  + v_k * sin(mu_k)                       * dT
  mu_{k+1} = mu_k + (v_k/L * tan(delta_k)
                     - kappa_ref_k * v_k * cos(mu_k))     * dT
  v_{k+1}  = v_k  + a_k                                   * dT
  ```
- **중요**: `kappa_ref_k`는 `ref_slice.kappa_ref[k]`에서 받음 (센터라인 곡률 → frenet kinematic에서 mu dot의 커플링 항). 이게 있어야 3D 센터라인 따라가는 feasibility가 살아남.
- Station `s = ref_slice.ref_s[k]` 고정 그리드 사용. 솔버 내부는 **xy를 절대 터치하지 않음** (CLAUDE.md 메모리: 3D 오버패스 층 aliasing 버그).

### 2. Cost (4항 + progress + slack)
```
J = q_n  · Σ n_k²                    contour
  + r_a  · Σ (Δa)²                   accel smooth
  + r_d  · Σ (Δdelta)²               steer smooth
  + r_r  · Σ delta_k²                steer zero-bias
  - γ    · Σ v_k · cos(mu_k) · dT    progress (maximize)
  + w_s  · Σ slack_k²                corridor slack penalty
```
- 진동 억제는 **Δu 스무스니스**에서 나오도록 → n(s)-only 시대의 점프는 원천 차단.
- 추월 유인은 `-γ · progress`로 **자연스럽게**. 기존처럼 장애물을 repulsive Gaussian으로 박지 않음.

### 3. Hard Constraints (cost 아님)
- **Corridor**: `n_k ∈ [-d_right + inflation + wall_safe, d_left - inflation - wall_safe]`  — slackable (slack은 비상용 여유, 큰 페널티).
- **Obstacle half-plane** (side decider 결과로 스위칭):
  - `SIDE_LEFT` : `n_k ≥ n_obs_k + gap_lat` for k in [k_obs_start, k_obs_end]
  - `SIDE_RIGHT`: `n_k ≤ n_obs_k - gap_lat`
  - `SIDE_TRAIL`: k-별 v 상한 캡 (ego가 뒤에 따라붙도록)
  - `SIDE_CLEAR`: 제약 없음
- Input/state 상한: v ∈ [v_min, v_max], a ∈ [a_min, a_max], delta ∈ [-δ_max, +δ_max], mu ∈ [-μ_max, +μ_max].

### 4. External Side Decider (NLP 바깥)
`planner/mpc_planner/src/side_decider.py`:
- Isotropic cost의 좌/우 ambiguity를 없애기 위해 **규칙 기반 + hysteresis**.
- Inputs per tick: 각 장애물의 `s0, n0, v_s_obs, half_width, d_L, d_R, ref_v`
- 규칙 (우선순위):
  1. `dv = ref_v - v_obs < trail_dv_thresh (0.5)` → **TRAIL** (ego가 충분히 빠르지 않음)
  2. `max(d_free_L, d_free_R) < 0` → TRAIL (어느 쪽도 안 들어감)
  3. `d_free_L ≥ d_free_R` → LEFT
  4. else → RIGHT
- Hysteresis: 새 side는 `hold_ticks=5` 동안 유지돼야 전환. 오실레이션 차단.

### 5. Debug pipeline (live rostopic echo 운용)
- `~debug/tick_json` (`std_msgs/String`) — tick마다 1줄 JSON. 필드: ego/u0/ref/traj/obs/side/cost/weights/constraints/jitter_rms/tier/ipopt_status 등.
- `~debug/markers` (`MarkerArray`) — RViz 시각화. corridor L/R LINE_STRIP, obstacle SPHERE_LIST+head ball, ref-slice centerline, `TEXT_VIEW_FACING` 상태판(tier/side/solve_ms 등).
- 운영 원칙 (CLAUDE.md 신규 섹션): Claude가 본인 background Bash에서 `rostopic echo`로 실시간 모니터링 → 수치 근거로 튜닝.

---

## 이번 세션에서 손댄 파일 (HJ 세션 스코프)

### 신규
- `planner/mpc_planner/src/frenet_kin_solver.py` ★ 새 MPC 솔버 본체 (~330 lines)
- `planner/mpc_planner/src/side_decider.py` ★ 외부 side decider (~110 lines)
- `planner/mpc_planner/src/frenet_d_solver.py` — 직전 세션의 n(s) 교란 솔버 (이번 세션에 추가된 게 아니라 untracked 상태). 레퍼런스/롤백용.
- `HJ_docs/mpc_redesign_frenet_kin_20260420.md` (이 문서)
- `HJ_docs/mpc_planner_state_machine_integration.md` (세션 초반 상태머신 통합 플랜)

### 백업 (롤백 포인트, `_backup_20260420` 접미사)
- `planner/mpc_planner/src/frenet_d_solver_backup_20260420.py`
- `planner/mpc_planner/src/mpcc_solver_backup_20260420.py`
- `planner/mpc_planner/node/mpc_planner_state_node_backup_20260420.py`
- `planner/mpc_planner/config/state_overtake_backup_20260420.yaml`
- `planner/mpc_planner/config/state_observe_backup_20260420.yaml`
- `planner/mpc_planner/config/state_recovery_backup_20260420.yaml`
- `planner/mpc_planner/launch/mpc_planner_state_backup_20260420.launch`

### 수정
- `planner/mpc_planner/node/mpc_planner_state_node.py` — **메인 통합 지점**
  - `FrenetKinMPC`, `SideDecider`, `SIDE_*` import
  - `solver_backend=frenet_kin`을 **새 기본**
  - 파라미터 로딩 추가 (q_n, gamma_progress, r_a, r_delta, r_steer_reg, gap_lat, gap_long, mu_max, a_min, a_max, side_hold_ticks, ego_half_width, trail_dv_thresh)
  - `_slice_local_ref`가 `kappa_ref`, `ref_psi` 필드를 반환 (frenet kin dynamics에 필요)
  - `_plan_loop`에 `frenet_kin` 분기: `mu0 = wrap(car_yaw - ref_psi[0])`로 초기 mu 세팅 → `_decide_side` → 솔버 호출
  - `_decide_side(obs_arr, ref_slice)` 추가 — obs_list 빌드 후 decider에 위임
  - `_lift_frenet_to_xy(traj_frenet, ref_slice)` 추가 — `center_points[k] + n_k·normal[k]`, z는 `lifter._interp(s, g_z)`. **xy round-trip 없음** (CLAUDE.md 메모리 준수, 주석 인용)
  - `pub_debug_tick` / `pub_debug_markers` 퍼블리셔 + `_publish_tick_live(fields)` + `_publish_debug_extra_markers(...)` 추가
  - `_debug_log` 맨 앞에서 `_publish_tick_live` 호출 — DebugLogger 활성/비활성과 독립적으로 매 tick 발행
  - `_last_frenet_traj` 스태시로 tick_json 정확도 보강
- `planner/mpc_planner/launch/mpc_planner_state.launch`
  - `instance_name` 기본값: `$(arg state)` → **`mpc_$(arg state)`**
  - 사유: `sampling_planner_state.launch`도 기본 `/overtake`라 충돌 (`new node registered with same name` 발생). 단독 기동 시 `/mpc_overtake`로 격리.
  - `solver_backend` arg 추가, 기본 `frenet_kin`
- `CLAUDE.md` — "⏱ 구현 속도" 및 "🖥 디버깅은 Claude가 직접 터미널에서 실시간 rostopic echo" 섹션 추가

### 세션 스코프 **외** (다른 세션/IY/공유 — 커밋 시 제외)
- `f110_utils/*`, `perception/2.5d_detection` (submodule), `state_estimation/*` (submodule), `stack_master/*` 거의 전부, `planner/3d_sampling_based_planner/*`, `planner/overtaking_iy/*`, `IY_docs/*`, 다수 맵/파라미터 yaml

---

## 3D 안전성 (CLAUDE.md 메모리 준수)

> 3D 트랙에서 Frenet xy 라운드트립 금지 — 오버패스 층 구분 실패 버그

이번 재설계에서 지킨 것:
- **솔버 내부는 순수 frenet** (`n`, `mu`, `v` 만 다룸). xy, z 없음.
- `_lift_frenet_to_xy`는 **프리컴퓨트된** `ref_slice.center_points`, `ref_dx/ref_dy`, `g_z` interp만 사용 → 솔버 결과 `n_k`를 접선/법선로 해서 xy로 역변환만 함. 재진입해서 `get_frenet_3d` 같은 거 **안 부름**.
- `kappa_ref`, `ref_psi`, `d_left/d_right`도 전부 ref_slice가 한 번 뽑은 값만 사용.

---

## 파라미터 (기본값 → yaml 튜닝 지점)

| name | default | 설명 |
|---|---|---|
| `N` | 20 | horizon step |
| `dT` | 0.05 | step 시간 (50ms) |
| `vehicle_L` | 0.33 | wheelbase |
| `q_n` | 3.0 | contour weight |
| `gamma_progress` | 10.0 | progress reward |
| `r_a` | 0.5 | Δa 스무스니스 |
| `r_delta` | 5.0 | Δδ 스무스니스 |
| `r_steer_reg` | 0.1 | δ zero-bias |
| `w_slack` | 2000.0 | corridor slack penalty |
| `v_min / v_max` | 0.5 / 8.0 | |
| `a_min / a_max` | -4.0 / 3.0 | |
| `delta_max` | 0.6 rad | |
| `mu_max` | 0.9 rad | |
| `inflation` | 0.05 | 차체 반 폭 infl |
| `wall_safe` | 0.15 | 벽에서 추가 여유 |
| `gap_lat` | 0.25 | 장애물 측면 여유 |
| `gap_long` | 0.8 | 장애물 길이방향 여유 |
| `side_hold_ticks` | 5 | hysteresis |
| `trail_dv_thresh` | 0.5 | ego가 이거 이상 빨라야 추월 고려 |

튜닝 순서 (라이브 숫자 기준):
1. `margin_L_min < 0.15` → `wall_safe` 0.15 유지, `inflation` 올리거나 corridor feasibility 확인
2. `jitter_rms p95 > 0.05` → `r_a`, `r_delta` 상향
3. `u0.v vs ego.v` 괴리 → `v_max`를 현실 컨트롤러 capable로 낮추거나 `gamma_progress` 하향
4. IPOPT iter_count 상한 근접 → warm-start seed 품질 점검 (`_seed_warm_start`)

---

## 알려진 위험 / TODO

- **Live-test 미완**. 첫 solve가 infeasible로 떨어질 가능성 있음 — warm-start 초기 시드 (`self._warm_X = None` 상태) 처리 동선 재확인 필요.
- `side=SIDE_TRAIL`일 때 per-k v cap 계산식이 `ref_v`에만 의존 — 장애물 가속/감속 예측 반영은 미포함 (GP predictor 연결은 다음 라운드).
- `_lift_frenet_to_xy`의 z가 ref_slice s-grid interp에 정밀 의존 — s-grid가 coarse하면 꺾임에서 z가 계단식이 될 수 있음. 필요 시 cubic interp로 업그레이드.
- State=`observe`/`recovery`도 같은 프레임 돌지만 role output만 다름. 라이브에서 세 state 모두 한 번씩 돌려볼 것.
- `_observation` suffix 유지 중 (Phase X gate 전). metric 합격 후 사용자 승인 받고 real topic으로 연결.

---

## 다음 세션 Commit 가이드 (IMPORTANT)

이 md와 위 세션 파일들은 **이번 세션 중 커밋 전에 다른 세션이 먼저 커밋**하기로 했음 (사용자 요청). 다른 세션 커밋 완료되면 HJ 세션 파일만 선별 스테이징:

```bash
# HJ 세션 스코프 — 정확히 이 경로들만
git add CLAUDE.md
git add planner/mpc_planner/node/mpc_planner_state_node.py
git add planner/mpc_planner/launch/mpc_planner_state.launch
git add planner/mpc_planner/src/frenet_kin_solver.py
git add planner/mpc_planner/src/side_decider.py
git add planner/mpc_planner/src/frenet_d_solver.py
git add planner/mpc_planner/src/frenet_d_solver_backup_20260420.py
git add planner/mpc_planner/src/mpcc_solver_backup_20260420.py
git add planner/mpc_planner/node/mpc_planner_state_node_backup_20260420.py
git add planner/mpc_planner/config/state_observe_backup_20260420.yaml
git add planner/mpc_planner/config/state_overtake_backup_20260420.yaml
git add planner/mpc_planner/config/state_recovery_backup_20260420.yaml
git add planner/mpc_planner/launch/mpc_planner_state_backup_20260420.launch
git add HJ_docs/mpc_redesign_frenet_kin_20260420.md
git add HJ_docs/mpc_planner_state_machine_integration.md

# 스테이징 검수 — HJ 외 경로 절대 섞이지 말 것
git status
git diff --cached --stat
```

커밋 메시지 (HEREDOC, Co-Authored-By 금지):
```bash
git commit -m "$(cat <<'EOF'
mpc_planner: frenet kinematic MPC + external side decider + live debug (WIP)

Redesign after 271-tick diagnosis: n(s)-only backend failed wall_safe/jitter/
speed-tracking. New solver = Liniger frenet kinematic bicycle with hard
corridor + half-plane obstacle constraints. Side-of-passing moved OUT of NLP
(rule-based + 5-tick hysteresis). Per-tick debug (~debug/tick_json,
~debug/markers) for live rostopic echo loop.
EOF
)"
```

푸시 전 delta 확인:
```bash
git branch -vv
git log origin/main..HEAD --oneline
git push origin main   # force 계열 금지
```
