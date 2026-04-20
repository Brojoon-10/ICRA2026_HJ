# MPC Frenet Kinematic — v3c live debugging 결과 (2026-04-21)

- **작성일**: 2026-04-21
- **상태**: v3c 핵심 3 fix + RViz tier-color + **HSL ma27 + CasADi JIT (solve_ms 누적 ~5× 가속, p50 9.3ms)** 적용, live 검증 통과 (solver_pass=1: 1997/1997, p99=32.5ms)
- **선행 문서**: [mpc_redesign_frenet_kin_20260420.md](mpc_redesign_frenet_kin_20260420.md) (v1~v3b)
- **남은 TODO**: obstacle cost의 차폭/장애물폭 body-edge 거리 기반 재정의 (본 문서 §5)

---

## 다음 세션 첫 10분 복귀 가이드

1. 빌드 / 런치
   ```bash
   docker exec icra2026 bash -c "source /opt/ros/noetic/setup.bash && source /home/nuc3/catkin_ws/devel/setup.bash && cd /home/nuc3/catkin_ws && catkin build mpc_planner"
   roslaunch stack_master 3d_base_system.launch map:=gazebo_wall_2 sim:=true
   roslaunch stack_master 3d_headtohead.launch dynamic_avoidance_mode:=NONE
   roslaunch mpc_planner mpc_planner_state.launch state:=overtake
   ```
2. Claude-side live 모니터 (background bash):
   ```bash
   docker exec icra2026 bash -c "source /opt/ros/noetic/setup.bash && source /home/nuc3/catkin_ws/devel/setup.bash && ROS_MASTER_URI=http://192.168.70.2:11311 ROS_HOSTNAME=nuc3 rostopic echo -p /mpc_overtake/debug/tick_json > /tmp/mpc_tick.log"
   ```
3. 30~60초 후 집계:
   ```bash
   docker cp /tmp/analyze_mpc2.py icra2026:/tmp/  # 본 repo의 /tmp 스크립트
   docker exec icra2026 bash -c "tail -n +2 /tmp/mpc_tick.log | cut -d',' -f2- | python3 /tmp/analyze_mpc2.py"
   ```
4. 합격선 (v3c 기준):
   - `solver_pass` 분포: `{1: ≥99%}` — retry(2) / fail(0) 거의 없어야 함
   - `solve_ms` p99 < 100ms, max < 150ms
   - `predict_age_s` p99 < 100ms (subscriber 독립 스레드 건강)
   - RViz 궤적: **빨간색**이 기본. 노랑/주황/초록 이상 뜨면 어느 tick에 무슨 input이었는지 `solver_input` / `solver_infeas` 필드로 역추적

---

## 1. v3b → v3c 에서 무엇을 고쳤나

### 1.1 병증 (v3b live, 2026-04-21 17:00 capture)
- 1185 tick 중 17 tick이 `solver_pass=3` (pass 1 → 2 → 3 체인으로 간신히 "성공")
- 17 tick 전부 `solver_pass_hist = [{pass:1,Infeasible,worst:v_over_max}, {pass:2,Infeasible,worst:v_over_max}, {pass:3,Solve_Succeeded}]`
- `solve_ms` p99 = **266ms**, max = **593ms** (plan loop 주기 33ms의 ~18배) → RViz "뚝뚝"
- `jitter_rms_m` p95 = 0.27m, max = 1.57m (target <0.10)

### 1.2 뿌리 원인 — 솔버는 죽은 게 아니라 **불가능한 문제를 받고 있었다**
- 동역학 등식: `v_[0] == P_v0 = 3.567 m/s` (현재 차량 속도는 못 바꿈)
- 하드 바운드: `v_[0] ≤ P_vmax[0]`
- `_build_vmax`가 SIDE_TRAIL일 때 **k=0부터 N까지 동일한 cap**을 적용: `cap = v_obs_s × 0.95 = 3.35 m/s`
- 결과: `3.567 ≤ 3.35` 제약 → 수학적 infeasible → IPOPT `Infeasible_Problem_Detected` (200 iter 소진 후 포기)
- Pass 3(옛 구현)가 obs 끄고 CLEAR로 cap 제거 → "성공" 찍혔지만 그 궤적은 **장애물 관통 + ref_v 그대로** 라는 최악 fallback
- 노드는 `success=True`로 보고 tier1/2/3 recovery ladder 진입조차 안 함 → 사용자 직감대로 **fallback이 작동 안 하는 것처럼 보인** 현상

### 1.3 v3c fix 3종
1. **Remove `v_[0] ≤ P_vmax[0]` (k=0 vmax bound 스킵)**
   - `v_[0] == P_v0` 가 이미 지배. 중복 + 충돌 유발.
   - [frenet_kin_solver.py:256-266](../planner/mpc_planner/src/frenet_kin_solver.py#L256-L266)
2. **`_build_vmax`에 감속 ramp 실장**
   - 이전: `vmax[:]` 전부 `cap`으로 uniform → v0 > cap이면 구조적 infeasible.
   - 이후: `vmax[k] = max(cap, v0 − k · dT · a_dec_ramp)`, `a_dec_ramp=3.0 m/s²`
   - `a_dec_ramp`는 `a_min=-4.0` 내부 (strictly feasible). k가 커질수록 cap에 수렴.
   - [frenet_kin_solver.py:396-440](../planner/mpc_planner/src/frenet_kin_solver.py#L396-L440)
3. **Pass 3 제거 — 2-pass ladder로 단순화**
   - pass 1: caller-side, obstacles on
   - pass 2: SIDE_TRAIL 강제 (이젠 ramp 덕분에 feasible), obstacles on
   - 둘 다 실패 시 `success=False` 정직 반환 → 노드 tier1 HOLD_LAST → tier2 GEOMETRIC → tier3 CONVERGENCE_QUINTIC → tier3 RACELINE_SLICE (사용자 지정 recovery 스타일)
   - [frenet_kin_solver.py:463-505](../planner/mpc_planner/src/frenet_kin_solver.py#L463-L505)

### 1.4 Before vs After (같은 시나리오 gazebo_wall_2, sim, overtake)

| metric | v3b | v3c | 개선 |
|---|---|---|---|
| total ticks | 1185 | 1997 | — |
| fail clusters (pass ≠ 1) | 17 | **0** | ✅ |
| solver_pass dist | `{1:1168, 3:17}` | `{1:1997}` | ✅ |
| ipopt_status | 100% Solve_Succeeded (pass 3 가짜 성공 포함) | 100% Solve_Succeeded (진짜) | ✅ |
| solve_ms p50 | 47ms | 48ms | — |
| solve_ms p99 | **266ms** | **91ms** | 2.9× |
| solve_ms max | **593ms** | **125ms** | 4.7× |
| iter p99 | — | 19 | 수렴 안정 |
| predict_age_s p99 | 48ms | 39ms | — (subscriber 독립성 유지 확인) |
| RViz stutter | "뚝뚝" 체감 | 매끄러움 | ✅ |

---

## 2. RViz marker 색깔 매핑 (v3c)

[mpc_planner_state_node.py:1784-1809](../planner/mpc_planner/node/mpc_planner_state_node.py#L1784-L1809)

| 색 | 조건 | 의미 |
|---|---|---|
| 🔴 red | `tier=0, solver_pass=1` | NLP 1-pass clean (정상) |
| 🟠 orange | `tier=0, solver_pass=2` | TRAIL retry로 겨우 성공 (경고) |
| 🟡 yellow | `tier=1` | HOLD_LAST (이전 tick 궤적 재사용) |
| 🟢 green | `tier=2` | GEOMETRIC_FALLBACK quintic Δs≈8m |
| 🩵 cyan | `tier=3 CONVERGENCE_QUINTIC` | ego_n→0 recovery quintic Δs≈4m |
| 🔵 blue | `tier=3 RACELINE_SLICE` | 마지막 보루 — raceline 슬라이스 |

각 tier 진입부에서 `self._viz_tier / _viz_status / _viz_pass`를 세팅하고, `_publish_debug_markers`가 이를 읽어 LINE_STRIP + SPHERE_LIST 색을 결정.

---

## 3. v3c 파라미터 레퍼런스 (state_overtake.yaml)

### 3.1 Horizon / Vehicle
| key | 값 | 의미 | 튜닝 노트 |
|---|---|---|---|
| `planner_freq` | 30 Hz | 메인 plan loop rate | 40ms budget → solve_ms p99=91ms 여유 |
| `N` | 20 | prediction horizon steps | |
| `dT` | 0.05 s | step duration → horizon 1.0s | |
| `vehicle_L` | 0.33 m | wheelbase | F1Tenth 기본 |

### 3.2 Kinematic limits
| key | 값 | 의미 | 노트 |
|---|---|---|---|
| `min_speed` | 0.0 | v_min (hard) | Phase 4: 형상 우선 솔버이므로 0 허용 |
| `max_speed` | 10.0 | v_max (hard) | ref_v < 10인 구간에서는 ref_v가 사실상 지배 |
| `max_steering` | 0.6 rad | δ hard bound | ±34° |
| `delta_rate_max` | 3.0 rad/s | δ̇ hard input bound | 1:10 R/C servo 보수치 |
| `a_min`, `a_max` | -4.0, 3.0 | 가속 한계 | 하드 바운드 |
| `a_dec_ramp` | 3.0 (v3c NEW) | TRAIL vmax ramp 감속 envelope | `a_min` 절대값보다 여유. 안정적 감속 |
| `mu_max` | 0.9 rad | |mu| 하드 바운드 | 역주행/스핀 방지 |

### 3.3 Cost weights (shape-first, Phase 4)
| key | 값 | 의미 | 튜닝 노트 |
|---|---|---|---|
| `q_n` | 3.0 | n^2 contour weight | centerline 복귀 세기 |
| `gamma_progress` | 10.0 | progress pull (-v cos mu) | 전진 유도 |
| `r_a` | 0.5 | Δa smoothness | |
| `r_delta` | 1.5 | δ̇ smoothness | v3: 5→1.5, 형상 부드럽게 |
| `r_steer_reg` | 0.1 | δ magnitude regulariser | tiny |
| `r_dd` | 5.0 | steer-rate penalty (input) | κ(s) C^0 |
| `r_dd_rate` | 1.0 | steer-jerk penalty | κ(s) C^1 |
| `q_n_term` | 10.0 | n[N]² 종단 비용 | raceline 복귀 |
| `q_v_term` | 0.5 | (v[N]-ref_v)² 종단 비용 | 종단 속도 맞춤 |
| `w_velocity` | 0.1 | v_k ref tracking | Phase 4 tiny anchor |
| `v_bias_max` | 3.0 | legacy MPCC only | |

### 3.4 Continuity / Smoothness (v3 신설)
| key | 값 | 의미 | 튜닝 이력 |
|---|---|---|---|
| `w_cont` | 200.0 | n_k ← n_prev 연속성 cost | v3b 20→200 (×10). jitter p95 0.28→0.05 목표 |
| `side_hold_ticks` | 10 | LEFT↔RIGHT 플립 지연 | |
| `trail_entry_ticks` | 3 | TRAIL 진입 빠른 게이트 | "못 갈 것 같으면 바로 감속" |
| `min_pass_margin` | 0.10 m | side_decider feasibility 잔여 마진 | 너무 좁은 slot 선택 방지 |
| `obs_ema_alpha` | 0.30 | obs s/n EMA (~3-tick τ) | predictor jitter 감쇠 |
| `trail_vel_ramp_ticks` | 8 | (node-side, 현재 미사용) | v3c 이후 solver ramp로 대체됨. 레거시 param |

### 3.5 Corridor / Wall safety
| key | 값 | 의미 | 주의 |
|---|---|---|---|
| `boundary_inflation` | 0.05 m | 벽 추정 오차 쿠션 | |
| `wall_safe` | 0.08 m | 바퀴↔벽 최소 여유 (BODY-EDGE) | 0.20m 실제 body-edge 안전은 `w_wall_buf=2500` soft cushion이 담당 |
| `w_slack` | 5000.0 | corridor 위반 soft penalty | v2c 2000→5000. obstacle cost보다 무겁게 |
| `ego_half_width` | 0.15 m (solver 기본) | F1Tenth 차 half-width | centroid bound = d_wall − (ego_half + inflation + wall_safe) = d_wall − 0.28m |
| `w_wall_buf` | 2500.0 | body-edge soft cushion | v3 400→2500. 벽 쪽은 hard+soft 둘 다 body-edge 반영 ✅ |
| `wall_buf` | 0.30 m | body-edge cushion 두께 | |

> 벽 쪽 실제 마진 요약:
> - 차 **centroid ↔ 벽** ≥ 0.28m (hard)
> - 차 **body-edge(바퀴 바깥) ↔ 벽** ≥ 0.13m (hard: inflation + wall_safe)
> - body-edge가 0.30m 안으로 들어오면 `w_wall_buf=2500` soft cushion 작동 (정상 주행 시 body-edge는 0.30m 밖 유지)

### 3.6 Obstacle cost (⚠️ body-edge/차폭 미반영 — §5 TODO)
| key | 값 | 의미 | 튜닝 노트 |
|---|---|---|---|
| `collision_mode` | soft | Gaussian bubble + side bias | |
| `w_obs` | 180.0 | obstacle bubble weight | v2c 300→180. wall cushion이 이겨야 함 |
| `sigma_s_obs` | 0.7 m | s-축 Gaussian std | ~차 2대 길이 |
| `sigma_n_obs` | 0.18 m | n-축 Gaussian std | v2a 0.30→0.18. corridor 전체 휩쓸기 방지 |
| `w_side_bias` | 25.0 | proximity-gated side bias | v2a 80→25. bias가 raceline보다 세지지 않게 |
| `gap_lat` | 0.25 m | side bias zero-crossing offset | **현재 centroid 기반**. 차폭/obs폭 미반영 |
| `gap_long` | 0.8 m | legacy gate (v2 이후 미사용) | |
| `n_obs_max` | 2 | solver NLP 슬롯 | 2 obstacle 동시 처리 |
| `w_obstacle` | 400.0 | legacy MPCC only | |

### 3.7 IPOPT
| key | 값 | 의미 | 튜닝 이력 |
|---|---|---|---|
| `ipopt_max_iter` | 200 | max iter | v3b 1000→200. 실패 시 ~40ms 컷 (이전 ~210ms) |
| `ipopt_print_level` | 0 | quiet | |
| `linear_solver` | `ma27` | IPOPT 선형 솔버 (HSL) | **v3c+ (2026-04-21)**. MUMPS→ma27 단독으로도 약 2×. libhsl.so 없을 때는 자동 mumps fallback (`_resolve_linear_solver`). |
| `ipopt_jit` | `true` | CasADi JIT (obj/constr/Jac/Hess 네이티브 C 컴파일) | **v3c+ (2026-04-21)**. ma27 위에 얹어 추가 ~2×. 첫 런치 시 10~20s 컴파일 지연 (런타임 영향 없음). `false`로 끌 수 있음. |

**apples-to-apples 누적 비교 (same Gazebo scenario, 60s)**

| metric | MUMPS (1088 ticks) | ma27 only (1416) | **ma27 + JIT (1419)** | MUMPS → 최종 |
|---|---|---|---|---|
| solve_ms p50 | 45.5 | 21.6 | **9.3** | **4.9×** |
| solve_ms p95 | 73.3 | 38.2 | **19.0** | 3.9× |
| solve_ms p99 | 91.1 | 48.6 | **32.5** | 2.8× |
| solve_ms max | 115.9 | 80.2 | **42.7** | 2.7× |
| iter p50 / p95 | 16 / 18 | 16 / 18 | 16 / 17 | 동일 (수렴 경로 불변) |
| jitter_rms p50 | 0.146 | 0.129 | 0.114 | 오히려 소폭 개선 |
| traj n range | −0.90 ~ +1.10 | −0.91 ~ +1.22 | −0.91 ~ +1.22 | ≈ 동일 |
| fails | 0 | 0 | 0 | — |

iter 수 불변 + solve_ms만 1/5 → **수치 품질 영향 없이 linear-solve + function-eval 둘 다 순수 가속**. p50 <10ms / max 42.7ms로 30Hz budget(33ms) 근접 진입.

### HSL 미설치 환경 자동 fallback
`_resolve_linear_solver`가 `linear_solver`를 검사해서 `ma*` 계열이면 `ca.__file__` 옆 `libhsl.so` 또는 `ctypes.CDLL('libhsl.so')`로 로드 가능성 검증. 실패 시 `warnings.warn(...)` + `mumps`로 자동 전환. 따라서 HSL 미설치 포크/환경에서도 **코드 수정 없이 동작**.

---

## 4. v3c 검증 루프 (재현 가능)

1. `mpc_planner_state.launch state:=overtake` 기동
2. `rostopic echo -p /mpc_overtake/debug/tick_json > /tmp/mpc_tick.log` (30~60s)
3. `/tmp/analyze_mpc2.py`로 파싱 → 핵심 필드:
   - `solver_pass` 분포: 1이 압도적
   - `solver_infeas.worst`: fails 에서 어떤 hard bound가 violated 됐는지 (v_over_max / mu_abs / delta_abs / corridor_*)
   - `solver_input.v0 / vmax_min / vmax_max`: 감속 ramp 제대로 박히는지
   - `side_scores`: d_free_L/R, can_pass_L/R, reason — SideDecider 판단 근거
   - `tier` / `_viz_tier`: 0이 대부분. 1이상이면 그 시점 RViz 궤적 색 변화로 바로 감지

---

## 5. 남은 TODO — **장애물 거리 cost의 차폭/장애물폭 반영 (P1)**

### 5.1 현재 상태 (v3c 기준)

| 영역 | half-width 반영? | 비고 |
|---|---|---|
| SideDecider feasibility | ✅ `d_free = eff_d - (n_o + w_o) - (ego_half + gap_lat)` | [side_decider.py:93-94](../planner/mpc_planner/src/side_decider.py#L93-L94) |
| Wall hard corridor | ✅ `margin = ego_half + inflation + wall_safe` | [frenet_kin_solver.py:381-386](../planner/mpc_planner/src/frenet_kin_solver.py#L381-L386) |
| Wall soft cushion | ✅ `buf_c = ego_half + wall_buf` | [frenet_kin_solver.py:316-320](../planner/mpc_planner/src/frenet_kin_solver.py#L316-L320) |
| **Obstacle bubble cost** | ❌ `P_obs_n` centroid 기반 Gaussian만. `sigma_n_obs=0.18`은 고정 std, 장애물 폭 `w_o`가 반영 안 됨 | [frenet_kin_solver.py:298-302](../planner/mpc_planner/src/frenet_kin_solver.py#L298-L302) |
| **Obstacle side bias** | ❌ `(P_obs_n ± gap_lat) - n_[k]` 힌지. `gap_lat=0.25`만, `ego_half + w_o` 미포함 | [frenet_kin_solver.py:303-306](../planner/mpc_planner/src/frenet_kin_solver.py#L303-L306) |
| **`_build_obs_params`** | ❌ `obs_arr[:,:,2]`를 active flag로만 사용, `w_o` 전달 안 됨 | [frenet_kin_solver.py:432-448](../planner/mpc_planner/src/frenet_kin_solver.py#L432-L448) |

### 5.2 운영상 버퍼 (왜 지금 문제가 안 나오는지)
- `sigma_n_obs + gap_lat ≈ 0.43m` (두 cost 메커니즘 총합) — 좁은 트랙(1.0m)에서는 센트로이드 간 거리만으로도 body-edge 안전 확보가 됨
- 하지만 **두꺼운 장애물(w_o ≥ 0.2m)** 또는 **넓은 트랙**에서는 body-edge 여유가 `0.05m` 수준으로 떨어질 수 있음
- 이론적 보장은 벽 쪽(`±0.13m body-edge hard`)보다 약함

### 5.3 제안 수정안 (한 번에 적용)

#### (1) `_build_obs_params`에 width 전달
```python
def _build_obs_params(self, obs_arr):
    N1 = self.N + 1
    s_mat = np.full((self.n_obs_max, N1), self.FAR, dtype=float)
    n_mat = np.full((self.n_obs_max, N1), self.FAR, dtype=float)
    w_mat = np.zeros((self.n_obs_max, N1), dtype=float)   # NEW
    active = np.zeros(self.n_obs_max, dtype=float)
    ...
    w_mat[o, :] = obs_arr[o, :, 3]   # half_width column (requires node populate)
    ...
    return s_mat, n_mat, w_mat, active
```

#### (2) Opti parameter 추가
```python
P_obs_w = opti.parameter(n_obs, N + 1)
```

#### (3) Bubble을 body-edge 거리 기준으로
```python
# 현재:
dy = (n_[k] - P_obs_n[o, k]) / self.sigma_n_obs
# 변경:
dn_raw = ca.fabs(n_[k] - P_obs_n[o, k])
dn_edge = ca.fmax(0.0, dn_raw - (self.ego_half + P_obs_w[o, k]))  # body-to-body gap
dy = dn_edge / self.sigma_n_obs
```
→ 두 body가 닿는 순간 cost가 peak. 떨어질수록 지수적 감쇠.

#### (4) Side bias의 zero-crossing을 body-edge + gap_lat으로
```python
margin_lat = self.ego_half + P_obs_w[o, k] + self.gap_lat
viol_L = ca.fmax(0.0, (P_obs_n[o, k] + margin_lat) - n_[k])
viol_R = ca.fmax(0.0, n_[k] - (P_obs_n[o, k] - margin_lat))
```

#### (5) 노드 쪽 `_build_obstacle_array` 업데이트
현재 `obs_arr[:,:,2] = (w * active_flag)` 로 active를 width로 표현 중. 4번째 column에 실제 half_width를 별도 저장하거나, column 3 의미를 split 해야 함. 가장 안전: `obs_arr`를 `(n_obs, N+1, 4)` [s, n, active_flag, half_width]로 확장.

### 5.4 합격선 / 검증 방법
- 두꺼운 장애물 시나리오 수동 생성 (Gazebo `opp` 스폰 시 w=0.30m 장애물)
- tick_json에 `obs_edge_margin_min` (ego_body ↔ obs_body 최소 거리) 필드 추가
- 합격: `obs_edge_margin_min ≥ gap_lat=0.25m` 유지

### 5.5 Non-goals
- Gaussian 대신 distance-field (SDF) 로의 완전 재설계는 비용 대비 이득 불확실 → 유지
- obstacle half_width 동역학적 추정 (predictor 쪽)은 범위 외

---

## 6. TODO: obs_prediction 개수 (`n_obs_max`) 재검토

### 6.1 현상황
- 노드 config: `n_obs_max: 2` (한 tick에서 solver로 넘기는 최대 장애물 수)
- tick_json 관측 (1997 ticks): `n_obs_raw` 최댓값이 **20**까지 관측됨 — predictor / tracker 쪽이 훨씬 많은 후보를 만들어주고 있음
- 노드 쪽 `_build_obstacle_array`에서 "closest-by-s (ahead)" 기준으로 상위 `n_obs_max`개만 골라서 solver에 주입 → **18개는 그냥 버림**

### 6.2 왜 다시 봐야 하나
- 현재 Gazebo single-opponent 시나리오에서는 `n_obs_max=2`가 과잉 (실제 유효 obstacle 1~2). 다만:
  - Multi-opponent (head-to-head 3대 이상) / static debris 섞인 시나리오에서 **2개는 빠듯**
  - Predictor가 동일 opponent를 "현재 + future-covariance-cloud"로 쪼개 보내는 경우 s-proximity sort가 엉뚱한 cluster만 남김 → **ghost sampling**
  - 반대로 키우면 NLP의 `n_obs × (N+1)` parameter/cost 항이 선형 증가 → solve_ms 잠재 악화 (v3c p99=91ms 마진 일부 소모)
- 현재 v3c는 solve_ms 여유가 있지만 (**max 125ms < 200ms IPOPT budget**), `n_obs_max` 올리기 전에 **양쪽 trade-off를 수치로** 확인 필요.

### 6.3 결정 포인트 (다음 세션 검토)
1. **🔥 실제 장애물 1개인데 2개로 찍히는지 우선 확인** — Gazebo에 opponent 1대만 떠 있는 상태에서 tick_json의 `obstacles` 배열 / `n_obs_active`를 확인. 만약 "1개 실물 → 2개 주입"이면 predictor / `_build_obstacle_array`의 중복 필터링 버그. 어떤 경로로 2개가 되는지 (same-s duplicate / predictor split / ghost track) 먼저 원인부터. 이게 버그면 `n_obs_max` 재검토보다 **중복 제거가 먼저**
2. **필터링 기준 재정의**: 단순 "ahead-by-s 최소 |Δs|" 대신
   - `|Δs| < s_horizon` 필터
   - `|Δn_raw| - (ego_half + obs_half) < lat_cutoff`로 lateral에서 멀리 떨어진 장애물 제외
   - time-to-collision (Δs / max(v_ego - v_obs, 0))로 급한 것만 우선
3. **동적 n_obs**: `n_obs_max`는 3~4로 상한 올리되, solver parameter는 **고정 크기 pre-allocate** + 남는 slot은 inactive (`active_flag=0`)로 채워서 NLP 구조 불변 유지 → 현재 코드 구조와 잘 맞음
4. **Predictor 중복 제거**: predictor 쪽이 opponent 1대당 1 track만 내도록 강제 (현재 `n_obs_raw=20`은 너무 sparse/ghost 가능성 큼 — 배경 static obstacle이 prediction으로 혼입될 수도 있음)
5. **수치 검증**: `n_obs_max ∈ {2,3,4,6,8}`로 실주행 로그 돌려서
   - `solve_ms p50/p95/p99` 변화
   - `obs_edge_margin_min` p5 (낮은 꼬리)
   - fallback tier 분포 변화
   - 주행 품질 (jitter, progress)

### 6.4 Non-goals
- Predictor 내부 알고리즘 수정 (이 문서 범위 밖)
- `n_obs_max` 무한대 (예: 20 전부 주입) — NLP 관점에서 비현실적

---

## 7. 참고 / 히스토리

- v1/v2/v3a/v3b 배경: [mpc_redesign_frenet_kin_20260420.md](mpc_redesign_frenet_kin_20260420.md)
- 재설계 최초 Phase 리스트: [mpc_planner_state_machine_integration.md](mpc_planner_state_machine_integration.md) (일부 대체됨)
- 3D overpass 층 분리 원칙: `CLAUDE.md` + `memory/project_3d_frenet_roundtrip.md`
- SideDecider feasibility v3 구현: [side_decider.py](../planner/mpc_planner/src/side_decider.py)
