# Sampling Planner 꼬불거림 (Wobble) 진단 + 다음 세션 복귀 가이드

- **작성일**: 2026-04-20 (비상 스냅샷) / **갱신 2026-04-21 (rate constraint fix + shifted-raceline refactor 완료 → 늦은 오후: soft-relax rate penalty + smootherstep blend + marker lifetime)**
- **상태**: 진단 완료 / **L1+B-plan+velocity-aware cap+rate-constraint-with-hold + shifted-raceline connector + soft-relax rate + C² smootherstep + marker lifetime 0.2s 모두 적용**. 20s live 회귀 (soft-relax): rollback 0/463 (0.0%), timing total_ms p99=44.7ms, best_idx changes 55.4%, `/best_sample/markers` 30Hz 안정. 다음 작업: **n 방향 샘플 개수 ↑, 범위 좁게, cost로 유도**.
- **대상 노드**: `planner/3d_sampling_based_planner/node/sampling_planner_state_node.py`
  + `planner/3d_sampling_based_planner/src/sampling_based_planner.py`
- **테스트 맵/모드**: `gazebo_wall_2` sim, HJ 모드, `state:=overtake`

---

## 다음 세션 첫 10분 복귀 가이드

1. 이 문서 + `TODO_HJ.md` 읽기.
2. `git log --oneline -5` 로 본 스냅샷 커밋(`sampling: tick_json debug pub + wobble diagnosis`) 확인.
3. 구동:
   ```bash
   # 호스트
   roslaunch stack_master 3d_base_system.launch map:=gazebo_wall_2 sim:=true
   roslaunch stack_master 3d_headtohead.launch dynamic_avoidance_mode:=NONE
   roslaunch sampling_based_planner_3d sampling_planner_state.launch state:=overtake
   # 노드 이름: /sampling_overtake (이번 세션에 standalone default 변경)
   ```
4. 모니터링 토픽:
   - `/sampling_overtake/debug/tick_json` — JSON 한 줄 per tick (std_msgs/String)
   - `/sampling_overtake/best_sample/markers`, `/sampling_overtake/candidates`, `/sampling_overtake/prev_best/markers` — RViz 마커 (기존 유지)
5. **다음 세션에서 적용할 fix**는 아래 "수정 계획 (L1/L2/L3 + YAML-overwrite)" 섹션 그대로. 모두 1시간 안쪽 규모.

---

## 요약 (결론 먼저)

꼬불거림은 **단일 원인이 아니라 3개 층이 중첩된 구조적 결함**. 하나만 고치면 부분만 해결됨.

| 층 | 원인 | 증거 | 수정 포인트 |
|---|---|---|---|
| **L1 (공간축)** | candidate endpoint의 tangent/곡률이 **벽 경계 finite-diff에 보간**됨 (`endpoint_chi_raceline_only=False`) | `sampling_based_planner.py:999-1004` L1003 interp / L965-987 one-step finite-diff | `state_overtake.yaml`, `cfg/SamplingCost.cfg` default → `True` |
| **L2 (시간축)** | `filter_alpha=1.0` (EMA off) + `w_cont=trajectory-integrated` → tick 간 endpoint 앵커링 제로 | s=50~70에서 45/140 tick이 `|Δn_end|>8cm`, 최대 97cm | YAML `filter_alpha: 0.6`, cfg default 동일; 또는 endpoint 전용 continuity cost |
| **L3 (초기 tangent)** | node state dict `n_dot_start=0` 하드코드, 실제 ego `chi_rel` 최대 26° | s=60.05에서 `chi_rel=+0.45 rad` 관측, `n_dot_start` 계산 훅 없음 | node `_build_state_dict()` 에 `n_dot = v·tan(chi)·(1-Ω_z·n)` 주입 |
| 공통 전제 | **dynamic_reconfigure 초기 콜백이 YAML 로드 위에 cfg default를 덮어씀** → YAML 튜닝 무효 | live params `w_pred=5000, w_race=0.1` 등이 `.cfg` default와 정확히 일치, YAML 값과 불일치 | `_weight_cb` 또는 서버 초기화 순서 정리 |

L1이 "다른 해를 더 이질적으로 갈라지게" → L2가 "그 이질 해로 매 tick flip 허용" → L3가 "flip된 해를 큰 kappa/kdot로 실현". 셋 다 끊어야 함.

---

## 이번 세션에 완료된 작업

### 1. 백업
- `planner/3d_sampling_based_planner/node/sampling_planner_state_node_backup_20260420.py`
- `planner/3d_sampling_based_planner/src/sampling_based_planner_backup_20260420.py`

### 2. upstream `calc_trajectory`에 `check_stats` 추가
`src/sampling_based_planner.py` — 각 kill 단계(`curvature/path/friction`) 전후 valid 수를 기록. `self.check_stats` dict.

### 3. 노드에 `~debug/tick_json` publisher 추가 (std_msgs/String, JSON payload)
`node/sampling_planner_state_node.py`:
- `import json`, `import math` 추가 직후.
- `self._cur_yaw = None` 초기화 + `_cb_odom` 에서 quaternion → yaw 추출.
- `self.pub_tick_json = rospy.Publisher('~debug/tick_json', String, queue_size=1)`
- 헬퍼: `_jnum` (NaN/Inf safe), `_compute_cost_stats`, `_ego_chi_vs_raceline`, `_build_tick_json`.
- `spin()` 재구성: `tick_fields` unconditional build → tick_json publish 독립.

JSON 스키마 (한 tick):
```
{
  tick, t_ros, state, status,
  ego: {s, n, v, chi_raceline_rel},
  candidates: {total, valid_before_any, killed_curvature, killed_path, killed_friction, valid_after_all},
  cost_stats: {n, min, q25, median, q75, max, mean, std, best, spread_ratio},
  best: {idx, n_end, v_end, total_progress_m, kappa_max, kappa_dot_max, n_swing, boundary_min_margin, pred_min_dist},
  mode: {chosen, prev, changed, lock_remain_s, cost_follow, cost_ot_left, cost_ot_right, best_idx_*},
  opp: {count, lead_id, lead_s, lead_n, lead_vs, lead_age_s},
  mppi: {eff_n, top1_w, T_used, n_blend},
  timing_ms: {calc, post_add, total},
  params: {w_race, w_vel, w_pred, w_cont, w_bound, w_side, w_progress, w_detour, safety_m, safety_dist, filter_alpha, mppi_enable, horizon, num_samples, n_samples, v_samples, endpoint_chi_raceline_only, relative_generation}
}
```

### 4. Standalone node 이름 충돌 방지
`launch/sampling_planner_state.launch`:
- `instance_name` default를 `$(arg state)` → `sampling_$(arg state)` 로 변경.
- Standalone: `/sampling_overtake`, `/sampling_recovery`. multi.launch는 `instance_name` override 사용 중이라 영향 없음 (`/sampling_planner/overtake` 등 유지).
- MPC 쪽은 `/mpc_overtake` 쓰도록 같은 세션에 HJ가 별도 수정.

---

## 수집된 데이터 (두 런)

### 런 1: opponent 없음 (60 tick, ego.s 23~30)
- `spread_ratio=1.0` 정확히 고정 → 구조적 (min-max normalize 특성). 무의미한 flat distribution.
- `chi_raceline_rel` 중앙값 -0.086 rad (-5°).
- best.n_end tick-to-tick: med 1.6mm, max 26.3mm (이때는 안정).
- `opp.count=0/60`.

### 런 2: opponent 있음 (60 tick, ego.s 23~30)
- 상대 lead_n=+0.30 (왼쪽), lead_s 28~34 (앞쪽 3~5m).
- `chi_rel` 범위 -0.17~+0.37 rad (21° swing).
- `kappa_max` med 0.45 / max 0.66, `kdot_max` max 2.24.
- best.n_end tick-to-tick: med 1.6mm, max 263mm (점프 발생).
- mode 30/30 `ot_right`.

### 런 3: opponent 있음 + s=50~70 구간 (400 tick, ego.s 21.95~74.54 중 s=50~70만 141 tick)
s-bucket 집계 (2m 빈):
```
s [50,52)  kmax med 0.14 max 0.20  kdot med 0.77 max 0.97
s [52,54)  kmax med 0.18 max 0.22  kdot med 1.27 max 1.48
s [54,56)  kmax med 0.18 max 0.63  kdot med 1.43 max 8.11  ← 급발생
s [56,58)  kmax med 0.61 max 0.65  kdot med 6.92 max 8.54  ← 폭주
s [58,60)  kmax med 0.60 max 0.79  kdot med 9.00 max 10.56 ← 최악
s [60,62)  kmax med 0.67 max 0.72  kdot med 6.91 max 9.39
s [62,64)  kmax med 0.52 max 0.60  kdot med 2.60 max 4.30
s [64,66)  kmax med 0.52 max 0.63  kdot med 3.29 max 3.99
s [66,68)  kmax med 0.28 max 0.59  kdot med 2.66 max 3.53
s [68,70)  kmax med 0.39 max 0.63  kdot med 3.13 max 4.64
```

best.n_end |Δ| in s=[50,70]: n=140 med=0.0041 max=**0.9680** (1m 가까이 점프). >8cm 점프 **45/140 (32%)**.

대표 대점프:
```
tick    ego.s   n_end 전   →  n_end 다음     |Δ|
18760   54.91   +0.156   →  -0.165       32cm
18762   55.60   -0.076   →  +0.241       32cm
18779   58.33   +0.452   →  +0.068       38cm
18821   63.64   +0.280   →  -0.473       75cm
18830   65.25   -0.480   →  +0.488       97cm
```

`pred_min_dist` 1.31~2.6m (런 2의 3~5m 대비 훨씬 가까움) → prediction cost saturates → 좌우 회피 cost 차 작음 → argmin이 미세 noise로 flip.

---

## 벽 경계가 샘플 궤적에 영향을 주는 경로 (이번 세션 코드 확인)

### (A) Lateral endpoint range 자체가 벽 폭에 묶임
`src/sampling_based_planner.py:958-962`
```python
n_min_track = track_handler.w_tr_right_interpolator(s_end).full().squeeze()
n_max_track = track_handler.w_tr_left_interpolator(s_end).full().squeeze()
n_min = n_min_track + self.vehicle_params['total_width']/2 + safety_distance
n_max = n_max_track - self.vehicle_params['total_width']/2 - safety_distance
n_end_values = np.concatenate((np.linspace(n_min, n_max, n_samples-1), [n_rl_eval[-1]]))
```

### (B) Endpoint chi가 벽 tangent로 보간 (**현재 ON — 주 원인**)
`src/sampling_based_planner.py:999-1004`
```python
if endpoint_chi_raceline_only:
    chi_end = chi_end_rl
    n_ddot_end = n_ddot_rl_eval[-1]
else:   # ← 현재 여기로 들어옴
    chi_end = np.interp(n_end,
                        [n_min_track, n_rl_eval[-1], n_max_track],
                        [chi_end_right_bound, chi_end_rl, chi_end_left_bound])
    n_ddot_end = np.interp(n_end,
                           [n_min_track, n_rl_eval[-1], n_max_track],
                           [0.0, n_ddot_rl_eval[-1], 0.0])
```

### (C) 벽 tangent가 1-스텝 finite-diff 라 잡음 직결
`src/sampling_based_planner.py:965-987`
```python
nearest_idx = (np.abs(track_handler.s - s_end)).argmin()
next_idx = nearest_idx+1 if nearest_idx+1 < self.track_handler.s.size else 1
left_bound_change_end  = self.left_track_bounds[:,next_idx]  - self.left_track_bounds[:,nearest_idx]
right_bound_change_end = self.right_track_bounds[:,next_idx] - self.right_track_bounds[:,nearest_idx]
...
chi_end_left_bound = np.arccos(np.dot(ref_normed, left_bound_normed))
```

→ 3D raycast로 채운 boundary라 원래 노이즈 있음. 여기서 **이웃 인덱스 2점 차분 + arccos** 로 쓰니 candidate간 endpoint heading이 벽 잡음 그대로 받음.

---

## 현재 live params (YAML과 불일치 — 버그)

```
w_race=0.1      (YAML: 30)
w_vel=100.0     (YAML: 50)
w_pred=5000.0   (YAML: 150)
w_cont=50.0     (YAML: 40)
w_bound=0.0     (YAML: 60)
w_side=100.0    (YAML: 300)
w_progress=5.0  (YAML: 5)     ← match
w_detour=0.0    (YAML: 0)     ← match
filter_alpha=1.0  (YAML: 1.0) ← match
mppi_enable=false (YAML: false) ← match
endpoint_chi_raceline_only=False (YAML: false) ← match (근데 True가 필요)
```

**불일치한 것들이 정확히 `.cfg` 파일의 `gen.add(..., default=X, ...)` 값과 일치** → dynamic_reconfigure 서버 초기화 시 cfg default가 YAML 로드 후 callback으로 들어와 덮어쓴다는 의미. HJ가 YAML에서 튜닝한 값은 한 번도 실제 실행에 반영된 적 없음.

---

## 수정 계획 (L1/L2/L3 + YAML-overwrite) — 다음 세션

### L1 — 벽 경계 영향 차단
- `planner/3d_sampling_based_planner/config/state_overtake.yaml:28` → `endpoint_chi_raceline_only: true`
- `planner/3d_sampling_based_planner/config/state_recovery.yaml` 에도 동일 추가 (아직 키 없을 수 있음)
- `planner/3d_sampling_based_planner/cfg/SamplingCost.cfg` L31 의 default `False` → `True` (YAML-overwrite 버그 미해결 상태에서도 안전)

### L2 — 시간축 smoothing
옵션 A (최소 수정, 권장 1차): `filter_alpha: 0.6` (YAML + cfg default). 기존 (n_end, v_end) EMA 훅이 살아남. 0.5~0.7 범위에서 튜닝.

옵션 B (근본적): endpoint 전용 continuity cost 추가
```python
# _compute_cost() 근처
if self._prev_best_n_end is not None:
    cost_end_cont = w_end_cont * (n_end_arr - self._prev_best_n_end) ** 2
    total_cost += cost_end_cont
```

일단 A로 가고 s=50~70에서 |Δn_end| med/max가 떨어지는지 보고 B 여부 결정.

### L3 — Heading feedback (Fix-2)
`node/sampling_planner_state_node.py` state dict 빌드 지점 (summary에 L1491 부근으로 기록됨):
```python
# 현재
state = {..., 'n_start': n_now, 'n_dot': 0.0, 'n_ddot': 0.0, ...}

# 변경
Omega_z = float(self.track_handler.Omega_z_interpolator(s_now))
n_dot_start = self._cur_v * math.tan(chi_raceline_rel) * (1.0 - Omega_z * n_now)
state = {..., 'n_start': n_now, 'n_dot': n_dot_start, 'n_ddot': 0.0, ...}
```
`chi_raceline_rel` 계산은 이미 `_ego_chi_vs_raceline()` 헬퍼 있음 (tick_json용). 같은 로직 재사용.

### YAML-overwrite 버그
`node/sampling_planner_state_node.py` `_weight_cb` 확인:
- `dynamic_reconfigure.server.Server(...)` 는 생성 시 config `level=~0` 으로 callback을 한 번 호출 → cfg default로 초기화됨.
- 이 시점이 `rospy.get_param(..., yaml_value)` 로드 **이후**면 덮어씀.

해결 패턴 (stack 내 `dynamic_statemachine_server.py` 관례):
1. YAML load를 먼저, dict로 들고 있음.
2. `Server(cfg, cb)` 에 넘길 때 `reconfigure_callback(default_config=yaml_dict)` 형태가 있으면 사용.
3. 또는 `_weight_cb` 첫 호출 플래그 `self._first_cb=True` 처리: 첫 호출일 때 **cfg가 준 값이 아니라 self.yaml_dict를 서버에 되돌려 넣기** (`self.dyn_srv.update_configuration(self.yaml_dict)`).

검증: 부팅 직후 `rostopic echo -n 1 /sampling_overtake/debug/tick_json` → `params.w_pred` 가 YAML 값과 일치하는지.

---

## 2026-04-21 후속 수정 요약 (wobble → rate-constrained smoothness)

**문제**: L1/L2/L3 + YAML-overwrite 수정 이후에도 best idx가 tick마다 바뀌면서 `n_end`가 0.3~0.8m 단위로 왕복. 사용자 피드백: "d 방향 변화는 고를때마다 순간이동하면 안돼. 제약으로 부드럽게 전환되게 해야지. valid 하지 못한 경로를 뽑게된다면 그냥 앞차를 따라가는 걸 선택하면 되는거야."

**진단** (30s tick_json 분석):
- Raceline-anchored B-plan + velocity-aware cap 적용 후에도 `|Δn_end|` p95=0.29m, max=0.76m. 특정 구간(s≈81, s≈15)에서 ping-pong.
- 원인: n_samples=11 grid 간격 0.15m > rate_cap 0.12m → rate window에 후보 0개 → fallback ("closest valid to prev")이 먼 후보 선택 → 다음 tick에서 다시 먼 후보 → 진동.
- 추가 원인: `calc_trajectory` 내부에서 `self.planner.trajectory` 를 이미 argmin으로 덮어씀. node가 rate-saturated fallback을 포기해도 planner의 unconstrained trajectory가 publish 됨.

**수정 3종**:
1. **Planner-side prev-anchored narrow linspace** — `sampling_based_planner.py:981-1001`. `self.prev_chosen_n_end` 있으면 해당 값 주변 `±min(3·rate_cap, d_side)` 범위로 `n_samples-1` 포인트 linspace + raceline point append. grid가 항상 rate window를 포함하도록 보장.
2. **Node-side hard rate constraint + hold fallback** — `sampling_planner_state_node.py:1853-1883`. mode별로 `rate_ok = |n_end - prev_n_end| ≤ rate_cap` 마스크 적용. rate-ok + valid 후보 없으면 `cost_per_mode[mode] = inf`, `bi = -1`. `_select_mode_with_hysteresis` 가 inf 모드를 필터링하므로 rate-ok 모드로 스위치되거나 전부 inf면 active mode 유지.
3. **Trajectory snapshot restore** — `sampling_planner_state_node.py:1760-1776, 1892-1896`. `calc_trajectory` 호출 전 이전 tick의 trajectory 스냅샷을 저장. rate-empty로 chosen mode의 `bi = -1` 이면 snapshot을 되돌려 놓아 "hold previous" 가 실제 publish까지 반영되게 함.
4. **prev_chosen_n_end 주입** — `sampling_planner_state_node.py:1760-1765`. `calc_trajectory` 직전에 `self.planner.prev_chosen_n_end = self._prev_chosen_n_end`, `self.planner.n_end_rate_cap = float(self.n_end_rate_cap)` 설정. `~n_end_rate_cap` rosparam default 0.12m.

**회귀 결과 (30s, 665 ticks, state=overtake, opp present)**:
| metric | before B-plan | after B-plan | **after rate+hold** |
|---|---|---|---|
| `|Δn_end|` p50 | 0.10m | 0.02m | **0.003m** |
| `|Δn_end|` p95 | 0.31m | 0.14m | **0.075m** |
| `|Δn_end|` max | 0.76m | 0.89m* | **0.120m** (cap) |
| violations >cap | N/A | 7.3% | **0.0%** |
| `boundary_min_margin` p05 | — | — | **0.449m** |
| solve_ms p95 | — | — | **41ms** |
| hold rate (idx unchanged) | — | — | **70.5%** |

\* rate constraint 단독 적용 후 (hold-snapshot 없이) 에는 planner 내부 argmin 으로 인해 오히려 max가 튐. hold+snapshot 추가 후 완전 해결.

**다음 과제**:
- Sampling 결과를 그대로 controller에 넘기는 대신 MPCC가 tracker 역할을 해 track/obstacle corridor 안에서 smooth 실행 — 사용자 언급한 canonical "sampling(decision) → MPCC(tracker)" 아키텍처.
- hold rate 70%는 정상(로컬 최적 고수)이나, 장기간 hold가 누적되면 snapshot이 stale 해짐. 필요 시 hold 카운터 한도 추가 → 한도 초과 시 prev를 raceline 쪽으로 rate_cap 만큼 이동.

---

## 커밋에 포함할 파일 (본 비상 스냅샷)

- `HJ_docs/sampling_planner_wobble_diagnosis.md` (this file, NEW)
- `TODO_HJ.md` (updated)
- `planner/3d_sampling_based_planner/src/sampling_based_planner.py` (check_stats)
- `planner/3d_sampling_based_planner/node/sampling_planner_state_node.py` (tick_json pub)
- `planner/3d_sampling_based_planner/launch/sampling_planner_state.launch` (default instance_name)
- `planner/3d_sampling_based_planner/src/sampling_based_planner_backup_20260420.py` (backup)
- `planner/3d_sampling_based_planner/node/sampling_planner_state_node_backup_20260420.py` (backup)

**제외** (다른 세션/다른 작업자 것): `planner/mpc_planner/`, `stack_master/maps/`, `f110_utils/`, submodule 들, `IY_docs/`, `HJ_docs/backup/`, `HJ_docs/debug/`, `HJ_docs/mpc_planner_state_machine_integration.md`, `planner/3d_gb_optimizer/`, etc. HJ 세션 내 sampling 작업분만 선별.

참고: `SamplingCost.cfg` / `state_overtake.yaml` / `debug_log/.gitignore` 도 modified로 찍히는데, 이번 세션 이전에 이미 건드린 것으로 보임 (diff 내용 확인 후 sampling fix 세션에서 같이 정리 예정). 비상 스냅샷엔 포함하지 않음 → 다음 세션에서 명확히 추적 후 commit.

---

## 2026-04-21 후속 수정 2: Shifted-raceline + Hermite connector sampling

### 배경
Rate constraint + hold-snapshot 으로 wobble은 잡혔지만, 근본 표현(quintic BVP in time)이
boundary chi/n_ddot 혼합 + time-domain 3차 도함수 제약이라 candidate 모양이 raceline과
"비슷하지만 다른 다항식" 이었음. 사용자 지시:

> "d 방향 변화는 고를때마다 순간이동하면 안돼. 제약으로 부드럽게 전환되게 해야지."
> "글로벌 레이스라인과 비슷하게 생긴 경로에 부드럽게 이어주는궤적이 샘플링 되는게 맞아?"
> "n은 그렇게 하고 s를 기존처럼 v기반으로 주고, 거기까지 이어지는걸 내 위치와 헤딩 고려한
>  recovery method로 샘플링해서 그중에 고르게 한다면?"

→ **Quintic BVP 를 버리고, 각 candidate = [ego → Hermite connector → 평행이동된 raceline tail]
구조로 전면 교체**. (recovery_spliner 의 BPoly 스타일과 동등한 아이디어)

### 새 구조 (generate_lateral_curves 재설계)
각 candidate 의 lateral profile n(s) 는 두 구간의 조합:

1. **Connector** (s ∈ [s0, s0+L_conn])
   - Cubic Hermite in s:
     `n(s) = h00·n_start + h10·L·m0 + h01·n_bc_end + h11·L·m1`
   - `n_bc_end = n_rl(s_trans) + d_i`, `d_i = n_end − n_rl(s_end)`
   - `m0 = n_dot_start / s_dot_start` (ego 기울기), `m1 = dn_rl/ds(s_trans)` (스플라이스에서 C¹)
2. **Tail** (s ∈ [s_trans, s_end])
   - `n_tail(s) = n_rl(s) + d_i`  (raceline 을 정확히 `d_i` 만큼 평행이동)
   - 자연스럽게 `n_tail(s_end) = n_rl(s_end) + d_i = n_end` (endpoint 정확 일치)

**3번째 샘플링 축 (factor=2 slot 재활용)**: `L_conn ∈ {3m, 6m}` 두 variant.
짧은 connector = sharp recovery, 긴 connector = gentle recovery. 전체 candidate 수는
2 × 11 × 5 = 110 유지.

**Endpoint sampling**: `n_end_values` 선택 로직(prev-anchored + velocity-aware cap + B-plan)
은 이전 세션 코드 그대로. 즉 **rate constraint 는 그대로 보존**.

**API 유지**: `raceline_tendency`, `endpoint_chi_raceline_only` 파라미터는 남아있지만 내부에서
무시됨 (tail 이 자동으로 raceline-parallel 이라 endpoint chi/n_ddot 가 강제로 raceline 것과
동일).

### 30s live 회귀 (shifted-raceline, 705 ticks, gazebo_wall_2)

| 지표 | Corner (s=50~70) | Straight (s=20~30) | Corner-exit (s=80~89) | 이전(quintic) 대비 |
|---|---|---|---|---|
| `|Δn_end|` p50 | 0.003m | 0.015m | 0.008m | ~동일 |
| `|Δn_end|` p95 | 0.096m | 0.089m | 0.079m | +0.013m (여전히 < cap) |
| `|Δn_end|` max | 0.120m | 0.110m | 0.108m | 동일 (cap 에 붙음) |
| violations > cap | 0% | 0% | 0% | **변함없이 0%** |
| `boundary_min_margin` p05 | 0.444m | 0.450m | 0.424m | −0.005m 정도 |
| `solve_ms` p95 | **25.4ms** | **27.4ms** | **26.3ms** | **quintic 41ms → 25ms (−40%)** |
| hold rate (전체) | — | — | — | 70.5% → 58.5% |

**해석**:
- Rate constraint / margin / feasibility 전부 regression 없음.
- Solve time 이 40% 줄어 MPCC tracker 붙이기 위한 여유 확보. 6×6 `np.linalg.solve` × 110번 → Hermite basis ×4 numpy ops × 110번.
- Hold rate 감소(70→58%) 는 오히려 긍정 — 평행이동 tail 이 raceline에 더 붙어서 argmin 이
  tick-to-tick 으로 더 자주 개선을 찾음. 절대 jump 는 rate cap 이 여전히 0.12m로 막고 있음.

### 변경 파일
- `planner/3d_sampling_based_planner/src/sampling_based_planner.py` —
  `generate_lateral_curves` quintic → Hermite+tail; `calc_trajectory` 가
  tight(3m)/loose(6m) 두 번 호출.
- 백업: `sampling_based_planner_backup_20260421_rate_constraint.py` (이 리팩터 직전의
  rate-constraint 최종본).

### 남은 과제
- MPCC tracker 붙이기 (canonical "sampling(decision) → MPCC(tracker)" 아키텍처).
- `L_conn_tight`/`L_conn_loose` 를 rosparam 으로 노출 (현재 하드코드 3.0/6.0).
- 저속 주행 시 `L_conn_loose=6m` 가 horizon s-range 를 초과 → tail 이 사라지는 경우
  connector-only 가 됨. 이 자체는 안전하나 두 variant 가 동일해져 실효 factor=1 로 떨어짐.
  필요 시 `L_conn = clip(horizon_s * α, 1.5, 6.0)` 식으로 속도적응.

---

## 2026-04-21 늦은 오후 세션 — κ-kick + 롤백 + 잔상 3종 fix

커밋: `sampling_planner: soft-relax rate cap + smootherstep blend + marker lifetime` (ebd93bc..f511831).

### 증상
- **(S1) best_sample/markers 에서 κ 봉우리**가 커브 구간에 관측 → V 가 뚝 떨어져 "차가 멈추는 것 같음". 실제 ego v_min=1.5 m/s (정지는 아님).
- **(S2) RViz 에서 마커 잔상 + 한참 뒤 재등장**.
- **(S3)** `/candidates` 빨강은 tick 마다 움직이는데 `/best_sample/markers` 는 가만히 있음 → "best 가 안 따라옴". 매우 자주 발생.

### 근본 원인 (세 개 독립)

1. **α-blend 가 C¹ smoothstep** (`2u³-3u²+1`) 이라 u=1 에서 `d²α/ds²` 에 `6/L²` jump.
   → n̈(s) 불연속 → κ 에 kick → GGV 로 V 급감 → 로컬 감속 "멈춤"처럼 보임.

2. **Marker lifetime=0** 이 기본값 → publish gap 이 생기면 RViz 가 이전 프레임을
   계속 보여주고, 새 tick 이 오면 DELETEALL 로 지웠다가 바로 새 marker 표시 →
   "잔상 + 끊김" 체감.

3. **n_end rate constraint 가 hard filter**: chosen_mode 의 모든 candidate 가
   `|Δn_end| > rate_cap` 이면 `best_idx_per_mode[chosen_mode] = -1` → spin 루프가
   `self.planner.trajectory = _prev_traj` 로 **이전 tick 경로를 고정** 재발행.
   - `/best_sample/markers` (= `/out/otwpnts`, 컨트롤러 입력)도 prev 경로 유지 → 차가 "옛날 plan" 을 추종.
   - `_publish_candidates(prev_idx)` 가 fresh candidate 배열에 prev 인덱스를 적용해서
     빨강 표시하므로 **표시되는 빨강은 사실 아무 의미 없는 후보**. "best 가 안 따라옴"
     체감의 원인.

### Fix 3종

| # | 포인트 | 코드 |
|---|---|---|
| 1 | α-blend 를 C² smootherstep `6u⁵-15u⁴+10u³` (1-S) 로 교체. u=0,1 양끝에서 α, α', α'' 전부 0 → n̈(s) 연속. | `sampling_based_planner.py:991-1005` |
| 2 | `_publish_best_markers` SPHERE/CYLINDER 마커에 `lifetime=rospy.Duration(0.2)` 부여. publish gap 에도 0.2s 후 자동 소멸. | `sampling_planner_state_node.py:2186-2223` |
| 3 | rate cap 을 **soft penalty** 로 변경. `rate_penalty = w_rate_penalty * (max(|Δn_end|-cap, 0)/cap)²` 를 mode cost 에 더하고 argmin → 항상 실제 인덱스 반환 → rollback 소멸. `~w_rate_penalty` rosparam (default 5.0). | `sampling_planner_state_node.py:181-189, 1858-1887` |
| + | `tick_json` 에 `rollback: bool` 필드 추가 (regression 방지). | `sampling_planner_state_node.py:1503, 1658` |

### 회귀 수치 (20s, 463 ticks)

| 지표 | soft-relax 적용 후 | 비고 |
|---|---|---|
| `rollback` 발생 | **0 / 463 (0.0%)** | hard-filter 시 "매우 자주" 라 사용자가 지적 |
| `status` 분포 | OK 307 / CONTINUITY_SWITCH 156 / NO_FEASIBLE 0 | 정상 |
| `mode` 분포 | ot_left 226 / ot_right 188 / follow 49 | left/right 양방 사용 |
| `best_idx` changes | 55.4% | tick 마다 실제로 재선택 중 |
| `total_ms` | p50=17.7, p95=30.5, p99=44.7, max=59.9 | ~30Hz 유지 |
| `/best_sample/markers` rate | 29.97 Hz, max gap 73ms | lifetime 0.2s 안쪽 |

### 남은 과제 (이 세션 이후)

- **n 샘플 그리드 재설계** (다음 세션 0순위):
  - 현재: `n_samples=11` 개를 벽 폭 전체(`n_min_track ~ n_max_track`, ~1.5m 범위)에 linspace
  - 문제: endpoint 간격이 넓어서 (~15cm) 최적해를 "중간 d_i" 로만 잡을 수 있고, 비슷비슷한
    후보들로 cost 표면이 평평해짐 → small perturbation 에 argmin 이 jitter.
  - 방향: `n_samples` 를 **증가 (예: 21~31)**, 범위는 **prev_chosen_n_end 중심의 좁은 윈도우**
    (예: ±0.4m, rate_cap 의 수 배) 로 제한 → 촘촘한 grid + 넓은 탐색은 cost 페널티로 유도
    (soft-relax rate penalty 가 이미 이 역할 일부 담당). racing line 쪽으로 `w_race` 가
    자연스럽게 끌고 가도록 유지.
  - 구현 위치 후보: `generate_lateral_curves` 의 `n_end_values` 생성부(`sampling_based_planner.py`
    상단부근) + `~n_samples`, `~n_end_window_half_m` rosparam 추가.

- soft-relax 전제에서 `w_rate_penalty`, `rate_cap` 의 실전 튜닝 (현재 5.0, 0.12 경험값).
- `_publish_candidates` 의 빨강 인덱스도 이론상 `optimal_idx` 와 항상 일치하게 됐으니,
  rollback flag 가 계속 0 이면 관련 가드 코드 완전 제거 가능.
