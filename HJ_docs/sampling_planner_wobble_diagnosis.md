# Sampling Planner 꼬불거림 (Wobble) 진단 + 다음 세션 복귀 가이드

- **작성일**: 2026-04-20 (비상 스냅샷 — 워크스페이스 유실 가능성 때문에 즉시 푸시)
- **상태**: 진단 완료 / 수정 미적용 / 다음 세션에서 코드 fix 들어가야 함
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
