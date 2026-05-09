# Debug: 3D Local Raceline Optimization

IY가 작성 중이던 `local_raceline_mux_node.py` (acados spatial MPC 기반 local racing line planner)의 **Frenet 좌표계 꼬임 문제**를 진단하고 수정한 과정. 원본은 그대로 두고 `_HJ` 버전을 새로 만들어 **주행 영향 없는 테스트 전용 노드**로 정리했다.

파일 위치: `planner/3d_gb_optimizer/global_line/local_racing_line/local_raceline_mux_node_HJ.py`

---

## 1. 배경 — Local Raceline MPC란

### 역할
Global raceline optimizer (`gen_global_racing_line.py`)가 **오프라인으로 전체 트랙 1회** 최적화하는 데 비해, 이 노드는 **실시간으로 앞 300m 구간만** 재최적화한다. 차가 전역 경로에서 벗어났을 때(> d_threshold) ACTIVE 모드로 전환해 **복귀 경로를 MPC로 생성**, PASSTHROUGH 모드에서는 global raceline을 그대로 사용한다.

### 왜 필요한가
- 실제 차는 컨트롤러 오차, 슬립, 외란으로 global raceline을 완벽히 못 따라감
- 이탈 후 global raceline에 강제 복귀하면 급조향/급가감속 발생 → 트랙 이탈 위험
- MPC는 **현재 상태에서 제약(트랙 경계, GGV) 만족하며 min-time 주행** → 자연스러운 복귀

### 내부 구조
- **solver**: acados SQP_RTI + HPIPM (IPOPT 아님. realtime 전용 stack)
- **state**: `[s, V, n, chi, ax, ay]` (centerline Frenet)
- **horizon**: 300m 앞 × 30 step 샘플
- **constraints**: GGV diamond (apparent acc 기반) + 트랙 경계 (soft) + V_max (soft)
- **cost**: `1/s_dot + w·jerk² + w_slack·ε` (min-time + smoothing)

---

## 2. 발견된 문제 — Frenet 좌표계 꼬임

### 증상
원본 노드는 `/car_state/odom_frenet` 의 `(s, d)` 를 그대로 MPC 입력으로 넣고, `Track3D.theta_interpolator(s)` 로 heading reference를 구해 `chi = yaw - theta` 를 계산했다. 결과적으로 **ACTIVE 모드에서 MPC가 엉뚱한 경로를 생성**하거나 수렴 실패.

### 원인
**두 가지 좌표계가 암묵적으로 섞였음:**

| 데이터 | 좌표계 | 출처 |
|--------|--------|------|
| `/car_state/odom_frenet` 의 `(s, d)` | **racing line 기준** | frenet_conversion 노드 (raceline을 reference로 build) |
| `Track3D` (smoothed.csv) | **centerline 기준** | `<map>_3d_smoothed.csv` |
| `global_traj_wpnts_iqp` (json) 의 `s_m` | **racing line 기준** | `export_global_waypoints.py` 에서 재파라미터화 |
| `raceline.csv` (`timeoptimal.csv`) 의 `s_opt` | **centerline 기준** | `gen_global_racing_line.py` 출력 |

같은 물리 위치 (x, y) 한 점을 두 좌표계로 표현하면 값이 다르다:
- centerline arc length `s_cent ≈ 49.7m`
- raceline arc length `s_race ≈ 47.4m`
- 레이스라인은 코너를 가로질러 약 5% 짧음

원본 코드는 **raceline `s`** 를 **centerline interpolator** 에 넣음:
```python
theta_track = track_handler.theta_interpolator(s)  # s 는 raceline 기준인데 centerline indexed!
chi = yaw - theta_track                            # 엉뚱한 heading 차이
planner.calc_raceline(s=self.cur_s, n=self.cur_d, chi=chi, ...)  # 모두 불일치
```

→ MPC가 받는 초기 상태가 물리적으로 **모순**. SQP가 infeasible을 만나 수렴 못 함.

### 검증 근거
`global_waypoints.json` 내부 비교:
```
centerline_waypoints     : n=250, s_last=49.680m
global_traj_wpnts_iqp    : n=475, s_last=47.407m
```
두 arc length가 다름 → 좌표계 확실히 구분돼야 함.

---

## 3. 수정 — A안 + B안 (입력 + 출력 좌표 정리)

### 전략
**cartesian `(x, y, z)` 를 공통 기준점**으로 삼아, 필요할 때마다 `FrenetConverter` 2개(raceline용, centerline용)로 변환.

```
                       /car_state/odom  (cartesian)
                              │
                 ┌────────────┴────────────┐
                 ▼                         ▼
   centerline FrenetConverter   raceline FrenetConverter
   (track_handler.x/y/z)         (json ref_x/y/z)
                 │                         │
                 ▼                         ▼
         (s_cent, n_cent)         (s_race, n_race)
                 │                         │
         MPC INPUT                 wpnt 인덱싱 / d_m 필드
```

### A안 — MPC 입력 좌표 수정

`_run_active_mode` 앞에서 cartesian → centerline frenet:
```python
fr = self.frenet_cent.get_frenet_3d(
    np.array([self.cur_x]), np.array([self.cur_y]), np.array([self.cur_z]),
    s=np.array([self.cur_s % self.track_length]),  # raceline s 를 초기값 힌트로
)
s_cent = float(fr[0][0]) % self.track_length
n_cent = float(fr[1][0])

theta_track = float(self.track_handler.theta_interpolator(s_cent))  # ← 이제 일치
chi = self._normalize_angle(self.cur_yaw - theta_track)
planner.calc_raceline(s=s_cent, n=n_cent, chi=chi, ...)             # ← 좌표계 맞춤
```

### B안 — MPC 출력 좌표 수정

Solver 출력은 centerline frenet. 퍼블리시할 `d_m` 은 raceline 기준 n 이어야 하고, waypoint grid 인덱스도 raceline s 기준.

```python
# solver 샘플 (x, y, z) 전체를 한 번에 raceline frenet으로 변환
fr_rl = self.frenet_race.get_frenet_3d(xs, ys, zs, s=s_guess_race)
solver_s_race = fr_rl[0]
solver_n_race = fr_rl[1]

# 같은 샘플을 centerline frenet으로도 한 번에 변환 (theta/bounds/mu 조회용)
fr_cl = self.frenet_cent.get_frenet_3d(xs, ys, zs, s=s_guess_cent)
solver_s_cent = fr_cl[0] % track_length
```

`FrenetConverter.get_frenet_3d` 가 **배열 입력/출력 지원**이라 **한 번 호출로 N개 샘플 전부 변환**. 비용은 밀리초 단위.

### 중간에 놓칠 뻔한 부분 — approximation 함정

초기 구현에서 `s_cent_approx = s_race × (L_cent / L_race)` 같은 **선형 스케일링**으로 근사했는데, 실제로는 위치마다 비율이 달라 (직선 구간 1.0, 코너 구간 1.15 등) **몇 cm ~ 10cm 오차** 발생. → **정확 변환 (get_frenet_3d 한 번 더 호출)** 로 교체. 추가 비용 미미, 정확도 풀.

### Deviation check / convergence check 도 좌표계 정리

- **Deviation check**: `cur_s` (raceline) vs `ref_s` (raceline) → 그대로 OK, 다만 `period=race_length` 로 수정
- **Convergence check**: solver 출력(centerline) vs ref(raceline) → solver tail (x,y,z) → raceline frenet 변환 후 비교

---

## 4. 테스트 전용 노드로 전환 (주행 영향 0)

### 문제 인식
원본은 MPC 출력을 `/global_waypoints_scaled` (controller 직접 구독) 에 재퍼블리시한다. 좌표계 버그가 있는 상태로 실제 주행에 영향을 주면 위험.

### 해결
HJ 버전은 **주행에 안 쓰이는 전용 토픽**에만 퍼블리시하도록 전환:

| 항목 | 원본 (IY) | HJ (테스트) |
|------|----------|------------|
| 주 출력 | `/global_waypoints_scaled` (덮어쓰기) | `/3d_optimized_local_waypoints` |
| 마커 | `/online_waypoints` (WpntArray) | `/3d_optimized_local_waypoints/markers` (Sphere)<br>`/3d_optimized_local_waypoints/vel_markers` (Cylinder) |
| status | `/local_raceline/status` | `/3d_optimized_local_waypoints/status` |
| node name | `local_raceline_mux` | `local_raceline_mux_hj_test` |

마커 스타일은 `3d_state_machine_node` 의 `/local_waypoints/markers`, `/vel_markers` 를 미러링 (속도별 RGB 그라데이션, cylinder 높이 = V·0.1317). 기존 RViz 설정 **토픽 prefix만 바꾸면** 그대로 재활용 가능.

### /global_waypoints_scaled_raw 구독 제거

원본은 이 토픽을 "덮어쓰기 base"로 썼는데, HJ 는 덮어쓰지 않으므로 불필요. **subscribe도 제거**하여 의존성 최소화. 남은 subscribe:
- `/car_state/odom_frenet` (deviation check용 raceline frenet s, d, v)
- `/car_state/odom` (MPC 입력용 cartesian + yaw)
- `/ekf/imu/data` (실차 only — MPC state 의 ax, ay 초기값)

### 이로써 확보한 것
- IY 원본 노드가 기존대로 `/global_waypoints_scaled` 발행 → **실제 주행 변화 없음**
- HJ 노드는 병렬 실행, RViz에서 **우리 MPC가 계획한 경로를 시각 오버레이** 만 함
- 두 노드 이름/토픽 모두 분리되어 launch 동시 기동 가능

---

## 5. 최종 노드 구조

### Sub (읽기만)
| 토픽 | 용도 |
|------|------|
| `/car_state/odom_frenet` | raceline frenet (s, d, vs) → deviation check |
| `/car_state/odom` | cartesian (x, y, z) + yaw → centerline 변환 |
| `/ekf/imu/data` | (실차) ax, ay → MPC 초기 상태 |

### Pub (전부 test-only)
| 토픽 | 메시지 | 스타일 |
|------|--------|--------|
| `/3d_optimized_local_waypoints` | WpntArray | raceline frenet `s_m, d_m`, cartesian `x_m/y_m/z_m` |
| `/3d_optimized_local_waypoints/markers` | MarkerArray | Sphere, 0.15m, 속도 컬러 (저속 빨강→고속 초록) |
| `/3d_optimized_local_waypoints/vel_markers` | MarkerArray | Cylinder, 높이 = V·0.1317, 속도 컬러 |
| `/3d_optimized_local_waypoints/status` | String | "PASSTHROUGH" / "ACTIVE" |

### 핵심 의사결정 요약
1. 좌표계는 cartesian 을 pivot 으로 통일
2. FrenetConverter 2개를 미리 생성해 재사용
3. approximation 쓰지 말고 정확 변환 (batch 호출이라 비용 미미)
4. 테스트 중에는 실주행 영향 0 → 별도 토픽 + 별도 노드 이름
5. 원본 파일 무수정 (`_HJ` 버전으로 분기)

---

## 6. Time MPC vs Spatial MPC — 왜 이 planner 는 spatial 인가

| | **Time MPC** | **Spatial MPC** (이 노드) |
|---|---|---|
| 독립 변수 | t (초) | s (m, 경로 arc length) |
| horizon | 예: 3초 앞 | 예: 300m 앞 |
| 단계 | N개 × Δt | N개 × Δs |
| dynamics | dx/dt = f(x, u) | dx/ds = f(x, u) / s_dot |
| 저속에서 | 시간 같아도 거리 줄어듦 | 거리 그대로, 잘 봄 |
| 고속에서 | 시간 같아도 거리 늘어남 | 거리 그대로, 잘 봄 |
| V→0 | OK (정차 가능) | 특이점 발산 |

### 레이싱에서 spatial 이 유리한 이유
1. **트랙 지형은 "장소" 에 있지 시간에 있지 않음** — 코너는 항상 s=20m, 40m... 에 있지 "3초 후" 가 아니다.
2. **preview 거리가 일정** — 시속 50km/h든 10km/h든 앞 300m는 항상 300m. 반면 time horizon 3s면 저속에서 80m, 고속에서 400m로 불균형.
3. **GGV 같은 속도 의존 제약이 깔끔** — grip 한계가 "현재 V에서" 라는 개념이라 frenet s + V state 구조와 자연스럽게 맞음.

### Time MPC 가 유리한 경우
- 주변 차량 예측 (opponent 의 위치는 시간 함수)
- 정차 / 후진 포함 기동 (V=0 이 흔함)
- 궤적 추적 일반 (path tracking controller)

### 컨트롤러로 그대로 쓰려면?
현재 spatial MPC를 컨트롤러로 쓸 수도 있지만 **V→0 근처의 수치적 위험**이 있다:
- 정지 상태 출발 불가 (1/s_dot 발산)
- Low V (< 1 m/s) 에서 수치 불안정

일반적으로는 **hybrid** 구조가 표준이다:
```
Spatial MPC (이 노드) → 경로/속도 plan
      ↓
Time-based controller (PID, pure pursuit, MPC tracker) → 실제 조향/스로틀 명령
```

---

## 7. 발전 방향

### 단기 (이 노드를 실주행에 투입)
- [ ] 좌표 변환 검증 (HJ 노드 로그 `RL → CL` 값이 예상 범위인지)
- [ ] RViz로 `/3d_optimized_local_waypoints/markers` 오버레이해서 실제 주행 경로와 MPC plan 비교
- [ ] 수렴 품질/속도 통계 (solver dt_ms, eps_n_max, convergence rate)
- [ ] 문제 없으면 원본과 동일한 파이프라인(`/global_waypoints_scaled` 덮어쓰기)으로 승격

### 중기 (장애물 회피 / 추월로 확장)
spatial MPC 프레임워크에 cost term 몇 개만 추가하면 racing 시나리오 확장이 자연스러움.

**정적 장애물 회피:**
```python
# point_mass_model.py 의 cost 에 추가
cost += w_obs * exp(-((x - obs_x)² + (y - obs_y)²) / σ²)
```
또는 장애물을 **가상 트랙 경계**로 취급하여 기존 `slack_n` 메커니즘 재활용.

**동적 상대차 추월:**
- opponent 의 미래 궤적을 `s_opp(t)` → horizon 내 `s_opp(s_ours)` 로 변환 (time→spatial 투영)
- 겹치는 s 구간에서 `n` 이 `n_opp ± margin` 영역을 피하도록 cost 추가
- overtaking 방향(인/아웃) 선택도 state-level decision 추가 가능

**GGV 재활용 장점:**
- 회피 기동의 물리적 가능 여부는 기존 GGV constraint 가 이미 처리
- "이 grip 에서 이 V 로 이 회피는 불가능" 같은 판단 자동

### 장기 (controller 통합)
spatial MPC + time MPC hybrid 가능:
```
[Global Raceline (offline)]
    ↓
[Spatial MPC (이 노드, 10~20Hz)]  ← 장애물/추월 포함 경로 계획
    ↓
[Time MPC tracker (50~100Hz)]    ← V→0 대응, 정지/저속 안정
    ↓
[Actuator commands]
```
- spatial MPC 가 **무엇을 할지** (경로/속도 profile)
- time MPC tracker 가 **어떻게 할지** (steering/throttle)

단일 MPC로 모두 하려면 V→0 특이점 회피 기법 (예: `s_dot` 하한 clamping + V 하한 제약) 추가 필요. 또는 time MPC로 전면 재설계.

### 수치적 이슈 후보
- `1 - n·Ω_z` 분모 특이점 (타이트 코너 + 큰 |n| 에서 부호 역전 가능) — soft barrier 또는 제약 추가로 회피
- V_min slack 의 dual multiplier 진동 — acados tolerance 튜닝 또는 scaling 재조정
- 좌표 변환 get_frenet_3d Newton 수렴 실패 시 fallback (global search)

---

## 관련 파일

- `planner/3d_gb_optimizer/global_line/local_racing_line/local_raceline_mux_node.py` — IY 원본 (무수정)
- `planner/3d_gb_optimizer/global_line/local_racing_line/local_raceline_mux_node_HJ.py` — 이번 수정본 (테스트 전용)
- `planner/3d_gb_optimizer/global_line/local_racing_line/sim_local_racing_line.py` — 시뮬 표준 스크립트 (참고)
- `planner/3d_gb_optimizer/global_line/src/local_racing_line_planner.py` — acados MPC 코어 (LocalRacinglinePlanner 클래스)
- `planner/3d_gb_optimizer/global_line/src/point_mass_model.py` — dynamics + cost function
- `planner/3d_gb_optimizer/global_line/src/track3D_lite.py` — pandas-free Track3D (centerline geometry)
- `f110_utils/libs/frenet_conversion/src/frenet_converter/frenet_converter.py` — FrenetConverter 클래스
- `HJ_docs/3d_optimized_vel_planner.md` — 관련 노드 (경로 고정 속도 재최적화)
- `HJ_docs/fast_ggv_gen.md` — GGV 고속 생성기
