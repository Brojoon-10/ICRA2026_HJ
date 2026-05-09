# RoboRacer 3D 트랙 글로벌 레이스라인 최적화 — 분석 및 설계 방향 정리

## 1. 배경 및 목표

### 상황
- UNICORN Racing (HMCL-UNIST) 팀이 ICRA 2026 Vienna RoboRacer Master Cup 준비 중
- 대회 트랙에 **다리(bridge)** 구간이 있어 xy 좌표가 겹치는 3D 트랙
- 트랙 특성: **뱅크(banking) 없음, 슬로프(slope)만 존재**, 다리 밑에 트랙이 지나감
- 차량: F1TENTH 스케일 (~4.5kg, BLDC 모터, 폼타이어, 3-12 m/s 속도 범위)

### 목표
- 3D 트랙(높이 변화, 다리)에서 중력 효과를 고려한 글로벌 레이스라인 생성
- 오르막/내리막에 따른 가감속 능력 차이를 속도 프로파일에 반영

---

## 2. 관련 GitHub 레포지토리 분석

### 2-1. TUMRT/online_3D_racing_line_planning
- **URL**: https://github.com/TUMRT/online_3D_racing_line_planning
- **방식**: CasADi 기반 NLP로 경로+속도 동시 최적화 (time-optimal)
- **차량 모델**: dallaraAV21 (750kg, 357kW, Magic Formula 타이어)
- **3D 표현**: Perantoni & Limebeer 방식의 arc-length parameterized curvilinear 좌표 (s, n)
  - 오일러 각도: theta (heading), mu (slope), phi (banking)
  - 각속도: Omega_x, Omega_y, Omega_z (rad/m)
- **gg-diagram**: 3D gggv — 축이 [V, g_tilde, alpha] → 속도, apparent gravity, 방향별 타이어 한계
- **최적화 옵션**: **time-optimal만 존재**, minimum curvature 옵션 없음
- **핵심 특징**:
  - `calc_apparent_accelerations()`에서 3D 효과(중력, 원심력 등)를 apparent acceleration으로 변환
  - `g_tilde = g·cos(mu)·cos(phi)` + 원심력 + 기타 항 → 유효 수직가속도
  - 이 `g_tilde`로 gg-diagram을 lookup하여 타이어 한계 결정

### 2-2. TUMFTM/global_racetrajectory_optimization
- **URL**: https://github.com/TUMFTM/global_racetrajectory_optimization
- **방식**: 4가지 최적화 모드
  - `shortest_path`: 최단 경로
  - `mincurv`: 곡률 최소화 (QP, 수 초)
  - `mincurv_iqp`: 반복적 곡률 최소화
  - `mintime`: CasADi NLP로 시간 최소화 (경로+속도 동시)
- **차량 모델**: 2D 평지 가정, ggv 기반
- **mincurv 후 속도 프로파일**: `trajectory_planning_helpers` 패키지의 `calc_vel_profile` 사용 (forward-backward integration)
- **3D/slope 지원 없음**: 평지만 가정

### 2-3. TUMRT/sampling_based_3D_local_planning
- **URL**: https://github.com/TUMRT/sampling_based_3D_local_planning
- 2024 IEEE IV 논문 기반, 로컬 플래닝용 (글로벌 최적화와는 별개)

---

## 2-A. 현재 코드베이스의 3D 데이터 파이프라인 (실제 파일 기준)

### 전체 데이터 흐름

```
Stage 0: PCD 포인트클라우드
  └─ wall.pcd (LiDAR 스캔)
        │
        ▼  pcd_to_track_csv_v4.py
Stage 1: 트랙 경계 CSV (raw_track_data/)
  └─ gazebo_wall_2_bounds_3d.csv
     컬럼: right_bound_x/y/z, left_bound_x/y/z, center_x/y/z
        │
        ▼  gen_3d_track_data_v2.py (generate_3d_from_3d_track_bounds)
Stage 2: 3D 트랙 CSV (3d_track_data/)
  └─ gazebo_wall_2_3d.csv  (353 pts)
     컬럼: s_m, x_m, y_m, z_m, theta_rad, mu_rad, phi_rad,
            dtheta_radpm, dmu_radpm, dphi_radpm,
            w_tr_right_m, w_tr_left_m,
            omega_x_radpm, omega_y_radpm, omega_z_radpm
        │
        ▼  smooth_3d_track.py (CasADi NLP + ipopt)
Stage 3: 스무딩된 트랙 CSV (smoothed_track_data/)
  └─ gazebo_wall_2_3d_smoothed.csv  (1799 pts, 더 조밀)
     컬럼: Stage 2와 동일, 값이 스무딩됨
        │
        ▼  gen_global_racing_line.py (CasADi NLP time-optimal)
Stage 4: 레이싱 라인 CSV (global_racing_lines/)
  └─ gazebo_wall_2_3d_rc_car_10th_timeoptimal.csv  (451 pts)
     컬럼: s_opt, v_opt, n_opt, chi_opt, ax_opt, ay_opt, jx_opt, jy_opt, laptime
        │
        ▼  export_global_waypoints.py (sn2cartesian + 재보간)
Stage 5: UNICORN 호환 JSON (global_racing_lines/ 또는 maps/)
  └─ global_waypoints.json
     → 런타임 시 global_trajectory_publisher.py가 이 JSON을 ROS 토픽으로 발행
```

### 파일 위치 이중 구조

**`planner/3d_gb_optimizer/global_line/data/`** — 옵티마이저 작업 디렉토리 (원본):
```
data/
├── raw_track_data/          # Stage 1 입력
├── 3d_track_data/           # Stage 2 출력
├── smoothed_track_data/     # Stage 3 출력
├── global_racing_lines/     # Stage 4-5 출력
├── vehicle_params/          # 차량 파라미터 (rc_car_10th.yml 등)
├── gg_diagrams/             # 3D gggv (풀스케일용, F1TENTH에선 미사용)
└── sector_config/           # 구간별 마찰/속도 설정
```

**`stack_master/maps/{map_name}/`** — 런타임 배포 디렉토리:
- `*_3d.csv` ← Stage 2 복사본
- `*_3d_smoothed.csv` ← Stage 3 복사본 (있는 경우)
- `*_bounds_3d.csv` ← Stage 1 복사본
- `*_timeoptimal.csv` ← Stage 4 복사본
- `global_waypoints.json` ← Stage 5 최종 산출물 (런타임에서 사용)

### CSV 컬럼 상세 — Stage 2/3 (15컬럼 트랙 데이터)

이것이 **TUMRT 3D 옵티마이저의 핵심 입력 포맷**:

| 컬럼 | 설명 | 단위 | 비고 |
|---|---|---|---|
| `s_m` | 센터라인 arc length | m | 누적 거리 |
| `x_m, y_m, z_m` | 센터라인 좌표 | m | 글로벌 프레임 |
| `theta_rad` | heading (yaw) | rad | |
| `mu_rad` | pitch (slope) | rad | 오르막 양수 |
| `phi_rad` | roll (banking) | rad | 형준 트랙에서 ≈0 |
| `dtheta_radpm` | dθ/ds | rad/m | |
| `dmu_radpm` | dμ/ds | rad/m | |
| `dphi_radpm` | dφ/ds | rad/m | |
| `w_tr_right_m` | 우측 트랙 폭 | m | 센터에서 우측 경계까지 (음수) |
| `w_tr_left_m` | 좌측 트랙 폭 | m | 센터에서 좌측 경계까지 (양수) |
| `omega_x/y/z_radpm` | 각속도 | rad/m | Euler→body 변환된 각속도 |

**주의:** `w_tr_right_m`이 음수인 경우가 있음. `gazebo_wall_2`에서는 `-0.89` 형태.

### CSV 컬럼 상세 — Stage 4 (레이싱 라인 출력)

| 컬럼 | 설명 | 단위 |
|---|---|---|
| `s_opt` | 센터라인 기준 arc length | m |
| `v_opt` | 최적 속도 | m/s |
| `n_opt` | 센터라인으로부터 횡방향 오프셋 | m |
| `chi_opt` | 센터라인 대비 방향 오프셋 | rad |
| `ax_opt` | 종방향 가속도 | m/s² |
| `ay_opt` | 횡방향 가속도 | m/s² |
| `jx_opt, jy_opt` | 종/횡 저크 | m/s³ |
| `laptime` | 총 랩타임 (모든 행 동일) | s |

### 이 파이프라인이 mincurv 접근에 주는 시사점

**핵심: Stage 2/3의 15컬럼 CSV가 이미 필요한 모든 정보를 포함.**

mincurv 접근에서 활용 가능한 부분:
1. **`x_m, y_m, w_tr_right_m, w_tr_left_m`** → 2D mincurv QP 입력으로 바로 사용
2. **`z_m, mu_rad`** → slope 보정에 직접 활용 (별도 계산 불필요)
3. **`omega_z_radpm`** → 곡률 κ와 동치 (2D 평면에서 `omega_z ≈ kappa`)
4. **`s_m`** → el_lengths 계산: `np.diff(s_m)`

즉, 새로운 트랙 처리 코드를 만들 필요 없이 **Stage 2/3 CSV를 그대로 읽어서 2D mincurv + slope 보정**에 필요한 모든 데이터를 뽑을 수 있음.

---

## 3. 핵심 기술적 논의 및 결론

### 3-1. TUMRT 3D 레포에 minimum curvature 옵션이 없는 이유

**2D에서는 경로와 속도를 분리할 수 있다:**
- Step 1: 곡률 최소화 QP로 경로만 최적화 (기하학적 문제)
- Step 2: 그 경로 위에 ggv 기반 forward-backward로 속도 프로파일 생성

**3D에서는 이 분리가 이론적으로 부정확하다:**
- `g_tilde`(apparent gravity)가 위치와 속도에 동시 의존
- 같은 곡률이라도 다운힐에서 더 빠르고 업힐에서 느림
- → 경로와 속도가 커플링되어 있어 동시 최적화(NLP)가 필요

**하지만 형준 트랙(뱅크 없음, 슬로프만)에서는 커플링이 약하다:**
- 뱅크가 없으면 lateral 이동 시 z 변화 없음
- 경로의 자유도는 여전히 n(s) 하나 → 본질적으로 2D 문제
- 3D 효과는 속도 프로파일에서 g·cos(μ), g·sin(μ)로 보정하면 충분

### 3-2. `gen_global_racing_line.py`를 수정해서 quasi-min-curvature 만들기 (실패)

시도: `w_T=0.0`, `w_dOmega_z=1e3`으로 곡률 정규화만 키우기
→ **NLP가 수렴하지 않음.** 시간 최소화 항이 없으면 V를 결정할 근거가 없어서 solver가 막힘.

### 3-3. 뱅크 vs 슬로프 — 왜 슬로프만 있으면 사실상 2D인가

| | 뱅크 (banking, φ) | 슬로프 (slope, μ) |
|---|---|---|
| 방향 | 트랙 횡방향 기울기 | 트랙 종방향 기울기 |
| n 이동 시 z 변화 | **있음** → 경로가 z에 영향 | **없음** → z는 트랙이 결정 |
| 경로 최적화 | 진짜 3D 문제 | 사실상 2D 문제 |
| xy 곡률 vs 실제 곡률 | **다름** (투영 왜곡) | **같음** (왜곡 없음) |

결론: 뱅크 없이 슬로프만 있으면 **xy 곡률 = 도로면 곡률**이므로 2D min curvature가 정확함.

### 3-4. mintime NLP vs mincurv + 속도 프로파일 — 트레이드오프

**mintime NLP (B 방향):**
- 장점: 경로+속도 동시 최적화, 이론적 최적
- 단점: 파라미터(타이어 마찰계수 등) 정확도에 크게 의존. 파라미터가 10% 틀리면 경로와 속도가 동시에 바뀌어서 **문제 원인 분리(디버깅)가 불가능**

**mincurv + forward-backward (A 방향):**
- 장점: 경로와 속도가 독립 → 문제 분리 가능, 디버깅 쉬움, 강건함
- 단점: 경로가 속도에 무관 → "고속 세팅이면 코너를 더 넓게" 같은 전략 못 잡음
- 하지만: mincurv 자체가 "가능한 한 곡률을 줄이는 경로" → 고속으로 갈수록 mincurv가 정답에 가까워짐
- forward-backward는 경로 고정 조건에서의 **time-optimal velocity profile**임 (friction ellipse를 매 포인트에서 꽉 채움)

**실전 판단:** F1TENTH에서 폼타이어 마찰계수를 정밀하게 아는 것이 어렵고, 대회 현장에서 바닥 재질이 바뀔 수 있으므로 — **mincurv + slope 보정 속도 프로파일이 실전에서 더 강건한 선택.**

---

## 4. 설계 방향: mincurv + slope 보정 forward-backward 속도 프로파일

### 4-1. 파이프라인

```
[Stage 2/3 트랙 CSV (15컬럼)]
        │
        ├─ (1) import_track: [x,y,z,w_tr_right,w_tr_left] 5컬럼 추출
        │      → z는 별도 보관, xy+w_tr만 prep_track으로 전달
        │
        ├─ (2) prep_track → opt_min_curv: 곡률 최소화 QP → alpha_opt
        │      → create_raceline: xy 리샘플링 (stepsize_interp_after_opt 간격)
        │      → 출력: raceline_interp(x,y), s_points_opt_interp, el_lengths, kappa
        │
        ├─ (3) z 복원 + slope 계산 (create_raceline 직후, 같은 s 위치에 끼워 넣기)
        │      ① alpha_opt 시점의 조밀 레이싱라인에 centerline_z 부여 (뱅크 없음 → z 동일)
        │      ② 조밀 포인트의 arc length s_coarse 계산
        │      ③ CubicSpline(s_coarse, z_coarse, bc_type='periodic') 피팅
        │      ④ s_points_opt_interp (create_raceline이 쓴 그 s 위치)에서 z, dz/ds 뽑기
        │      ⑤ slope: μ = -arcsin(dz_ds / sqrt(dx_ds² + dy_ds² + dz_ds²))
        │
        └─ (4) calc_vel_profile 수정판: slope 보정 forward-backward
               → V_opt(s), ax_opt(s)
```

**설계 핵심 — z를 별도 파이프라인이 아닌 기존 흐름에 끼워 넣기:**

`tph.create_raceline`은 PyPI 패키지 함수라 수정 불가. 하지만 이 함수가 리샘플링에 사용한
`s_points_opt_interp`를 반환하므로, **같은 s 위치에 z를 CubicSpline으로 보간**하면 된다.
x, y가 보간되는 타이밍에 z도 같은 s 좌표로 태워 보내는 구조.

```python
# ── 기존 코드 (건드리지 않음) ──
raceline_interp, ..., s_points_opt_interp, ..., el_lengths_opt_interp = \
    tph.create_raceline.create_raceline(refline=reftrack_interp[:, :2],
                                         normvectors=normvec_normalized_interp,
                                         alpha=alpha_opt,
                                         stepsize_interp=pars["stepsize_opts"]["stepsize_interp_after_opt"])

# ── z 복원 + slope 계산 (추가 코드, ~10줄) ──
from scipy.interpolate import CubicSpline

# ① 조밀 레이싱라인에 z 부여 (alpha로 밀어도 뱅크 없으면 z 동일)
raceline_coarse_xy = reftrack_interp[:, :2] + np.expand_dims(alpha_opt, 1) * normvec_normalized_interp
raceline_z_coarse = centerline_z  # import_track 시점에 별도 보관해둔 z

# ② 조밀 포인트의 arc length
ds_coarse = np.linalg.norm(np.diff(raceline_coarse_xy, axis=0), axis=1)
s_coarse = np.insert(np.cumsum(ds_coarse), 0, 0.0)

# ③ z 스플라인 피팅 + ④ create_raceline이 쓴 s 위치에서 z, dz/ds 뽑기
z_spline = CubicSpline(s_coarse, raceline_z_coarse, bc_type='periodic')
z_fine = z_spline(s_points_opt_interp)
dz_ds = z_spline(s_points_opt_interp, 1)  # 1차 미분 (해석적, 정확)

# ⑤ slope 계산
dx_ds = np.gradient(raceline_interp[:, 0], s_points_opt_interp)
dy_ds = np.gradient(raceline_interp[:, 1], s_points_opt_interp)
slope = -np.arcsin(np.clip(dz_ds / np.sqrt(dx_ds**2 + dy_ds**2 + dz_ds**2), -1.0, 1.0))
```

**CubicSpline을 쓰는 이유:**
- linear 보간 시 slope(dz/ds)가 구간마다 상수 → 계단 함수 → 속도 프로파일에 불필요한 급변
- CubicSpline은 slope가 연속이고, `.derivative(1)`로 해석적 미분 가능 → `np.gradient` 같은 수치 미분 불필요
- `bc_type='periodic'`로 closed track 경계 조건 자동 처리

### 4-2. xy 교차(다리) 문제 — mincurv QP에서 에러 나지 않음

**코드 분석 결과 (import_track.py → prep_track.py → opt_min_curv 추적):**

`import_track`은 CSV를 순서대로 `[x, y, w_tr_right, w_tr_left]`로 읽고,
`prep_track`에서 `tph.calc_splines(path)`로 **매개변수 스플라인** `x(t), y(t)`를 피팅한다.
t는 포인트 인덱스 기반 매개변수이지 xy 좌표가 아니다.

따라서 다리 위(t=50)와 다리 아래(t=200)가 같은 xy여도:
- **t가 다르므로 스플라인에서 완전히 별개의 구간**
- QP는 "xy가 겹치냐"를 검사하지 않음 — 인덱스 순서의 스플라인 곡률만 최소화
- normal vector 교차 체크도 **주석 처리**되어 있음 (`prep_track.py:57-81`)

**유일한 주의점:** `set_new_start` 옵션이 xy 최근접점으로 시작점을 찾는데,
다리에서 다른 층의 포인트를 잡을 수 있음. **`set_new_start: False`로 두면 문제 없음.**

**입력 포맷 호환:** `import_track`이 5컬럼 CSV(`[x, y, z, w_tr_right, w_tr_left]`)을 지원하며,
이 경우 z를 자동으로 무시하고 xy만 취하는 로직이 이미 있음 (`import_track.py:39-42`).
15컬럼 3D CSV에서 5컬럼만 추출한 CSV를 만들면 바로 입력 가능.

### 4-3. slope 보정 — calc_vel_profile 수정

현재 `calc_ax_poss` 함수 (trajectory_planning_helpers 내):

```python
ax_max_tires = mu * interp(vx_start, ggv[:, 0], ggv[:, 1])
ay_max_tires = mu * interp(vx_start, ggv[:, 0], ggv[:, 2])
ay_used = vx_start² / radius

radicand = 1.0 - (ay_used / ay_max_tires)^dyn_model_exp
ax_avail = ax_max_tires * radicand^(1/dyn_model_exp)

ax_drag = -vx_start² * drag_coeff / m_veh

# 현재: ax_final = ax_avail + ax_drag
```

**수정 사항 (2가지 slope 효과):**

**효과 1 — 종방향 중력 외력 (g·sin(μ)):**
- 오르막(μ>0): 가속 시 중력이 저항 → 실제 가속도 감소
- 내리막(μ<0): 가속 시 중력이 도움 → 실제 가속도 증가
- 드래그와 동일한 구조로 외력으로 추가

**효과 2 — 수직 하중 변화 (g·cos(μ)) → 그립 스케일링:**
- slope에 의해 유효 수직가속도가 g·cos(μ)로 변함
- 타이어 그립이 수직 하중에 비례 → ggv를 cos(μ)로 스케일링
- 형준 트랙에서 slope 10도라 해도 cos(10°) = 0.985 → 1.5% 차이 (작지만 적용 어렵지 않음)

```python
def calc_ax_poss(vx_start, radius, ggv, mu_friction,
                 slope,  # ← 추가 (rad, 오르막 양수)
                 dyn_model_exp, drag_coeff, m_veh,
                 ax_max_machines, b_ax_max_machines,
                 mode='accel_forw'):

    # 효과 2: 그립 스케일링 — 수직 하중 변화 반영
    scale = math.cos(slope)
    ax_max_tires = mu_friction * np.interp(vx_start, ggv[:, 0], ggv[:, 1]) * scale
    ay_max_tires = mu_friction * np.interp(vx_start, ggv[:, 0], ggv[:, 2]) * scale

    # 기존: mode에 따라 ax_max_tires 부호 결정
    # accel_forw, decel_backw → 양수, decel_forw → 음수
    # (실제 코드: vel_planner.py:603-608 참조)

    # friction ellipse 계산
    ay_used = vx_start**2 / radius
    radicand = 1.0 - (ay_used / ay_max_tires)**dyn_model_exp
    ax_avail_tires = ax_max_tires * radicand**(1.0/dyn_model_exp) if radicand > 0 else 0.0

    # 모터/브레이킹 한계 적용
    if mode == 'accel_forw':
        ax_max_m = np.interp(vx_start, ax_max_machines[:, 0], ax_max_machines[:, 1])
        ax_avail_vehicle = min(ax_avail_tires, ax_max_m)
    else:  # decel_forw, decel_backw
        bx_max_m = np.interp(vx_start, b_ax_max_machines[:, 0], b_ax_max_machines[:, 1])
        ax_avail_vehicle = min(ax_avail_tires, bx_max_m)

    # 드래그
    ax_drag = -vx_start**2 * drag_coeff / m_veh

    # 효과 1: 종방향 중력 외력
    ax_gravity = -9.81 * math.sin(slope)

    # 외력 적용: forward는 그대로, backward는 부호 반전
    # (드래그와 동일한 구조: vel_planner.py:639-643 참조)
    if mode in ['accel_forw', 'decel_forw']:
        ax_final = ax_avail_vehicle + ax_drag + ax_gravity
    else:  # decel_backw
        ax_final = ax_avail_vehicle - ax_drag - ax_gravity

    return ax_final
```

**backward에서 `-ax_gravity`인 이유:**

실제 코드에서 backward pass는 두 가지가 결합됩니다:
1. **배열 flip** (`np.flipud`): 포인트 순서 N→0으로 역순 탐색 — slope 배열도 flip되어 `slope[N-i]` 적용
2. **외력 부호 반전** (`-ax_drag`, `-ax_gravity`): 물리적 역방향 주행이므로 드래그/중력의 작용 방향 반전

⚠️ **주의: flip ≠ 부호 반전.** flip은 순서만 뒤집고 부호는 안 바꿈. 부호 반전은 `calc_ax_poss` 내의 mode 분기에서 처리.

**오르막/내리막 비대칭 자동 반영:**
- forward pass 오르막(μ>0): `ax_gravity < 0` → 가속 어려움
- forward pass 내리막(μ<0): `ax_gravity > 0` → 가속 도움
- backward pass: slope 배열이 flip되어 역순, 그리고 `-ax_gravity`로 부호 반전 → 올바른 감속 한계 계산

### 4-4. 구간별 마찰계수 적용

`calc_vel_profile`에 이미 **`mu` 파라미터** (포인트별 마찰계수 배열)가 존재:

```python
vx_profile = tph.calc_vel_profile.calc_vel_profile(
    ggv=ggv,
    mu=mu_array,       # ← 포인트별 마찰계수 배열
    kappa=kappa_opt,
    el_lengths=el_lengths,
    ...
)
```

사용: `ax_max_tires = mu * interp(...)` 로 ggv에 mu를 곱해 스케일링.

```python
mu = np.ones(n_points)         # 기본 1.0
mu[bridge_section] = 0.7       # 다리 구간 (다른 재질)
mu[rough_section] = 0.8        # 거친 바닥 구간
```

---

## 5. ggv (가감속 한계 다이어그램) 관련

### 5-1. 2D ggv vs 3D gggv

**2D ggv (TUMFTM):** `[v, ax_max, ay_max]`
- 속도별 타이어 가감속/횡가속 한계
- 수직 하중 = m·g 고정 (평지 가정)
- csv 파일로 수기 작성 또는 실측

**3D gggv (TUMRT):** `[v, g_tilde, ax_max, ay_max]`
- g_tilde 축 추가 → 수직 하중이 m·g_tilde로 가변
- 각 (V, g_tilde) 조합마다 NLP로 full vehicle dynamics 풀어서 타이어 한계 계산:
  - 4륜 Magic Formula 비선형 타이어
  - 4륜 수직 하중 분배 (종/횡 하중 이동)
  - 공력 다운포스/드래그
  - 조향각 한계, 구동력 배분, 엔진 파워 한계
- g_tilde 범위: 0.5g ~ 3.5g (다운포스, 뱅크 효과)

**F1TENTH에서 3D gggv가 불필요한 이유:**
- 다운포스 없음 → g_tilde ≈ g·cos(μ) ≈ 9.81 ± 2%
- 하중 이동 미미 (3.5kg, 낮은 무게중심)
- 폼타이어는 단순 friction circle에 가까움 (비선형 load sensitivity 무시 가능)
- **2D ggv + cos(μ) 스케일링으로 충분**

### 5-2. F1TENTH용 ggv 만드는 법

별도 생성 코드 없이 **실차 측정** 또는 **계산**으로 csv 작성:

**실차 측정:**
1. 직선 풀가속 로그 → VESC 속도 데이터 미분 → 속도별 `ax_max` 테이블
2. 직선 풀브레이킹 로그 → 속도별 `ax_min` (감속 한계)
3. 원형 주행 (반경 고정, 속도 점진 증가) → 슬립 발생 지점에서 `ay_max = v²/R`
4. `ax_max_machines` (모터 한계)는 별도 csv로: 고속에서 모터 토크 한계로 가속 감소

**계산 (초기 추정):**
```
ay_max ≈ μ_tire · g   (모든 속도에서 동일, 다운포스 없으므로)
ax_max_tire ≈ μ_tire · g
ax_max = min(ax_max_tire, ax_max_motor(v))
```

**ggv.csv 예시:**
```
# v_mps, ax_max_mps2, ay_max_mps2
0.0,  4.5, 5.0
2.0,  4.5, 5.0
4.0,  4.0, 5.0
6.0,  3.0, 4.8
8.0,  2.0, 4.5
10.0, 1.5, 4.5
```

### 5-3. ggv의 역할 — 타이어와 외력의 분리

**핵심 개념:** 타이어는 노면 접촉면에서 종방향(ax)과 횡방향(ay) 두 방향의 힘만 냅니다. 이것이 ggv diagram.

Slope, 드래그, 다운포스 등은 전부 **타이어 바깥의 외력**:

```
타이어가 낼 수 있는 힘 (ggv)  =  차가 필요로 하는 가속도 + 외력들

ax_tire_capacity  ≥  ax_required + drag + gravity
ay_tire_capacity  ≥  ay_required (= v²/R)
```

그래서 **ggv는 평지에서 측정/계산하면 되고, slope 효과는 외력으로 별도 처리.**
3D gggv가 g_tilde 축을 추가한 이유는 다운포스 등으로 수직 하중이 크게 변하는 풀스케일 레이싱카를 위한 것이지, F1TENTH에는 해당 없음.

---

## 6. forward-backward 속도 프로파일 — 동작 원리

### 알고리즘 요약

**Step 1 — 코너링 속도 상한 계산:**
```
각 포인트에서: v_max_corner = √(ay_max / κ)
직선(κ≈0) → v_max, 타이트한 코너 → 낮은 속도
```

**Step 2 — Forward pass (가속):**
```
i=0부터 끝까지:
  ay_used = v² · κ  (현재 사용 중인 횡가속)
  ax_avail = friction_ellipse에서_남은_종방향_여유
  v_next = √(v_now² + 2·ax_avail·ds)
  if v_next > v_max_corner[i+1]: clip
```

**Step 3 — Backward pass (감속):**
```
끝에서 i=0으로 역순:
  같은 논리를 역방향으로
  "여기까지 감속해야 코너 진입 가능" 계산
```

**결과:** "직선 풀가속 → 코너 직전 풀브레이킹 → 코너 속도 유지 → 탈출 가속" 패턴이 **자동으로** 나옴. 별도 로직이 아니라 forward-backward의 결과.

### 이것이 time-optimal인 이유

매 포인트에서 friction ellipse를 꽉 채워서 씀:
- 가속 구간: ay 쓰고 남은 여유를 ax로 전부 사용
- 감속 구간: 코너 진입 속도를 맞추기 위해 ax를 최대한 사용

**경로가 고정된 상태에서 이보다 더 빠른 속도 프로파일은 물리적으로 불가능.** 따라서 `mincurv + forward-backward = "곡률 최소 경로 위에서의 time-optimal 속도"`.

### Quasi-steady-state 성격

순수 kinematics도, 순수 dynamics도 아닌 혼합 구조:
- `ay_used = v²/R` → kinematics
- `ax_avail = ggv에서 남은 여유` → dynamics (타이어 한계)
- `ax_drag = -v²·C_d/m` → dynamics (외력)
- `ax_gravity = -g·sin(μ)` → dynamics (외력) ← **추가할 항목**

드래그가 들어갈 수 있으면 중력도 같은 방식으로 들어갈 수 있음. 물리적으로 완전히 일관된 접근.

---

## 7. 구현 TODO 정리

### 7-0. 파이프라인 연결 — 2D mincurv ↔ 3D 트랙 데이터 합치기

현재 코드베이스에 **두 가지 독립 경로**가 존재:
- **경로 A** (`planner/gb_optimizer/`): 2D TUMFTM — mincurv QP 가능, **z 좌표 없음**
- **경로 B** (`planner/3d_gb_optimizer/`): 3D TUMRT — z 좌표 있음, **NLP time-optimal만**

**작업 패키지:**
- `planner/2.5d_gb_optimizer/` — `gb_optimizer` 복사본 (패키지명: `gb_optimizer_25d`)
- `f110_utils/libs/2.5d_vel_planner/` — `vel_planner` 복사본 (패키지명: `vel_planner_25d`, Python import: `vel_planner_25d`)
- 기존 `gb_optimizer`, `vel_planner`는 **수정하지 않음** (2D 파이프라인 보존)

제안하는 접근은 **두 경로를 합치되, 기존 흐름에 z를 끼워 넣는 구조**:

```
[Stage 2/3 트랙 CSV (15컬럼)]
        │
        ├─ import_track: [x,y,z,w_tr_right,w_tr_left] 추출
        │   z는 centerline_z로 별도 보관, xy+w_tr만 기존 파이프라인으로
        │
        ├─ prep_track → opt_min_curv → alpha_opt
        │
        ├─ create_raceline (기존 tph 함수, 수정 없음)
        │   → raceline_interp(x,y), s_points_opt_interp, el_lengths, ...
        │
        ├─ ★ z 끼워 넣기 (create_raceline 직후, ~10줄 추가)
        │   ① 조밀 레이싱라인에 centerline_z 부여 (뱅크 없음 → z 동일)
        │   ② 조밀 포인트의 arc length → CubicSpline(s, z, periodic) 피팅
        │   ③ s_points_opt_interp에서 z, dz/ds 뽑기 (해석적 미분)
        │   ④ slope 계산: μ = -arcsin(dz_ds / |dr_ds|)
        │
        └─ calc_vel_profile (slope 배열 추가 전달)
            → V_opt(s), ax_opt(s)
```

**설계 원칙 — 기존 파이프라인 최소 침습:**

- `tph.create_raceline`은 PyPI 패키지 함수 → **수정하지 않음**
- 이 함수가 반환하는 `s_points_opt_interp`에 **z를 CubicSpline으로 보간**
- x, y가 리샘플링되는 타이밍에 z도 같은 s 좌표로 태워 보내는 구조
- `import_track` 수정: z를 버리지 않고 별도 보관 (1줄 추가)
- `trajectory_optimizer.py` 수정: create_raceline 호출 직후 z 복원 + slope 계산 (~10줄 추가)

**CubicSpline을 쓰는 이유 (linear 보간 안 함):**

- linear: dz/ds가 구간마다 상수 → slope가 계단 함수 → 속도 프로파일에 불필요한 급변
- CubicSpline: slope가 연속 + 해석적 1차 미분 `.derivative(1)` 사용 가능
  → `np.gradient` 같은 수치 미분 불필요, 정확도 높음
- `bc_type='periodic'`로 closed track 경계 조건 자동 처리

**왜 센터라인 mu_rad를 직접 쓰지 않고 재계산하는가:**

레이싱라인은 센터라인 대비 횡방향 오프셋(`alpha_opt`)이 s에 따라 변하므로,
진행 방향이 센터라인과 약간 다르다 (경사면을 "대각으로" 오르는 효과).
실제 체감 slope: `μ_eff = arctan(tan(μ_center) · cos(α))`, α = 방향 차이.
뱅크 없는 트랙에서 이 차이는 보통 1% 미만이지만,
z 복원 + slope 재계산 비용이 수십 ms로 무시할 수 있으므로 근사 없이 정확하게 계산한다.
재계산하면 대각 등반 효과가 자동 반영됨.

**Stage 2/3 CSV 데이터 활용 매핑:**

| CSV 컬럼 | 용도 |
|---|---|
| `x_m, y_m` | mincurv QP 입력 (센터라인) |
| `w_tr_right_m, w_tr_left_m` | mincurv QP 입력 (트랙 폭 제한) |
| `z_m` | import_track에서 별도 보관 → alpha_opt 후 레이싱라인에 부여 → slope 재계산 |
| `s_m` | `el_lengths = np.diff(s_m)`, 조밀 포인트 arc length 기준 |

### 7-1. 필수 — 코드 수정 체인

`2.5d_vel_planner/src/vel_planner_25d/vel_planner.py` 수정 대상 함수 (전체 호출 체인):

| 순서 | 함수 | 수정 내용 | 라인 |
|---:|---|---|---|
| 1 | `calc_vel_profile()` | `slope` 파라미터 추가 (np.ndarray, 기본값 None→np.zeros) | L6 |
| 2 | `__solver_fb_unclosed()` | slope 전달 | L224 |
| 3 | `__solver_fb_closed()` | **slope triple 처리** (`np.concatenate` 3회) + 전달 | L305 |
| 4 | `__solver_fb_acc_profile()` | **slope `np.flipud` 처리** (backwards 시) + `calc_ax_poss`에 전달 | L411 |
| 5 | `calc_ax_poss()` | cos(μ) 그립 스케일링 + g·sin(μ) 외력 추가 | L536 |

⚠️ **closed loop 주의:** `__solver_fb_closed`에서 3랩을 이어 붙여 boundary effect를 제거하는 구조 (L362-368). slope 배열도 동일하게 triple 처리 필수:
```python
slope_triple = np.concatenate((slope, slope, slope), axis=0)
```

### 7-2. 필수 — 데이터 준비

1. **3D 트랙 데이터**: ✅ 이미 존재 (`experiment_3d_2_3d.csv` — z_m, mu_rad 포함)
2. **slope angle 계산**: ✅ 이미 구현됨 (`export_global_waypoints.py:224`)
   - `mu = -arcsin(dz_ds / sqrt(dx_ds² + dy_ds² + dz_ds²))`
3. **F1TENTH용 ggv.csv**: 차량별로 존재 (`stack_master/config/{VEHICLE}/veh_dyn_info/ggv.csv`)
   - 현재 SIM용은 전 속도 12.0 m/s² — **실차 측정 또는 현실적 추정값으로 교체 필요**

### 7-3. 필수 — 상위 호출 코드 수정

`2.5d_gb_optimizer` 내에서 `vel_planner_25d`를 import하도록 변경하고, slope 배열을 전달:

| 수정 대상 (2.5d_gb_optimizer 내) | 원본 경로 | 설명 |
|---|---|---|
| `src/.../trajectory_optimizer.py` | `planner/gb_optimizer/src/.../` | `import vel_planner_25d`, slope 전달 |
| `src/global_trajectory_publisher.py` | 동일 | 패키지명 변경 반영 |

런타임에서 slope를 사용하는 노드들은 추후 3D 파이프라인 통합 시 별도 처리:

| 파일 (기존, 향후 수정) | 설명 |
|---|---|
| `stack_master/scripts/velocity_planner.py` | 런타임 속도 플래너 |
| `planner/spliner/src/smart_static_avoidance_node.py` | 장애물 회피 재계획 |
| `state_machine/src/state_machine_node.py` | 상태머신 내 속도 재계산 |

### 7-4. 3D 인터랙티브 시각화 도구 (섹터 지정용)

**동기:** 3D 트랙(다리 구간 등)에서 기존 2D 뷰는 xy 겹침으로 인해 섹터 경계를 지정하기 어려움.
자유롭게 회전/줌하면서 s 좌표를 확인하고 섹터를 지정할 수 있는 인터랙티브 3D 뷰가 필요.

**구현 도구:** `plotly` (브라우저 기반, 마우스로 3D 회전/줌/팬, hover로 데이터 확인)

**기능 요구사항:**

1. **3D 트랙 뷰**
   - 센터라인 + 좌/우 트랙 경계를 3D로 표시
   - 레이싱라인을 **속도 기반 색상**으로 오버레이 (느린 구간 = 파랑, 빠른 구간 = 빨강)
   - hover 시 해당 포인트의 `s_m`, `v_opt`, `mu_rad`, `z_m`, 포인트 인덱스 표시

2. **보조 2D 그래프** (같은 대시보드에 subplot)
   - s vs v_opt (속도 프로파일)
   - s vs z_m (고도 프로파일)
   - s vs mu_rad (slope 프로파일)
   - 3D 뷰에서 hover한 포인트가 2D 그래프에서도 하이라이트

3. **섹터 지정 지원**
   - 현재 섹터 구간을 색 띠로 3D 트랙 위에 표시
   - hover로 s 값 확인 → `speed_scaling.yaml`의 `start/end` 인덱스 결정에 활용
   - (선택) 클릭으로 섹터 시작/끝 지점 지정 → yaml 자동 생성

**입력 데이터:**
- Stage 2/3 트랙 CSV (센터라인 + 경계)
- Stage 4 레이싱라인 CSV 또는 `global_waypoints.json` (속도 포함)
- `speed_scaling.yaml` (기존 섹터 설정, 있으면 로드)

**출력:**
- 브라우저에서 열리는 인터랙티브 HTML (plotly `fig.show()` 또는 `fig.write_html()`)

**의사코드:**
```python
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import numpy as np

# ── 데이터 로드 ──
track = pd.read_csv('experiment_3d_2_3d_smoothed.csv')
raceline = pd.read_csv('experiment_3d_2_3d_rc_car_timeoptimal.csv')

# 레이싱라인 3D 좌표 복원 (sn2cartesian 또는 이미 export된 JSON에서)
# ...

# ── 3D + 2D 서브플롯 구성 ──
fig = make_subplots(
    rows=2, cols=2,
    specs=[[{"type": "scene", "colspan": 2}, None],
           [{"type": "xy"}, {"type": "xy"}]],
    subplot_titles=["3D Track View", "Velocity Profile", "Elevation & Slope"],
    row_heights=[0.65, 0.35]
)

# ── 3D 트랙 ──
# 센터라인
fig.add_trace(go.Scatter3d(
    x=track.x_m, y=track.y_m, z=track.z_m,
    mode='lines', line=dict(color='gray', width=2),
    name='Centerline',
    customdata=np.stack([track.s_m, track.mu_rad], axis=-1),
    hovertemplate='s=%{customdata[0]:.1f}m<br>z=%{z:.3f}m<br>μ=%{customdata[1]:.3f}rad'
), row=1, col=1)

# 좌/우 경계 (bounds CSV 또는 w_tr로 계산)
# ...

# 레이싱라인 (속도 색상)
fig.add_trace(go.Scatter3d(
    x=rl_x, y=rl_y, z=rl_z,
    mode='lines+markers', marker=dict(size=2, color=raceline.v_opt, colorscale='RdYlBu_r',
                                       colorbar=dict(title='v [m/s]')),
    name='Racing Line',
    customdata=np.stack([raceline.s_opt, raceline.v_opt, raceline.n_opt], axis=-1),
    hovertemplate='s=%{customdata[0]:.1f}m<br>v=%{customdata[1]:.2f}m/s<br>n=%{customdata[2]:.3f}m'
), row=1, col=1)

# 섹터 구간 하이라이트 (speed_scaling.yaml에서 로드)
# ...

# ── 2D 서브플롯 ──
# s vs velocity
fig.add_trace(go.Scatter(x=raceline.s_opt, y=raceline.v_opt, name='v_opt'), row=2, col=1)
# s vs elevation + slope
fig.add_trace(go.Scatter(x=track.s_m, y=track.z_m, name='z_m'), row=2, col=2)

fig.update_layout(height=900, title='3D Raceline Visualization & Sector Tuning')
fig.show()  # 또는 fig.write_html('track_3d_viz.html')
```

**기존 시각화와의 차이:**
| | 기존 (matplotlib/RViz) | 제안 (plotly 3D) |
|---|---|---|
| 3D 회전 | RViz에서만 가능, 데이터 hover 없음 | 브라우저에서 자유 회전 + hover |
| s 좌표 확인 | 포인트 클릭 후 별도 확인 | hover만으로 즉시 확인 |
| 섹터 시각화 | 별도 스크립트 필요 | 같은 뷰에 색 띠로 표시 |
| 속도+고도+slope | 별도 2D 플롯 | 같은 대시보드 하단에 연동 |
| 공유 | RViz 실행 필요 | HTML 파일 하나로 공유 |

### 7-5. 선택

5. **구간별 마찰계수 배열**: 다리 구간 등 바닥 재질 차이 반영 (mu 배열 — 이미 지원됨)
6. **`ax_max_machines.csv`**: VESC 모터 한계 (속도별 최대 가속) 별도 정의
7. **검증**: slope=0인 평지 구간에서 기존 2D 결과와 비교 → slope 수정이 regression 없음을 확인
