# 3D Velocity Planner 비교 분석

> 3가지 속도 플래닝 알고리즘의 3차원 슬로프 처리 방식 비교
> 뱅크 없는 3D 슬로프 트랙 레이싱 기준

---

## 1. 알고리즘 개요

| 항목 | 3d_gb_optimizer | FBGA | global_velocity_planner_3d |
|------|----------------|------|---------------------------|
| **풀네임** | 3D Global-line-Based Optimizer | Forward-Backward Generic Acceleration constraints | 2.5D Velocity Planner |
| **핵심 알고리즘** | SQP (Sequential Quadratic Programming) via ACADOS | Forward-Backward 2-pass + Brent-Dekker root finding | Forward-Backward iterative solver (Python) |
| **물리 모델** | Point mass + 3D track dynamics | Generic constraint functions (사용자 정의) | Generalized Kamm circle + drag + gravity |
| **독립 변수** | Arc length `s` (거리 기반) | Arc length `s` | Waypoint index (이산) |
| **최적화 대상** | 시간 최소화 (+ jerk 페널티) | 시간 최소화 (최대 속도 프로파일) | 시간 최소화 (최대 속도 프로파일) |
| **경로 최적화** | O (경로 + 속도 동시 최적화) | X (고정 경로, 속도만) | X (고정 경로, 속도만) |
| **구현 언어** | Python (CasADi + ACADOS) | C++ | Python |
| **실시간성** | 낮음 (NLP 풀이) | 높음 (O(n) 복잡도) | 중간 |

---

## 2. 3차원 슬로프 처리 방식 비교

### 2.1 핵심 개념: Apparent Gravity (g_tilde)

세 알고리즘 모두 슬로프의 영향을 **유효 중력 가속도(g_tilde)** 개념으로 처리한다. 이것이 핵심이다.

**기본 원리:** 경사면에서는 타이어에 작용하는 수직항력이 변한다.
- 오르막: 수직항력 감소 -> 그립 감소
- 내리막: 수직항력 증가 -> 그립 증가
- 이 변화를 g_tilde로 정량화하여 GGV에 반영

### 2.2 각 알고리즘의 g_tilde 계산

#### A. 3d_gb_optimizer (가장 정밀)

**파일:** `planner/3d_gb_optimizer/global_line/src/track3D.py:804-862`

```python
# 완전한 3D 유효 가속도 (CasADi 심볼릭)
ax_tilde = ax + omega_y_dot*h - omega_z*omega_x*h 
         + g*(-sin(mu)*cos(chi) + cos(mu)*sin(phi)*sin(chi))

ay_tilde = ay + omega_x_dot*h + omega_z*omega_y*h 
         + g*(sin(mu)*sin(chi) + cos(mu)*sin(phi)*cos(chi))

g_tilde = fmax(w_dot - V_omega + (omega_x^2 - omega_y^2)*h 
              + g*cos(mu)*cos(phi), 0)
```

**특징:**
- 차량 heading angle(chi)까지 고려 -> 경사면에서 차가 직진이 아닌 방향으로 달릴 때의 중력 분해
- 각속도 변화율(omega_dot) 반영 -> 경사 전이 구간의 동적 하중 이동
- 수직 속도(w)와 그 변화율 포함
- CoG 높이(h) 효과 포함
- **뱅크 없는 트랙(phi=0)에서 단순화됨:**

```python
# phi=0일 때:
ax_tilde = ax + omega_y_dot*h - omega_z*omega_x*h - g*sin(mu)*cos(chi)
ay_tilde = ay + omega_x_dot*h + omega_z*omega_y*h + g*sin(mu)*sin(chi)
g_tilde  = fmax(w_dot - V_omega + (omega_x^2 - omega_y^2)*h + g*cos(mu), 0)
```

#### B. global_velocity_planner_3d (중간 수준)

**파일:** `f110_utils/libs/2.5d_vel_planner/src/vel_planner_25d/vel_planner.py:658-686`

```python
# 유효 중력 계산
V_omega = omega_y * vx^2           # 수직 곡률 × 속도² (원심력)
centrifugal = (omega_x^2 - omega_y^2) * h * vx^2  # CoG 높이 효과
g_tilde = max(-V_omega + centrifugal + g*cos(mu)*cos(phi), 0)

grip_scale = g_tilde / g  # 그립 스케일링 팩터
```

**특징:**
- omega_y (피치 레이트)에 의한 원심력 효과 반영
- CoG 높이에 의한 동적 하중 이동 반영
- **그러나** 차량 heading angle(chi) 미고려 -> 항상 경로 방향으로 달린다고 가정
- omega_dot (각속도 변화율) 미포함
- **뱅크 없는 트랙(phi=0)에서:**

```python
# phi=0일 때:
g_tilde = max(-omega_y*vx^2 + (omega_x^2 - omega_y^2)*h*vx^2 + g*cos(mu), 0)
```

#### C. FBGA (외부 의존)

**파일:** `f110_utils/libs/FBGA/src/GIGI/FWBW.hxx:64-66, 87-92`

```cpp
// FBGA는 g_tilde를 직접 계산하지 않음!
// 외부에서 계산된 apparent gravity 벡터를 입력받음
void compute(
  std::vector<real> const &SS,   // arc length
  std::vector<real> const &KK,   // curvature  
  std::vector<real> const &GG_,  // <<< 외부에서 계산된 apparent gravity
  real v0
);
```

**특징:**
- g_tilde 계산 자체는 FBGA 라이브러리 밖에서 수행
- 모든 constraint 함수 `gg_Upper(ay, v, g)`, `gg_Lower(ay, v, g)`에 g를 전달
- **유연성:** 어떤 g_tilde 계산 방식이든 사용 가능 (3d_gb_optimizer 수준도, 단순 cos(mu) 수준도)

---

## 3. GGV 다이어그램 활용 비교

### 3.1 GGV가 무엇인가?

GGV = (lateral acceleration ay, longitudinal acceleration ax, velocity V) 공간에서의 차량 가속도 한계 엔벨로프.
3D 확장 시 g_tilde 차원이 추가되어 **GG(V, g_tilde)** 형태가 됨.

### 3.2 각 알고리즘의 GGV 활용

#### A. 3d_gb_optimizer

**GGV 생성:** 사전 계산 (오프라인)
- **파일:** `planner/3d_gb_optimizer/gg_diagram_generation/gen_gg_diagrams.py`
- 2-track 차량 모델 + Magic Formula 타이어 모델로 물리 시뮬레이션
- V × g_tilde 2D 격자에서 각 조합마다 GGV 엔벨로프 생성
- g_tilde 범위: 0.8g ~ 1.2g (슬로프 ±~35도)

**GGV 표현:** Diamond approximation (4 파라미터)
- **파일:** `planner/3d_gb_optimizer/gg_diagram_generation/gen_diamond_representation.py:54-60`
- 파라미터: `gg_exponent, ax_min, ax_max, ay_max`
- 제약식: `(|ax/ax_lim|^p + |ay/ay_max|^p) <= 1`
- CasADi 2D 보간으로 (V, g_tilde) -> 4개 파라미터 매핑

**GGV 적용:** NLP 제약 조건으로 직접 삽입
- **파일:** `planner/3d_gb_optimizer/global_line/src/point_mass_model.py:104-146`
- 매 최적화 스텝에서 현재 (V, g_tilde)에 대한 가속도 한계를 제약 조건으로 적용

#### B. global_velocity_planner_3d

**GGV 입력:** CSV 파일 (외부 제공)
- **파일:** `stack_master/scripts/global_velocity_planner_3d.py:47-52`
- `ggv.csv`: [vx, ax_max, ay_max] 3열 테이블
- `ax_max_machines.csv`: 모터 가속 한계
- `b_ax_max_machines.csv`: 브레이크 감속 한계

**GGV 적용:** grip_scale로 스케일링
- **파일:** `f110_utils/libs/2.5d_vel_planner/src/vel_planner_25d/vel_planner.py:689-690`
```python
ax_max_tires = mu * interp(vx, ggv[:,0], ggv[:,1]) * grip_scale
ay_max_tires = mu * interp(vx, ggv[:,0], ggv[:,2]) * grip_scale
```
- grip_scale = g_tilde / g
- **2D GGV를 g_tilde로 선형 스케일링** -> 단순하지만 물리적으로 합리적

**가속도 할당:** Generalized Kamm circle
```python
ax_avail = ax_max * (1 - (ay/ay_max)^p)^(1/p)
```

#### C. FBGA

**GGV 입력:** 함수 객체로 전달 (가장 유연)
- **파일:** `f110_utils/libs/FBGA/src/GIGI/FWBW.hxx:54-55, 77-81`
```cpp
FWBW(
  std::function<real(real, real, real)> gg_Upper,  // (ay, v, g) -> ax_max
  std::function<real(real, real, real)> gg_Lower,  // (ay, v, g) -> ax_min
  gg_range_max_min gg_range                        // (v, g) -> ay_min, ay_max
);
```

**GGV 활용 방식:**
1. **Vmax 계산** (`FWBW.cc:88-130`): 각 waypoint에서 `k*v^2 = gg_range.max(v, g)` 풀어서 최대 속도 결정
2. **Forward pass** (`FWBW.cc:182-240`): `gg_Upper(ay, v, g)`로 최대 가속 결정
3. **Backward pass** (`FWBW.cc:248-330`): `gg_Lower(ay, v, g)`로 최대 감속 결정
4. **Constraint 체크**: `signed_distance()` 함수로 현재 (ax, ay)가 GGV 엔벨로프 안에 있는지 확인

**FBGA 테스트에서의 GGV 예시** (`src_tests/GIGI_test_unicorn.cc:64-155`):
- 3d_gb_optimizer와 동일한 diamond GGV 테이블을 CSV에서 로드
- (V, g_tilde) 2D 보간으로 `ax_max, ax_min, ay_max, gg_exponent` 조회
- `ax_upper = ax_max * (1 - (|ay|/ay_max)^p)^(1/p)` 형태로 제약

---

## 4. 슬로프에서의 종방향 힘 처리

### 4.1 중력의 종방향 성분 (g*sin(mu))

| 알고리즘 | 처리 방식 | 위치 |
|---------|----------|------|
| 3d_gb_optimizer | ax_tilde에 `-g*sin(mu)*cos(chi)` 포함 (heading 고려) | `track3D.py:858` |
| global_velocity_planner_3d | `ax_gravity = g*sin(slope)` 별도 항 | `vel_planner.py:734` |
| FBGA | 외부에서 계산 후 GG constraint에 반영 | 사용자 정의 |

### 4.2 공기 저항 (Drag)

| 알고리즘 | 처리 방식 |
|---------|----------|
| 3d_gb_optimizer | GGV 생성 시 F_D 포함 + 최적화 내 별도 처리 |
| global_velocity_planner_3d | `ax_drag = -vx^2 * drag_coeff / m` 명시적 |
| FBGA | Constraint 함수에서 사용자가 처리 |

### 4.3 뱅크 없는 슬로프에서의 물리 (phi=0)

뱅크가 없으므로 lateral방향 중력 성분이 없다. 슬로프의 영향은:

1. **수직항력 감소:** `N = m*g*cos(mu)` -> 그립 감소
2. **종방향 중력 성분:** `F_gravity_x = m*g*sin(mu)` -> 오르막 감속, 내리막 가속
3. **수직 곡률 효과:** 언덕 정상/골짜기에서 원심력에 의한 수직항력 변화
4. **경사 전이 구간:** 경사 변화율에 의한 동적 하중 이동

---

## 5. 통합 방안: FBGA + 3D 물리

### 5.1 왜 FBGA가 통합에 유리한가

1. **속도:** C++ O(n) 알고리즘 -> 실시간 가능
2. **유연성:** GGV constraint를 함수 객체로 받음 -> 어떤 물리 모델이든 플러그인 가능
3. **이미 g_tilde 지원:** 모든 인터페이스가 (ay, v, **g**) 3인자
4. **테스트 완료:** `GIGI_test_unicorn.cc`에서 diamond GGV 룩업 테이블 동작 확인

### 5.2 통합 아키텍처

```
[3d_gb_optimizer의 GGV 생성]     [global_velocity_planner_3d의 g_tilde 계산]
         |                                    |
   Diamond GGV Table              Per-waypoint g_tilde 계산
   (V, g_tilde) -> (ax_max,       (mu, omega_y, vx, h_cog)
    ax_min, ay_max, exponent)              |
         |                                 |
         +------ FBGA에 입력 ------+--------+
                    |
              FWBW::compute(SS, KK, GG, v0)
                    |
              실시간 속도 프로파일
```

### 5.3 구체적 구현 단계

#### Step 1: GGV 테이블 준비

3d_gb_optimizer의 GGV 생성 코드를 사용하여 diamond 파라미터 테이블 생성:
- **입력:** 차량 파라미터 (질량, 타이어, 공력 등)
- **출력:** CSV 테이블 `[V, g_tilde, ax_max, ax_min, ay_max, gg_exponent]`
- **파일:** `planner/3d_gb_optimizer/gg_diagram_generation/gen_gg_diagrams.py` + `gen_diamond_representation.py`

#### Step 2: g_tilde 계산 모듈

global_velocity_planner_3d의 `_build_track_3d_params` + `calc_ax_poss` 내 g_tilde 로직을 분리:

```python
def compute_g_tilde_per_waypoint(waypoints, v_estimate, h_cog):
    """
    각 waypoint에서의 유효 중력 계산
    
    뱅크 없는 트랙 (phi=0) 기준:
    g_tilde = max(-omega_y * vx^2 + g*cos(mu), 0)
    
    omega_y = d(mu)/ds * cos(phi) = d(mu)/ds  (phi=0)
    """
    mu = waypoints['mu']           # 슬로프 각도 [rad]
    kappa = waypoints['kappa']     # 2D 곡률
    ds = waypoints['ds']           # 구간 길이
    
    dmu_ds = np.gradient(mu, ds)   # 슬로프 변화율
    omega_y = dmu_ds               # phi=0이므로 단순화
    
    # 속도 추정치 사용 (이전 iteration 또는 초기 추정)
    vx = v_estimate
    
    g_tilde = np.maximum(-omega_y * vx**2 + 9.81 * np.cos(mu), 0)
    return g_tilde
```

#### Step 3: FBGA Constraint 함수 구성

`GIGI_test_unicorn.cc`의 패턴을 따라:

```cpp
// Diamond GGV 룩업 테이블에서 constraint 함수 생성
auto gg_Upper = [&table](real ay, real v, real g) -> real {
    auto [ax_max, ay_max, exponent] = table.lookup_upper(v, g);
    real ratio = std::abs(ay) / ay_max;
    if (ratio >= 1.0) return 0.0;
    return ax_max * std::pow(1.0 - std::pow(ratio, exponent), 1.0/exponent);
};

auto gg_Lower = [&table](real ay, real v, real g) -> real {
    auto [ax_min, ay_max, exponent] = table.lookup_lower(v, g);
    real ratio = std::abs(ay) / ay_max;
    if (ratio >= 1.0) return 0.0;
    return ax_min * std::pow(1.0 - std::pow(ratio, exponent), 1.0/exponent);
};
```

#### Step 4: 종방향 중력 보정

FBGA의 ax에 `g*sin(mu)` 보정을 추가하는 방법:

**방법 A: Constraint 함수 내부에서 보정**
```cpp
// ax_gravity를 constraint에 반영
auto gg_Upper_with_slope = [&](real ay, real v, real g, real mu) -> real {
    real ax_grip = gg_Upper(ay, v, g);
    return ax_grip + 9.81 * sin(mu);  // 내리막이면 가속 한계 증가
};
```

**방법 B: 사후 보정 (간단)**
- FBGA로 계산된 속도 프로파일에 `g*sin(mu)` 항을 별도로 적용
- global_velocity_planner_3d가 이 방식 사용 (`vel_planner.py:734`)
- 주의: grip_scale은 이미 g_tilde에 반영되어 있으므로, 중력 종방향 성분만 별도 처리

#### Step 5: 전체 파이프라인

```python
# 1. 3D 경로 데이터 로드
waypoints = load_3d_waypoints("experiment_3d_2")

# 2. 초기 속도 추정 (단순 곡률 기반)
v_init = estimate_initial_velocity(waypoints['kappa'])

# 3. g_tilde 계산 (반복 가능)
g_tilde = compute_g_tilde_per_waypoint(waypoints, v_init, h_cog=0.16)

# 4. FBGA 실행
fbga = FWBW(gg_Upper, gg_Lower, gg_range)
total_time = fbga.compute(
    SS=waypoints['s'],      # arc length
    KK=waypoints['kappa'],  # curvature
    GG=g_tilde,             # apparent gravity
    v0=v_init[0]
)

# 5. 속도 프로파일 추출
velocity_profile = [fbga.evalV(s) for s in waypoints['s']]

# 6. (선택) g_tilde 재계산 후 반복 (속도가 바뀌면 g_tilde도 변함)
for iteration in range(3):
    g_tilde = compute_g_tilde_per_waypoint(waypoints, velocity_profile, h_cog)
    fbga.compute(SS, KK, g_tilde, velocity_profile[0])
    velocity_profile = [fbga.evalV(s) for s in waypoints['s']]
```

---

## 6. 핵심 차이 요약

### 6.1 정밀도 vs 속도 트레이드오프

```
정밀도    3d_gb_optimizer >>>>>> global_velocity_planner_3d >> FBGA(단순 g_tilde)
                                                              ≈ FBGA(정밀 g_tilde)

실시간성  FBGA >>>>> global_velocity_planner_3d >> 3d_gb_optimizer

유연성    FBGA ≈ 3d_gb_optimizer >> global_velocity_planner_3d
```

### 6.2 뱅크 없는 슬로프 트랙에서의 실질적 차이

| 물리 효과 | 3d_gb_optimizer | vel_planner_3d | FBGA |
|-----------|:-:|:-:|:-:|
| g*cos(mu) 수직항력 감소 | O | O | O (g_tilde 입력) |
| g*sin(mu) 종방향 중력 | O (ax_tilde) | O (ax_gravity) | 사용자 구현 |
| 수직 곡률 원심력 (-omega_y*v^2) | O | O | O (g_tilde 입력) |
| CoG 높이 효과 | O | O | O (g_tilde 입력) |
| Heading angle 영향 (chi) | O | X | X |
| 각속도 변화율 (omega_dot) | O | X | X |
| 수직 속도 (w, w_dot) | O | X | X |
| 경로 최적화 (lateral) | O | X | X |

### 6.3 결론

**뱅크 없는 슬로프 전용 레이싱에서:**

1. **Heading angle(chi) 효과:** 고정 경로(racing line 이미 결정)에서 chi≈0이므로 영향 작음
2. **omega_dot, w_dot 효과:** 급격한 경사 변화가 없으면 2차 효과 수준
3. **실질적 핵심:** `g*cos(mu)` + `omega_y*v^2` + `g*sin(mu)` 3가지가 지배적

**따라서 FBGA + global_velocity_planner_3d 수준의 g_tilde 계산으로 충분히 정확한 3D 속도 플래닝이 가능하다.**

추가로 3d_gb_optimizer의 GGV 생성 코드를 사용하면, 단순 선형 스케일링(`grip_scale = g_tilde/g`)보다 물리적으로 정확한 가속도 한계를 얻을 수 있어 최적의 조합이 된다.

---

## 7. 각 알고리즘의 완성도 현황

### 7.1 3d_gb_optimizer — 완성도 높음 (오프라인 전용)
- GGV 생성, diamond 피팅, SQP 최적화 모두 동작
- 단, 실시간 사용 불가 (NLP 풀이 시간)
- 오프라인 레이싱 라인 + 속도 프로파일 생성용

### 7.2 FBGA — 코어 + 파이프라인 완성 (오프라인)
C++ 코어: IY가 3인자 `(ay, v, g)` 확장 완료.
Python 파이프라인: `run_fwbw.py`에서 전체 흐름 구현 완료.

**`run_fwbw.py` (planner/3d_gb_optimizer/global_line/global_racing_line/run_fwbw.py):**
- g_tilde 계산: `g_tilde = 9.81*cos(mu) - v²*dmu/ds` (line 290-299)
- N-laps closed loop 트릭 (3바퀴 이어붙이기, line 192-206)
- GGV .npy → binary 변환 (line 350-368)
- GIGI_test_unicorn.exe subprocess 호출 (line 374-386)
- 결과 → ROS waypoint JSON export (line 414-499)
- v↔g_tilde fixed-point iteration 설계 언급 (line 287)

**남은 미완성:**

| 항목 | 상세 |
|------|------|
| **CATKIN_IGNORE** | `f110_utils/libs/FBGA/CATKIN_IGNORE` → catkin build 제외 |
| **ROS 실시간 노드 없음** | 현재 오프라인 CLI 전용. ROS topic subscribe → publish wrapper 없음 |
| **g*sin(mu) 종방향 보정** | GGV constraint에 종방향 중력 성분 미포함 (grip만 g_tilde로 보정) |
| **RUNNER_BIN 경로 하드코딩** | `/home/iy/Desktop/...` 으로 고정 (line 65) |

### 7.3 global_velocity_planner_3d — 구조 완성, 물리 정밀도 부족

| 미완성 항목 | 상세 |
|------------|------|
| **g_tilde ↔ velocity 반복 수렴 없음** | g_tilde가 vx에 의존하는데 (`V_omega = omega_y * vx^2`), 한 번의 FW+BW만 수행하고 재계산 안 함 |
| **초기 Vmax에 slope 미반영** | `vx = sqrt(ay_max * radius)` — grip_scale 없이 평면 기준으로 추정 |
| **w_dot 생략** | `w_dot ≈ 0` 가정. 급경사 전이 구간에서 부정확 |
| **backward pass omega 부호** | `track_3d_params`를 flipud하는데, omega_y 같은 방향성 값은 부호 반전도 필요할 수 있음 |
| **heading angle(chi) 무시** | 레이싱 라인이 중심선 방향이면 OK, 벗어나면 오차 |

---

## 8. 권장 구현 우선순위

1. **[필수]** FBGA CATKIN_IGNORE 해제 + catkin build 확인
2. **[필수]** FBGA ROS wrapper 노드 작성 (waypoints → g_tilde 계산 → FWBW → velocity profile publish)
3. **[필수]** g_tilde 계산 모듈 구현 (vel_planner_3d의 로직을 C++/Python wrapper로 포팅)
4. **[필수]** 종방향 중력 보정 (`g*sin(mu)`) — constraint 함수 또는 사후 보정
5. **[필수]** closed loop 처리 (3-lap 트릭 또는 반복 수렴)
6. **[권장]** 3d_gb_optimizer의 GGV 생성기로 차량-specific diamond 테이블 binary 생성
7. **[권장]** g_tilde ↔ velocity 반복 수렴 (2-3회 반복이면 충분)
8. **[선택]** global_velocity_planner_3d의 backward pass omega 부호 수정
9. **[선택]** 3d_gb_optimizer 수준의 정밀 g_tilde (omega_dot 등) 구현

---

## 부록: 핵심 파일 경로

| 구성요소 | 파일 |
|---------|------|
| **3d_gb_optimizer** | |
| Point mass 동역학 | `planner/3d_gb_optimizer/global_line/src/point_mass_model.py` |
| 3D Track + g_tilde | `planner/3d_gb_optimizer/global_line/src/track3D.py:804-862` |
| GGV 생성 | `planner/3d_gb_optimizer/gg_diagram_generation/gen_gg_diagrams.py` |
| Diamond 피팅 | `planner/3d_gb_optimizer/gg_diagram_generation/gen_diamond_representation.py` |
| GGV 보간 | `planner/3d_gb_optimizer/global_line/src/ggManager.py` |
| SQP 최적화 | `planner/3d_gb_optimizer/global_line/src/local_racing_line_planner.py` |
| **FBGA** | |
| 핵심 알고리즘 | `f110_utils/libs/FBGA/src/FWBW.cc` |
| 인터페이스 | `f110_utils/libs/FBGA/src/GIGI/FWBW.hxx` |
| Constraint 유틸 | `f110_utils/libs/FBGA/src/gg_utils.cc` |
| 3D 테스트 | `f110_utils/libs/FBGA/src_tests/GIGI_test_unicorn.cc` |
| **global_velocity_planner_3d** | |
| ROS 노드 | `stack_master/scripts/global_velocity_planner_3d.py` |
| 속도 계산 엔진 | `f110_utils/libs/2.5d_vel_planner/src/vel_planner_25d/vel_planner.py` |
| g_tilde 계산 | `vel_planner.py:658-686` |
| 종방향 중력 보정 | `vel_planner.py:728-742` |
