# MPC Planner Design — From Zero

**작성일**: 2026-04-21
**대상 독자**: MPC / 레이싱 플래너를 한 번도 본 적 없는 사람. 수학 기호는 필요한 만큼만, 대신 "이 항이 무엇을 하는가"를 말로 풀어 설명함.
**범위**: `planner/mpc_planner/` 스택 전체 — FrenetKinMPC v3c 솔버, state-machine 노드, External SideDecider, 4-tier fallback, IPOPT/HSL/JIT 옵션, 모든 파라미터의 실제 역할.

---

## 0. 이 문서가 답해주는 질문들

1. "MPC가 대체 뭔데?" → §1
2. "왜 Frenet 좌표계? Cartesian 아니고?" → §2
3. "상태변수가 왜 `[n, μ, v, δ]`야? δ를 왜 상태에 넣어?" → §3
4. "cost의 q_n, w_obs, w_wall_buf ... 이 weight들 각각 **진짜** 뭐하는 애들이야?" → §4 (가장 긴 섹션)
5. "어떤 건 hard 제약, 어떤 건 soft 페널티로 넣었는데 기준이 뭐야?" → §5
6. "장애물 회피 방향(왼/오)은 누가 어떻게 정해?" → §6 (External SideDecider)
7. "솔버가 실패하면 어떻게 복구해? RViz 무지개색은 뭘 뜻해?" → §7 (4-tier fallback)
8. "IPOPT, ma27, JIT — 누가 무슨 일 하는 거야?" → §8
9. "ROS 토픽은 어떻게 들어오고 어디로 나가?" → §10
10. "디버그는 어떻게 봐야 해?" → §11

---

## 1. MPC 기본기 — 30초 설명

**Model Predictive Control (MPC)**: 매 tick마다 **미래 N 스텝**을 최적화로 한 번에 풀고, 그중 **첫 번째 제어만 실제로 실행**한 뒤, 다음 tick에서 또 새로 푸는 방식. "receding horizon" (후퇴하는 지평선) 이라고도 부름.

왜 첫 스텝만 실행하나? 미래는 모델 오차·교란이 있으니까. 솔버가 20 스텝짜리 계획을 짜도, 실제 세계에서 1 스텝 굴러간 뒤에는 새 관측이 들어옴 → 그걸 반영해 다시 최적화. **현재 스택**에서는

- horizon `N = 20` 스텝, `dT = 0.05 s` → 미래 **1.0초**를 내다봄
- 플래너 주기 `planner_freq = 30 Hz` → 33 ms마다 풀어서 첫 스텝만 쓰고 버림
- solve time p50 ~9 ms, p99 ~32 ms (JIT+ma27 기준)

**한 tick 안에서 솔버가 푸는 것**: "지금 상태 `x0`에서 출발해, 동역학 제약을 지키면서, cost `J`를 최소화하는 제어 시퀀스 `u[0..N-1]`를 찾아라."

MPC가 매력적인 이유 (대체 왜 PID/PurePursuit 같은 거로 안 쓰고?)
- **제약(constraints)을 직접** 문제에 넣을 수 있음 — 벽, 속도 상한, 조향각 범위 등
- **미래 예측(prediction)**을 자연스럽게 통합 — 상대 차가 1 초 뒤 어디 있을지 알면 cost에 반영 가능
- **smoothness 페널티**로 조향 jerk 억제 — PID로는 못 하는 일

---

## 2. 왜 Frenet 좌표계?

### 2.1 Cartesian(x,y)의 문제

트랙 위에서 자동차를 제어할 때 "좌표계를 뭘로 할까?"는 생각보다 큰 결정. 지도 좌표 `(x, y)`로 상태를 표현하면:
- **raceline(기준선)과의 관계가 암시적**. "raceline에서 10cm 오른쪽" 을 말하려면 매 지점의 tangent 방향을 먼저 구해서 투영해야 함
- 코너에서 곡선 경로가 **복잡한 `(x(t), y(t))` 쌍**이 돼서 MPC가 "progress(많이 앞으로 가고 싶음)"를 표현하기 어려움

### 2.2 Frenet(s,n) 해결

트랙을 따라가는 arc-length `s`와, 그 점에서의 **법선 방향 횡오프셋 `n`** 으로 표현:
- `s` = raceline을 따라 얼마나 진행했는지 (miles on the road)
- `n` = raceline에서 얼마나 옆으로 벗어났는지 (+는 왼쪽, −는 오른쪽)
- `μ` (mu) = raceline tangent 대비 차체가 얼마나 틀어졌는지 (yaw error)
- 벽까지 거리는 `d_left(s)`, `d_right(s)` 두 스칼라만 있으면 됨 → 제약이 한줄로 끝남

그럼 **progress는 `−v·cos(μ)·dT`를 최소화** (음수 곱해 크게) = `s` 방향 속도 최대화 — 직관적.

### 2.3 이 스택의 3D 특수성

평평한 트랙이 아니라 **오버패스(다리)** 가 있는 3D 맵 (`gazebo_wall_2`). 같은 `(x,y)` 라도 **층이 다를 수 있음** → Cartesian nearest로 s를 찾으면 아래층/위층이 섞임.

대처:
- **솔버는 순수하게 `(s, n)` 평면**에서 2D NLP로 풀고
- 결과 trajectory를 `(s, n, μ, v) → (x, y, z, ψ)` 로 **리프팅(lifting)** 할 때만 3D 정보를 씀 ([mpc_raceline_lifter.py](planner/mpc_planner/src/mpc_raceline_lifter.py))
- `z` 는 raceline 의 `g_z[s]` 를 선형보간으로 얻음 — **절대 xy → frenet 라운드트립 금지** (overpass 버그)

이게 "**2D NLP + Track3D 3D lift**"라고 부르는 설계.

---

## 3. 문제 정식화 (Problem Formulation)

### 3.1 상태 & 제어 변수

**State** `x ∈ ℝ⁴`:

| 기호 | 물리 의미 | 단위 | 왜 필요 |
|---|---|---|---|
| `n`   | Frenet 횡오프셋 (raceline 법선방향) | m     | 벽/장애물 회피 |
| `μ`   | raceline tangent 대비 heading error | rad   | 언더스티어/오버스티어 표현 |
| `v`   | 종방향 속도 | m/s   | 가감속 제어 |
| `δ`   | **조향각** (v3에서 상태로 승격) | rad   | 조향 연속성 — §3.3 참조 |

**Control** `u ∈ ℝ²`:

| 기호 | 물리 의미 | 단위 |
|---|---|---|
| `a`   | 종방향 가속도 | m/s² |
| `δ̇` (delta_dot) | **조향각 변화율** (v3에서 제어로 변경) | rad/s |

### 3.2 동역학 (Kinematic Bicycle in Frenet)

이산시간, `dT = 0.05s` explicit-Euler:

```
n_{k+1} = n_k + v_k · sin(μ_k) · dT
μ_{k+1} = μ_k + (v_k/L · tan(δ_k) − κ_ref(s_k) · v_k · cos(μ_k)) · dT
v_{k+1} = v_k + a_k · dT
δ_{k+1} = δ_k + δ̇_k · dT
```

- `L = 0.33 m` (`vehicle_L`): 1:10 스케일 F1Tenth 휠베이스
- `κ_ref(s_k)`: raceline 의 곡률(curvature). `μ` 방정식의 두 번째 항 `− κ·v·cos μ · dT` 는 "raceline 이 휘는 만큼 heading이 자동으로 회전" 효과를 뺀다 (coordinate rotation term). 직선 트랙이면 `κ_ref = 0`.

이 모델은 **kinematic bicycle** — tire slip 을 무시하는 저속/중속 모델. F1Tenth 스케일에서 drift 안 일어나는 구간이면 충분.

### 3.3 왜 δ를 상태에, δ̇를 제어에? (v3 핵심 재설계)

v2 버전에서는 `δ` 자체를 제어(input)로 썼고, `(Δδ)²` 에만 페널티를 뒀음. 문제:

1. **δ가 knot 사이에서 piece-wise constant** → `κ = tan(δ)/L` 이 knot 마다 **불연속** → 궤적이 "직선으로 꺾이는" 모양
2. **tick-to-tick 점프가 unbounded** → 사용자 리포트 "번개 치듯 discrete 하게 바뀜"

v3 해결:
- `δ` 를 **state 에 승격** → 동역학 `δ_{k+1} = δ_k + δ̇_k·dT` 에 의해 **horizon 전체에서 연속 함수**. κ가 `C¹` 보장 → knot 꺾임 제거.
- 제어는 `δ̇` (steering rate). 여기에 `r_dd·Σδ̇²` (rate 페널티) + `r_dd_rate·Σ(Δδ̇)²` (jerk-like 페널티) 걸면 κ̇ 까지 매끄럽게.
- 매 tick 풀 때 **직전 solve의 `δ[1]` 을 이번 tick의 `δ[0]` 에 hard 고정** (`P_de0`, `P_de0_active`) → 물리적 조향각이 연속으로 이어짐.

> **핵심 교훈**: "제어 신호 자체를 매끄럽게 만들고 싶으면, 그 신호의 변화율을 제어로 삼고 신호를 상태로 넣는다."

---

## 4. Cost 함수 — 항별 해부

솔버가 매 tick 최소화하는 총 cost:

```
J = J_contour + J_reg + J_dd + J_dd_rate + J_smooth_a
  + J_progress + J_obs + J_bias + J_wall
  + J_cont + J_term + J_slack
```

11개 항. 각각 물리적으로 어떤 역할을 하는지, 어느 파라미터가 어떻게 튜닝에 영향 주는지 아래에 정리.

### 4.1 Running (horizon 전체) cost

#### 4.1.1 `J_contour = q_n · Σ n_k²`
**역할**: "raceline 에 붙어있어" 당기는 항. `n` 이 0에서 멀어질수록 이차로 비용 붐.
- **파라미터**: `q_n = 3.0`
- **튜닝 감도**:
  - 높이면 → 장애물이 있어도 raceline에 더 붙으려 함 → 회피 폭 좁아짐
  - 낮추면 → 넓게 swing 가능 하지만 곧게 달릴 때도 중앙 고집 약함
- **v3에서 중요**: stage ramp 제거. v2에서는 horizon 앞쪽 q_n을 약하게(0.4배) 뒤쪽을 강하게(2배) 했는데 → 앞쪽에서 벽쪽으로 붙어놓고 끝에서만 복귀하는 "U자" 궤적 문제. v3는 **모든 k 동일 weight** + 터미널 항으로 끝 복귀 유도.

#### 4.1.2 `J_reg = r_steer_reg · Σ δ_k²`
**역할**: 조향각 0 근방 편향 (zero-bias regulariser). 수치적으로 δ가 불필요하게 큰 값 갖는 것 방지.
- **파라미터**: `r_steer_reg = 0.1` (매우 약함)
- **튜닝 감도**: 보통 건드릴 일 없음. degeneracy 방어용 수준.

#### 4.1.3 `J_dd = r_dd · Σ δ̇_k²`  ← **중요, v3 신설**
**역할**: 조향 **rate** 페널티. `κ̇ ∝ δ̇` 이므로 직접적으로 **곡률변화율 매끄러움** 강제.
- **파라미터**: `r_dd = 5.0`
- **튜닝 감도**:
  - 높이면 → 조향이 더 매끄러움. 회피 반응성 ↓.
  - 낮추면 → 빠른 lane-change 가능 하지만 chattering 위험.

#### 4.1.4 `J_dd_rate = r_dd_rate · Σ (δ̇_{k+1} − δ̇_k)²`  ← **중요, v3 신설**
**역할**: 조향의 **jerk** 페널티 (변화율의 변화율). `κ̇` 의 **연속성** 담보.
- **파라미터**: `r_dd_rate = 1.0`
- **튜닝 감도**: v3에서 knot 불연속 방지의 수학적 핵심. 낮추면 knot마다 "톡톡" 끊김.

#### 4.1.5 `J_smooth_a = r_a · Σ (a_{k+1} − a_k)²`
**역할**: 가속도 변화(jerk) 페널티. 승차감/trackability.
- **파라미터**: `r_a = 0.5`
- **튜닝 감도**: 높이면 속도프로파일 매끈, 가감속 반응성 ↓.

#### 4.1.6 `J_progress = −γ · Σ v_k · cos(μ_k) · dT`  ← **레이싱의 원동력**
**역할**: "앞으로 가고 싶다." `v·cos(μ)·dT` 는 한 step 동안 s-방향 이동량. **음수**로 넣어 최소화 = 최대화.
- **파라미터**: `gamma_progress = 10.0`
- **튜닝 감도**:
  - 낮추면 → 솔버가 안전만 추구하며 느려짐
  - 높이면 → 과속 추구 → corridor 제약으로 몰림 → slack 활성 위험

#### 4.1.7 `J_obs = w_obs · Σ_obstacles Σ_k prox_s · exp(−(Δn/σ_n)²)`  ← **장애물 회피 핵심**
**역할**: 장애물 주변에 **Gaussian 반발 버블** 생성. 버블 안에 들어가면 cost 폭증.
- **파라미터**:
  - `w_obs = 180.0` — 버블 peak 세기
  - `sigma_s_obs = 0.7` — 종방향 스케일 (≈ 차 2대 길이)
  - `sigma_n_obs = 0.18` — 횡방향 스케일
- **구조**: 버블은 `(Δs, Δn)` 둘 다 Gaussian.
  - `prox_s = exp(−(Δs/σ_s)²)` : 장애물 s 근처에서만 **"gate"** 로 켜짐. 멀리 있으면 ≈0 → horizon 앞쪽 장애물이 horizon 뒤쪽 궤적에 영향 X.
  - `exp(−(Δn/σ_n)²)` : 장애물의 `n_o` 에 가까우면 폭증.
- **튜닝 이력** (state_overtake.yaml 주석에도 기록):
  - σ=0.35 (초기): d=1m cost 8 → 회피 못 함 (σ 좁음)
  - σ=0.7, w=800: horizon 전체에 영향 → 여러방향 밀림 → 궤적 꼬불거림 (σ 넓음)
  - σ=0.5/0.18, w=180 (현재): σ_s=0.7로 복원, σ_n 작게. d=1m에서 54, d=0.5m에서 242 → 근거리만 강하게 밀어냄
- **왜 이 값?** `w_obs` 는 **wall cushion(`w_wall_buf=2500`)보다 작아야** 솔버가 "벽 지키며 우회" 선택. 장애물은 **선호**, 벽은 **절대**.

#### 4.1.8 `J_bias = w_side_bias · Σ prox_s · hinge(side)²`  ← **방향 편향**
**역할**: SideDecider 가 LEFT/RIGHT 결정했을 때 **한쪽으로만 밀어내는 일차평면 페널티**. 장애물 반대쪽으로 가도록 유도.
- **구조**: **one-sided quadratic hinge**.
  - SIDE_LEFT 선택되면 `bias_L = w_side_bias`, bias_R = 0 → `n < n_obs + gap_lat` 일 때만 페널티
- **파라미터**:
  - `w_side_bias = 25.0` — 낮음 (v2a에서 80은 과하게 작동 — raceline 안 따라감)
  - `gap_lat = 0.25` — bias의 zero-line 가로 clearance
- **중요 설계**: `prox_s` 로 gate → 장애물에서 멀면 bias 영향 없음. LEFT↔RIGHT 뒤집혀도 **장애물 근처** 만 뒤집힘 → 원거리 궤적은 요동 안 함.

#### 4.1.9 `J_wall = w_wall_buf · Σ (hinge_up² + hinge_dn²)`  ← **벽 안전**
**역할**: **소프트 벽 쿠션**. `wall_buf` 안쪽으로 들어가면 이차 폭증. hard corridor에 닿기 전에 밀어냄.
- **파라미터**:
  - `w_wall_buf = 2500.0` — 매우 강함 (v3에서 ×6 증폭)
  - `wall_buf = 0.30 m` — 쿠션 두께
- **구조**: `viol_up = max(0, n − (d_L − ego_half − wall_buf))` → 센트로이드가 벽 30cm 안으로 들어가면 hinge 작동
- **왜 w_wall_buf 를 이렇게 크게?**: 2500 × (0.30m)² = **225 per step** → 장애물 버블 peak(180)보다 **항상** 큼 → "벽 뚫고 장애물 피하기" 선택 불가. 벽이 **언제나** 이김.

#### 4.1.10 `J_cont = w_cont · cont_active · Σ (n_k − n_prev_k)²`  ← **tick-to-tick 연속성**
**역할**: 이번 tick 의 `n[k]` 가 **직전 tick 의 `n[k+1]`** (한 칸 shift)에 가깝게 당김. 동일 장애물/벽 상황에서 tick 마다 최적해가 요동치는 것 방지.
- **파라미터**:
  - `w_cont = 200.0` — v3b에서 20→200 으로 ×10 (jitter_rms p95=0.281m → 목표 <0.10m)
  - `cont_active = 1.0` (첫 solve는 0, 이후 1)
- **동작**: 이전 solve의 `n_sol[1:]` 을 `n_prev[:-1]` 로 shift 해서 파라미터로 전달.
- **중요**: `w_cont` 너무 크면 장애물/벽 변화를 쫓아가지 못함 — "고집스러운" 솔버. 200이 현재 sweet spot.

#### 4.1.11 `J_slack = w_slack · Σ slk_k²`
**역할**: corridor 제약을 **soft하게** 만든 벌금. slk ≥ 0 인 슬랙변수로 `n` 이 벽 바깥 나가면 `slk`가 양수 → 2차 페널티.
- **파라미터**: `w_slack = 5000.0` (v2c 2000→5000)
- **왜 큼?**: v2b 캡처에서 솔버가 corridor 뚫고 들어가기(slack≈0.22m) vs 장애물 피하기 중 전자가 더 쌌음. 2.5배 올려서 corridor 위반이 항상 더 비싸게.

### 4.2 Terminal (horizon 끝) cost

#### 4.2.1 `J_term = q_n_term · n_N² + q_v_term · (v_N − v_ref_N)²`
**역할**: "horizon 끝에서는 raceline 중앙에 근처 & 속도가 ref 에 근접" 유도.
- **파라미터**:
  - `q_n_term = 10.0` — n_N 매우 강하게 → 회피 후 raceline 복귀 확실히
  - `q_v_term = 0.5` — 속도는 약하게 (velocity planner가 덮어쓰므로)
- **왜 필요?**: v2에서 stage ramp로 끝쪽 q_n 키우던 걸 **명시적 terminal cost** 로 대체. 훨씬 깔끔.

### 4.3 Cost weight 전체 요약 (state_overtake.yaml 기준)

| 항 | weight | 역할 | 튜닝 민감도 |
|---|--:|---|---|
| `q_n`         | 3.0   | raceline contour | 높으면 회피 좁음 |
| `q_n_term`    | 10.0  | terminal n | 회피 후 복귀 강도 |
| `q_v_term`    | 0.5   | terminal v | 약. velocity planner가 덮음 |
| `gamma_progress` | 10.0 | progress pull | 레이스 속도 추구 |
| `r_a`         | 0.5   | accel smoothness | — |
| `r_steer_reg` | 0.1   | δ zero-bias | degeneracy 방어 |
| `r_dd`        | 5.0   | δ̇ rate | κ 매끄러움 |
| `r_dd_rate`   | 1.0   | δ̈ jerk | knot 연속성 |
| `w_obs`       | 180.0 | obstacle bubble peak | 근거리 회피 강도 |
| `w_side_bias` | 25.0  | side 편향 | LEFT/RIGHT gate 세기 |
| `w_wall_buf`  | 2500  | 벽 쿠션 | **절대적 벽 안전** |
| `w_cont`      | 200   | tick-to-tick 연속 | jitter 억제 |
| `w_slack`     | 5000  | corridor 위반 | 거의 hard처럼 |

> **튜닝 원칙**: `w_slack > w_wall_buf × (wall_buf)² > w_obs × (obstacle_peak)` 순으로 내려가야 의미가 유지. 현재: 5000 > 225 > 180. ✓

---

## 5. Hard Constraints — "절대로" 규칙

cost 는 "선호" 지만, constraint 는 "절대". IPOPT 가 hard constraint 만족 못 하면 infeasible 판정. 솔버가 갖는 hard 제약:

### 5.1 동역학 (Equality)
```
n[k+1] = n[k] + v[k]sin(μ[k])·dT
μ[k+1] = μ[k] + (v[k]/L·tan(δ[k]) − κ·v[k]cos(μ[k]))·dT
v[k+1] = v[k] + a[k]·dT
δ[k+1] = δ[k] + δ̇[k]·dT
```
당연. 이 방정식을 안 지키면 물리 위반.

### 5.2 초기 조건 (Equality)
- `n[0] = n0_clamped` (차 현재 n)
- `μ[0] = μ0`
- `v[0] = v0`
- `δ[0] = prev_de1` (직전 solve의 `δ[1]`, tick-to-tick 연속성)

> **중요 트릭**: `v[0] = v0` 가 hard equality 이므로 만약 `vmax[0] < v0` 면 infeasible. 그래서 TRAIL 진입 시 **v_max 는 v0 에서 cap 까지 a_dec_ramp 로 램프** 해서 `vmax[0] = v0` 유지 (§6.3).

### 5.3 입력/상태 바운드 (Inequality)
| 변수 | 하한 | 상한 | YAML 파라미터 |
|---|--:|--:|---|
| `a`   | −4.0  | 3.0   | `a_min`, `a_max` |
| `δ̇`   | −3.0  | 3.0   | `delta_rate_max` |
| `δ`   | −0.6  | 0.6   | `max_steering` |
| `μ`   | −0.9  | 0.9   | `mu_max` |
| `v`   | 0.0   | vmax_k | `min_speed`, `max_speed` |
| `slk` | 0     | ∞ | — |

### 5.4 Corridor (slackable hard)
```
nlb[k] − slk[k] ≤ n[k] ≤ nub[k] + slk[k]
```
여기서 `nlb[k]` 와 `nub[k]` 는 `_build_wall_bounds` 에서:
```
margin = ego_half(0.15) + inflation(0.05) + wall_safe(0.08)  = 0.28 m
nub = d_L − margin    (positive  — left wall)
nlb = −(d_R − margin) (negative  — right wall)
```

**즉** 센트로이드가 `|n| ≤ d_wall − 0.28m` 안에 있어야 함. **차체 엣지(centroid ± 15cm) 기준으로 벽에서 13cm 확보**.

> **wall_safe 는 "body edge" 기반**. 센트로이드 기반이면 차폭 15cm 가 고려 안 돼 벽 스침. v3b에서 `ego_half` 를 명시적으로 corridor 및 wall cushion 수식에 포함.

### 5.5 Soft vs Hard 설계 기준

| 위반하면 | 처리 |
|---|---|
| 동역학 / 초기조건 / 입력 바운드 | **Hard equality / inequality** (IPOPT 가 풂) |
| 벽 corridor (`wall_safe`+inflation+ego_half) | **Hard + slack** (slack은 매우 비쌈: w_slack=5000) |
| 벽 쿠션 `wall_buf` (corridor 안쪽 30cm 층) | **Soft quadratic hinge** (w_wall_buf=2500) |
| 장애물 회피 | **Soft Gaussian bubble** (w_obs=180) |
| 방향(LEFT/RIGHT) 편향 | **Soft one-sided hinge** (w_side_bias=25) |
| raceline 복귀 | **Soft contour + terminal** (q_n=3, q_n_term=10) |

**원칙**: "물리적 한계(acc, steer)는 hard, 안전(벽)은 거의 hard, 전술(장애물 회피 방향)은 soft" — soft한 쪽이 많을수록 NLP 가 feasible 영역을 찾기 쉬움.

---

## 6. External SideDecider — "어느 쪽으로 뚫어?"

### 6.1 왜 NLP 밖에서 정하나?

NLP 가 LEFT/RIGHT 를 **스스로 선택** 하려면 two-sided bubble + logical OR 이 필요 → 비볼록(non-convex), 로컬미니마 남발. v2c 까지 이 방식 시도했으나 **warm-start가 좌·우 전환 시 꼬여서 궤적 튕김**.

v3 해결: **룰 기반 바깥 결정자** 가 매 tick LEFT/RIGHT/TRAIL/CLEAR 중 하나 뽑음 → NLP 에는 **한 방향 one-sided hinge** 만 심어서 볼록성 유지.

### 6.2 결정 로직 ([side_decider.py](planner/mpc_planner/src/side_decider.py))

입력:
- `ego_v`: 현재 속도
- 장애물 리스트: 각 항목에 `(s0, n0, v_s_obs, d_L, d_R, ref_v)`

절차 (가장 가까운 장애물 기준):
1. **d_free 계산** — 장애물 옆으로 빠지는 유효 slot 폭:
   ```
   eff_dL = d_L − wall_safe − inflation          (솔버가 실제 허용하는 벽)
   d_free_L = eff_dL − (n_o + w_o) − (ego_half + gap_lat)
   d_free_R = (n_o − w_o) − (−eff_dR) − (ego_half + gap_lat)
   ```
2. **can_pass 게이트**: `d_free ≥ min_pass_margin (0.10m)` 이면 그 쪽 통과 가능.
3. **결정**:
   - `dv < trail_dv_thresh(0.5)` → **TRAIL** (속도차 작으면 굳이 회피 X)
   - 둘 다 can_pass 불가 → **TRAIL** (유일하게 안전)
   - 한쪽만 가능 → 그 쪽
   - 둘 다 가능 → `d_free` 넓은 쪽
4. **히스테리시스**:
   - LEFT↔RIGHT flip: `hold_ticks = 10` (약 0.33s 관성) — flip 남발 방지
   - ANY → TRAIL: `trail_entry_ticks = 3` (약 0.1s) — "못 갈 것 같으면 바로 감속"

### 6.3 TRAIL 동작 — v_max 램프

TRAIL 결정 시 솔버는 `vmax[k]` 를 **장애물 s-속도 × 0.95** 로 cap. 그런데 여기서 문제:
- `v[0] = v0` (현재 속도) 는 **hard equality**
- 만약 `vmax[0] = v_obs × 0.95 < v0` → IPOPT infeasible_problem_detected

해결 ([`_build_vmax`](planner/mpc_planner/src/frenet_kin_solver.py#L473)):
```
ramp[k] = max(cap, v0 − a_dec_ramp·dT·k)      (a_dec_ramp = 3.0 m/s²)
vmax[k] = min(ref_v[k], ramp[k])
```

즉 k=0 에서 v0 로 시작 → 매 step 3 m/s² 씩 감속 → 수 step 뒤에 cap 도달. 솔버는 이 램프 profile 안에서 feasible 해를 찾으면 됨. 자연스러운 감속.

노드 쪽 `trail_vel_ramp_ticks = 8` 은 컨트롤러에 넘기는 명령 속도 자체를 부드럽게 만드는 별도 완충.

---

## 7. 4-tier Fallback Ladder

NLP 는 가끔 실패함 — narrow corridor, 장애물 충돌, numerical 이슈. 그때 **안전한 뭔가라도 발행해야** 해서 4단계 복구 체계:

| tier | 상태 | 내용 | 언제 |
|--:|---|---|---|
| 0 | `OK`                      | NLP 성공 (pass 1: caller side) | 정상 |
| 0 | `OK (pass 2)`             | NLP 성공 (pass 2: TRAIL 재시도) | LEFT/RIGHT 실패 후 TRAIL 시도해서 성공 |
| 1 | `HOLD_LAST`               | **직전 성공 궤적을 s-shift 해서 재발행** | `fail_streak ≤ fail_streak_H(5)` 일 때 |
| 2 | `GEOMETRIC_FALLBACK`      | Quintic(5차 다항식) 기하 궤적 Δs≈8m | 1이 소진되면 |
| 3 | `CONVERGENCE_QUINTIC`     | 짧은 Δs≈4m quintic | 2도 실패 |
| 3 | `RACELINE_SLICE`          | raceline 의 n=0 slice 그대로 | 최후의 보루 |

quintic fallback 은 [`geometric_fallback.py`](planner/mpc_planner/src/geometric_fallback.py) 의 `build_quintic_fallback` — 현재 (s, n) 에서 목표 (s+Δs, 0) 로 C²-연속 5차 다항식 생성.

### RViz 무지개색 매핑 (v3c)

궤적 marker 색상은 **솔버 건강 상태의 직관적 지표**:

| 색 | 의미 |
|---|---|
| 🔴 red    | tier0 pass1 — NLP 1-pass 클린 (정상) |
| 🟠 orange | tier0 pass2 — TRAIL 재시도로 구제 |
| 🟡 yellow | tier1 — HOLD_LAST (직전 궤적 재사용) |
| 🟢 green  | tier2 — quintic Δs=8m |
| 🔵 cyan   | tier3 — CONVERGENCE_QUINTIC Δs=4m |
| 🔷 blue   | tier3 — RACELINE_SLICE (최후) |

RViz 보면서 **색이 빨강 → 파랑으로 치우치면 솔버 건강 악화**. 튜닝 중 가장 빠른 시각 피드백.

---

## 8. IPOPT & 솔버 가속

### 8.1 IPOPT 가 무엇?

**IPOPT** (Interior Point OPTimizer)는 비선형 NLP 를 푸는 open-source 솔버. Newton 스텝으로 KKT 조건 만족하는 해 찾음. 한 IPOPT iteration 에서 제일 비싼 건 **linear system solve** (KKT 행렬의 factorize).

### 8.2 Linear solver 선택

| 옵션 | 속도 | 라이선스 | 의존성 |
|---|---|---|---|
| `mumps` (default) | 느림 | free | CasADi 기본 포함 |
| `ma27` (HSL) | **~2× 빠름** | 학술/비상용 무료 | `libhsl.so` 필요 |
| `ma57`/`ma86`/`ma97` | 문제 크기/희소성에 따라 | HSL | 동상 |

현재 설정: `linear_solver: ma27`. 효과 (v3c+ 벤치):
- MUMPS: p50 45.5 ms
- ma27 단독: p50 21.6 ms (**2.11×**)
- ma27 + JIT: p50 9.3 ms (**4.9×**)
- p99: 91.1 → 32.5 ms (**2.8×**)

### 8.3 CasADi JIT

**JIT (Just-In-Time) compilation**: CasADi 가 생성한 symbolic 함수(objective, constraint, Jacobian, Hessian)를 **setup 시 native C로 컴파일** → 매 tick 함수 평가 비용 ↓.

- 파라미터: `ipopt_jit: true`, `ipopt_jit_flags: ['-O3', '-march=native', '-ffast-math']`
- 비용: 노드 시작 시 ~10-20초 추가 컴파일
- 이득: runtime 함수평가 가속 (HSL 이후 남은 bottleneck)
- 끄려면: YAML에서 `ipopt_jit: false`

### 8.4 HSL 없는 환경에서의 자동 fallback

`_resolve_linear_solver` 가 `libhsl.so` 로드 가능 여부를 **ctypes.CDLL probe** 로 확인:
- 있으면 요청한 ma27 사용
- 없으면 `warnings.warn` 띄우고 MUMPS 로 자동 전환

→ 다른 팀원 기기에서도 YAML 수정 없이 돌아감. HSL 빌드는 `planner/3d_gb_optimizer/fast_ggv_gen/solver/setup_hsl.sh`.

### 8.5 기타 IPOPT 파라미터
- `ipopt_max_iter = 200` — 반복 상한. 보통 ~15회면 수렴. 실패 시 40 ms에 clip → fallback ladder 빠르게 작동.
- `ipopt_print_level = 0` — 로그 침묵.

---

## 9. 전체 파라미터 레퍼런스 (state_overtake.yaml)

한 눈에 보는 cheat sheet:

```yaml
# ─── 기본 ───
planner_freq: 30           # Hz, plan loop rate
N: 20                      # horizon steps
dT: 0.05                   # s per step → 1.0s lookahead
vehicle_L: 0.33            # wheelbase (m)

# ─── 물리 한계 ───
max_speed: 10.0            # hard upper v bound
min_speed: 0.0
max_steering: 0.6          # rad (~34°)
delta_rate_max: 3.0        # rad/s
# (a_min/a_max 는 코드에서 −4/+3)
a_dec_ramp: 3.0            # TRAIL 감속 램프 (m/s²)

# ─── Cost: raceline contour ───
q_n: 3.0                   # Σn² running
q_n_term: 10.0             # n_N² terminal
q_v_term: 0.5              # (v_N − v_ref)² terminal

# ─── Cost: smoothness ───
r_a: 0.5                   # Δa²
r_steer_reg: 0.1           # δ²
r_dd: 5.0                  # δ̇²  (rate)
r_dd_rate: 1.0             # Δδ̇²  (jerk)

# ─── Cost: progress ───
gamma_progress: 10.0       # −γ·Σv·cos(μ)·dT

# ─── Cost: obstacle bubble ───
w_obs: 180.0               # peak
sigma_s_obs: 0.7           # 종방향 scale
sigma_n_obs: 0.18          # 횡방향 scale
n_obs_max: 2               # 동시 처리 장애물 수 (TODO: 재검토)

# ─── Cost: side bias (LEFT/RIGHT) ───
w_side_bias: 25.0
gap_lat: 0.25              # bias zero-line clearance

# ─── Cost: wall cushion ───
w_wall_buf: 2500           # 매우 강함 (w_obs 대비 ×13)
wall_buf: 0.30             # 쿠션 두께

# ─── Cost: continuity ───
w_cont: 200                # tick-to-tick n 고정

# ─── Cost: slack ───
w_slack: 5000              # corridor 위반

# ─── Corridor geometry ───
wall_safe: 0.08            # body-edge 최소 여유 (hard)
boundary_inflation: 0.05
# (ego_half_width = 0.15 는 코드 default)

# ─── SideDecider ───
side_hold_ticks: 10        # LEFT↔RIGHT flip 관성
trail_entry_ticks: 3       # ANY→TRAIL 빠른 진입
min_pass_margin: 0.10      # feasibility gate

# ─── IPOPT ───
ipopt_max_iter: 200
ipopt_print_level: 0
linear_solver: ma27        # mumps 자동 fallback
ipopt_jit: true

# ─── 기타 ───
obs_ema_alpha: 0.30        # 장애물 s/n EMA (3-tick 시상수)
trail_vel_ramp_ticks: 8    # 컨트롤러 명령 부드럽게
collision_mode: soft       # (legacy, v3에서는 무의미)
```

---

## 10. Data Flow — ROS 토픽 지도

### 10.1 입력 (Subscribers)
| 토픽 | 용도 |
|---|---|
| `/global_waypoints` (WpntArray)      | raceline (s, x, y, z, ψ, κ, d_L, d_R, v_x) cache |
| `/car_state/pose` (PoseStamped)      | ego pose |
| `/car_state/odom` (Odometry)         | ego v, yaw_rate |
| `/car_state/odom_frenet` (Odometry)  | ego (s, n) — **3D-aware** — 직접 사용, xy→frenet 라운드트립 금지 |
| `/opponent_prediction/obstacles`     | Primary 예측 장애물 (100ms stale threshold) |
| `/tracking/obstacles`                | Fallback detection-only |
| `/opponent_trajectory`               | 예측기 kinematic cache |

### 10.2 출력 (Publishers)
| 토픽 | 타입 | 용도 |
|---|---|---|
| `~out/otwpnts` → `/planner/avoidance/otwpnts_observation` | OTWpntArray | **주 출력** (observation: state_machine 직접 연동 X) |
| `~best_trajectory_observation` (WpntArray) | 최종 후보 경로 | 다른 플래너와 포맷 호환 |
| `~best_sample/markers` (MarkerArray) | RViz trajectory (무지개색) |
| `~debug/tick_json` (String) | **매 tick JSON** — Claude 라이브 디버깅 핵심 |
| `~debug/markers` (MarkerArray) | corridor, 장애물 버블, ref-slice 시각화 |
| `~status` (String, latched) | "INIT"/"OK"/"HOLD_LAST"/... |
| `~timing_ms` (Float32) | solve_ms |

**관건**: `~out/otwpnts` 의 **remap suffix `_observation`** 이 중요. state_machine 은 이 토픽 안 본다 — 완전 검증 전까지 observation-only 운용.

### 10.3 장애물 주입 우선순위
```
/opponent_prediction/obstacles      (fresh < 100ms)  ← primary
 └─ stale → /tracking/obstacles                      ← fallback
     └─ stale → cost 비활성화 (w_obs·0, CLEAR)
```

---

## 11. Live Debugging Workflow

CLAUDE.md 원칙: **Claude 가 직접 터미널에서 `rostopic echo` 돌리며 수치 기반 디버깅**. 이 노드가 그 루프의 주 도구인 `~debug/tick_json` 을 발행.

### 11.1 Tick JSON 주요 필드
매 tick (30Hz) `std_msgs/String` JSON 한 줄:

| 필드 | 의미 |
|---|---|
| `tick`, `dt_ms`       | 번호, solve time |
| `ipopt_status`, `iter` | IPOPT 반환/반복 |
| `tier`, `status`, `pass` | fallback 상태 |
| `side`, `side_str`, `scores` | SideDecider 결정 + d_free_L/R + reason |
| `cost`: {contour, progress, obs, bias, wall, cont, term, slack} | cost breakdown |
| `margin_L_min`, `margin_R_min` | horizon 중 벽 최소 여유 (centroid) |
| `slack_max` | slack 최대값 (0 이면 corridor 깨끗) |
| `jitter_rms_m` | 직전 tick 궤적 대비 RMS (<0.1 m 목표) |
| `n_obs_used`, `obs_source` | 주입된 장애물 수 + 소스 태그 |
| `last_input`, `last_infeas_info` | 실패 시 재현/진단용 |

### 11.2 디버깅 루프 (CLAUDE.md 방식)
1. 수정 → Docker 안 `catkin build mpc_planner`
2. `roslaunch mpc_planner mpc_planner_state.launch state:=overtake` (background)
3. **별도 터미널**: `rostopic echo -c /planner/mpc_planner/debug/tick_json`
4. 수치 읽고 가설 수립 ("cost_wall=500 < cost_obs=300 → wall 약함, w_wall_buf↑")
5. 수정 → 다시 관측 → 수렴까지 반복
6. 합격선 진입 (`solve_ms p95 < 25`, `margin_min > 0.15m`, `jitter < 0.05m`) → 사용자에게 숫자와 함께 리포트

### 11.3 "느낌상 좋아졌다" 금지
무조건 **구체 숫자** 인용. 예: "ma27 적용 후 `dt_ms` p50 45.5 → 21.6 ms" — 바로 검증 가능.

---

## 12. 알려진 한계 & TODO

### 해결됨
- ✅ v2c knot-wise κ 불연속 → v3 δ-state + δ̇-control
- ✅ tick-to-tick n jitter p95 0.28m → w_cont=200 으로 <0.1m
- ✅ wall 침투 > 장애물 회피 → w_slack=5000, w_wall_buf=2500 로 역전
- ✅ TRAIL 진입 시 infeasibility → v_max ramp from v0 (a_dec_ramp=3)
- ✅ body-edge 안 고려 → ego_half 를 corridor/cushion 에 포함
- ✅ solve time → ma27 + JIT (p50 45.5 → 9.3 ms, 4.9×)
- ✅ HSL 없는 환경 → MUMPS 자동 fallback

### Open
- **장애물 cost 를 body-edge distance 기반으로 재정의** — 현재는 centroid–centroid distance. 장애물이 작지 않은 경우 회피 timing 미세 어긋날 여지. (`HJ_docs/mpc_frenet_kin_v3c_live_debugging_20260421.md §5`)
- **`n_obs_max = 2` 재검토** — 실제 runtime 상대차 수 vs 솔버 cost. 1 이면 충분할 수도. (§6)
- **실상 1대 장애물이 2개로 찍히는지 체크** — predictor 쪽 duplicate 의심. (`§6.3` 에 TODO 기록)
- `carstate_node` 필요 여부 — GLIL base_odom 이 v_y, pitch 충분한가? (프로젝트 루트 CLAUDE.md 미확인)

### 바꾸지 말 것 (교훈)
- `w_wall_buf >> w_obs` 비율 (벽은 절대)
- `w_cont` 200 ± 몇배 내에서만 (0 이면 jitter 부활, 1000 이면 장애물 추종 실패)
- `δ` 를 다시 input 으로 되돌리기 (knot 불연속 부활)
- stage ramp 부활 (U자 궤적 부활)

---

## 13. 한 장 요약 (Elevator Pitch)

> 우리 MPC는 **Frenet (s, n) 평면**에서 **kinematic bicycle** 동역학을 쓰는 **20-step, 0.05s, 30Hz receding-horizon NLP**다. 상태에 조향각 δ까지 포함시켜 곡률 연속성(C¹) 을 보장하고, 입력은 가속도·조향률이다. cost는 (1) raceline pull, (2) Gaussian 장애물 버블, (3) 벽 쿠션, (4) tick-to-tick 연속성, (5) progress, (6) smoothness 페널티들을 가중합. 장애물 회피 **방향** 은 NLP 밖의 룰 기반 **SideDecider** 가 결정 → NLP는 한쪽 hinge만 풀어 볼록성 유지. 실패 시 HOLD_LAST → quintic → raceline 순의 **4-tier fallback**. 솔버는 IPOPT+HSL(ma27)+CasADi JIT로 p50 9ms. "벽은 절대, 장애물은 선호, raceline은 귀환선" — 세 계층이 w_slack(5000) > w_wall_buf(2500) > w_obs(180) 순으로 비용 서열을 이뤄 의도된 행동이 나옴.

---

## 부록 A. 자주 나오는 질문

**Q1. `w_obs` 랑 `w_wall_buf` 둘 다 "피하기" 인데 왜 구분?**
A. 벽은 **물리적 절대** (뚫으면 크래시), 장애물은 **전술적 선호** (돌아갈 수 있으면 돌지만 안 되면 느리게라도 따라감). 비용 서열이 뒤집히면 솔버가 벽으로 밀려가 장애물 피하는 망측한 플랜을 뽑음.

**Q2. `planner_freq = 30` 인데 더 올리면?**
A. solve_ms p50 9 ms → 이론상 100 Hz까지 됨. 하지만 (a) controller 주기(40 Hz)와의 정합성 (b) predictor freshness (10–20 Hz), (c) warm-start 이득 감소 때문에 30이 sweet spot. 노드 `rospy.get_param('~planner_freq', 30)` 에서 변경 가능.

**Q3. solver 실패 시 궤적 안 바뀌면 로봇은 뭘 함?**
A. tier1 (HOLD_LAST) — 직전 solve 의 `n_prev` 를 s 방향으로 한 칸 shift 해서 재발행. 위치 일관성 유지. `fail_streak_H = 5` tick 동안만 유효 → 그 뒤 tier2 기하학 fallback.

**Q4. SideDecider 가 `trail` 쪽으로 붙여놨는데 실제로 "뒤에 따라붙음" 을 어떻게 보장?**
A. `_build_vmax` 에서 `vmax[k] = min(ref_v[k], v_obs × 0.95 ramped)` 로 cap. 솔버의 progress 항 `−γv·cos μ` 는 이 상한 안에서 최대 속도를 당길 뿐. 그래서 "obstacle 속도 × 0.95" 로 자연히 수렴.

**Q5. 튜닝 시작 시 어떤 파라미터부터?**
A. 우선순위:
1. **`q_n` vs `gamma_progress`** — 레이싱 공격성. 보통 `gamma / q_n ≈ 3–5` 가 균형.
2. **`w_obs`, `sigma_n_obs`** — 장애물 회피 거리감. d=1m 에서 cost≈50 이 1차 목표.
3. **`w_wall_buf`** — 반드시 `w_obs × 10` 이상.
4. **`w_cont`** — jitter 보고 가감.
5. 나머지 smoothness 항들은 마지막.

---

## 부록 B. 참고 문서

- [mpc_redesign_frenet_kin_20260420.md](mpc_redesign_frenet_kin_20260420.md) — v3 재설계 배경
- [mpc_frenet_kin_v3c_live_debugging_20260421.md](mpc_frenet_kin_v3c_live_debugging_20260421.md) — v3c 튜닝/HSL 벤치마크
- [mpc_planner_state_machine_integration.md](mpc_planner_state_machine_integration.md) — 상위 연동 계획
- [mpcc_planner_trial_v1.md](mpcc_planner_trial_v1.md) — MPCC 실험 기록
