# 3D Optimized Velocity Planner

## 개요

3D 경로 고정 상태에서 **속도 프로파일만 재최적화**하는 ROS 노드.  
기존 `gen_global_racing_line.py`는 경로(n, chi)와 속도를 동시에 최적화해서 ~1분 걸리는데, 이 노드는 **경로를 고정**하고 속도만 최적화해서 **~3초**에 끝난다.

**목적**: 타이어 파라미터/GGV 튜닝 시, 같은 경로 위에서 새 GGV로 속도 프로파일이 어떻게 변하는지 빠르게 확인.

파일 위치: `stack_master/scripts/3d_optimized_vel_planner.py`

---

## 구현 내용

### 핵심 아이디어: Reduced-state NLP

기존 NLP (`gen_global_racing_line.py`)는 **state = [V, n, chi, ax, ay]** (5개), **control = [jx, jy]** (2개)로 경로 전체를 최적화한다. 우리는 **n과 chi를 고정 파라미터로** 취급해서 NLP를 축소한다.

| | 원본 | 이 노드 |
|---|---|---|
| state | V, n, chi, ax, ay | **V, ax** |
| control | jx, jy | **jx** |
| ay | decision variable | **algebraic** (대수적으로 결정) |
| 변수 수 (~400 pts) | ~3,155 | **~1,193** |
| 수렴 iter | ~수백 | **~35** |
| 시간 | ~60초 | **~3초** |

### 수학적 근거: 결과가 동일한가?

원본 NLP의 dynamics 식 `dchi/ds = ay/(V·s_dot) - Ω_z`에서,
**chi(s)가 고정되면** ay는 다음과 같이 **대수적으로** 결정된다:
```
ay = V · s_dot · (dchi/ds + Ω_z)
```

즉 원본 최적해 `(V*, n*, chi*, ax*, ay*)`에서 위 관계가 **이미 성립**하므로, n/chi를 고정한 새 NLP의 해는 `(V*, ax*)`와 **같다** (같은 비용, 같은 제약, 같은 최적점).

나머지는 전부 원본과 동일:
- `Track3D.calc_apparent_accelerations(V, n, chi, ax, ay, s, ...)` 그대로 호출 (n, chi만 상수로)
- GGV diamond 제약 동일
- Cost function `min lap time + jerk regularization` 동일
- RK4 4차 적분 동일

### 경로 고정 (loop closure)

Track3D의 `resample(step_size)`는 고정 step을 쓰므로 `(N-1)·step ≠ L_track`인 경우가 많다. 원본 optimizer는 이 차이로 마지막 0.2m가량 최적화되지 않고 periodic boundary `V[0] == V[N-1]`가 실제 loop closure와 엄밀히 일치하지 않는다.

이 노드는 **트랙 길이에 정확히 맞는 step을 자동 계산**한다:
```python
L_track = track.s[-1] + track.ds
N = round(L_track / desired_step)    # ≈ 398
actual_step = L_track / N             # ≈ 0.19983
track.resample(actual_step)
```

→ NLP 그리드가 `[0, actual_step, ..., (N-1)·actual_step]`로 **전체 트랙을 정확히** 덮고, periodic boundary가 **s=0 ↔ s=L_track** (실제 loop closure)과 일치.

### Dynamics (reduced)

```
s_dot  = V·cos(chi) / (1 - n·Ω_z(s))              ← n, chi는 고정 함수
ay     = V·s_dot·(dchi/ds + Ω_z(s))                ← 대수적으로 결정
dV/ds  = ax / s_dot
dax/ds = jx / s_dot                                 ← jx만 control
```

GGV diamond 제약은 `Track3D.calc_apparent_accelerations`로 apparent (ax_tilde, ay_tilde, g_tilde)를 계산해서 그대로 적용.

### Warm start

`<raceline>.csv`의 `v_opt`, `ax_opt`를 IPOPT 초기 guess로 넣는다. cold start (V=3.0 균일)로도 같은 해에 수렴하지만 warm start가 iter 수를 크게 줄인다.  
**이 NLP는 거의 convex 구조**라서 초기값에 따른 local optimum 편향 위험 없음.

---

## ROS 토픽 구조

```
subscribe: /global_waypoints  (WpntArray, latched)
            ↓
            [첫 메시지 하나만 받고 unregister]
            ↓
            메시지를 템플릿으로 사용 (x, y, z, psi, kappa, ...)
            ↓
            NLP 결과(V, ax)를 각 waypoint의 s_m 기준으로 보간해서 덮어쓰기
            ↓
publish:   /global_waypoints  (same topic, latched)
```

**같은 토픽에 재퍼블리시**하므로 기존 소비자(vel_planner, controller 등)는 추가 수정 없이 새 속도 프로파일을 사용한다. latched message 특성상 우리가 발행한 게 덮어쓴다.

NLP는 **노드 시작 시 1회**만 돌고, 토픽 수신은 메시지 템플릿 취득용. 한 번 퍼블리시 후 spin만 한다.

### 출력 보간 (periodic wrap)

NLP가 푸는 그리드 길이와 토픽의 waypoint 수는 다르므로(예: 398 vs 750), 결과를 토픽의 `s_m`에 맞춰 보간해야 한다. periodic closure를 유지하기 위해:
1. `V_opt` 끝에 `V_opt[0]` 추가 → 순환 배열
2. 토픽의 `s_m`을 NLP 그리드 범위로 스케일링
3. `np.interp` 선형 보간

→ 시작점과 끝점이 부드럽게 연결된다.

---

## 사용법

### 기본 명령

```bash
rosrun stack_master 3d_optimized_vel_planner.py \
    --map <MAP> \
    --raceline <RACELINE_VARIANT> \
    --vehicle_yml <VEHICLE_YML_FILENAME> \
    --gg_dir <GG_DIR_NAME>
```

### 경로 자동 해결

폴더 구조는 **고정**, 파일명만 인자로 넘긴다.

| 인자 | 실제 경로 |
|------|----------|
| `--map eng_0410_v5` | `stack_master/maps/eng_0410_v5/` (map folder) |
| | → track: `<map>/<map>_3d_smoothed.csv` (자동) |
| `--raceline rc_car_10th` | → raceline: `<map>/<map>_3d_<raceline>_timeoptimal.csv` |
| `--vehicle_yml params_rc_car_10th.yml` | `planner/3d_gb_optimizer/global_line/data/vehicle_params/params_rc_car_10th.yml` |
| `--gg_dir rc_car_10th` | `planner/3d_gb_optimizer/global_line/data/gg_diagrams/rc_car_10th/velocity_frame/` |

### 예시

```bash
# 기본 rc_car_10th
rosrun stack_master 3d_optimized_vel_planner.py \
    --map eng_0410_v5 \
    --raceline rc_car_10th \
    --vehicle_yml params_rc_car_10th.yml \
    --gg_dir rc_car_10th

# latest 세팅으로
rosrun stack_master 3d_optimized_vel_planner.py \
    --map eng_0410_v5 \
    --raceline rc_car_10th \
    --vehicle_yml params_rc_car_10th_latest.yml \
    --gg_dir rc_car_10th_latest

# v1 세팅으로
rosrun stack_master 3d_optimized_vel_planner.py \
    --map eng_0410_v5 \
    --raceline rc_car_10th \
    --vehicle_yml params_rc_car_10th_v1.yml \
    --gg_dir rc_car_10th_v1
```

`--raceline`은 **고정할 경로 정보 (n, chi) 소스**이고, `--vehicle_yml + --gg_dir`는 **재최적화에 쓸 차량/GGV**다. 이 둘을 다르게 주면 "**X 세팅으로 만든 경로를 Y 세팅의 GGV로 재평가**" 같은 비교가 가능하다.

### 선택 인자

| 인자 | 기본값 | 설명 |
|------|-------|------|
| `--step_size_opt` | 0.2 | NLP 그리드 간격 (m). 실제 step은 트랙 길이에 맞게 자동 조정됨. |
| `--V_min` | 0.0 | 최소 속도 하한 (m/s) |
| `--gg_margin` | 0.0 | GGV 여유 |

---

## 실행 전제 조건

- `roscore`가 이미 떠 있을 것
- `/global_waypoints` 토픽이 latched로 발행되어 있을 것 (예: `stack_master` base_system.launch)
- 컨테이너: `icra2026` Docker 안에서 실행 (`rosrun` 가능한 환경)
- `catkin build` 완료 (package 인식 필요)

---

## 동작 예시 로그

```
[velopt] map=eng_0410_v5
[velopt] track    : .../eng_0410_v5_3d_smoothed.csv
[velopt] raceline : .../eng_0410_v5_3d_rc_car_10th_timeoptimal.csv
[velopt] vehicle  : .../params_rc_car_10th.yml
[velopt] gg       : .../gg_diagrams/rc_car_10th/velocity_frame
[velopt] L_track=79.5320m, desired_step=0.2000, actual_step=0.199829 (N=398)
Resampled track: 398 points
[velopt] Track3D + GGManager + raceline ready, grid=398 pts
[velopt] fixed n  : [-0.396, 0.698] m
[velopt] fixed chi: [-0.543, 0.481] rad
[velopt] solving NLP ...
[velopt] NLP: 1193 vars, 1990 constraints, 398 points
[velopt] solver built in 0.58s, solving...
[velopt] IPOPT: 2.52s, success=True, laptime=18.8604s
[velopt] V range [1.89, 6.96] m/s  success=True
[velopt] waiting for /global_waypoints template message ...
[velopt] published new /global_waypoints (velocity overwritten)
```

총 소요: **~3~4초** (build 0.6s + NLP 2.5s + message I/O)

---

## 제약 & 주의사항

1. **경로는 절대 안 바뀐다** — n, chi는 입력 csv 고정. 경로 자체를 바꾸고 싶으면 원본 `gen_global_racing_line.py` 사용.
2. **GGV와 raceline이 너무 달라지면** IPOPT가 수렴 못 할 수 있음. 예를 들어 매우 낮은 grip GGV로 고그립 raceline을 풀면 feasibility 문제 생김.
3. **파일 생성 없음**. 토픽에만 퍼블리시. 저장하려면 따로 작업 필요.
4. **한 번만 풀고 spin**. rqt 기반 실시간 튜닝은 추후 확장 (노드 재실행 또는 topic-triggered re-solve로).

---

## 향후 계획 (rqt 실시간 튜닝 연동)

현재 구조는 다음 워크플로우의 기반:

```
[rqt 슬라이더]
  타이어 파라미터 (lambda_mu, P_max, ...) 조절
       │
       ├── fast_ggv_gen으로 GGV 재생성 (~2초)
       │
       ├── 3d_optimized_vel_planner로 속도 재계산 (~3초)
       │    (노드 재실행 or topic-triggered re-solve)
       │
       └── /global_waypoints에 새 속도 덮어쓰기
           → 실차/시뮬에서 즉시 반영
```

파라미터 변경 → 시뮬 반영까지 **< 10초** 루프 가능.

---

## 관련 파일

- `stack_master/scripts/3d_optimized_vel_planner.py` (이 노드)
- `planner/3d_gb_optimizer/global_line/global_racing_line/gen_global_racing_line.py` (원본, 참고용, **수정하지 않음**)
- `planner/3d_gb_optimizer/global_line/src/track3D.py` (Track3D, library로 import)
- `planner/3d_gb_optimizer/global_line/src/ggManager.py` (GGManager, library로 import)
- `planner/3d_gb_optimizer/fast_ggv_gen/` (GGV 고속 생성기, 같이 쓰면 튜닝 루프 완성)
- `HJ_docs/fast_ggv_gen.md` (fast GGV 문서)
