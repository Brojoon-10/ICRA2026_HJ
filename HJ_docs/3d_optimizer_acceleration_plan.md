# 3D Optimizer Acceleration Plan

`planner/3d_gb_optimizer/global_line/global_racing_line/gen_global_racing_line.py` 의 전체 최적화 (경로 + 속도 동시) 속도를 개선하기 위한 전략 정리.

---

## 0. 현재 상태

- **NLP 크기**: ~3,155 변수 / ~3,608 제약 (트랙 수백 m, step 0.2 m 기준)
- **기존 속도**: 약 60초 (MUMPS + limited-memory Hessian + cold start)
- **현재 속도**: HSL ma27 적용 → 예상 20~40초 (2~3배 향상)

**이미 적용된 최적화:**
- ✅ HSL ma27 선형 solver + MUMPS fallback probe  
  ([참고: fast_ggv_gen/solver/README.md](../planner/3d_gb_optimizer/fast_ggv_gen/solver/README.md))
- ✅ IY의 tolerance 완화 (`tol`/`dual_inf_tol`/`acceptable_*`) — V_min 디제너러시 대응
- ✅ 기본 scaling (a_x_s, u_s, delta_s 등)

---

## 🟢 쉬움 + 효과 있음

### 1. Exact Hessian 전환

**변경:**
```python
"ipopt.hessian_approximation": "exact"  # 현재: "limited-memory"
```

**내용:**
- **L-BFGS (limited-memory)**: 매 iter 근사 Hessian. iter당 빠르지만 많이 필요.
- **exact**: CasADi가 symbolic으로 계산한 진짜 Hessian. iter당 느리지만 훨씬 적게 수렴.
- 중간 규모 NLP (수천 변수) 에서는 exact가 **20~50% 더 빠른 경우 많음**.
- 실패해도 한 줄 원복하면 끝.

**테스트 비용**: 1분.  
**위험도**: 낮음. 메모리 사용량이 늘어서 수렴 실패하면 원복.

---

### 2. Warm start 구조화

**상황:**
- 현재: `V_guess = 3.0` 균일 cold start
- 파라미터/GGV만 바꿔서 여러 번 돌릴 때, 이전 해를 초기값으로 쓰면 **iter 수 절반 이하**
- 튜닝 루프 ([fast_ggv_gen + 3d_optimized_vel_planner + 경로 재최적화](./fast_ggv_gen.md)) 에서 특히 유효

**구현 방향:**
- 이전 실행 결과(`raceline.csv`)의 `n_opt, chi_opt, v_opt, ax_opt, ay_opt`를 초기 `w0`에 채움
- cost function 값이 이미 낮은 지점에서 시작 → IPOPT가 빠르게 수렴

**테스트 비용**: 반나절 (w0 구성 로직 추가).  
**위험도**: 낮음. 초기값이 나빠도 IPOPT가 수렴해야 정상 동작하는 구조.

---

### 3. Step size 키우기

**변경:**
```python
'step_size_opt': 0.3   # 현재 0.2
```

**효과:**
- NLP 그리드 ~400 → ~270 포인트
- **1.5~2배 빠름**
- 정확도 약간 저하 (코너 주변 해상도 감소)

**트레이드오프:**
- 짧은 트랙, 매우 타이트한 코너 많으면 0.2 유지
- 긴 트랙, 완만한 코너 위주면 0.3~0.5까지 가능

**테스트 비용**: 5분.  
**위험도**: 낮음. 결과 품질만 비교하면 됨.

---

### 4. 곡률 기반 초기 속도 프로파일

**상황:**
- cold start에서도 `V_guess = 3.0` 균일은 너무 naive
- 직선에서는 V 높고 코너에서 낮아야 하는데, 균일 초기값이면 IPOPT가 많은 iter 소비해서 이걸 맞춰야 함

**구현:**
```python
# 트랙 곡률(kappa) 기반 heuristic
V_init(s) = V_max * sqrt(1 - kappa(s) / kappa_max)
```
- 즉 코너에서 미리 감속한 프로파일로 시작
- **수렴 iter 약 절반**

**테스트 비용**: 1~2시간 (Track3D에서 kappa 추출 + w0 구성).  
**위험도**: 낮음.

---

## 🟡 중간 난이도

### 5. NLP scaling 재조정

**상황:**
- 원본에 이미 scaling factor (`a_x_s = a_max` 등)가 들어 있음
- IY가 V_min 등 추가한 뒤로 scaling이 여전히 최적인지 검증 안 됨
- IPOPT의 자동 scaling도 옵션:
```python
"ipopt.nlp_scaling_method": "gradient-based"
```

**효과:**
- 수렴 iter 감소 (20~40%)
- numerical conditioning 개선

**테스트 비용**: 반나절.  
**위험도**: 중간. scaling이 틀리면 오히려 수렴 실패.

---

### 6. 약한 제약 완화

**상황:**
- 현재 `constr_viol_tol = 1e-4`로 타이트 (물리적 유효성 보장)
- GGV diamond 제약은 부드러운 곡선인데, IPOPT가 **exact boundary를 찾느라 많은 iter 소비**
- 허용 가능한 범위라면 `1e-3`까지 완화 가능

**효과:** iter 10~20% 감소, lap time 오차 무시 가능 수준

**테스트 비용**: 30분.  
**위험도**: 중간. 물리적으로 유효한 trajectory가 나오는지 검증 필요.

---

## 🔴 큰 변경 (효과 크지만 리스크)

### 7. CasADi JIT 컴파일

**변경:**
```python
opts['jit'] = True
opts['jit_options'] = {"flags": ["-O2"]}
```

**내용:**
- NLP의 모든 함수(f, g, Jacobian, Hessian)를 C로 컴파일
- Call당 훨씬 빠름 (Python/CasADi 오버헤드 제거)

**트레이드오프:**
- **빌드에 수 초** 걸림
- 한 번만 실행이면 오히려 느려질 수도
- **여러 번 반복 실행** (튜닝 루프, 스윕) 할 때 유효

**테스트 비용**: 반나절 (build cache 경로 설정).  
**위험도**: 중간. 컴파일러 의존성 문제 가능.

---

### 8. Problem reformulation

**접근 방향들:**

**A. State 축소** — 이미 `3d_optimized_vel_planner.py`에서 경로 고정 버전으로 시도 (V, ax만). 원본은 경로도 최적화해야 하므로 단순 축소 불가.

**B. 변수 선택 변경** — 예:
- 현재: `x = [V, n, chi, ax, ay]`
- 대안: `x = [V, n, chi, F_x, F_y]` (힘을 직접 decision variable로)
- IPOPT가 다르게 접근할 수도

**C. Multi-phase / multi-segment** — 트랙을 여러 구간으로 나눠 따로 풀고 경계 매칭. 각 phase가 작아짐 → 병렬화 가능.

**테스트 비용**: 수일 이상.  
**위험도**: 높음. 결과 검증 많이 필요.

---

## 추천 실행 순서

| 순위 | 항목 | 예상 효과 | 노력 |
|-----|------|----------|------|
| 1 | exact Hessian 실험 | +20~50% | 🟢 1분 |
| 2 | step size 0.2 → 0.3 | +50~100% | 🟢 5분 |
| 3 | 곡률 기반 초기값 | +50~100% | 🟢 1~2시간 |
| 4 | warm start 구조화 | 반복 실행 +50%+ | 🟢 반나절 |
| 5 | Scaling 재조정 | +20~40% | 🟡 반나절 |
| 6 | Problem reformulation | 잠재적 2배+ | 🔴 수일 |

**단기 목표**: 1+2+3 조합해서 **60s → 10~20s** 달성.  
**중기 목표**: 위 + warm start로 반복 실행 시 **< 5초**.  
**장기 목표**: reformulation으로 근본적 속도 개선.

---

## 참고

- `planner/3d_gb_optimizer/global_line/global_racing_line/gen_global_racing_line.py` — 원본
- `gen_global_racing_line_backup_0415.py` — HSL 적용 직전 스냅샷
- `planner/3d_gb_optimizer/fast_ggv_gen/solver/README.md` — HSL 설치 가이드
- `HJ_docs/3d_optimized_vel_planner.md` — 경로 고정 속도 최적화 노드 (유관 최적화 사례)
- `HJ_docs/fast_ggv_gen.md` — GGV 생성 가속화 사례 (parametric NLP + fork 병렬)
