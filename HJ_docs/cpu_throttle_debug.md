# CPU Throttling on NUC14RVB — Diagnosis & Fix

> 환경: ASUS NUC14RVK-B / Core Ultra 7 155H (Meteor Lake-H, 6 P-core + 8 E-core + 2 LP-E, 22 logical CPU) / Ubuntu 24.04 / 4S 배터리 운용
> 대상: 같은 stack을 다른 머신에 올리거나, 이 머신이 다시 throttle 걸릴 때 빠르게 진단/대응하기 위한 문서

---

## 한 줄 요약

> Intel ITMT가 단일 favorite core를 5GHz까지 boost 시키는데, 그 순간 **per-core 전류 한도(ICCMAX)** 에 부딪혀 throttle. 온도도 패키지 전력도 사유 아님. **boost 클럭 천장만 70% (~3.3GHz)로 잡으면 끝.** 명령 한 줄: `set_pl.sh set 64 64 70`

---

## 1. 증상

ROS/glim 풀스택 돌리는 중 다음이 동시에 발생:

- **컴퓨터 전체가 들쑥날쑥 느려짐** (jitter — ROS 콜백 지연, control 노드까지 영향)
- `core_throttle_count` 카운터가 초당 5~14회 증가
- 코어 온도는 80~85°C — Tjmax(110°C) 한참 못 미침
- 패키지 전력 27W — PL1 cap(40~60W)도 한참 못 미침

**겉보기 모순**: 온도도, 전력도, 사용률도 다 여유 있는데 throttle이 끊임없이 발생.

## 2. 진짜 원인 — ICCMAX

`thermal_throttle/core_throttle_count`에는 **6가지 사유가 모두 합산**됩니다:

| 사유 | 본다 | 발동 시간 |
|---|---|---|
| Tjmax | 코어 온도 | 100~110°C |
| PL1 (long_term) | 패키지 12초 평균 전력 | seconds |
| PL2 (short_term) | 패키지 2.44ms burst 전력 | ms |
| **ICCMAX** | **단일 코어 instantaneous 전류** | **µs** |
| VR Therm Alert | VRM 온도 | ms |
| PROCHOT (외부) | 보드/VRM 신호 | ms |

이름이 "thermal"이지만 실제로는 어느 사유든 합산됨. **우리 케이스는 100% ICCMAX**.

### 메커니즘

```
1. Intel ITMT/HFI: "Core 12 = 이 칩에서 가장 빠른 코어, 5GHz 가능"으로 마킹
2. Linux 스케줄러가 새 work를 그 코어에 우선 배치
3. Core 12가 5GHz boost 시도 → 그 코어 instantaneous 전류 폭증
4. 칩의 per-core 전류 한도(ICCMAX) 초과
5. 펌웨어가 즉시 그 코어 클럭 강등 (전류 보호 목적, 온도와 무관)
6. throttle counter +1
7. ms 단위로 다시 boost 시도 → 같은 사이클 → 초당 ~14회 반복
8. 매 throttle마다 HWP가 모든 코어 P-state 재계산 → 시스템 전체에 jitter 전파
```

핵심: **온도도 전력도 사유가 아니다. 단일 코어 5GHz boost가 끌어쓰는 전류만 사유.**

## 3. 왜 다른 시도들이 안 됐나

### ❌ PL1 줄이기 (40W, 30W)
PL은 패키지 평균 전력 한도. ICCMAX는 단일 코어 instantaneous 전류 한도. **차원이 다름**. PL을 줄여도 boost 시도는 그대로 → ICCMAX 그대로 → throttle 그대로. 오히려 PL2 cap에 더 빨리 부딪혀 throttle 빈도가 늘어남.

### ❌ PL1 키우기 (80W, 100W)
같은 이유. PL을 풀어도 ICCMAX는 별개. throttle 그대로.

### ❌ favorite core (cpu3-4) offline
ITMT는 살아있는 코어 중 가장 빠른 걸 항상 favorite로 지정. 하나 빼면 다음 P-core(예: cpu1-2)가 새 favorite. **두더지 잡기**. 근본 해결 아님.

### ❌ Fan 더 돌리기
- 우리 throttle 사유는 전류지 온도가 아님 → fan 효과 없음.
- 추가로 NUC14RVB는 **OS에서 fan PWM 미노출** (`/sys/class/hwmon`에 pwm 파일 없음, asus-nb-wmi에도 fan 인터페이스 없음). BIOS 외엔 못 만짐.

### ❌ Turbo 완전 OFF (`no_turbo=1`)
효과는 확실 (throttle 0). 단점: 모든 코어가 base 클럭(P-core 1.4GHz, E-core 0.9GHz)에 고정 → 너무 박해서 SLAM 처리속도 손해.

## 4. 해결책 — Boost 클럭 천장 잡기

`max_perf_pct`로 boost ratio를 70%로 잡으면 **모든 코어의 클럭 천장 = 4.5GHz × 0.7 ≈ 3.3GHz**.

```bash
echo 70 | sudo tee /sys/devices/system/cpu/intel_pstate/max_perf_pct
```

또는 `set_pl.sh` 사용 (아래).

### 왜 이게 듣나

```
Boost 시도 자체가 5GHz가 아니라 3.3GHz로 제한됨
   ↓
그 코어가 끌어쓰는 instantaneous 전류 = 4.5GHz 시절의 절반 이하 (V²f scaling)
   ↓
ICCMAX에 절대 안 닿음
   ↓
throttle 발생 자체가 사라짐
```

### 부수 효과

| 지표 | 70% 적용 후 |
|---|---|
| throttle | **0/sec** |
| 코어 peak 온도 | 73~77°C (이전 80~95°C 대비 -10~20°C) |
| 패키지 평균 전력 | 19W (이전 27~80W 대비 30~75% 절감) |
| glim Hz | 75.8Hz 일정 (jitter 사라짐) |
| 시스템 jitter | 거의 없음 |

V²f scaling 때문에 **클럭 30% 줄이면 전력 약 65% 줄어듦**. throttle 사라지면서 전력/발열도 큰 폭 절감.

## 5. 운영 명령 (`set_pl.sh`)

위치: `stack_master/scripts/set_pl.sh`

### 핵심 사용법

```bash
# 현재 상태 (root 불필요)
set_pl.sh show

# 권장 적용 (HJ default — PL1=64W, PL2=64W, max_perf_pct=70%)
set_pl.sh set
# 또는 명시적으로
set_pl.sh set 64 64 70

# max_perf_pct만 변경 (load 늘었을 때 임시로 ↑)
set_pl.sh pct 80

# 1초 간격 실시간 모니터링 (Ctrl-C 종료, 로그는 ~/.set_pl_monitor/<ts>.log)
set_pl.sh monitor 1

# 디폴트 200/40/100 으로 되돌리기 (디버깅용)
set_pl.sh reset

# 의존성 자동 설치 (msr-tools, sysstat, lm-sensors)
set_pl.sh install
```

### 매 부팅마다 적용 필요

부팅 시 `power-profiles-daemon`이 PL1=200W로 덮어씀, `intel_pstate`도 max_perf_pct=100으로 리셋. 부팅 후 한 번:

```bash
~/icra2026_ws/ICRA2026_HJ/stack_master/scripts/set_pl.sh set
```

영구화는 systemd unit으로 가능 (TODO):

```ini
# /etc/systemd/system/hj-cpu-tune.service
[Unit]
Description=HJ CPU power/boost tuning
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/home/nuc3/icra2026_ws/ICRA2026_HJ/stack_master/scripts/set_pl.sh set
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

`After=multi-user.target` 으로 PPD가 PL1=200W 박은 다음에 우리가 덮어쓰는 순서 보장.

## 6. 효율 최적화 — pct를 어디까지 낮출 수 있나

해결책으로 pct=70을 잡았지만, **거기서 멈출 필요는 없음**. SLAM 같은 워크로드는 보통 클럭 budget을 다 안 쓰므로, hz 손실 없는 한도에서 pct를 더 낮추면 **발열·전력·배터리 시간이 그대로 이득**.

### 왜 더 낮춰야 하나 — V²f scaling

코어 전력 = `V² × f` (대략). Linux/Intel은 클럭(f)을 낮출 때 전압(V)도 같이 낮추므로, 실효적으로 **전력은 클럭의 cubic에 가깝게 떨어짐**:

| 클럭 비율 | 대략 전력 비율 |
|---:|---:|
| 100% | 100% |
| 80% | ~50% |
| 70% | ~35% |
| 60% | ~22% |
| 50% | ~13% |

→ **불필요한 boost 천장 = 그대로 발열·배터리 낭비**. 같은 일을 더 낮은 클럭으로 하면 시간만 살짝 길어지고 전력은 큰 폭으로 줄어듦.

### 측정된 비교 (이 머신, glim+ROS 풀스택)

| pct | freq peak | pkg avg power | core peak | glim Hz | throttle |
|---:|---:|---:|---:|---:|---:|
| 100 (boost full) | 4.5GHz | ~30W (cap에 자주 부딪힘) | 95~100°C | 75 (들쑥날쑥) | 14/sec |
| 80 | 3.8GHz | ~22W | 80~82°C | 75.8 (안정) | 0 |
| **70** | **3.3GHz** | **19W** | **73~77°C** | **75.8 (안정)** | **0** |
| 60, 50 | 3.0GHz, 2.5GHz | (미측정) | (미측정) | **이 영역에서 sweet spot 찾기** | 예상 0 |

→ 100→70 만으로 throttle 사라짐 + 전력 ~37% 절감 + 발열 -20°C. 더 낮출 여지 큼.

### 측정 방법 (단계적 step-down)

```bash
# 1) 베이스라인 — 우리가 지킬 hz
timeout 30 rostopic hz /<glim의 출력 토픽>

# 2) pct를 단계적으로 낮춰가며 같은 hz 측정
for p in 70 60 55 50; do
  ~/icra2026_ws/ICRA2026_HJ/stack_master/scripts/set_pl.sh pct $p
  sleep 5  # 적용 + 안정화
  echo "=== pct=$p ==="
  timeout 30 rostopic hz /<topic> 2>&1 | tail -3
  ~/icra2026_ws/ICRA2026_HJ/stack_master/scripts/set_pl.sh show | grep throttle
done

# 3) hz가 떨어지기 시작 직전 값이 sweet spot
```

### 판단 기준

| 결과 | 의미 | 행동 |
|---|---|---|
| hz 그대로, throttle 0 | 아직 여유 있음 | 한 단계 더 낮춰봄 |
| hz 1~2% 떨어짐 | 임계 도달 | 한 단계 위가 답 |
| hz 5% 이상 떨어짐 | 너무 박함 | 두 단계 위로 복귀 |

### 운영 시 — burst margin 추가

평소 hz가 유지돼도 **순간 peak 부하** (loop closure, 장애물 폭증, detection 추가, 새 노드 띄우는 순간) 시 부족할 수 있음.

→ 측정한 sweet spot의 **+10~15 pct가 안전 운용 값**.

예: 측정 결과 pct=50 까지 hz 안 떨어지면 → 운영은 pct=60 으로. 평소 약간 여유 두고, peak에 대응.

## 7. Throttle이 다시 발생하면 — 진단 흐름

증상: `set_pl.sh monitor` 에서 `thr_dt > 0` 보일 때.

```
1. set_pl.sh show
   → max_perf_pct=70 인지 확인. 100이면 누가/뭔가 풀어둔 것.
     해결: set_pl.sh set
                                      ↓ (70인데도 throttle이면)
2. CPU 부하 조사: htop 또는 mpstat -P ALL 1 5
   → 평균 사용률 50% 넘는지. 새로운 무거운 노드 추가됐는지.
                                      ↓
3. 부하가 늘었다면 (예: detection 추가):
   → set_pl.sh pct 80   (3.8GHz로 천장 ↑, throttle 다시 0 가능)
   → 그래도 부족: set_pl.sh pct 85
                                      ↓ (pct=85로도 throttle 발생하면)
4. 진짜 한계 도달. 옵션:
   (a) 워크로드 줄임: 노드 disable, hz target 낮춤, OMP_NUM_THREADS↓
   (b) 한 노드를 다른 코어에 격리: taskset 또는 cgroup
   (c) NUC 쿨링 보강: 외부 송풍, 페이스트 재도포
```

### load 늘 때 pct를 어디까지 올려도 되나

| pct | freq 천장 | throttle 위험 | 발열 |
|---:|---:|---|---|
| 70 | 3.3GHz | 거의 0 | 70°C대 |
| 80 | 3.8GHz | 0~1/sec | 80°C대 |
| 85 | 4.1GHz | 1~3/sec | 85°C대 |
| 90 | 4.2GHz | 3~5/sec | 90°C대 |
| 100 | 4.5GHz boost full | **14/sec** (원위치) | **100°C+** |

**원칙**: throttle 0 유지하면서 가장 낮은 pct가 정답. 90 넘기면 ICCMAX 영역 다시 들어감.

## 8. 진단 도구 (직접 검증)

### MSR로 throttle 사유 확인 (sudo)

```bash
sudo modprobe msr

# bit 10 = Power Limit (PL1/PL2/ICCMAX/VR therm 전부 합산)
for c in $(seq 0 21); do
  v=$(sudo rdmsr -p $c 0x19C -f 10:10 2>/dev/null)
  [ "$v" = "1" ] && echo "cpu$c: POWER LIMIT throttle ACTIVE"
done

# bit 0 = Thermal (Tjmax 사유) — 이게 켜지면 진짜 온도 문제
for c in $(seq 0 21); do
  v=$(sudo rdmsr -p $c 0x19C -f 0:0 2>/dev/null)
  [ "$v" = "1" ] && echo "cpu$c: THERMAL throttle ACTIVE"
done

# Tjmax 값 확인
sudo rdmsr -p 0 0x1A2 -f 23:16 -d   # 110 (정상)
sudo rdmsr -p 0 0x1A2 -f 29:24 -d   # TCC offset (3 = throttle threshold 107°C)
```

### 실측 패키지 전력 (sudo 필요, RAPL covert channel mitigation)

```bash
sudo turbostat --quiet --show PkgWatt,CorWatt,Avg_MHz,Bzy_MHz,PkgTmp \
  --interval 1 --num_iterations 10
```

### 실시간 종합 모니터

```bash
sudo set_pl.sh monitor 1   # sudo 붙여야 패키지 전력도 같이 보임
```

## 9. 자주 헷갈리는 점

| 관찰 | 잘못된 해석 | 정확한 해석 |
|---|---|---|
| `thermal_throttle_count`가 늘어남 | "온도가 높다" | 사유 6가지 합산 — 사유는 MSR로 따로 봐야 함 |
| 코어 온도 80°C | "곧 throttle 위험" | Tjmax 110°C 기준 30°C 여유, 안전 |
| BIOS의 "max 85°C" 설정 | "85°C에서 throttle" | 그건 fan curve top, throttle threshold는 별도 (107°C) |
| pct=70로 낮췄더니 CPU% 올라감 | "코어가 더 일하니 손해" | 같은 일을 낮은 클럭으로 하니 시간 점유율만 늘어 보임. 처리량은 동일 |
| pct=80일 때 freq peak 3800MHz | "boost 풀로 동작" | 천장이 3.8GHz일 뿐 ICCMAX 트리거하는 5GHz는 안 시도 |

## 10. 일반화 — 다른 머신에서

이 진단/해결 패턴은 NUC14RVB에 한정 아님. 다음 조건이면 같은 문제 / 같은 해결:

- Intel 12th gen 이상 (P/E core 분리 + ITMT)
- 컴팩트 폼팩터 (mini-PC, NUC, 노트북)
- 단일 코어 boost 클럭이 base의 2배 이상
- 워크로드가 multi-thread parallel + memory bound (SLAM, 영상 처리, 시뮬레이션 등)

이 조합이면 거의 ICCMAX-driven throttle 발생. **`max_perf_pct=60~80`으로 천장만 잡으면 즉시 해결**되는 경우가 많음.

다른 머신 적용 시:
1. `set_pl.sh show`로 현재 PL/pct 확인
2. throttle 카운트 증가 속도 측정 (`monitor` 모드)
3. `pct 70` 적용 후 같은 모니터로 0 되는지 확인
4. 안 되면 `pct 60` 까지 시도. 그래도 안 되면 진짜 다른 사유 (Tjmax / VR therm) — MSR bit 0 / 4로 분리 진단
