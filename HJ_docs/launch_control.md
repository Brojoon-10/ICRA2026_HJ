# Launch Control v1 — 진행 인수인계

- **작성일**: 2026-05-01
- **마지막 업데이트**: 2026-05-01 (VESC 직접 current 명령 sweep 진행 중)
- **현재 상태**: 🟡 **사전 진단 단계** — 코드 한 줄 안 짬. VESC가 명령된 전류를 진짜로 흘리는지 격리 검증 중.
- **다음 세션 첫 10분 복귀 가이드**:
  1. 본 문서 읽기 (5분)
  2. `stack_master/launch/test_vesc.launch` 가 신규 추가된 파일임을 확인 ([파일](../stack_master/launch/test_vesc.launch))
  3. 차량 컴(차량에 연결된 PC) 켜고 차륜 들린 상태 확인
  4. "현재 멈춘 지점" 섹션의 가설 검증 — speed(ERPM) sweep 명령부터 재개

---

## 1. 목표 (한 줄)

ICRA 2026 staggered start 대응. **퀄리(SOLO) 한정으로 정지 → 첫 0.75초 가속을 traction limit 직전까지 짜내는 launch control 추가**. Staggered 2대 분기, 합류 spline은 다음 세션.

## 2. 합의된 v1 설계 (단순화 후)

핵심: **기존 파이프라인 영향 0**. launch_controller 노드가 안 떠 있으면 평소 동작 100% 동일.

- **건드리지 않음**: state_machine, spline / `OTWpntArray` / `cur_start_wpnts`, `simple_mux`, 다른 컨트롤러/플래너 전부.
- **신규**:
  - `controller/launch_controller/` 패키지 (노드 + msg + cfg + launch). `/launch_signal` (std_msgs/Header) 받으면 0.75s 동안 50Hz로 `LaunchCommand`(자체 msg) publish.
  - `controller/controller_manager.py`에 hook 1군데: `create_ack_msg` 직전에 launch_override active면 `(speed, accel, jerk, steering)` 덮어씀.
  - `sensors/vesc/vesc_ackermann/src/custom_ackermann_to_vesc.cpp:124` 직전에 **`if (cmd->drive.jerk == 511)` 신규 분기**: launch torque current mode. `launch_accel_to_current_gain × accel + clip(0, max_launch_current)` → `/commands/motor/current`. watchdog (마지막 511 수신 0.8s 초과면 0 publish).
  - racecar별 `vesc.yaml`에 4 params: `launch_accel_to_current_gain`, `launch_velocity_ff_gain`, `max_launch_current`, `launch_max_duration_sec`.

### Speed/torque profile (1차 안)

| 구간 | 시간 | mode | accel | speed (sim ref) | jerk |
|---|---|---|---|---|---|
| step | [0, 0.15s] | current | `ax_step` (시작 8.0 m/s²) | v_ref(t) integrated | **511** |
| ramp | [0.15, 0.6s] | current | linear `ax_step → ax_steady` (5.0) | v_ref(t) | **511** |
| crossfade | [0.6, 0.75s] | speed | unused | (1-α)·v_ref + α·racing_line_vx | 0 |
| done | t > 0.75s | inactive | — | controller 원본 그대로 | 0 |

첫 0.3s는 `steering_angle = 0` lock (정지 yaw 추정 불안정 회피).

> **단, 현재 진단 결과에 따라 첫 단계가 "spin-up (duty 또는 speed)"으로 바뀔 가능성 큼.** 아래 진단 진행 후 확정.

## 3. VESC 액추에이션 체인 (조사 결과)

```
controller(AckermannDriveStamped, jerk=N)
  → /vesc/high_level/ackermann_cmd_mux/input/nav_1
  → simple_mux (50Hz timer로 zero burst 가능 — 진단 시 위험!)
  → /vesc/low_level/ackermann_cmd_mux/output
  → custom_ackermann_to_vesc.cpp:124  ─┬─ jerk==512 → /commands/motor/{current|brake} (브레이크, 기존)
                                       ├─ jerk==511 → ★ 신규 launch torque (예정)
                                       └─ else      → /commands/motor/speed (ERPM)
  → vesc_driver.cpp:258 → current_limit_.clip(0~100A) → vesc_.setCurrent → VESC HW
```

조사로 확정된 것:
- `/sensors/core` publisher는 vesc_driver 단 1개 ([sensors/vesc/vesc_driver/src/vesc_driver.cpp:78](../sensors/vesc/vesc_driver/src/vesc_driver.cpp#L78)). 다른 노드는 read-only.
- ROS 단에서 후가공 0 — `current_motor`는 raw 디코드 ([vesc_packet.cpp:147-154](../sensors/vesc/vesc_driver/src/vesc_packet.cpp#L147-L154)).
- VESC 펌웨어 측에는 LP filter, FOC Iq 추정, slew rate 적용된 후 보고됨 (펌웨어 내부 가공). 정상 상태 평균 전류는 ground truth로 신뢰 가능.
- jerk 필드 grep 결과: 코드베이스에서 0 또는 512만 set. **511은 충돌 0** 확정.
- sim에선 vesc_driver 자체 미기동 + simulator는 AckermannDriveStamped만 구독 → jerk==511 cpp 분기 sim 영향 0.
- Mux 우회: launch_controller가 actuation에 직접 publish하면 race 발생 → **controller에 hook inject 방식**으로 가야 publisher 단일 유지.

위험 (이미 알고 있는 것):
- `simple_mux.py:94-98` — RB 한 번이라도 만지면 `release_time` 후 1초 동안 50Hz zero ackermann publish → custom_ackermann_to_vesc가 speed=0 발행 → 우리 current 명령 즉시 덮임. **진단 단계엔 simple_mux/ackermann_to_vesc 절대 띄우면 안 됨**.
- `custom_ackermann_to_vesc.cpp:154-168` brake 분기는 `if(true)` 강제로 데드코드 (유지, 우리가 안 건드림).
- `vesc_driver.cpp` 의 `if (driver_mode_ = MODE_OPERATING)` 할당문 버그. 우리가 만질 자리 아님.

## 4. 진단용 도구 (이미 만든 것)

### `stack_master/launch/test_vesc.launch` (신규 — 이번 세션 추가)

vesc_driver 단독 기동. simple_mux / custom_ackermann_to_vesc / joy 다 미기동. publisher race 0.

```bash
roslaunch stack_master test_vesc.launch
# 또는 racecar 인자 명시
roslaunch stack_master test_vesc.launch racecar_version:=<CAR_NAME>
```

검증 (별도 터미널):
```bash
rostopic info /vesc/commands/motor/speed                    # Publishers: None 이어야 함
rostopic info /vesc/low_level/ackermann_cmd_mux/output      # Unknown topic 또는 Publishers: None
rostopic info /vesc/commands/motor/current                  # Subscribers: vesc_driver_node 만, Publishers: None
rostopic hz /vesc/sensors/core                              # ~50Hz
```

## 5. 현재까지의 측정 결과

| 명령 | 결과 |
|---|---|
| `current = 3.0A` | `current_motor=3.0` 일치, 차륜 안 돔 (cogging — 정지 상태 BLDC sensorless 시동 실패) |
| `duty_cycle = 0.05` | 회전 안 함, current는 흐름 (시동 임계 못 넘음) |
| `duty_cycle / speed / current 더 큰 값들로 sweep` | **회전 시작했고 매우 빠르게 회전** |
| `current ≥ 20A 부근` | **소리가 포화됨** — 사용자 가설: ERPM cap에 걸려 회전수는 더 안 오르고 토크만 더 들어가는 상태 |

`fault_code: 88, 231` 등 매번 다른 값이 나옴 → byte offset misalignment 확정 (펌웨어/driver 버전 mismatch). 단:
- `voltage_input: 16.7V` — 4S LiPo 만충값과 일치 → 그 offset까지는 정상
- `current_motor` 가 명령값(3.0A)과 정확히 일치 → 이 필드는 신뢰 가능
- **fault_code 필드만 깨진 듯**. 다른 측정값으로 진행 가능.

`temperature_pcb`, `speed (rpm)`, `duty_cycle` 의 신뢰성은 **회전 중일 때 한 번 sanity check 필요** (다음 단계에서).

## 6. 현재 멈춘 지점 + 가설

### 멈춘 지점
**~20A 부근에서 RPM/소리 포화. 이상 current 명령 줘도 회전수 안 오르고 토크만 들어가는 듯한 음향 변화.**

### 사용자 가설
- **VESC 펌웨어 또는 모터 셋팅에서 max ERPM cap이 걸려있음**. 그 cap에 도달하면 RPM은 더 못 오르고 펌웨어가 current를 cap 안으로 계속 흘리지만 회전수는 정체 → 소리만 더 거칠어짐.

### 검증 방법 (다음 세션 1순위)

#### A. ERPM(speed) sweep으로 cap 위치 직접 측정 (코드 0줄, 가장 빠름)

```bash
# test_vesc.launch 띄운 상태에서, 차륜 들린 채로
timeout 3 rostopic pub -r 50 /vesc/commands/motor/speed std_msgs/Float64 "data: 5000.0"
timeout 3 rostopic pub -r 50 /vesc/commands/motor/speed std_msgs/Float64 "data: 10000.0"
timeout 3 rostopic pub -r 50 /vesc/commands/motor/speed std_msgs/Float64 "data: 20000.0"
timeout 3 rostopic pub -r 50 /vesc/commands/motor/speed std_msgs/Float64 "data: 30000.0"
timeout 3 rostopic pub -r 50 /vesc/commands/motor/speed std_msgs/Float64 "data: 50000.0"
```

각 단계에서 별도 터미널 echo:
```bash
rostopic echo -c /vesc/sensors/core | grep -E "speed|duty_cycle|current_motor|current_input|voltage_input"
```

명령 ERPM ↑ 해도 측정 `speed(rpm)` 가 어느 시점에서 정체 → 거기가 cap. duty_cycle이 1.0 부근으로 saturate면 전압 한계, 0.5 부근에서 정체면 ERPM cap.

#### B. VESC Tool 직접 확인 (가능하면)

차량 컴 또는 별도 노트북에 VESC Tool 설치 → USB로 VESC 직결 → 다음 4개 메모:
- 펌웨어 버전 (`Welcome → Firmware`)
- Motor mode (BLDC vs FOC)
- `Motor Max` (절대 모터 전류 한계)
- `Battery Max` (배터리 측 전류 한계)
- `Maximum ERPM` (회전 한계 — 사용자 가설의 진짜 후보)

VESC Tool RT Data로 명령 publish 시 phase current, ERPM, duty 실시간 그래프 — 가장 확실한 ground truth.

## 7. 진행 체크리스트

### 완료
- [x] 기존 START 모드의 한계 분석 ([Controller.py:155](../controller/combined/src/Controller.py#L155) `start_mode` 효과 = steering 보정 비활성화뿐)
- [x] VESC 액추에이션 체인 전체 추적 (controller → simple_mux → custom_ackermann_to_vesc → vesc_driver → HW)
- [x] jerk magic value 충돌 검사 (511은 코드베이스 어디에서도 미사용 — 안전)
- [x] sim 영향 0 확정 (vesc_driver는 sim launch 미기동, simulator는 AckermannDriveStamped만 구독)
- [x] launch_controller architecture 결정 (Controller에 hook inject 방식 — publisher 단일 유지, mux race 0)
- [x] ref path 빼는 단순화 (SOLO는 racing line 시작점 정렬이라 spline 불필요)
- [x] `stack_master/launch/test_vesc.launch` 추가 (vesc_driver 단독 기동, publisher race 0 보장)
- [x] `/sensors/core` 측정 신뢰성 1차 확인 (`current_motor` 명령 일치, `voltage_input` 합리적, **`fault_code`만 깨짐**)
- [x] 정지 상태에서 current 직접 명령 → cogging (BLDC sensorless 정상 동작)
- [x] 큰 명령으로 회전 시작 확인 (chain alive 결정적 증거)

### 진행 중 (다음 세션 1순위)
- [ ] **ERPM cap 위치 측정** (사용자 가설 검증) — Section 6의 A 또는 B 절차
- [ ] 회전 중일 때 `speed(rpm)`, `duty_cycle` 신뢰성 sanity check
- [ ] 현재 모터 모드(BLDC vs FOC) 확인 — production launch profile에 spin-up 단계 필요 여부 결정 (BLDC면 필요, FOC면 불필요)

### 그 다음 (코드 작업 진입)
- [ ] `controller/launch_controller/` 패키지 스캐폴딩 (package.xml, CMakeLists.txt, msg/, cfg/, launch/, src/)
- [ ] `LaunchCommand.msg` 정의
- [ ] `LaunchOverride` 헬퍼 (50줄) + `controller_manager.py`에 init + 구독자 + `create_ack_msg` 직전 hook 1줄
- [ ] `launch_controller_node.py` 베이스라인: `/launch_signal` 구독 → ax(t) profile + v_ref(t) integration + mode FSM + LaunchCommand publish + debug JSON
- [ ] `custom_ackermann_to_vesc.cpp` jerk==511 분기 + 4 params + watchdog
- [ ] `vesc.yaml` 4 params 추가 (보수값으로 시작: `max_launch_current=30A`, `launch_accel_to_current_gain=8.0`)
- [ ] catkin build (`launch_controller` + `vesc_ackermann`만)
- [ ] sim 회귀 (변화 0 확인)
- [ ] 실차 보수 calib + 합격선 sweep
- [ ] abort path 검증

### 다음 세션 (Staggered)
- [ ] `LaunchCommand.msg`의 `role` 필드 활성화 (POLE/P2/SOLO)
- [ ] opponent cap 활성화 (longitudinal v_cap from Δs)
- [ ] 합류 spline 자동 생성 (정지위치 → racing line, cubic Bézier — 처음 plan에 있던 것)
- [ ] state_machine `StartTransition` 4조건 강화 (속도 + |d| + heading + timeout)

## 8. 합격선 (변경 없음)

- 0 → 3.1 m/s ≤ 0.7 s
- 0 → 4.6 m/s ≤ 1.5 s
- 1 s 동안 이동 거리 ≥ 1.8 m
- 휠스핀 카운트 (slip > 0.15 가 50 ms 지속) ≤ 1 / launch
- 첫 코너(s≈10 m) 진입 vx가 racing line 그 지점 vx ±0.3 m/s
- launch_controller 미기동 시 모든 동작 평소와 동일 (회귀 0)

## 9. 안전 / 진행 원칙 (반복 강조)

- **진단 단계엔 `low_level.launch` 절대 띄우지 말 것**. simple_mux의 50Hz zero burst가 우리 current 명령을 즉시 덮음.
- **진단은 `test_vesc.launch` (vesc_driver 단독)** 만 사용.
- 차륜 항상 들린 상태로. 차 진행방향 5m 이상 비움.
- RC 송신기 OFF, USB joy 분리 (혹시 simple_mux가 떠있더라도 안전 장치).
- 명령 publish는 `timeout N` 으로 자동 종료시키기. `Ctrl+C` 하면 rospy traceback이 길게 뜨지만 무해.
- VESC Tool 사용 가능하면 RT Data를 ROS echo와 동시에 띄워 비교 (ground truth 검증용).

## 10. 보류된 의문 (해결 우선순위)

1. **ERPM cap 위치** ← 가장 시급 (현재 멈춘 지점)
2. **모터 모드 (BLDC vs FOC)** — production launch profile에 spin-up 단계 필요 여부 결정
3. **byte offset misalignment 정확한 위치** — `current_motor` 만으로 calib 가능하므로 production에 영향 없음. 단 `fault_code` 모니터링 못 하는 상태이므로, **production deploy 전엔** vesc_driver를 새 펌웨어 호환 버전으로 교체 검토 (fork: f1tenth/vesc 새 버전).
4. **VESC 펌웨어 버전** — VESC Tool 한 번 연결로 즉시 해결.

## 11. 참고 파일 / 위치

- 본 인수인계: `HJ_docs/launch_control.md` (이 파일)
- 단계별 plan 원본: `~/.claude/plans/functional-churning-bubble.md` (Claude plan agent 결과 + 단순화 후 합의안)
- 신규 진단 launch: [stack_master/launch/test_vesc.launch](../stack_master/launch/test_vesc.launch)
- 향후 수정 대상:
  - [controller/controller_manager.py](../controller/controller_manager.py) (hook 추가)
  - [sensors/vesc/vesc_ackermann/src/custom_ackermann_to_vesc.cpp:124](../sensors/vesc/vesc_ackermann/src/custom_ackermann_to_vesc.cpp#L124) (jerk==511 분기)
  - [sensors/vesc/vesc_ackermann/include/vesc_ackermann/custom_ackermann_to_vesc.h](../sensors/vesc/vesc_ackermann/include/vesc_ackermann/custom_ackermann_to_vesc.h) (멤버 추가)
  - `stack_master/config/<CAR_NAME>/devices/vesc.yaml` (4 params 추가)
- 향후 신규:
  - `controller/launch_controller/` (패키지 전체)

## 12. TODO_HJ.md 갱신 필요

다음 세션 시작 시 `TODO_HJ.md`에 다음 항목 추가/체크:
- [ ] **launch_control v1**: VESC ERPM cap 측정 → spin-up 단계 필요 여부 결정 → 코드 진입
