# Servo Calibration — 치명 버그 2건 수정

- **작성일**: 2026-04-20
- **상태**: 코드 수정 완료 / 실차 검증 대기 (Gazebo sim에서는 vesc가 안 돌아서 이 경로 검증 불가능. 실차 필수)
- **브랜치**: main
- **영향 범위**: `calibration:=true` 로 기동하는 경우에만. 평소 주행/캘리 false 시에는 동작 변화 없음
- **수정된 파일 (이 세션 작업물)**:
  - `stack_master/scripts/record_steering_bags.py`
  - `stack_master/launch/3d_base_system.launch`
- **인접 관련 파일 (변경 없음, 참고용)**:
  - `stack_master/launch/servo_calibration.launch`
  - `stack_master/scripts/fit_steering_poly.py`
  - `stack_master/launch/middle_level.launch` → `low_level.launch` → `vesc.launch`
  - `stack_master/config/SRX1/devices/vesc.yaml` (현재 `enable_nonlinear_servo_gain: false`)
  - `sensors/vesc/vesc_ackermann/src/custom_ackermann_to_vesc.cpp` (param 읽는 노드 코드)

---

## 다음 세션 첫 10분 복귀 가이드

1. 본 문서 읽기 → 두 버그가 무엇이었고 어떤 형태로 수정됐는지 파악.
2. `git log --oneline -3` 으로 `stack_master: servo calibration bug fixes ...` 커밋 확인.
3. 실차 세션이면 바로 **검증 섹션** 순서대로 실행. sim/HJ 개발 세션이면 다른 태스크로 넘어가도 됨 (이 수정은 calibration=false 경로에 영향 없음).
4. 실차 검증 끝나면 본 문서 맨 아래 **체크리스트**에 결과 기입 후 커밋.

---

## 문제 요약

### 버그 1: `MIN_VALID_SECS` 하드코딩으로 모든 bag이 INVALID 판정

- 위치: `stack_master/scripts/record_steering_bags.py:211-212` (수정 전 기준 L208)
- 증상: `record_sec` 기본값이 5.0초인데 검증 임계값도 하드코딩 5.0초.
  실제로 bag에 기록된 첫 odom 메시지 타임스탬프 → 마지막 메시지 타임스탬프까지의 `duration`은 subscribe latency + rate.sleep gap으로 인해 항상 `record_sec` 보다 수십 ms 짧음 (보통 ~4.97~4.99s).
  → 검증 조건 `duration < MIN_VALID_SECS` 가 **항상 참** → 모든 스윕 포인트가 INVALID → 무한 retry 루프.
- 발견 계기: 코드 리뷰 (사용자: "3d_base_system의 calibration 버그 찾아줘")

### 버그 2: `<param>` override가 rosparam load에 묻혀 효과 없음 (잠복 버그)

- 위치: `stack_master/launch/3d_base_system.launch:31-35` (수정 전 기준)
- 증상:
  - 수정 전, `calibration:=true` 일 때 `/vesc/enable_nonlinear_servo_gain=false` 로 강제하는 `<param>` 이 `middle_level.launch` include **이전**에 있음.
  - roslaunch는 `<param>` / `<rosparam>` 을 **XML 문서 순서**로 수집해 순차 upload → 같은 키는 **later wins**.
  - middle_level → low_level → vesc.launch 내부 `<rosparam file="vesc.yaml" command="load"/>` 이 뒤에 처리되며 `/vesc/enable_nonlinear_servo_gain` 키가 **yaml 값으로 덮어써짐**.
  - 현재 vesc.yaml에 `enable_nonlinear_servo_gain: false` 이어서 **우연히** 동작 중.
  - 사용자가 fitting 결과 적용 후 `vesc.yaml` 에서 `true` 로 바꾸면, 다음 캘리 시도에서 launch의 false override 가 먹지 않아 nonlinear 모드가 살아있음 → recorder 의 safety 체크([record_steering_bags.py:44-49](../stack_master/scripts/record_steering_bags.py#L44-L49))가 `rospy.logfatal + sys.exit(2)` 로 종료됨.

---

## 수정 내용

### 버그 1 수정

`MIN_VALID_SECS` 를 `record_sec` 상대값 + 0.5s 여유로 변경.

```python
# stack_master/scripts/record_steering_bags.py, L204-212
# ### HJ : validate recorded bag — if odom samples or duration is too low,
# the data is unusable (corruption, bad timing, etc). Treat as failed so
# the run loop retries this point instead of silently skipping it.
# Duration threshold is relative to record_sec with a small tolerance:
# first/last odom bag timestamps never span the full record window
# (subscribe latency + rate.sleep gap), so a hard 5.0s threshold rejected
# every bag when record_sec itself was 5.0.
MIN_VALID_ODOM  = 200
MIN_VALID_SECS  = max(1.0, self.record_sec - 0.5)
```

- `record_sec` 을 늘리거나 줄여도 마진이 따라감.
- 바닥 1.0s 로 극단적인 낮은 설정(`record_sec < 1.5s`)에서도 음수/0 안 되게 보호.

### 버그 2 수정

override `<param>` 을 `middle_level` include **뒤**로 이동 + `<group unless="$(arg sim)">` 안으로 배치 (sim 에서는 vesc 자체가 없으므로 의미 없음).

```xml
<!-- stack_master/launch/3d_base_system.launch, L31-47 -->
<!-- ══════════════════ REAL CAR ══════════════════ -->
<group unless="$(arg sim)">
  <!-- Low level + State Estimation + Frenet -->
  <include file="$(find stack_master)/launch/middle_level.launch">
    <arg name="racecar_version" value="$(arg racecar_version)" />
    <arg name="map" value="$(arg map)" />
    <arg name="SE" value="$(arg SE)" />
    <arg name="use_vesc_imu" value="$(arg use_vesc_imu)" />
  </include>
  <!-- ### HJ : during calibration, force nonlinear mapping OFF so the algebraic trick
       delta_cmd = (target_servo - offset)/gain produces deterministic raw servo values.
       Placed AFTER the middle_level include so this <param> is processed after
       vesc.launch's rosparam load of vesc.yaml (roslaunch: later-in-document wins).
       Sim path is excluded on purpose: middle_level / vesc are not launched there. -->
  <param if="$(arg calibration)"
         name="/vesc/enable_nonlinear_servo_gain" value="false" />
</group>
```

---

## 동작 원리 검증 근거 (다음 세션이 의심스러우면 재확인할 것)

- `custom_ackermann_to_vesc_node` 의 param 읽기 경로: [custom_ackermann_to_vesc.cpp:62](../sensors/vesc/vesc_ackermann/src/custom_ackermann_to_vesc.cpp#L62)
  ```cpp
  nh.param<bool>("enable_nonlinear_servo_gain", enable_nonlinear_servo_gain_, false);
  ```
  - `nh` = 상대 NodeHandle. 노드가 `/vesc/` 네임스페이스에서 spawn되므로 조회 경로 = `/vesc/enable_nonlinear_servo_gain`.
  - 생성자에서 1회만 읽음. dynamic_reconfigure 없음. 노드 시작 전 param 서버에 최종값이 세팅돼 있으면 OK.
- roslaunch 의 param 처리 순서: XML 문서 순서로 수집 → 노드 spawn 전 일괄 XML-RPC setParam → 같은 키는 마지막 set 이 이김 (later wins).
- 따라서 우리 수정본은:
  1. vesc.launch → rosparam load vesc.yaml → `/vesc/enable_nonlinear_servo_gain = <yaml 값>` (e.g. true or false)
  2. 우리 `<param if="calibration" value="false"/>` → `/vesc/enable_nonlinear_servo_gain = false` **강제**
  3. 노드 spawn → 생성자에서 `false` 로 읽음 → 선형 매핑 활성 → recorder 의 algebraic trick 성립.

---

## 엣지 케이스 매트릭스

| calibration | sim | 동작 |
|---|---|---|
| false (기본) | false | `<param if>` 무시. vesc.yaml 값 그대로. **기존 동작 100% 보존** |
| false | true | 동일. 평소 sim 주행에 아무 영향 없음 |
| true | false | override 적용 → `/vesc/enable_nonlinear_servo_gain=false` 강제, recorder + fitter 정상 진행 |
| true | true | `<group unless=sim>` 로 스킵됨. middle_level 자체가 없어 vesc 노드도 없음. 의미 없는 조합 (운영상 사용 금지) |

---

## 검증 계획 (실차 세션에서 실행)

터미널 1 (host 또는 container):
```bash
# 평소 주행 — 아무 변화 없는지
roslaunch stack_master 3d_base_system.launch map:=<map> sim:=false
# 다른 터미널
rosparam get /vesc/enable_nonlinear_servo_gain
# 기대: vesc.yaml 에 쓰여있는 값 그대로 (false 또는 true)
```

터미널 1 (calibration 모드):
```bash
roslaunch stack_master 3d_base_system.launch map:=<map> sim:=false calibration:=true
# 다른 터미널
rosparam get /vesc/enable_nonlinear_servo_gain
# 기대: False (vesc.yaml 값 무관)
```

추가 검증 시나리오: vesc.yaml 에 `enable_nonlinear_servo_gain: true` 로 임시 변경 후 `calibration:=true` 재기동 → 여전히 `False` 나오면 override 완성.

**버그 1 검증**: 위 calibration 세션에서 스윕 1개 수행.
- RB 눌러 첫 delta (+0.05) 기록 → 로그에 `[sweep] delta=+0.050 OK (N odom, Ys)` 가 찍히면 성공.
- 수정 전이라면 항상 `[sweep] delta=+0.050 INVALID: ... < 5.0s ...` 였음.

---

## 리스크 / 롤백

- 리스크: 실차 검증 전까지는 잠재적 회귀 가능성 존재. 단, `calibration:=false` 일 때는 launch XML 경로상 이 `<param>` 자체가 무시되므로 기존 주행 경로 영향 없음.
- 롤백: 두 파일 모두 단일 커밋에 묶여 있으므로 `git revert <commit-sha>` 로 즉시 원복 가능.

---

## 미확인/보류 (다음 세션 확인 대상)

- `calibration:=true` + 실차 스윕 시 record 타이밍이 충분한지. 25 포인트 × 약 (warmup 2.5s + record 5s + 조작 시간) ≈ 10분. 오퍼레이터 피로도 vs 데이터 품질 트레이드오프는 실운용에서 판정.
- `fit_steering_poly.py` 로 생성된 `nonlinear_servo_mapping.png` 이 적어도 1회 실차 주행 전에 눈으로 검증되어야 함 — 포인트들이 단순한 S 곡선이 아니라 symmetry 기울어지면 linkage 문제 가능성.
- vesc.yaml 에 fitter 가 poly_coeffs 를 자동으로 쓸 때 주석 보존 여부는 `fit_steering_poly.py:_update_vesc_yaml` 에서 텍스트 기반으로 처리 중. 여기도 구조적으로 의심해볼 여지는 있으나 이번 세션 범위는 아님.

---

## 체크리스트 (실차 검증 시 기입)

- [ ] `calibration:=false` 로 기동 → `rosparam get /vesc/enable_nonlinear_servo_gain` 결과: ______
- [ ] `calibration:=true` 로 기동 → 같은 결과: ______ (기대: False)
- [ ] 스윕 1 포인트 `OK` 로그 확인
- [ ] 전체 스윕 (25 pts) 완주 여부
- [ ] `nonlinear_servo_mapping.png` 생성 확인
- [ ] vesc.yaml 에 `steering_servo_poly_coeffs` 자동 업데이트 확인
- [ ] `enable_nonlinear_servo_gain: true` 로 바꾼 뒤 평소 주행해서 차가 정상 steering 하는지 확인
