# Isolation Sweep — controlled A/B/C measurement of isolation configs

> 작성: 2026-05-24 (HJ)
> 목적: glim burst가 sm/cm/vesc/frenet/rslidar 등에 미치는 영향을 isolation
> 조건별로 **동일 시나리오·동일 측정 함수**로 비교해서, 어떤 조합이 실제로 효과
> 있는지 / 어떤 게 의미 없는지 수치로 판정. 이전 세션 분석이 흩어진 이유는 매번
> 조건과 측정이 일관되지 않았기 때문 — 이 디렉터리는 그걸 한 번에 통제함.

## 다음 세션 첫 10분 복귀 가이드
1. 컨테이너 안에서 sim + 3d_base + 3d_headtohead 띄우기 (HJ 기본):
   ```bash
   roslaunch stack_master 3d_base_system.launch map:=gazebo_wall_2 sim:=true
   roslaunch stack_master 3d_headtohead.launch dynamic_avoidance_mode:=NONE
   ```
2. 차량 정지 상태로 두고 (overtaking 백엔드는 OFF) sweep 실행:
   ```bash
   docker exec -it icra2026 bash -c '
     source /opt/ros/noetic/setup.bash &&
     source $HOME/catkin_ws/devel/setup.bash &&
     sudo -E bash $HOME/catkin_ws/src/race_stack/HJ_docs/debug/isolation_sweep/run_sweep.sh'
   ```
3. ~16분 후 `HJ_docs/debug/isolation_sweep/runs/<ts>/_comparison.txt`
   와 `_comparison.csv` 확인.

## 무엇을 측정하나
각 condition마다 `measure_one.py`가 **동일한 방식**으로 측정 (= 비교 가능):

1. 50ms tick으로 `/proc/<pid>/task/*/stat` + `schedstat` 수집 → glim
   R-count ≥ 8인 tick을 **BURST**, ≤ 2인 tick을 **IDLE**로 분류.
2. 같은 tick window 안에서 sm/cm/vesc/frenet/rslidar의
   - CPU jiffies/sec (= % of 1 core)
   - wait_ns/sec (스레드 합산, 콜백 기다린 총 시간)
   - 50ms 이상 wait 이벤트 횟수
   를 BURST vs IDLE로 분리해서 평균/p95/max 산출.
3. 토픽 `/rslidar_points`, `/glim_ros/base_odom`, `/vesc/odom`의
   inter-arrival을 BURST window vs IDLE window로 분리해서 mean/stdev/p95/max
   비교 (publish 자체가 흔들리는지 확인).
4. cpu0/5(ctrl), cpu12(lidar)의 %active를 BURST vs IDLE로 비교 — 격리한 코어
   안으로 glim work가 **새는지**(LEAK) 감지.
5. `core_throttle_count` 총합의 측정 윈도우 동안 증분 → ICCMAX throttle 빈도.

이걸 **8가지 isolation 조건**에 대해 같은 측정자로 돌려서 한 테이블로 비교.

## Condition matrix (run_sweep.sh 안에 하드코딩, 편집 가능)

| label | 무엇을 알아내려는 건가 |
|---|---|
| `none` | 아무 격리 안 한 baseline. burst 시 sm/cm/rslidar가 얼마나 망가지는지 |
| `perf_only` | governor=performance 효과만 — burst 자체에는 영향 없을 거라는 가설 검증 |
| `ctrl_main` | sm/cm/vesc/frenet "메인 스레드만" cpu 0,5 pin (worker는 0-21 자유). 가장 가볍게 격리한 효과 |
| `ctrl_main_rt` | + SCHED_FIFO 50. RT priority가 worker 격리보다 효과적인지 |
| `ctrl_workers` | WORKER_ISO=1 — sm/cm/vesc/frenet 모든 스레드 cpu 0,5 pin. 메인 격리 vs 워커까지 격리 정량 비교 |
| `workers_lidar_e` | + LIDAR_MODE=e (cpu12 단독). **현재 default**. lidar 격리 추가 효과 |
| `workers_lidar_p` | LIDAR_MODE=p (P-core HT pair). lidar에 더 좋은 코어 줬을 때 효과 |
| `workers_lidar_lp` | LIDAR_MODE=lp (LP E-core + 카메라랑 공유). 가장 약한 lidar 격리 |

기대하는 비교 패턴 (실측으로 검증):
- `none` 의 sm wait BURST/IDLE ratio가 매우 클 것 (예: 5–10×)
- `ctrl_main` → `ctrl_main_rt` 차이로 **RT priority의 단독 기여도** 가시화
- `ctrl_main_rt` → `ctrl_workers` 차이로 **worker isolation의 단독 기여도**
  - 워커까지 격리하면 2 core 안에 100+ 스레드가 몰리므로 **오히려 IDLE 시
    callback latency가 올라갈 가능성** — 이걸 측정으로 검증
- `ctrl_workers` → `workers_lidar_*` 차이로 **lidar 격리 단독 기여**
- `workers_lidar_e/p/lp` 비교로 lidar에 어느 코어 도메인이 베스트인지

## 시나리오 통일 가이드 (중요)
조건 비교가 의미 있으려면 **각 condition의 외부 환경이 같아야** 함. 권장:

- **차량 정지 + 센서 라이브** (glim/rslidar/imu 모두 ON, controller도 ON,
  state_machine은 GBT라서 모터 명령 0). 글림은 ~10s 주기로 backend 최적화
  burst를 일으키므로 stationary에서도 isolation 효과 비교에 충분.
- Overtaking 백엔드(sampling, mpc)는 **OFF**. 비교 대상은 isolation이지
  overtaking 코드가 아님.
- 측정 동안 다른 작업(rosbag record, RViz interaction 등)은 안 함.
- 추후 검증 단계에서 **동일 rosbag replay** 또는 **동일 sim 주행 패턴**으로
  같은 sweep을 한 번 더 돌려서 결과가 일관되는지 확인.

## 출력 구조
```
HJ_docs/debug/isolation_sweep/runs/<ts>/
  ├── none/
  │     ├── apply_reset.log        # reset 결과
  │     ├── apply.log              # apply_isolation.sh 결과 (none은 BASELINE 표시)
  │     ├── stdout.txt             # measure_one.py 실행 전체 stdout
  │     └── summary.json           # 구조화된 결과 (summarize_sweep.py가 읽음)
  ├── perf_only/
  │     ...
  ├── workers_lidar_e/
  │     ...
  ├── _comparison.txt              # summarize_sweep.py가 만든 비교 테이블
  ├── _comparison.csv              # 같은 데이터 flat CSV (스프레드시트용)
  └── _final_reset.log
```

## 부분 실행 / 디버깅
```bash
# 일부 조건만:
ONLY=ctrl_workers,workers_lidar_e DURATION_S=60 \
  sudo -E bash run_sweep.sh

# 측정 한 번만:
python3 measure_one.py 60 --label test --json-out /tmp/test.json

# 다른 sweep 결과 다시 요약:
python3 summarize_sweep.py runs/20260524_HHMMSS
```

## 해석 가이드
- **sm wait_ms/s ratio (BURST/IDLE)** 가 핵심 지표. 잘 격리됐다면 ≤ 1.5×, 격리
  실패면 5–10×. `none` baseline에서 얼마나 떨어지는지로 효과 측정.
- **rslidar inter-arrival BURST mean/max** 가 100ms 근처면 한 프레임 통째로
  큐에 밀린 것. LIDAR_MODE 다르게 했을 때 이게 어떻게 변하는지가 lidar 격리의
  실제 이득.
- **cpu_leak_pp > 10** 이면 격리한 코어 안으로 glim work가 새고 있음 — taskset이
  새 스레드를 못 잡고 있다는 신호. 그 condition은 격리가 명목상일 뿐.
- **throttle_per_min** 이 condition별로 비슷해야 정상. 한 condition만 유난히
  높으면 그 sweep 중 보드 온도 환경이 달랐던 것 — 같은 조건 한 번 더 돌릴 것.

## 다음 단계 (이 디렉터리 결과로 답할 질문들)
- [ ] 현재 default(`workers_lidar_e`)가 정말 ICRA 본선에 쓸 만한 최적인가?
      아니면 `workers_lidar_p`가 더 나은가?
- [ ] WORKER_ISO=1 의 worker compaction 비용이 IDLE wait를 키우지 않는가?
- [ ] 주행 중 불안정은 (a) glim burst 시 sm wait spike 때문인가, (b) ICCMAX
      throttle 때문인가, (c) 둘 다인가? — sweep 결과에서 burst 시 sm wait가
      이미 통제됐는데도 throttle/min이 높은 condition을 보면 답이 나옴.
