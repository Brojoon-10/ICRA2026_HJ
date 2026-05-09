# TODO_HJ

## Rolling-Horizon Overtake Planner (2026-04-19 시작)

설계 문서: `IY_docs/rolling_horizon_overtake_planner.md`
구현 플랜: `~/.claude/plans/casadi-radiant-elephant.md`

### 설계 결정
- [x] 출력 배선: 새 토픽 `/planner/rolling/otwpnts` → launch에서 `/planner/avoidance/otwpnts`로 remap. state_machine / controller 0 수정.
- [x] Solver: CasADi + IPOPT (true warm-start). scipy SLSQP 대비 dual/active-set 재사용 가능.
- [x] Launch 분리: `stack_master/launch/headtohead_rolling.launch` 신규 (기존 `headtohead.launch` 불변).
- [x] Rate: 20Hz (기존 SQP 노드와 동일).

### 구현 현황
- [x] 새 패키지: `planner/overtaking_iy/` (package.xml, CMakeLists.txt)
- [x] `src/sqp_casadi.py` — CasADi NLP (regularization + IPOPT warm-start)
- [x] `src/warm_start.py` — prev_d shifted to new s-grid
- [x] `src/velocity_profiler.py` — vel_planner FB pass 래퍼, IC matching
- [x] `src/abort_checker.py` — safety / performance abort + hysteresis + cooldown
- [x] `src/overtaking_iy_node.py` — 20Hz main node (기존 sqp_avoidance_node 입력 토픽 동일 구독)
- [x] `config/overtaking_iy.yaml`
- [x] `stack_master/launch/headtohead_rolling.launch`
- [ ] Docker 컨테이너에서 `catkin build overtaking_iy` — **다음 단계**
- [ ] 런치 dry-run: `/planner/rolling/*` 토픽 발행 확인
- [ ] 시뮬 회귀 검증: 일정 속도 opponent / 랜덤 블로킹 / 불가능 상황별 경로 morph, abort 동작
- [ ] solve time 99th percentile < 50ms (20Hz 예산) 확인
- [ ] Controller 연속성: `/vesc/commands` 점프 없음 확인 (rosbag)

### 재사용 자산 (코드 수정 없이 의존만 추가)
- `f110_msgs/Wpnt` (`kappa_radpm`, `ax_mps2` 필드 존재 → 새 msg 불필요)
- `f110_msgs/OTWpntArray`
- `prediction/gp_traj_predictor/gaussian_process_opp_traj.py` (`/opponent_trajectory`)
- `f110_utils/libs/vel_planner/src/vel_planner/vel_planner.py` (`calc_vel_profile`, `v_start` 지원)
- `stack_master/config/<racecar>/veh_dyn_info/ggv.csv`, `ax_max_machines.csv`, `b_ax_max_machines.csv`
- `sqp_planner/update_waypoints.py` (launch에서 그대로 include)

### 미확인 / 판단 보류
- **CasADi 설치**: icra2026 컨테이너에 `casadi` pip 패키지 존재 여부. 없다면 `pip install casadi` 필요 (IPOPT는 wheel 번들).
- **homotopy_lock=true가 좌/우 전환을 너무 경직되게 만들지 검증 필요**. 최초 한 번 side 결정 후 고정되는데, opponent가 반대쪽으로 크게 빠지면 재평가 필요할 수 있음. 현재는 `_more_space`로 side 산정하되 `homotopy_lock` 플래그일 때만 `last_desired_side` 고정.
- **shift_lookahead_s=0.05s**: solve 시간이 실측 50ms를 넘으면 ego state prediction이 과소평가됨 → 동적 조정 필요.

### 실행 레시피 (2026-04-20 확정)
`overtaking_iy.launch`는 `/planner/rolling/otwpnts → /planner/avoidance/otwpnts` remap 내장. `3d_headtohead.launch`는 건드리지 않고 `dynamic_avoidance_mode:=NONE`으로 SQP를 끈 뒤 rolling 런치로 takeover.

```bash
# 터미널 1: base system
roslaunch stack_master 3d_base_system.launch map:=experiment_3d_2

# 터미널 2: headtohead (SQP 비활성화)
roslaunch stack_master 3d_headtohead.launch dynamic_avoidance_mode:=NONE

# 터미널 3: overtaking_iy (필요할 때만 on/off)
roslaunch overtaking_iy overtaking_iy.launch
```

- 터미널 3 OFF → `/planner/avoidance/otwpnts` publisher 0개 → overtaking 시도 없음.
- 터미널 3 ON → rolling 단독 발행 (single publisher, race 없음) → state_machine takeover.

### 주의
- `dynamic_avoidance_mode:=SQP`(기본값)로 띄운 상태에서 `overtaking_iy.launch`를 추가로 실행하면 dual publisher race 발생. 반드시 `NONE`으로 띄울 것.
- Docker 내 git 조작 금지.
- 3D 확장 시: 현재는 Frenet d(s) 기반 2D → 3D waypoint 전환 시 `Wpnt.z_m`, `Wpnt.mu_rad` 활용한 별도 작업 필요.

---

## gg_tuner_node: raceline 파이프라인 rqt 반영 + raceline_ggv 신설 (2026-04-20)

구현 플랜: `~/.claude/plans/cryptic-noodling-mccarthy.md`

### 이슈 / 기능
- [x] **버그**: rqt `RACELINE_KEYS` (V_min, w_T, w_jx, w_jy, w_dOmega_z) 가 `run_ggv=False` 일 때 raceline 에 반영 안 됨.
  - 원인: `_write_latest_params_yml` 이 `run_ggv=True` 분기에서만 호출됨. raceline 옵티마이저는 yml 경로로만 이 키들을 받음 (`gen_global_racing_line.py:146-147` 의 `params.update`).
  - 해결: `_update_raceline_keys_in_yml(vehicle_name, tuning)` 헬퍼 추가 → `regen_raceline=True` 시 raceline 실행 직전 `_latest.yml` 에 RACELINE_KEYS 만 overlay. tire/vehicle/post 값은 불변.
- [x] **기능**: raceline 전용 GGV 선택 칸 신설.
  - rqt 파라미터: `raceline_ggv` (str, Raceline group, 기본값 `""`).
  - `run_ggv=False` + `raceline_ggv=<snapshot>` → `_activate_snapshot_as_latest` 가 `gg_diagrams/<snapshot>/` + `params_<snapshot>.yml` 을 `_latest` 자리로 복사 (원본 스냅샷은 불변).
- [x] **확인**: friction 동일한 sector 에 대한 GGV 재사용은 이미 `_generate_sector_ggvs` 의 `fresh_cache` 로 구현되어 있음 (중복 friction 시 `fast_ggv` 1회만 실행, 결과를 `_latest_sec<i>` 로 복사).

### 변경 파일
- `stack_master/cfg/GGTuner.cfg`: `raceline_ggv` str 파라미터 추가 (Raceline group)
- `stack_master/scripts/gg_tuner_node.py`:
  - `_update_raceline_keys_in_yml` 추가 (`_restore_to_latest` 위)
  - `_activate_snapshot_as_latest` 추가 (동일 위치)
  - `reconfigure_cb` 의 `run_opts` 에 `raceline_ggv` 수집
  - `_run_full_pipeline`: `run_ggv=False` 분기 진입 시 snapshot activation, `regen_raceline=True` 직전 RACELINE_KEYS overlay

### 검증 체크리스트
- [ ] 컨테이너에서 `catkin build stack_master` 통과 (cfg regen 포함)
- [ ] rqt 에서 `raceline_ggv` 칸 표시 확인
- [ ] `run_ggv=false, raceline_ggv=""`, `V_min` 변경 후 Apply → `_latest.yml` 에 `V_min` 만 바뀜, raceline 결과 반영됨
- [ ] `run_ggv=false, raceline_ggv=<기존 snapshot>` Apply → `_latest` 가 snapshot 으로 교체, 원본 snapshot 디렉토리/yml 불변
- [ ] `run_ggv=true` Apply (회귀): 기존 동작 그대로 (전체 merged yml 기록)
- [ ] `regen_raceline=false, run_fbga=true` Apply: yml 변경 없음, FBGA 만 재기동
