---
name: Phase 1 — ObstacleHorizon spec
description: GP timestep-별 (s, d) 시퀀스를 obs_arr 에 직접 사용. vs/vd constant 외삽 폐기.
phase: 1
created: 2026-05-02
status: in-progress
related_phases: [0 (완료), 4 (후행 — Phase 1 의 정확한 obs_arr 가 plan-aware MPC 의 전제)]
---

# Phase 1 — ObstacleHorizon spec sketch

## 목표 (한 줄)

`/opponent_prediction/obstacles` 의 N timestep (s, d) 시퀀스를 그대로 `obs_arr` 에 채우고, NLP / corridor / directional bubble 이 매 horizon timestep 의 정확한 obs 위치를 보고 풀이하게 한다. 현재 `_extrapolate_obs_traj` 의 vs/vd constant 외삽은 폐기.

## 의존성

- 선행 phase: Phase 0 완료 (publish gate / SM 정리). 코드는 b8580b4, a982af7 commit 이 이미 main 에 있음.
- 외부 모듈 / 토픽:
  - `/opponent_prediction/obstacles` (`ObstacleArray`, `prediction/gp_traj_predictor/src/3d_opp_prediction.py` publish). 한 opponent 당 N=20 개 Obstacle 메시지, 각 메시지가 timestep 0~N-1.
  - `/tracking/obstacles` (`ObstacleArray`, perception). 1 obstacle 당 1 msg.
- 후행 phase: Phase 4 (PlanAwareMPC) 가 이 obs_arr 를 cost / constraint 에 활용.

## 핵심 결정

- **prediction 메시지의 N timestep 시퀀스 보존** (현재는 timestep 0 만 추출 후 vs/vd 외삽).
- **tracking 을 canonical 로** (이전 결정 유지) — prediction 의 N msg 를 한 opponent 로 묶을 때 tracking 매칭이 anchor.
- **prediction horizon (~1~2초) 이후 tail 처리**: 마지막 timestep 의 (s, d) 를 hold (constant 외삽이 아니라 정지 가정). NLP horizon 이 prediction 보다 길 때 안전.
- **정지 obs**: prediction msg 가 없거나 vs<0.05 → (s_center, d_center) 를 N+1 모두 동일 채움 (현재 구현 그대로).

## 수정 파일 / 함수

| 파일 | 함수 / 위치 | 변경 |
|------|-----------|------|
| `planner/mpc_planner/node/mpc_planner_state_node.py` | `_merge_obs_sources` (line 869~964) | 출력 구조 변경. 매칭된 prediction opponent 의 N timestep 전체 보관. |
| 동상 | `_extrapolate_obs_traj` (line 966~985) | **폐기** (또는 backward-compat 용으로 남기되 호출 X). |
| 동상 | `_build_obstacle_array_xy` (line 987~1037) | xy backend 도 sequence 사용 (legacy 경로지만 일관성). |
| 동상 | `_build_obstacle_array_frenet` (line 1041 부근, `is_static_flag` 분기 line 1140~1148) | dynamic 분기에서 vs/vd 외삽 제거, prediction sequence 직접 사용. |
| 동상 | tick_json publish (`_publish_tick_json` 또는 동등) | `obs_horizon_source` 필드 추가 (debug). |

## 데이터 구조

### 이전 (`_merge_obs_sources` 출력)

```python
out: list[Obstacle]   # tracking 1개당 1 entry
                      # vs, vd, vs_var, vd_var 가 매칭된 prediction msg 의 timestep 0 값으로 덮어씀
```

### 신규

```python
@dataclass
class ObsHorizonEntry:
    # tracking-canonical 식별
    obs_id:         int             # tracking id
    is_static:      bool

    # 시퀀스 (N+1, ) — N 은 NLP horizon
    s_seq:          np.ndarray      # (N+1,) timestep 별 s_center
    d_seq:          np.ndarray      # (N+1,) timestep 별 d_center
    confidence_seq: np.ndarray      # (N+1,) prediction 영역 1.0, tail 감쇠

    # 부속 정보 (현재 코드와 호환)
    half_width:     float           # obs.size/2 (또는 fixed)
    vs_var_max:     float           # prediction variance 최댓값 (Phase 5 safety buffer 용)
    vd_var_max:     float

merged_obs: list[ObsHorizonEntry]
```

`_merge_obs_sources` → `(merged_obs, tag)` 반환. `tag` 형식 그대로 (`'tanchor:T%d+P%d'`).

### `obs_arr` shape

현재:
- `frenet_kin`: `(n_obs_max, N+1, 3)` — `[s, d, w_gate]`.

신규:
- 같은 shape `(n_obs_max, N+1, 3)`. **shape 유지**. 단지 `[:, :, 0]` 와 `[:, :, 1]` 채울 때 prediction sequence 직접 인덱싱.

## 알고리즘 의사코드

### `_merge_obs_sources` (신규)

```python
def _merge_obs_sources(self):
    pred_fresh  = self._is_fresh(self._obs_predict_t) and ...
    track_fresh = self._is_fresh(self._obs_track_t) and ...

    if not track_fresh:
        # tracking 끊김. legacy fallback (정지 obs 처리 또는 빈 list).
        return [], 'none'

    # 1. prediction msg 들을 opponent 별로 그룹핑
    #    같은 opponent 의 timestep 0..N-1 을 묶음. anchor = (timestep0.s, .d).
    pred_groups = group_prediction_by_anchor(self._obs_predict.obstacles)
    # pred_groups: list[ list[Obstacle] (sorted by timestep) ]

    out = []
    for tracked_obs in self._obs_track.obstacles:
        match_group = find_closest_group(tracked_obs, pred_groups,
                                         s_thresh=1.5, d_thresh=0.5)
        if match_group is not None:
            s_seq, d_seq = extract_seq_from_group(match_group, N+1)
            confidence_seq = build_confidence_seq(len(match_group), N+1,
                                                   tail_decay=0.5)
            entry = ObsHorizonEntry(
                obs_id=tracked_obs.id,
                is_static=False,
                s_seq=s_seq,
                d_seq=d_seq,
                confidence_seq=confidence_seq,
                half_width=...,
                vs_var_max=max(p.vs_var for p in match_group),
                vd_var_max=max(p.vd_var for p in match_group),
            )
        else:
            # prediction 매칭 없음 → 정지 가정.
            entry = ObsHorizonEntry(
                obs_id=tracked_obs.id,
                is_static=True,
                s_seq=np.full(N+1, tracked_obs.s_center),
                d_seq=np.full(N+1, tracked_obs.d_center),
                confidence_seq=np.full(N+1, 1.0),
                half_width=...,
                vs_var_max=0.0, vd_var_max=0.0,
            )
        out.append(entry)
    return out, f'tanchor:T{len(out)}+P{n_matched}'
```

### `extract_seq_from_group` 로직

```python
def extract_seq_from_group(group, target_len):
    # group: list[Obstacle] (timestep 0..M-1, M ≤ target_len)
    M = len(group)
    s_seq = np.zeros(target_len)
    d_seq = np.zeros(target_len)
    for k in range(target_len):
        if k < M:
            s_seq[k] = group[k].s_center
            d_seq[k] = group[k].d_center
        else:
            # tail: prediction horizon 이후. 마지막 (s, d) 유지.
            # 즉 vs/vd constant 외삽 안 함. obs 가 prediction 끝 위치에 정지한다고 가정.
            s_seq[k] = group[M-1].s_center
            d_seq[k] = group[M-1].d_center
    return s_seq, d_seq
```

### `_build_obstacle_array_frenet` (변경 부분)

기존 `is_static_flag` 분기 (line 1140~1148):
```python
if is_static_flag:
    obs_arr[n_used, :, 0] = s0
    obs_arr[n_used, :, 1] = d0
else:
    vs = float(o.vs); vd = float(o.vd)
    for k in range(N_plus_1):
        obs_arr[n_used, k, 0] = s0 + vs * k * dT
        obs_arr[n_used, k, 1] = d0 + vd * k * dT
```

신규:
```python
# entry: ObsHorizonEntry (이전 단계에서 _merge_obs_sources 가 반환)
obs_arr[n_used, :, 0] = entry.s_seq            # timestep 별 s
obs_arr[n_used, :, 1] = entry.d_seq            # timestep 별 d
# (s wrapping 은 entry 생성 시 처리하거나 여기서 한 번 더 mod track_length)
```

훨씬 간단. `is_static_flag` 분기 자체 제거 가능 (entry.s_seq 가 정지면 N+1 모두 동일).

## 실패 케이스 + 처리

| 케이스 | 트리거 | 처리 |
|-------|-------|------|
| prediction msg 안 옴 (predictor 죽음) | `pred_fresh=False` | tracking-only. 모든 obs 를 정지로 처리 (entry.is_static=True, s/d_seq 상수). |
| tracking 끊김 (perception 죽음) | `track_fresh=False` | `[], 'none'` 반환. obs_arr 모두 FAR_XY. NLP 가 obstacle 없음으로 풀이. |
| prediction msg 가 N 보다 짧음 (M < N+1) | `len(group) < N+1` | tail (k≥M) 은 마지막 (s, d) 유지. confidence tail decay 0.5. |
| 매칭 안 되는 tracked obs | `find_closest_group` 가 None | 정지 가정 (entry.is_static=True). 안전한 default. |
| tracking 1개 ↔ prediction 2 group 매칭 | 가까운 group 둘 다 있음 | score 기반 best 1 개 선택 (현재 코드 패턴 유지). |

## 튜닝 파라미터 + 초기값

| 파라미터 | 단위 | 초기값 | 의미 |
|---------|------|-------|------|
| `prediction_match_s_thresh` | m | 1.5 | tracking ↔ prediction matching s 거리 최대 |
| `prediction_match_d_thresh` | m | 0.5 | 동상 d 거리 |
| `prediction_match_score_d_weight` | — | 2.0 | matching 점수에서 d 가중 (s 보다 강조) |
| `tail_confidence_decay` | — | 0.5 | prediction horizon 이후 confidence (Phase 2 forecaster 가 쓸 입력) |

대부분 기존 코드의 hard-coded 값 그대로. rosparam 화는 Phase 5 튜닝 시.

## 검증

### 시나리오

#### bag 1: 정적 + 동적 obs 1대씩 (정상 케이스)
- gazebo_wall_2 sim, 동적 obstacle s=ego.s+8m, vs=5 m/s.
- 정적 obstacle s=ego.s+15m.
- 30s 녹화.

#### bag 2: prediction 짧은 horizon (M < N+1 케이스)
- predictor 의 prediction horizon 을 의도적으로 짧게 (~5 timestep).
- tail 처리가 정확한지 확인.

#### bag 3: tracking 1초 끊김 (graceful degradation)
- perception node 잠깐 죽이거나 throttle.
- obs_arr 가 FAR_XY 로 채워지는지 + NLP 정상 풀이 확인.

### 분석 스크립트

새 파일: `HJ_docs/debug/0429_debug/scripts/analyze_phase1_obs_horizon.py`

기능:
- bag 의 `/mpc_auto/debug/tick_json` 에서 `obs_horizon_source`, `obs_arr[0,:,0]`, `obs_arr[0,:,1]` 시퀀스 추출.
- 같은 시점 `/opponent_prediction/obstacles` 의 timestep 별 (s, d) 와 비교.
- 일치율 측정 (각 timestep 의 |obs_arr - pred| 평균).
- constant 외삽 baseline 과 비교 표.

### 통과 기준 (정량)

| 지표 | 기준 |
|------|------|
| `obs_arr[o, k, 0]` ↔ prediction sequence 의 k-th timestep s 일치율 | 95%+ (반올림 오차 1cm 안) |
| NLP `solve_ms` p99 변화 | ±10% 이내 (구조 변화로 인한 latency 차이 작아야) |
| `_validate_publish_wpnts` 통과율 | Phase 0 baseline 동일 (회귀 없음) |
| Tracking 끊김 시나리오 | NLP 풀이 OK + obs_arr 가 FAR_XY 채움 |
| Collision count (30분 bag) | baseline 대비 동등 또는 감소 (개선 없음 OK, 증가 안 됨) |

## 빌드 & 검증 명령

```bash
# 빌드
docker exec icra2026 bash -c 'source /opt/ros/noetic/setup.bash && \
    source $HOME/catkin_ws/devel/setup.bash && \
    cd $HOME/catkin_ws && catkin build mpc_planner'

# 시뮬 + bag (별도 터미널)
roslaunch stack_master 3d_base_system.launch map:=gazebo_wall_2 sim:=true
roslaunch stack_master 3d_mpc_headtohead.launch dynamic_avoidance_mode:=NONE
roslaunch mpc_planner mpc_planner_state.launch state:=auto

# bag (사용자 직접 실행)
rosbag record -O 2026-05-XX-phase1.bag /car_state/odom_frenet \
    /tracking/obstacles /opponent_prediction/obstacles \
    /mpc_auto/debug/tick_json /mpc_auto/status \
    /planner/mpc/wpnts /local_waypoints

# 분석
docker exec icra2026 bash -c 'source /opt/ros/noetic/setup.bash && \
    python3 $HOME/catkin_ws/src/race_stack/HJ_docs/debug/0429_debug/scripts/\
    analyze_phase1_obs_horizon.py $HOME/catkin_ws/src/race_stack/bag/<bag>.bag'
```

## Milestone 분해 (TODO §1.5 의 1.1~1.3 와 매핑)

| TODO milestone | 본 spec 의 작업 | 예상 시간 |
|----------------|---------------|----------|
| 1.1 `_merge_obs_sources` 신규 출력 구조 | `ObsHorizonEntry` dataclass + 그룹핑 + 매칭 + tail 처리 | 1.5h |
| 1.2 `obs_arr` 빌드 변경 | `_build_obstacle_array_frenet` 의 dynamic 분기 교체. xy backend 동일 패턴 적용. | 1h |
| 1.3 검증 | tick_json 필드 추가 + 분석 스크립트 작성 + bag 실행 | 1.5h |

총 예상: **4시간** (한 세션 안 마무리 가능).

## 변경 history (구현 중 발견된 결정)

| 날짜 | 결정 / 발견 | 사유 |
|------|------------|------|
| 2026-05-02 | `_ObsWithHorizon` wrapper class 도입. spec 의 dataclass 안 → 기존 Obstacle msg attribute 그대로 + sidecar 만 추가. | ROS msg `__slots__` 라 임의 attribute attach 불가. 첫 시도에서 AttributeError 발생. wrapper 의 `__getattr__` 으로 underlying delegation. 기존 코드 (vs, vd, is_static 등 attribute 사용) 무영향. |
| 2026-05-02 | tail (k≥M) 처리: 마지막 점 hold → vs/vd extrapolate. | spec 초안의 "마지막 점 hold" 로 진행했더니 prediction horizon (~0.5s) 이후 obs sequence 가 freeze → NLP 가 obs 가 정지한 것처럼 봄 → side_decider/NLP disagree → SIDE DECISION 토글 + `Infeasible_Problem_Detected`. 사용자 bag `2026-05-01-10-07-02` 로 진단. tail 도 vs/vd constant extrapolate (matched_seq 의 마지막 두 점 finite-diff) 로 변경 → baseline (Phase 0) 과 일관. |
| 2026-05-02 | obs_arr 빌드의 s 처리: 통째 offset → per-timestep ego_s 도메인 unwrap. | 사용자 bag `2026-05-01-10-19-58` 분석에서 `s_seq=[83.66, 84.23, 84.80, 41.77, 42.34, ...]` lap 경계 wrap 발견. prediction 의 `s_center % track_length` modulo 가 sequence 후반부에서 wrap → 통째 `s_offset = s0 - obs.s_center` 적용 시 wrap 후 timestep 들이 잘못된 도메인 (ego 뒤 -42m) 에 들어감. NLP 가 obs 가 시간 중간 점프한 것으로 봄. fix: 매 timestep `while s_k - ego_s > tl/2: s_k -= tl` 로 ego_s 의 ±tl/2 도메인으로 정규화. |
| 2026-05-02 | SM `get_recovery_wpts` None defense + GB raceline fallback. | 본 phase 와 별개 부수 fix. 이전 세션 (2026-05-01) 의 NonObstacleTransition_GBMode 폴백을 `LOSTLINE, GB_TRACK` → `LOSTLINE, RECOVERY` 로 바꾼 후, `cur_recovery_wpnts.is_init=False` 인 spawn 직후 케이스에서 `_pub_local_wpnts` crash. fix: `get_recovery_wpts` 에 GB raceline slice fallback (recovery publisher 자체 부재 시만). user directive ("ego 멀면 GB 안 씀") 의 명시 예외. |
| 2026-05-02 | PLAN_TRAIL: `w_obs` 25.0 → 0.0, `q_n_target` 25.0 → 0.0. | 사용자 본질 지적: NLP infeasibility 11건 분석 결과 corridor 1mm 위반인데 IPOPT 가 strict fail. 원인 = TRAIL plan 인데도 obs Gaussian bubble (w_obs=25) 이 ego 를 lateral 로 obs 반대 방향 push → corridor 위반. 사용자 비전 ("trail 은 lateral free + 종방향 ref_v cap"). fix: TRAIL 의 obs cost 폐기 → NLP 가 corridor + progress + smoothness + ref_v tracking 만으로 풀이 → 항상 feasible. ref_v cap 은 이미 `mpc_planner_state_node.py:3013-3022` 에서 `obs.vs - 0.3` 적용 중. |
| 2026-05-02 | tick_json 에 `obs_horizon_source`, `s_seq`, `n_seq`, `midN_s`, `midN_n` 필드 추가. | Phase 1 검증 / 분석 스크립트 입력. |
| 2026-05-02 | 분석 스크립트 3종 신규 (`analyze_phase1_obs_horizon.py`, `analyze_prediction_sweep.py`, `analyze_infeas_root.py`). | bag 의 sequence 정합성 / GP sweep / NLP infeas 원인 정량 검증. |

## 검증 상태

- 빌드 / syntax 모두 통과.
- 사용자 bag 3건 (`10-07-02`, `10-19-58`, `10-26-16`) 으로 단계적 진단 + fix.
- **새 bag (PLAN_TRAIL w_obs=0 적용 후) 검증 대기 중**. 통과 기준: SIDE DECISION 토글 빈도 baseline 회귀 + NLP infeasibility 0~1건.
- 검증 통과 시 status → `implemented` + commit.
