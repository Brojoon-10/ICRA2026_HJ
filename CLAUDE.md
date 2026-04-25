# CLAUDE.md - Project Context

## ⚠️ 세션 시작 시 필수 질문 (매 세션 첫 응답에서 반드시 수행)

새 세션이 시작되면 **어떤 사용자 작업도 시작하기 전에** 가장 먼저 사용자에게 물어볼 것:

> **"이번 세션은 HJ 모드인가요, IY 모드인가요?"**
> - HJ 모드 (기본): Sampling + MPPI, MPCC 백엔드 작업. `overtaking_iy.launch` 미기동, `dynamic_avoidance_mode:=NONE`.
> - IY 모드: rolling-horizon (`planner/overtaking_iy/`, `IY_docs/`) 작업. IY 사용자가 직접 요청한 경우에만.

- 이 질문은 **매 세션 시작 시 한 번** 해야 함 (사용자가 먼저 모드를 명시하지 않은 경우).
- 사용자 답변에 따라 작업 범위, 커밋 경로, 참조 문서가 달라지므로 **답을 받기 전에는 파일 수정/빌드/커밋 같은 쓰기 작업 금지**.
- 단순 질의응답(읽기 전용)은 먼저 답해도 되지만, 그 응답 끝에 모드 확인 질문을 붙일 것.
- 사용자가 이미 "HJ로 작업 시작"/"IY 쪽 커밋" 등으로 모드를 암시했다면 질문 생략하고 진행해도 됨.

## 프로젝트 개요
UNICORN Racing Team의 자율주행 레이싱 스택 (ROS1, Noetic).
2D 평면 레이싱 → **3D 맵 주행** 확장은 대부분 완료 (ICRA 2026 RoboRacer).
현재 주 작업: **3D 트랙에서 안정적·효율적 overtaking 파이프라인** 제작.
- 역할 분담:
  - **HJ (본 세션): sampling-based, MPC(MPCC) 백엔드 집중**
  - IY: rolling-horizon(CasADi+IPOPT, `overtaking_iy`) 담당 → HJ 작업 시에는 해당 런치 구동하지 않음
  - SQP(SLSQP, `3d_sqp_avoidance_node`)는 공용 베이스라인
- 우선은 state_machine과의 직통 연동은 TODO, 각 플래너는 별도 observation 토픽으로 경로의 valid함(충돌 없음 + 연속성 + feasibility)을 증명
- 테스트 맵: `gazebo_wall_2` (`sim:=true`, Gazebo는 컨테이너 바깥 호스트 ROS에서 구동 중)

## 언어
- 한국어로 설명하고 대화할 것

## 작업 원칙

### ⏱ 구현 속도 (매우 중요)
- **기본 기대치: 한 기능 / 한 재설계는 "하루 안에" 코드까지 완성**. 멀티페이즈 문서를 길게 늘어놓고 "Phase 1만 오늘 완료"하는 식으로 일정을 늘리지 말 것.
- ICRA 2026 마감이 가깝기 때문에, "Phase 1~9, 각각 며칠"식 로드맵은 **사치**. 대부분의 재설계는 **하루 집중 세션 한 번으로 구현 완료 + 가동 확인**이 디폴트.
- Claude는 "안전하게 작게 쪼개자"는 성향이 강한데, 이 프로젝트에서는 **쪼갤수록 총 기간이 늘어남** → 위험. 한 방향으로 결정되면 **그 세션 안에 MVP가 돌아가는 상태까지** 밀고 나갈 것.
- 쪼개도 되는 경우: (1) 사용자가 명시적으로 단계화를 지시한 경우, (2) 이틀 이상 소요가 명백한 대규모 포팅(예: 스택 전체 3D 전환), (3) 실험/튜닝이 포함돼 반복 관측 필요.
- 작업 시작 전 "전체 phase N개, 며칠 예정"처럼 긴 타임라인을 제시하는 습관 **금지**. 대신 "오늘 안에 X를 돌아가게 하고, 튜닝은 다음 세션"처럼 **하루 단위로 완성 기준을 명시**.
- 중간 산출물(임시 래퍼, 비활성 플래그, "다음 세션에서 붙일 TODO 스텁")을 남기는 것도 비효율. 가능한 **한 번에 live 상태**로 완료.

### 🖥 디버깅은 Claude가 직접 터미널에서 실시간 `rostopic echo`로 한다 (HJ 모드 기본 운용 방식)

**핵심 원칙**: 복잡한 플래너/컨트롤러(MPC, sampling, prediction 등)를 만들거나 튜닝할 때, Claude가 **본인이 직접 관리하는 background Bash 터미널에서 `rostopic echo -c` / `rostopic hz`로 디버그 토픽을 실시간 모니터링**하며 디버깅할 것. 이 방식이 HJ 모드의 **디폴트** 디버깅 루프다. 사용자가 RViz를 눈으로 보고 구두로 리포트해주는 걸 기다리는 수동 루프 금지.

**디버그 토픽 설계 (노드 쪽):**
- 모든 복잡 노드는 디버깅에 필요한 **모든 내부 상태를 한 토픽에 담아** 발행 — 예: `~debug` (`PlannerDebug.msg` 신규 정의) 또는 급할 때 `std_msgs/String`에 JSON 한 줄. 필드 예시:
  - NLP 상태: `ipopt_status`, `iter_count`, `solve_ms`, `success`
  - Cost breakdown: `cost_contour`, `cost_progress`, `cost_input_rate`, 기타 항목별
  - Corridor margin: `margin_left_min`, `margin_right_min` (N+1 step 중 최소)
  - Obstacle 주입: `n_obs_used`, `obs_source_tag`, `obs_freshness_ms`
  - Side decision: `side` (`left|right|trail|clear`), `score_left`, `score_right`, `score_trail`
  - Warm-start / fallback: `warm`, `fallback_tier`, `fail_streak`
  - Tick-to-tick jitter: `traj_jitter_rms` (이전 tick 대비 L2)
- 필드 이름은 **사람이 눈으로 읽어서 이해 가능**하게. 단위 포함(`solve_ms`, `margin_min_m`, `n_traj_max_m`).
- 기본 주기 발행, `~publish_debug:=false`로 끄기 가능. 디버그 토픽이 이미 있으면 **갈아엎지 말고 확장**.

**Claude의 디버깅 루프 (이 순서로 돈다):**
1. 노드 수정 후 Docker 안에서 빌드 → 런치 기동 (background Bash)
2. **별도 background Bash에서 `rostopic echo -c /<node>/debug` 또는 필요한 경우 여러 토픽 동시 echo**. 이 터미널은 계속 살려두고 수정 루프 내내 주시.
3. 증상 관측 → 숫자 기반으로 가설 수립(예: "wall_margin_min=0.05면 hard corridor 아닌 것", "cost_obs=946 vs cost_wall=1244면 wall이 obstacle에 진다")
4. 코드 수정 → 다시 echo로 숫자 변화 확인 → 수렴할 때까지 반복
5. 수치가 합격선(예: `solve_ms p95 < 25`, `margin_min > 0.15m`, `traj_jitter < 0.05m`)에 들어오면 사용자에게 **수치 증거와 함께 리포트**
6. Background Bash 관리: `run_in_background=true`로 띄우고, 분석 후 세션 종료 전에 정리(죽이기). 떠 있는 터미널들을 Claude가 항상 파악하고 있을 것.

**수치 기반 판정 원칙:**
- "느낌상 좋아졌다" 금지. 항상 echo 결과의 **구체적 숫자**를 인용해서 판단. 사용자 리포트에도 수치 첨부.
- 토픽에 담긴 필드 이외의 정보가 필요하다고 판단되면, **코드에 해당 필드를 추가 발행하는 것부터 선행**. 디버깅을 위한 토픽 확장은 망설이지 말 것.

### 공통 원칙
- Overtaking 파이프라인의 목표: **벽/장애물 무충돌 + 주행 연속성(궤적 진동 억제) + feasibility(solve 성공/abort 동작) 확보**
- 2D 원본(`base_system.launch`, `headtohead.launch`, `sqp_avoidance_node.py` 등)은 **사용자가 요청할 때만** 참조. 기본은 3D 버전(`3d_*`) 기준으로 작업
- 3D 버전은 `3d_` 프리픽스 신규 파일로 분리, 2D 원본은 보존
- 판단에는 항상 근거와 증거(파일 경로, 라인 번호, 토픽명 등)를 제시할 것
- 노드 분석 시 반드시 실제 launch 파일의 인자(arg/param)를 추적하여, 해당 인자 조합에서의 실제 실행 경로를 파악할 것. 코드만 보고 "이 기능이 있다"가 아니라 "이 launch 설정에서 이 코드 경로가 실행된다"를 확인
- 3D/오버테이킹 관련 수정에는 `### HJ :` 주석 사용
  - 예: `### HJ : add z for 3D closest-point search`
- 작업 중 발생하는 이슈, 해결 방법, 해결된 사항을 `TODO_HJ.md`에 즉시 업데이트할 것
  - 새 이슈 발견 시 TODO 항목 추가
  - 해결 완료 시 체크박스 체크 + 해결 방법 간단히 기록
  - 판단 보류 사항은 별도 섹션에 근거와 함께 기록

## 핵심 경로

### 맵 / 웨이포인트
- 맵 데이터: `stack_master/maps/<map_name>/`
- **테스트 맵 (현재 기본)**: `stack_master/maps/gazebo_wall_2/`
  - `global_waypoints.json`, `gazebo_wall_2_3d_rc_car_10th_timeoptimal.csv`, `gazebo_wall_2_bounds_3d.csv`
- 웨이포인트 메시지: `f110_utils/libs/f110_msgs/msg/Wpnt.msg` (z_m, mu_rad 포함)

### Frenet
- C++ (3D): `f110_utils/libs/frenet_conversion/src/frenet_conversion.cc`
- Python (3D): `f110_utils/libs/frenet_conversion/src/frenet_converter/frenet_converter.py`
  - `get_frenet_3d`, `get_cartesian_3d` — height filter + boundary raycast + 회전 검색 + fallback
- 서비스 srv: `Glob2Frenet`, `Glob2FrenetArr`, `Frenet2Glob`, `Frenet2GlobArr` (z 필드 포함)

### 런치 (3D 운용)
- Base: `stack_master/launch/3d_base_system.launch` (localization + frenet + sector + sim/real 분리)
- Controller + state_machine + planners: `stack_master/launch/3d_headtohead.launch`
- Overtaking 서브-런치 (사용자별):
  - IY 모드: `planner/overtaking_iy/launch/overtaking_iy.launch` (HJ 작업 시 **미기동**)
  - HJ 모드: sampling은 `3d_headtohead.launch sampling_planner_enable:=true`, MPC는 `mpc_planner/launch/mpc_planner_state.launch` (observation 토픽)

### 상태/제어/플래너
- 컨트롤러: `controller/controller_manager.py`, `controller/combined/src/Controller.py` (L1/PP, 3D 대응)
- 상태머신(3D): `state_machine/src/3d_state_machine_node.py`
- 동적 회피 계열 플래너 (병렬 개발 중, 모두 **observation 토픽**으로 유효성 증명 단계):
  - `planner/sqp_planner/src/3d_sqp_avoidance_node.py` — SLSQP 기반 `d(s)` 프로파일 회피 (one-shot, 공용 베이스라인)
  - **[HJ 집중]** `planner/3d_sampling_based_planner/node/sampling_planner_state_node.py` — state-aware sampling + MPPI, 연속성 필터
  - **[HJ 집중]** `planner/mpc_planner/node/mpc_planner_state_node.py` — kinematic MPCC (2D NLP + Track3D 3D lift)
  - [IY 담당] `planner/overtaking_iy/src/overtaking_iy_node.py` — CasADi + IPOPT rolling-horizon SQP (20Hz)
- Prediction: `prediction/gp_traj_predictor/src/3d_opp_prediction.py`, `3d_opponent_trajectory.py`

## Docker 환경
- 컨테이너 이름: `icra2026`
- 호스트 → 컨테이너 경로 매핑: `$HOME/icra2026_ws/ICRA2026_HJ` → `$HOME/catkin_ws/src/race_stack`
  (컨테이너 안에서 `$HOME`은 호스트 사용자명과 동일하게 구성됨)
- **빌드, import 테스트, 코드 실행은 반드시 Docker 컨테이너 안에서 할 것**
- **Gazebo 시뮬레이터는 컨테이너 바깥(호스트)에서 ROS로 구동 중** — 컨테이너 ↔ 호스트 간 ROS master 공유
```bash
# 컨테이너 내부에서 명령 실행 ($HOME은 컨테이너 안에서도 호스트와 동일하게 매핑됨)
docker exec icra2026 bash -c 'source /opt/ros/noetic/setup.bash && source $HOME/catkin_ws/devel/setup.bash && <명령>'

# 빌드 예시
docker exec icra2026 bash -c 'source /opt/ros/noetic/setup.bash && source $HOME/catkin_ws/devel/setup.bash && cd $HOME/catkin_ws && catkin build'
```
- 로컬 파일 수정은 호스트에서, 빌드/테스트는 Docker에서
- **로컬 git에 영향주는 docker 명령 (fetch, checkout 등) 절대 금지**

## 빌드
```bash
cd $HOME/catkin_ws && catkin build
# HJ 집중 패키지
catkin build mpc_planner 3d_sampling_based_planner sqp_planner obstacle_publisher
```

## 실행 (gazebo_wall_2, sim — HJ 기준)

시뮬레이터(Gazebo)는 **호스트**에서 별도 기동. 아래는 컨테이너 내부에서 순서대로.

**원칙**: `3d_headtohead.launch`에서 overtaking 관련 노드는 **항상 끄고**(`dynamic_avoidance_mode:=NONE`, `sampling_planner_enable:=false` 등) 운용. 각 overtaking 백엔드는 **별도 런치로 따로** 기동. headtohead 내부에서 overtaking 백엔드를 직접 켜는 것은 **완전 통합 검증이 끝난 백엔드에 한해** 허용.

```bash
# 터미널 1: 3D base (localization + frenet + sector + odom relay)
roslaunch stack_master 3d_base_system.launch map:=gazebo_wall_2 sim:=true

# 터미널 2: controller + state_machine + 기본 planners (overtake 백엔드는 OFF)
roslaunch stack_master 3d_headtohead.launch dynamic_avoidance_mode:=NONE

# ─── HJ 작업용 overtaking 백엔드 (별도 런치, 택일) ───
# (A) Sampling + MPPI state-aware
roslaunch 3d_sampling_based_planner sampling_planner_state.launch state:=overtake
# (B) MPCC state-aware (observation 토픽)
roslaunch mpc_planner mpc_planner_state.launch state:=overtake
```

> - IY 모드(`overtaking_iy.launch`)는 **IY가 직접 요청할 때만** 기동. HJ 세션에서는 안 띄움.
> - `dynamic_avoidance_mode:=NONE` + `sampling_planner_enable:=false`가 HJ 기본. SQP / sampling을 headtohead 안에서 켜는 건 통합 단계로 올라간 뒤에만.
> - 여러 overtaking 노드를 동시에 띄우면 `/planner/avoidance/otwpnts` dual publisher race 발생 — 별도 런치는 **반드시 한 번에 하나만**, 또는 observation suffix(`_observation`) 토픽으로 격리된 것끼리만.

## 세션 인수인계 (HJ 기본 원칙)

- 기본 가정: **이 저장소에서의 작업은 HJ이 주도**. 세션 간 맥락 상속이 중요.
- 세션 중 의미 있는 설계 결정/디버깅/실험 결과는 **`HJ_docs/<topic>.md`에 즉시 정리**
  - 새 주제는 새 파일, 기존 주제는 해당 파일 갱신 (예: `sampling_planner_state_machine_integration.md`, `mpc_planner_state_machine_integration.md`)
  - 문서 맨 위에 작성일 / 상태 / "다음 세션 첫 10분 복귀 가이드" 블록 유지
  - 진행 체크리스트 + 다음 할 일 섹션을 끝에 둬서 다음 세션이 바로 이어받을 수 있게
- 단기 작업 체크리스트는 `TODO_HJ.md` (프로젝트 루트)에 유지, 장기·설계는 `HJ_docs/`에 분리
- 백업이 필요한 파일 수정 시 `<name>_backup_YYYYMMDD.<ext>` 규칙으로 먼저 복사 (sampling/mpcc 통합 플랜과 동일)

## Git 작업 원칙 (매우 중요)

이 저장소는 HJ / IY가 같은 머신에서 교대로 작업하는 공유 트리이며, **커밋되지 않은 로컬 변경사항이 날아가는 것이 가장 큰 리스크**. Claude는 아래 규칙을 무조건 지킬 것.

### 절대 하지 말 것 (명시 요청 없이는 금지)
- `git checkout <branch>` / `git switch` — 브랜치 변경 (로컬 변경사항 덮어쓸 수 있음)
- `git pull`, `git fetch && git merge`, `git rebase` — 원격 반영 (충돌/덮어쓰기 위험)
- `git reset --hard`, `git checkout .`, `git restore .`, `git clean -fd` — **로컬 변경 전량 소실 가능**
- `git stash` 후 잊기 — stash pop을 잊으면 실질적 소실
- 넓은 범위의 `git add -A` / `git add .` — **다른 사람(IY)의 미커밋 변경이 섞여 들어감**
- Docker 컨테이너 안에서의 git 조작 (fetch/checkout/reset 등) — 호스트 git에 영향. 절대 금지
- `git push --force`, `--force-with-lease` (main 포함) — 명시 요청 전까지 금지
- 커밋 메시지에 Claude Co-Authored-By 트레일러 추가 (memory 규칙)

### 커밋 전 필수 상황 파악 루틴

커밋/푸시/체크아웃 등 git 쓰기 작업 직전에는 **반드시** 아래를 먼저 확인하고, 결과를 사용자에게 1~2줄로 요약 보고 후 승인받기:

```bash
git status              # untracked + modified 목록
git diff --stat         # 변경 규모
git log -5 --oneline    # 최근 커밋 히스토리 (브랜치 상태)
git branch -vv          # 현재 브랜치 + 원격 추적 상태 (ahead/behind)
```

특히 다음이 보이면 즉시 멈추고 사용자에게 확인:
- 이번 세션에서 건드리지 않은 파일의 modified 상태 (다른 세션/IY 작업일 가능성)
- `HJ_docs/backup/`, `IY_docs/`, `*_backup_*` 류의 변경 (백업 건드림)
- submodule (`m `로 표시되는 항목 — `f110_utils/libs/ccma`, `perception/2.5d_detection` 등) — **submodule은 별도 저장소**, 함부로 커밋하지 말 것

### 커밋 규칙 (HJ 세션)

- **세션에서 직접 작성/수정한 파일만** 선별해서 `git add <path>`로 명시 추가. `-A`/`.` 금지.
- 커밋 한 번에 **하나의 논리적 단위**만. 여러 주제가 섞였으면 커밋 여러 개로 쪼갤 것.
- 커밋 메시지 스타일은 `git log --oneline` 기준 기존 관례 따르기: `<scope>: <짧은 요약>` (예: `mpc_planner: Phase 3.5/4/5 obstacle cost + dynreg APIs`).
- Co-Authored-By 트레일러 **금지**.
- 푸시는 사용자 명시 요청이 있을 때만. 푸시 전 `git branch -vv`로 원격 추적/ahead 수 확인 → 사용자에게 보고.

### IY가 IY 작업만 커밋할 때 가이드 (Claude가 IY를 돕는 경우)

IY는 주로 `planner/overtaking_iy/`, `IY_docs/` 아래에서 작업. IY가 "내 작업만 커밋하고 싶다"고 하면 **아래를 그대로 안내**:

1. 현재 상태 파악 — 본인 변경과 HJ 변경이 섞여 있을 가능성:
   ```bash
   git status
   git diff --stat
   ```
2. IY 작업 범위에 해당하는 경로만 골라서 staging. **경로를 하나씩 확인**하며 추가:
   ```bash
   # 예시: overtaking_iy 패키지 + IY_docs 갱신
   git add planner/overtaking_iy/
   git add IY_docs/

   # 특정 파일만 추가할 때 (전체 디렉터리 변경 중 일부만)
   git add planner/overtaking_iy/src/overtaking_iy_node.py
   git add planner/overtaking_iy/config/overtaking_iy.yaml
   ```
3. staging 결과 확인 — **HJ 파일이 섞여 들어가지 않았는지 반드시 눈으로 확인**:
   ```bash
   git status                    # Staged / Not staged 분리 확인
   git diff --cached --stat      # staging된 변경 규모
   git diff --cached             # staging된 실제 내용 (필요시)
   ```
   Staged 쪽에 `HJ_docs/`, `planner/mpc_planner/`, `planner/3d_sampling_based_planner/`, `stack_master/maps/`, `state_machine/src/3d_*`, `controller/` 등 **HJ가 건드리는 경로가 있으면 잘못 추가된 것**. `git restore --staged <path>`로 빼기.
4. HEREDOC으로 커밋 (메시지에 Claude 트레일러 금지):
   ```bash
   git commit -m "$(cat <<'EOF'
   overtaking_iy: <간결한 요약 한 줄>

   <필요시 본문 몇 줄>
   EOF
   )"
   ```
5. 푸시 전 브랜치 상태 확인:
   ```bash
   git branch -vv                # 현재 브랜치 + ahead/behind
   git log origin/main..HEAD --oneline   # 푸시될 커밋만
   ```
6. 문제 없으면 푸시:
   ```bash
   git push origin <current-branch>
   ```
   main으로 바로 푸시하는 관행이라면 `git push origin main`. **force 계열 금지**.

만약 **HJ 변경이 working tree에 남아 있는 상태**라면, 그대로 둔 채 IY 경로만 `git add`하면 HJ 변경은 unstaged로 유지되어 안전. 절대 `git stash`나 `git checkout`으로 지우려 하지 말 것.

### 충돌 / 애매한 상황

- working tree에 예상 못 한 파일이 있으면 **삭제 전에 사용자에게 출처를 묻기** (IY 작업, 다른 세션, 빌드 아티팩트 등 판별)
- 원격이 앞서 있어서 (`behind`) pull이 필요해 보여도 **자동으로 pull하지 말고** 사용자에게 "원격이 N commits 앞서 있습니다. pull 할까요?" 물어볼 것
- merge/rebase 충돌이 생기면 즉시 멈추고 상황 보고. `--abort` 여부도 사용자가 결정

## 주의사항
- f1tenth_simulator는 2D only → 3D 맵에서는 사용 불가. Gazebo(호스트)로만 시뮬
- 2D 전용 `base_system.launch`, `headtohead.launch`는 3D에서 **사용하지 않음** (참조용으로만 존재)
- Overtaking 백엔드들은 **단일 publisher 원칙**. 동시 기동 시 반드시 한쪽은 observation suffix(`_observation`) 토픽으로 발행하거나, 기존 SQP를 `NONE`으로 꺼야 함
- TODO 관리: `TODO_HJ.md` (프로젝트 루트), 장기 설계/인수인계 문서: `HJ_docs/<topic>.md`

## 미확인 사항 (판단 보류)
- carstate_node.py: GLIL 환경에서 필요 여부 미확정
  - vy를 pose 미분+moving average로 자체 계산, `/car_state/pitch` 발행 등의 기능이 있음
  - GLIL base_odom이 vy, pitch를 충분히 제공하는지 확인 필요 (현재 3d_base_system.launch에서는 제거)
