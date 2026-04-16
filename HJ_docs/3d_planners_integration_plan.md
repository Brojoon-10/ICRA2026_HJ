# 3D Planners Integration Plan

> **Goal**: Add two new planner packages — `3d_sampling_based_planner` (TUM sampling) and `mpcc_planner` (Heedo MPCC, extended for 3D) — under `planner/`, run them in **observation mode** (publish output topics only, no pipeline injection), and prepare a clean container-side install path.

---

## 0. Scope & Non-Goals

**In scope (Phase 0–2)**
- Two ROS1 packages built in workspace
- Each subscribes to existing stack topics, publishes its own trajectory topic
- Self-contained install scripts that set up Python deps **inside the `icra2026` container**
- Per-package data assets reuse from `3d_gb_optimizer/global_line/data/`
- RViz markers for visual comparison vs current global raceline

**Out of scope (deferred to later phases)**
- Wiring into `state_machine` (no pipeline takeover yet)
- Replacing `controller_manager` outputs
- Dynamic obstacle handling (static evaluation first)
- Cost morphing / state-aware cost (architecture only — implementation in Phase 3)

---

## 1. Reference Repos

| Package | Source | License | Role |
|---|---|---|---|
| `3d_sampling_based_planner` | https://github.com/TUMRT/sampling_based_3D_local_planning | **GPLv3** | Frenet sampling + gg-validation, 3D native |
| `mpcc_planner` | https://github.com/heedo7425/icra_unicorn_heedo (`mpc/`) | None (ask author) | CasADi+IPOPT MPCC, 2D → extend for 3D |

**License note**: GPLv3 contagion risk for sampling planner. Keep it as a **separate, optional package** — never link statically into core stack code. ROS topic boundary is sufficient isolation per FSF interpretation but verify with team before publishing.

---

## 2. Directory Layout

```
planner/
├── 3d_sampling_based_planner/         # NEW (GPLv3 isolated)
│   ├── CMakeLists.txt
│   ├── package.xml                    # depends: rospy, f110_msgs, visualization_msgs
│   ├── README.md
│   ├── LICENSE                        # GPLv3 (inherited)
│   ├── install/
│   │   ├── install_deps.sh            # pip installs inside container
│   │   └── requirements.txt           # numpy, scipy, shapely, pandas, joblib, casadi==3.5.6rc2
│   ├── src/                           # vendored from upstream + modifications
│   │   ├── sampling_based_planner.py  # core (LocalSamplingPlanner)
│   │   ├── track3D.py                 # SHARE w/ 3d_gb_optimizer if API matches
│   │   ├── ggManager.py               # SHARE w/ 3d_gb_optimizer if API matches
│   │   └── trajectory_generator.py    # polynomial samples
│   ├── nodes/
│   │   └── sampling_planner_node.py   # ROS wrapper, observation mode
│   ├── config/
│   │   └── default.yaml               # n_range, K, weights
│   ├── launch/
│   │   └── sampling_planner_observe.launch
│   └── rviz/
│       └── sampling_observe.rviz
│
├── mpcc_planner/                      # NEW
│   ├── CMakeLists.txt
│   ├── package.xml                    # depends: rospy, f110_msgs, ackermann_msgs, nav_msgs
│   ├── README.md
│   ├── LICENSE                        # MIT or copy-from-upstream-after-asking
│   ├── install/
│   │   ├── install_deps.sh            # pip install casadi (IPOPT bundled)
│   │   └── requirements.txt           # casadi>=3.6, numpy, scipy
│   ├── src/
│   │   ├── mpcc_solver.py             # vendored, modified for 3D
│   │   ├── mpcc_solver_3d.py          # NEW: 3D-augmented model
│   │   └── path_lut_3d.py             # 3D B-spline LUT (x, y, z, theta_pitch, theta_roll)
│   ├── nodes/
│   │   └── mpcc_planner_node.py       # ROS wrapper, observation mode
│   ├── config/
│   │   ├── mpcc_2d.yaml               # baseline (Heedo's params)
│   │   └── mpcc_3d.yaml               # 3D-augmented
│   ├── launch/
│   │   └── mpcc_planner_observe.launch
│   └── rviz/
│       └── mpcc_observe.rviz
│
└── 3d_gb_optimizer/                   # EXISTING — code shared via symlink or sys.path
    └── global_line/
        └── data/                      # gg_diagrams, vehicle_params, smoothed_track_data
```

### Code Sharing Strategy

The TUM sampling repo bundles `track3D.py`, `ggManager.py`, `point_mass_model.py` — **identical or near-identical** to `3d_gb_optimizer/global_line/src/`. Two options:

- **Option A (recommended for Phase 0)**: Vendor copies into `3d_sampling_based_planner/src/`. Pros: fully isolated, no sys.path hacks. Cons: code duplication.
- **Option B (Phase 2 cleanup)**: Extract `track3D`, `ggManager` into a shared library (`f110_utils/libs/track3d_utils/`) and import from both packages.

Start with A, refactor to B once both packages are running stably.

---

## 3. Required Inputs & Where They Come From

Both planners consume the same physical "what is the world right now" inputs. Listed once, then per-planner specifics.

### 3.1 Static Assets (loaded at startup)

| Asset | Source | Used by |
|---|---|---|
| 3D smoothed track CSV (`*_3d_smoothed.csv`) | `stack_master/maps/<map>/` | sampling, mpcc-3d |
| Track bounds CSV (`*_bounds_3d*.csv`) | `stack_master/maps/<map>/` | sampling |
| Global raceline CSV (`*_timeoptimal.csv`) | `stack_master/maps/<map>/` | both (reference / contour) |
| `global_waypoints.json` | `stack_master/maps/<map>/` | both (Wpnt grid template) |
| GG diagram (`gg_diagrams/rc_car_10th_latest/`) | `3d_gb_optimizer/global_line/data/` | sampling (and mpcc-3d for friction limits) |
| Vehicle params (`params_rc_car_10th_latest.yml`) | `3d_gb_optimizer/global_line/data/` | both |

Pass these via launch params (`map_name:=gazebo_wall_2_iy`) — node resolves paths from `$(find stack_master)/maps/$(arg map_name)/`.

### 3.2 Runtime Subscriptions

| Topic | Type | Source node | Used as |
|---|---|---|---|
| `/car_state/odom_frenet` | `nav_msgs/Odometry` | `frenet_odom_republisher` | s, n, vs |
| `/car_state/odom` | `nav_msgs/Odometry` | `glim_ros` (relayed) | x, y, z, yaw |
| `/car_state/pose` | `geometry_msgs/PoseStamped` | `extract_pose_from_odom.py` | pose (alt) |
| `/global_waypoints` | `f110_msgs/WpntArray` | `global_planner` (offline-loaded) | reference raceline |
| `/global_waypoints_scaled` | `f110_msgs/WpntArray` | `vel_scaler` | scaled v_ref |
| `/ekf/imu/data` *(real car only)* | `sensor_msgs/Imu` | IMU driver | ax, ay (sim: derive from prev solution) |
| `/dynamic_obstacles` *(later)* | `f110_msgs/ObstacleArray` | perception | obstacle predictions (Phase 3) |

### 3.3 State Variables Mapping

```
solver state          ROS source                              fill rule
─────────────────────────────────────────────────────────────────────────
s                     /car_state/odom_frenet pose.x          direct
n                     /car_state/odom_frenet pose.y          direct
V (=vs)               /car_state/odom_frenet twist.linear.x  direct
chi (heading err)     yaw(/car_state/odom) - theta_track(s)  computed, wrap [-pi,pi]
ax                    /ekf/imu/data lin_acc.x  OR  prev      conditional on real_car arg
ay                    /ekf/imu/data lin_acc.y  OR  prev      conditional on real_car arg
z                     /car_state/odom pose.z                 direct (3D only)
pitch, roll           track3D.theta_*(s)                     interpolator output (3D only)
```

---

## 4. Per-Planner Specs

### 4.1 `3d_sampling_based_planner`

**Algorithm**: For each timer tick, sample K lateral offsets `n_k ∈ [n_min, n_max]` over horizon `s ∈ [s_now, s_now+L]`, generate cubic-polynomial Frenet trajectories, validate against gg-diagram + track bounds, score by cost, publish best.

**Key params (config/default.yaml)**:
```yaml
horizon_m: 8.0          # lookahead distance
n_samples_lateral: 11   # candidate count
n_range_m: [-0.5, 0.5]  # lateral offset range
dt: 0.1                 # internal time step
publish_rate_hz: 20

cost:
  w_raceline_dev: 5.0
  w_smoothness:   2.0
  w_progress:    -1.0   # negative → reward
  w_velocity:     1.0
```

**Subscriptions**: §3.2 set
**Publications**:
- `/sampling_planner/best_trajectory` (`f110_msgs/WpntArray`) — chosen trajectory
- `/sampling_planner/all_candidates` (`visualization_msgs/MarkerArray`) — colored by cost
- `/sampling_planner/status` (`std_msgs/String`) — "OK" / "NO_FEASIBLE" / etc.
- `/sampling_planner/timing_ms` (`std_msgs/Float32`) — solve time

**3D handling**: native via `Track3D` (z, pitch, roll come from interpolators).

### 4.2 `mpcc_planner`

**Baseline** (Phase 1): vendored Heedo MPCC, 2D kinematic bicycle, N=6, dT=0.1, IPOPT.

**3D Extension Plan** (Phase 2, see §6):
- Augment path LUT with `(x(s), y(s), z(s), theta_pitch(s), theta_roll(s))` — built from `Track3D`
- Modify cost: contour error in 3D (project onto track tangent plane, not horizontal)
- Add slope-aware velocity limit: `v_max(s) = √(μ·g·cos(pitch) / κ_eff(s))`
- Friction circle uses gg-diagram entry indexed by current `a_z` (vertical accel from slope)

**Key params (config/mpcc_3d.yaml)**:
```yaml
N: 10
dt: 0.1
vehicle_L: 0.36
max_speed: 15.0
max_steer: 0.4

cost:
  w_contour: 10.0
  w_lag:      5.0
  w_progress: 1.0
  w_v_ref:    2.0
  w_d_steer:  0.5
  w_d_v:      0.3
```

**Subscriptions**: §3.2 set + needs `/global_waypoints` for reference path B-spline build (one-shot at startup).

**Publications**:
- `/mpcc_planner/horizon_trajectory` (`f110_msgs/WpntArray`) — N predicted points
- `/mpcc_planner/horizon_path` (`nav_msgs/Path`) — RViz friendly
- `/mpcc_planner/predicted_v` (`std_msgs/Float32MultiArray`) — v profile
- `/mpcc_planner/status` (`std_msgs/String`)
- `/mpcc_planner/timing_ms` (`std_msgs/Float32`)

---

## 5. Container Setup (Install Scripts)

**Container**: `icra2026` (existing, from CLAUDE.md). Workspace mounted at `/home/unicorn/catkin_ws/src/race_stack`.

### 5.1 `3d_sampling_based_planner/install/install_deps.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail
# Run INSIDE the icra2026 container.
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[3d_sampling] Installing Python deps into container Python..."
pip3 install --no-cache-dir -r "${PKG_DIR}/install/requirements.txt"

# Optional: acados is NOT required for sampling-only mode — skip
echo "[3d_sampling] Done."
```

`requirements.txt`:
```
numpy>=1.21
scipy>=1.7
shapely>=1.8
pandas>=1.3
joblib>=1.1
matplotlib>=3.5
```

### 5.2 `mpcc_planner/install/install_deps.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[mpcc] Installing CasADi (bundles IPOPT) into container Python..."
pip3 install --no-cache-dir -r "${PKG_DIR}/install/requirements.txt"

# Sanity check
python3 -c "import casadi; print('CasADi:', casadi.__version__)"
echo "[mpcc] Done."
```

`requirements.txt`:
```
casadi>=3.6.0
numpy>=1.21
scipy>=1.7
```

### 5.3 Invocation

```bash
# Host (one-liners; install scripts run inside container)
docker exec icra2026 bash /home/unicorn/catkin_ws/src/race_stack/planner/3d_sampling_based_planner/install/install_deps.sh
docker exec icra2026 bash /home/unicorn/catkin_ws/src/race_stack/planner/mpcc_planner/install/install_deps.sh
```

**Idempotency**: `pip install` is idempotent. Scripts can be re-run safely.

**Build**:
```bash
docker exec icra2026 bash -c "source /opt/ros/noetic/setup.bash && \
  cd /home/unicorn/catkin_ws && catkin build 3d_sampling_based_planner mpcc_planner"
```

**Env isolation**: NOT using virtualenv to avoid ROS Python path conflicts. Container's system Python is the target. If conflict arises, switch to `--user` install per package.

---

## 6. MPCC 3D Extension — Honest Assessment & Plan

### 6.0 Reality Check

**Q: Is MPCC commonly done in 3D?**
**A: No.** Almost all published MPCC (Liniger 2014/2015 origin, EVO-MPCC, F1TENTH variants) is **planar**. 3D MPCC exists for aerial vehicles (SE(3)), but for ground racing on banked/sloped tracks it's rare. The standard approach when the track is 3D is to **bake 3D physics into an offline raceline + speed profile** (which `3d_gb_optimizer` already does), and let MPCC track that 2D-projected reference. So "MPCC in 3D" usually means "MPCC tracking a 3D-aware reference," not "MPCC with 3D dynamics."

**Q: Is g_tilde (effective gravity along track tangent plane) considered?**
**A: Not in kinematic bicycle.** g_tilde requires force/mass — kinematic bicycle has neither. It's a pure geometry+velocity model:
```
ẋ = v cos(ψ),  ẏ = v sin(ψ),  ψ̇ = v/L · tan(δ)
```
There is no `m`, no `g`, no `μ`. You **cannot inject gravity into the dynamics** — only into **constraints** added externally.

**Q: Does friction increase/decrease appear?**
**A: No, not natively.** Friction enters only if:
- (a) Model becomes dynamic (lateral force balance: `m·v²·κ = F_y ≤ μ·F_z`)
- (b) You add a **constraint** like `v² · κ ≤ μ_eff · g_normal` outside the dynamics

Kinematic bicycle gives you (b) at best — and that's a **velocity ceiling**, not a model of how friction shapes trajectory.

### 6.1 What's Actually Possible (3 honest options)

| Option | Model | g_tilde / friction | Effort | Verdict |
|---|---|---|---|---|
| **A. Kinematic + 3D constraints** | Heedo MPCC unchanged | Velocity ceiling only (constraint, not dynamics) | Low | Hacky but workable for small slopes (≤10°) |
| **B. Dynamic bicycle 2.5D** | Replace with dynamic bicycle (lateral tire force, mass) + 3D-tilted gravity vector | g_tilde in force balance, μ in tire model | Medium-high | Correct physics, but bigger rewrite. CasADi handles it fine. |
| **C. Point-mass on 3D Frenet** | Same as `3d_gb_optimizer/point_mass_model.py` | Native 3D, gg-diagram | High | Becomes what we're trying to escape. Defeats the purpose. |

### 6.2 Recommended: Option A for Phase 2a, Option B if needed

**Option A "Kinematic + 3D Reference + Slope-aware Velocity Cap"** — what we *actually* implement:

1. **Reference is 3D**: Path LUT extended with `(x(s), y(s), z(s), pitch(s), roll(s), kappa(s))` from `Track3D`. MPCC contour/lag computed in **track tangent plane** (project car position into local Frenet frame at nearest s).

2. **Dynamics stay 2D kinematic**: Solver still works in `(x, y, ψ, s)`. Banking does NOT enter dynamics. The car "thinks" it's flat.

3. **Velocity ceiling as soft state constraint** (this is the only place 3D physics enters):
   ```
   At each prediction step k:
     pitch_k = pitch_LUT(s_k)
     roll_k  = roll_LUT(s_k)
     g_n     = g · cos(pitch_k) · cos(roll_k)         # gravity normal to track
     a_lat_max = μ · g_n                              # friction circle (lateral budget)
     v_max(s_k) = √( a_lat_max / |κ(s_k)| )
     constraint: v_k ≤ v_max(s_k)  (soft, with slack)
   ```
   This is a **kinematic-level approximation** of "you slip on a steep tilted curve."

4. **Optional: longitudinal slope effect as feedforward**
   ```
   At each step:
     a_long_gravity = -g · sin(pitch_k)               # downhill = positive accel
   ```
   Add as a **bias term in v_dot prediction**, NOT a force balance. Correct sign helps the planner anticipate downhill speedup / uphill slowdown but is still a hack.

5. **Cost adds for 3D awareness**:
   - `w_z_dev * (z_car - z_ref(s))²` — discourages geometric drift on rolled banks
   - `w_v_slope * (v - v_max_slope)²` — soft preference for slope-respecting speed

### 6.3 What Option A does NOT do (be honest)

- ❌ Weight transfer (front-rear, left-right)
- ❌ Slip angle / tire saturation
- ❌ Coupling between steering and longitudinal force
- ❌ Aerodynamic effects (irrelevant at 15 m/s anyway)
- ❌ Gyroscopic effects on banked turns
- ❌ Real friction-circle interaction (we only cap lateral; longitudinal-lateral coupling ignored)

In short: **Option A is "MPCC pretending the track is flat, with a 3D-shaped speed limit bolted on."** It is not "3D MPCC" in any rigorous sense.

### 6.4 Why this is still useful

- Global raceline (`*_timeoptimal.csv`) already encodes 3D-optimal speed profile (computed offline by acados with full point-mass 3D physics)
- Online MPCC's job: **track that reference** + react to small disturbances
- Slope-aware velocity ceiling prevents MPCC from "improving" on the offline profile in ways that ignore slope physics
- This is exactly how Roborace, Indy Autonomous, and most racing stacks do it: **3D in offline planner, 2D in online tracker**, with velocity caps as the bridge

### 6.5 When to upgrade to Option B (dynamic bicycle)

Promote only if **all** of these are true:
- Phase 2a gives observed slip / corner-cutting on banked sections
- Lap-time gap to offline raceline > 5% on 3D maps
- We have measured tire μ values for the RC car (currently estimated)
- Team has 1-2 weeks to rewrite + retune

If we go to Option B, the dynamics become:
```
ẋ = v cos(ψ + β)
ẏ = v sin(ψ + β)
ψ̇ = v · cos(β) / L · tan(δ)
v̇ = (1/m) [ F_drive - F_drag - m·g·sin(pitch_proj) ]
β = atan( (lr/L) · tan(δ) )               # slip angle approx
F_y_max = μ · m · g · cos(pitch) · cos(roll)
constraint: |m · v² · κ| ≤ F_y_max         # friction circle (lateral)
```
Still a single-track approximation but now `m, g, μ, pitch, roll` actually appear in the dynamics. This is what most people mean by "3D-aware MPC."

### 6.6 Decision

- **Phase 2a (immediate)**: Implement **Option A**. Document explicitly that it is kinematic+caps, not full 3D dynamics.
- **Phase 2b (deferred)**: Option B only if Phase 2a shows clear failure modes. Estimate 1-2 weeks rewrite.
- **Reject**: Option C. If we want point-mass 3D, just use the existing `3d_gb_optimizer` online — that's what it is.

---

## 7. Implementation Phases

### Phase 0 — Skeletons (Day 1)
- [ ] Create both package directories with `package.xml`, `CMakeLists.txt`
- [ ] Vendor source files from upstream repos (clean, attribute, mark license)
- [ ] Write `install/install_deps.sh` and `requirements.txt`
- [ ] Run install scripts in container, verify imports
- [ ] `catkin build` both packages successfully
- **Verify**: `rosrun 3d_sampling_based_planner sampling_planner_node.py --help` runs

### Phase 1 — Observation-mode publishers (Day 2-3)
- [ ] Sampling node: subscribe to topics in §3.2, run sampler, publish `/sampling_planner/*`
- [ ] MPCC 2D node: subscribe, run baseline solver, publish `/mpcc_planner/*`
- [ ] Launch files for each, with map_name arg
- [ ] RViz config to visualize candidates + chosen trajectory
- **Verify**: With car driving global raceline (sim or playback), both nodes produce sensible markers; no exceptions for 5 min run

### Phase 2a — MPCC kinematic + 3D constraints (Day 4-5)
- [ ] Build 3D path LUT from `Track3D` (x,y,z,pitch,roll,kappa over s)
- [ ] Modify contour/lag cost: project car position into local Frenet tangent plane
- [ ] Add soft constraint `v_k ≤ √(μ·g·cos(pitch)·cos(roll) / |κ|)` per step
- [ ] Optional: longitudinal slope feedforward `a_long_g = -g·sin(pitch)`
- [ ] New launch (`mpcc_planner_observe_3d.launch`) with `mpcc_3d.yaml`
- **Verify**: On 3D map (gazebo_wall_2_iy), predicted v at known steep section is lower than 2D version; contour error stays bounded on banked turns
- **Document explicitly**: this is kinematic+caps, NOT 3D dynamics

### Phase 3 — State-aware cost (separate plan)
- Architecture only, not implemented here. See [State-aware cost plan TBD].

### Phase 4 — Pipeline integration (separate plan)
- Mux into `/global_waypoints_scaled` only after Phase 1-3 validated. Out of scope for this doc.

---

## 8. Validation Checklist

For each planner, before declaring Phase 1 done:

- [ ] Solve time < 50 ms (sampling) or < 30 ms (MPCC) on NUC
- [ ] No infeasibility for 90%+ of timer ticks during straight + corner driving
- [ ] Output WpntArray fields populated: `s_m, d_m, x_m, y_m, z_m, vx_mps, psi_rad, kappa_radpm, ax_mps2`
- [ ] RViz markers visible and sane
- [ ] Status topic transitions logged
- [ ] Logged 5-minute rosbag with all `/sampling_planner/*` and `/mpcc_planner/*` topics

---

## 9. Risks & Open Questions

| # | Risk | Mitigation |
|---|---|---|
| 1 | GPLv3 contagion from sampling planner | Isolate as separate package, communicate via topics only, document |
| 2 | CasADi/IPOPT version conflict in container | Pin CasADi 3.6.x; fallback to `--user` install if needed |
| 3 | `track3D.py` API drift between vendored and `3d_gb_optimizer` copies | Phase-2 cleanup: extract to shared lib |
| 4 | sim vs real ax/ay handling | Use `real_car:=true/false` arg; default to prev-solution-derived in sim |
| 5 | MPCC 2D contour cost meaningless on banked corners | 3D extension (§6) addresses |
| 6 | gg-diagram lookup latency in tight inner loop | Precompute LUT, no per-step interpolation |

**Open questions**:
- Heedo repo license? (currently absent — must request before vendoring publicly)
- Do we replicate `Track3D` and `GGManager` or share via `f110_utils/libs/`? (Phase 2 decision)
- Real-car IMU topic name confirmation needed (`/ekf/imu/data` assumed from existing code)

---

## 10. Quick-Start Commands (Once Built)

```bash
# Sampling planner observe (directory is 3d_sampling_based_planner/;
# ROS package name is sampling_based_planner_3d — ROS 1 doesn't allow pkg
# names that start with a digit, so directory and pkg name intentionally differ)
roslaunch sampling_based_planner_3d sampling_planner_observe.launch \
  map_name:=gazebo_wall_2_iy vehicle_name:=rc_car_10th_latest

# MPCC 2D observe (not yet implemented)
roslaunch mpcc_planner mpcc_planner_observe.launch \
  map_name:=gazebo_wall_2_iy use_3d:=false

# MPCC 3D observe (not yet implemented)
roslaunch mpcc_planner mpcc_planner_observe.launch \
  map_name:=gazebo_wall_2_iy use_3d:=true
```

---

## 11. Phase 0/1 구현 일지 — sampling_based_planner_3d

### 11.1 실제 셋업 순서

1. TUM 레포를 `planner/3d_sampling_based_planner/`에 통째로 복사 (`src/`, `data/`, `local_sampling_based/` 등 그대로 보존)
2. ROS 스킨 추가: `package.xml`, `CMakeLists.txt`, `config/default.yaml`, `launch/`, `node/sampling_planner_node.py`, `install/install_deps.sh` (pip3)
3. `ggManager.py`를 `3d_gb_optimizer`와 sys.path로 공유 (분리 시점에 byte-identical 확인). `track3D.py`, `point_mass_model.py`는 gb_optimizer에 HJ 수정본이 있어 로컬 유지.
4. ROS 패키지명 `3d_sampling_based_planner → sampling_based_planner_3d` (ROS 1은 숫자로 시작하는 이름 불가). 폴더명은 유지.
5. 런타임 버그 순차 수정 (아래 표).

### 11.2 버그 추적 로그 (시간 순)

| # | 증상 | 진단 | 해결 |
|---|---|---|---|
| 1 | `KeyError: 's_m'` 시작 시 | raceline CSV 컬럼이 `s_opt/v_opt/...`이지 `s_m/vx_mps`가 아님 | `_load_raceline_dict`에서 후보 컬럼명 리스트 탐색 |
| 2 | `AttributeError: Track3D has no track_length` | Upstream은 호길이 배열을 `self.s`로 저장, 스칼라 없음 | `self.track.s[-1]` 사용 |
| 3 | `KeyError: 't'` (calc_trajectory 내부) | raceline dict에 `t/V/chi/ax/ay` 키 누락 | `t = cumsum(ds/v)` 유도, CSV 나머지 필드 복사 |
| 4 | `KeyError: 't'` (2차) | `prediction`은 opponent id별 dict-of-dicts인데, 평면 dict를 넘김 | `prediction = {}` (상대차 없음) |
| 5 | 궤적 s가 랩 중간에서 24m 폭주 (1초, V≤5.8) | 2-lap raceline concat이 `np.interp(..., period=L)`의 xp 중복 유발 | 단일 lap으로 되돌림 |
| 6 | Stack frenet과 Track3D centerline 불일치 | `/car_state/odom_frenet`은 raceline 기준, Track3D는 centerline 기준 | `/car_state/odom` (cartesian) 구독 + `_cart_to_cl_frenet_exact`로 재투영 |
| 7 | 랩 끝에서 sample 꼬리 "역행" | Quintic 다항식이 끝 속도/가속도 BC만 있고 위치 BC 없어 overshoot | §11.3 참고 |
| 8 | 랩 끝 점들이 한 곳에 "몰림/뭉개짐" | 내가 넣은 `cummax` band-aid가 backward 점들을 같은 s에 stack | `cummax`/`stack-snap` 제거, `|ds|>0.20m` 시에만 truncate |
| 9 | 랩 wrap 직후 첫 tick에서 궤적이 트랙 한참 너머로 튐 | **근본 원인 = `np.interp` periodic 정밀도 버그** → §11.4 |
| 10 | Candidate 샘플이 best만 보이고 나머지 안 보임 | `_publish_candidates`에서 invalid skip + best skip → valid가 best 하나뿐이면 0개 | invalid도 옅게, best도 빨강으로, 모두 표시 |

### 11.3 다항식 꼬리 overshoot (#7, #8)

**현상.** 궤적 마지막 1~5개 sample의 s가 직전 대비 3~4cm 감소. RViz에서는 안 보이는 수준이지만, 첫 band-aid인 `np.maximum.accumulate`가 이 점들을 같은 s에 고정 → 같은 cartesian 위치에 중첩 → "점이 뭉친다".

**원인.** Upstream의 s(t)는 quintic jerk-optimal 다항식. 경계조건이 시작 (s, ṡ, s̈) + 끝 (ṡ, s̈) = 5개이고 **끝 위치 제약이 없음**. 다항식이 끝 속도/가속도 맞추다가 미세하게 역진 가능.

**선택한 해결:** 의미 있는 backward (> 0.20 m)에서만 truncate.
- 미세 떨림(3~4cm)은 무시 (RViz에서 안 보임)
- cummax는 점 stacking 유발, `xs[k+1:]=xs[k]`도 동일 → 둘 다 제거
- upstream 다항식 차수 변경은 jerk-optimality 깨뜨림 → PoC에서는 과함

### 11.4 랩 경계 "궤적 전역 폭주" 버그 (#9) — 핵심 이슈

**증상.** 차가 s=L→s≈0 넘은 직후 첫 tick에서 `s_raw[0]=0.17, s_raw[1]=63.6`. 0.033초에 63m 진행 — V=2인데 — 물리적 불가능.

**배제한 가설:**
- Frenet 투영 드리프트 — 매 tick `ds ≈ cart_jump` 검증 완료
- 2-lap raceline 확장 — 시도했으나 `period=L` interp에 xp 중복 유발, 악화
- `_cart_to_cl_frenet_exact`가 잘못된 세그먼트 선택 — top-3 nearest 로그로 확인, 항상 정확

**진짜 원인:** `np.interp(s_query, raceline['s'], raceline['t'], period=L)` 호출에서:

1. numpy 내부: `xp_wrap = xp % period` 적용
2. raceline s[-1] = 89.875073 == L **정확히 같음** → `89.875073 % 89.875073 = 0`
3. 정렬(sort) 시 이 wrap된 마지막 점(s≈0, t=18.184)이 배열 앞쪽으로 이동
4. s≈0.17 조회 시 numpy가 (s=0.000, t=18.184)와 (s=0.2, t=0.067) 사이를 보간
5. 결과: **t ≈ 2.5초** — lap 끝 시간(18초)과 lap 시작 시간(0초) 사이의 무의미한 보간값
6. 이 잘못된 t로 `postprocess_raceline`이 엉뚱한 인덱스에서 slice
7. `s_post` 첫 step이 12m 점프 → 다항식 경계조건 폭주 → 모든 candidate가 10~30m 날아감

**디버깅 과정:**
1. cartesian gap > 3m 감지 로그 추가 → JUMP 포착
2. `HJ_DEBUG` 환경변수로 랩 경계 근처 per-tick 상세 덤프 (state, postprocessed raceline, per-candidate 다항식 계수)
3. 3분 주행으로 실제 wrap 통과 포착
4. `s_post[-1] = 17.38 @ t_post[-1] = 1.51초` 확인 — V=3인 구간에서 11 m/s 진행이면 slice가 잘못됐다는 증거
5. Python 독립 실행으로 `np.interp(0.173, s, t, period=L) = 2.49` vs `period=None: 0.058` 재현
6. numpy 내부 동작 직접 추적 (`xp % period → sort → fp reorder → 경계 pad → 보간`) — wrap된 마지막 sample의 fp=T_lap이 앞쪽에 끼어든 것 확정

**해결 (5줄, `_load_raceline_dict`):** raceline s[-1]을 `L - 1e-6`로 pin.
- 데이터를 고쳐서 upstream 알고리즘이 설계대로 동작하게 함
- 증상 위에 band-aid 쌓기 X (cummax, JUMP-mitigation 오히려 악화시킴)
- closed-loop CSV에서 마지막 점은 첫 점과 동일 → 1µm 조정은 무해

### 11.5 `postprocess_raceline` wrap-attach

pin fix 이후에도 차가 랩 끝 ~0.5초 이내면 tail slice가 `horizon × 1.5`보다 짧음 → `np.interp`가 마지막 s에 clamp → sample 꼬리가 s=L에 정체.

**해결:** `src/sampling_based_planner.py::postprocess_raceline` 내부에서 time shift 직후, tail이 짧으면 raceline HEAD를 `s+L`, `t+tail_end+dt_first`로 concat. 바로 뒤의 `horizon × 1.5` clip이 잉여 제거 → 과잉 attach 무해.

### 11.6 시각화 파이프라인 (최종 형태)

```
traj['s'] (upstream, line 582에서 mod L 적용)
    │
    ├─ ds < -L/2 감지  → 이후 sample에 +L 누적 (unwrap)
    │
    ├─ ds < -0.20 m 감지 → 해당 인덱스에서 truncate
    │
    ├─ sn2cartesian(s_unwr % L, n) → wrap 일관적 cartesian 재계산
    │
    └─ 발행:
       ├─ ~best_trajectory : WpntArray (s_m 단조 증가)
       ├─ ~best_sample     : nav_msgs/Path (3D z 포함)
       ├─ ~best_sample/markers     : SPHERE per point (속도별 색)
       ├─ ~best_sample/vel_markers : CYLINDER (높이 = V × 0.1317)
       └─ ~candidates      : MarkerArray LINE_STRIP
            ├─ valid non-best : 회색, 얇게, 반투명
            ├─ invalid        : 짙은 회색, 매우 얇게, 거의 투명
            └─ best           : 빨강, 굵게, 불투명 (맨 마지막에 그려서 위에)
```

### 11.7 후보 시각화 이슈 (#10)

**현상.** `~candidates` 토픽에서 best만 보이고 나머지 후보가 안 나올 때가 있음.

**원인.** 초기 코드가 `if not valid: skip` + `if i == optimal_idx: skip` 처리 — 마찰원/경계 검증에서 대부분 탈락하고 valid가 best 하나뿐이면 그려지는 후보가 0개.

**해결.** 모든 55개(= v_samples × n_samples) 후보를 예외 없이 그림:
- best: 빨강 (1.0, 0.1, 0.1), 두께 0.07m, alpha 1.0
- valid non-best: 회색, 두께 0.03m, alpha 0.35
- invalid: 짙은 회색, 두께 0.015m, alpha 0.15

upstream에 `self.candidates` dict 노출 추가 (`s_array`, `n_array`, `valid_array`, `optimal_idx`).

### 11.8 현재 상태

- 랩 경계 시각적으로 매끄러움; 점프·몰림 없음
- 풀 horizon (npts=30) 유지
- `[wrap]` 로그가 랩 통과 시 정확히 1회 발생, unwrap 범위 정상
- `[trunc]`는 드물게 발생 (다항식이 실제로 > 20cm overshoot 시에만)
- 솔브 시간: 10Hz에서 8~22ms per tick
- 55개 후보 LINE_STRIP 중 valid 몇 개 + best 빨강 1개 항상 표시

잔여 외부 이슈:
- `/car_state/odom`이 가끔 순간이동 (GLIM 재초기화) — planner 영역 밖

---

## 12. 주요 설계 판단 근거 요약

| 판단 | 고려한 대안 | 선택 이유 |
|---|---|---|
| TUM upstream 통째로 wrapping | 자체 sampler 재구현 | upstream이 검증됨; 포팅 비용이 thin ROS wrapper보다 훨씬 큼 |
| 패키지명 `sampling_based_planner_3d`, 폴더명 `3d_sampling_based_planner` | 통일 (폴더 rename) | ROS 1이 숫자 시작 이름 금지; rename하면 기존 로그/IDE 참조 깨짐 |
| Track3D centerline에 직접 투영 (FrenetConverter 미사용) | 스택 frenet 서버 공유 | Track3D 자체 폴리라인이 solver 내부에서 사용됨; FrenetConverter는 다른 spline → cm급 drift → chi/bounds 불일치 |
| `raceline['s'][-1] = L - 1e-6` pin (trim 대신) | 마지막 점 삭제 | 삭제하면 `[s[-1], L)` 구간 gap이 mirror 실패 모드 유발; pin은 period를 정확히 채움 |
| `postprocess_raceline` 내부에 wrap-attach | node에서 감싸기 | attach는 slice와 clip *사이*에 일어나야 함 — node에서 하면 postprocess 전체 재구현 필요 |
| backward > 20cm에서만 truncate | cummax / stack-snap | cummax = 점 stacking; stack-snap = cartesian 몰림. truncate만 sample 유일성 보존 |
| 단일 lap raceline (2-lap 아님) | 2-lap concat | 2-lap은 `np.interp(period=L)` 깨뜨림 (xp 중복); periodic-interp + attach 조합이 안전 |
| `HJ_DEBUG` 로그 코드 유지 (env 제어) | 수정 후 삭제 | 이 영역이 깨지기 쉬움; 계측 남겨두면 향후 버그 빠르게 발견 가능 |
| 후보 전체 시각화 (invalid 포함) | valid non-best만 표시 | valid가 best 하나뿐이면 화면에 0개 → 디버깅 불가; 전부 그려야 "어디까지 시도했나" 파악 가능 |

---

## 13. TODO

### 13.1 Cost function 설계 + State Machine 연동

**목표.** Sampling planner의 cost function을 state-aware로 만들어 `state_machine`의
각 discrete state가 상황에 맞는 cost 프로파일을 선택하게 함. Sampling은 이 구조에
이상적 (cost가 순수 scoring 함수 — 미분 필요 없음, MPC와 달리).

**설계 개요:**
- cost term을 weight로 노출: `w_time`, `w_track`, `w_lat`, `w_vel`, `w_obs`, `w_smooth`
- state별 target 곡선: `n_target(s)`, `V_target(s)`
- state별 sample 분포 가변: `n_range`, `K` (추월 시 넓게, raceline tracking 시 좁게)

**State × Weight 테이블 (초안):**

| State | w_time | w_track | w_lat | w_vel | w_obs | w_smooth | n_target | V_target |
|---|---|---|---|---|---|---|---|---|
| `GB_TRACK` | 0 | 10 | 0 | 5 | 0 | 1 | 0 (raceline) | v_ref(s) |
| `GB_OPTIMIZER` | 1.0 | 0.1 | 0 | 0 | 0 | 1 | — | — |
| `OVERTAKE` | 0.5 | 0 | 5 | 1 | 50 | 2 | n_avoid(s) | v_ref·0.9 |
| `SMART_STATIC` | 0 | 0 | 10 | 5 | 100 | 1 | n_avoid_static | v_ref(s) |
| `TRAILING` | 0 | 5 | 0 | 20 | 30 | 3 | 0 | v_lead − Δ |
| `RECOVERY` | 0 | 0 | 0 | 10 | 0 | 5 | 0 | V_safe (~2) |
| `START` | 0.2 | 0 | 0 | 5 | 0 | 1 | 0 | ramp(t) |

**작업 목록:**
- [ ] `config/default.yaml`에 weight/target을 ROS param으로 노출 (`GB_TRACK` 행부터)
- [ ] 노드에서 `calc_trajectory(...)`를 state별 cost 번들로 호출하도록 리팩터
- [ ] `/state_machine/state` 구독 (없으면 `GB_TRACK` fallback)
- [ ] 상태별 `n_target(s)` / `V_target(s)` 주입 설계 (OVERTAKE, SMART_STATIC, TRAILING, RECOVERY, START)
- [ ] State별 sample 분포 (`n_range`, `K`) — 2~3가지 분포로 시작
- [ ] 검증: sim에서 각 state 통과하며 선택된 궤적이 의도에 맞는지 확인 (시각 + 로그)
- [ ] 파이프라인 주입을 `use_sampling_as_planner:=true` 하나로 gate; 기본값은 observation-only

**합리성 점검.** Sampling-based cost 교체는 가장 가벼운 통합 경로:
노드가 이미 state를 구독하고 자체 토픽에 발행 중이며 cost function은 순수 Python 점수 함수.
IPOPT/acados 재컴파일 불필요. State 전환 시 lateral 점프 없음 (sample 분포가 연속 변경).
채팅에서 도달한 설계 의도 ("planner 여러 개" 대신 "cost function을 state화") 그대로 구현.

### 13.2 Sampling planner 내부 구조 및 파라미터 가이드

#### 13.2.1 알고리즘 요약 — Solver 아님, 후보 선택 방식

이 planner는 **MPC/NLP solver를 사용하지 않는다.** 후보 궤적의 다항식 계수는
5×5 / 6×6 선형 연립방정식의 **직접 풀이** (`np.linalg.solve`). 반복 최적화가 아니라서
수렴 실패, warm-start 깨짐, NaN 발산 같은 solver 고유 위험이 없음.

```
[Step 1] 종방향 후보 v_samples개 생성 (4차 다항식, 끝 속도 목표별)
[Step 2] 횡방향 후보 n_samples개 × 종방향 v_samples개 = 총 후보 수
[Step 3] 물리 검증: 트랙 경계, 곡률, 마찰원(gg-diagram) → valid/invalid 분류
[Step 4] valid 후보에 cost 평가 → argmin = best sample
```

**"Infeasible"의 의미가 다름:**
- MPC: solver가 수학적 해를 못 찾음 (위험: 제어 입력 NaN 가능)
- Sampling: **55개 후보 전부 검증 탈락** → 고를 게 없음 (안전: 그냥 이번 tick skip)

최악 케이스가 "궤적 없음" → 기존 raceline으로 fallback 가능. 이 특성은 경쟁 주행에서
MPC 대비 안전성 이점.

**Spatial MPC 대비 저속 안정성 이점:**
Spatial MPC는 독립 변수가 arc-length(s)이므로 ds/dt → 0 (정지/저속)에서 수학적 특이점
발생 — 분모가 0으로 수렴하며 수치 발산 위험. 별도의 최소 속도 가드(`v ≥ ε`)나
temporal MPC 전환이 필요. 반면 sampling planner는 시간(t) 기반 다항식이라 **V=0에서도
정상 동작** — 정지 출발, 급감속, 스핀 후 저속 복귀 같은 상황에서 별도 처리 없이 안정적.
RC카 스케일에서 출발/정지/코너 저속 구간이 빈번하므로 이 장점이 실용적으로 큼.

#### 13.2.2 핵심 파라미터 (config/default.yaml)

| 파라미터 | 현재값 | 의미 | 영향 |
|---|---|---|---|
| `rate_hz` | 10 | 계획 주기 (Hz) | ↑ 올리면 반응 빨라짐. CPU 허용 시 20~30 가능 |
| `horizon` | 1.0 | 예측 시간 (초) | ↑ 멀리 보지만 다항식 overshoot 위험 ↑. RC카 1~2초 적정 |
| `num_samples` | 30 | 시간 discretization (궤적 당 점 수) | ↑ 궤적 해상도 ↑. 30~50이면 충분 |
| `v_samples` | 5 | 종방향 속도 목표 수 | ↑ 다양한 속도 프로파일 탐색. 10이면 더 정밀 |
| `n_samples` | 11 | 횡방향 위치 목표 수 | ↑ lateral 해상도 ↑. 15~21로 올리면 좁은 갭 통과 가능 |
| `safety_distance` | 0.20 | 트랙 경계 여유 (m) | ↓ 공격적, ↑ 보수적 |
| `gg_abs_margin` | 0.0 | 마찰원 절대 마진 (m/s²) | ↑ 보수적 (마찰원을 줄임) |
| `s_dot_min` | 1.0 | 최소 종방향 속도 (m/s) | 정지 상태 방지용 |

**총 후보 수 = v_samples × n_samples = 현재 55개.** 주요 튜닝 포인트:

| 목표 | 변경 | 효과 | CPU 비용 |
|---|---|---|---|
| 더 빠른 반응 | `rate_hz` 20~30 | 0.05~0.03초 간격 재계획 | ~2×~3× |
| 더 정밀한 경로 선택 | `v_samples=10, n_samples=21` → 210개 | 좁은 갭 통과, 미세한 speed 차이 반영 | ~4× |
| 더 먼 예측 | `horizon=2.0` | 15 m/s에서 30m 앞까지 | 동일 (후보 수 불변) |
| 후보 해상도만 올리기 | `num_samples=50` | 궤적 내 점 밀도 ↑ (시각화/보간 정밀도) | ~1.7× |

**실측 솔브 시간 (현재 설정, NUC):** 8~22ms per tick.
v_samples=10 + n_samples=21이면 ~50~80ms 예상. 10Hz 유지 가능. 30Hz는 버거울 수 있음.

#### 13.2.3 CPU 허용 시 올릴 수 있는 것

```yaml
# 공격적 설정 예시 (검증 필요)
rate_hz: 20
v_samples: 8
n_samples: 15      # 120개 후보
num_samples: 40
horizon: 1.5
```

120개 × 40점 = 4800 평가점. 현재 55×30=1650 대비 ~3배. NUC에서 30~60ms 예상.
20Hz (50ms budget) 내에 들어갈 가능성 있음. **실측 후 판단.**

### 13.3 Raceline 기반 Frenet 전환 가능성

현재 planner는 **centerline 기반 Track3D** 위에서 동작. 스택 나머지는 raceline
기반 frenet. 인터페이스에서 cartesian 변환으로 연결 중 (§11.2 #6).

**Raceline 기반 전환 시 이점:**
- 장애물 데이터가 raceline frenet으로 오면 변환 불필요
- `n=0 = raceline` 이라 tracking cost가 단순히 `n²`
- 스택 전체 frenet 통일 → 모듈 간 데이터 직접 교환 가능

**Raceline 기반 전환 시 필요 작업:**
- Raceline 곡선으로 Track3D 재빌드 (centerline CSV 대신 raceline CSV 입력)
  - 이건 Track3D 생성자에 다른 CSV 넣으면 되는 수준
- gg-diagram 재생성 (raceline 위의 slope/banking 기준)
  - `fast_ggv_gen/run.sh` 한 번 실행
- 트랙 경계 재계산: centerline 기준 `w_left/w_right` → raceline 기준으로 shift
  - `w_left_rl(s) = w_left_cl(s_cl) - n_rl(s_cl)` 같은 변환
- `sampling_based_planner.py` 내부의 `period=L` 호출을 raceline 호길이로 교체
- Wrap 버그 재검증 (raceline 호길이 ≠ centerline 호길이)
- **raceline이 바뀔 때마다 이 전체를 다시 돌려야 함** (offline 파이프라인 추가)

**현재 판단:** centerline 유지가 안전하고 실용적. 단, raceline 전환을 배제하지 않음.
장애물 통합이 본격화되면 frenet 통일의 이점이 커질 수 있으므로, 그 시점에 재평가.
핵심 판단 기준 = **장애물 데이터가 어떤 좌표계로 들어오는가** (cartesian이면 centerline
유지, raceline-frenet이면 전환 고려).

### 13.4 State Machine 연동 발전 방향

**Phase 3 (cost function state화):**
1. `config/default.yaml`에 state별 weight 테이블 노출
2. `/state_machine/state` 구독, 상태 변경 시 weight swap
3. 같은 55개 후보 중 **다른 기준으로 "best"를 고름** → state 전환 시 궤적 점프 없음

**Phase 4 (prediction 연동):**
1. `gp_traj_predictor` 출력 → `prediction` dict-of-dicts으로 변환
2. 장애물 위치가 cartesian이면 centerline 투영, raceline-frenet이면 변환
3. `prediction_cost_weight` 활성화 → OVERTAKE/TRAILING state에서 진짜 상대차 회피

**Phase 5 (pipeline 주입):**
1. observation → `use_sampling_as_planner:=true`로 `/global_waypoints_scaled`에 주입
2. State machine이 기존 spline planner 대신 sampling 출력 소비
3. 기존 planner와 A/B 비교 (같은 주행에서 둘 다 publish, 하나만 controller에 연결)

**Phase 6 (고급 튜닝):**
- State별 `horizon` / `n_range` / `v_samples` 동적 조정
- Adaptive rate: 장애물 감지 시 rate_hz 자동 상승 (20→30 Hz)
- Multi-horizon: 가까운 건 짧은 horizon (반응성), 먼 건 긴 horizon (예측성)

### 13.5 기타 계획 항목

- [ ] MPCC planner 패키지 (`mpcc_planner/`) — Phase 0 골격
- [ ] Dynamic bicycle MPC + 3D `g_tilde` + slope-aware 마찰원 (Phase 2b)
- [ ] 공유 `track3d_utils` 라이브러리 (`f110_utils/libs/`)
- [ ] Dynamic obstacle 처리 (`prediction` dict-of-dicts 연동)
- [ ] 3~4cm 다항식 꼬리 떨림 재검토 (끝 위치 soft BC 추가 가능)
- [ ] Raceline 기반 Track3D 전환 PoC — 장애물 통합 본격화 시점에 실행 여부 결정

---

**담당**: HJ
**생성**: 2026-04-16
**갱신**: 2026-04-17 — §11~13 추가 (구현 일지, 시각화, 파라미터 가이드, 발전 방향).
**상태**: sampling_based_planner_3d Phase 0/1 안정 (observation mode, 랩 경계 해결,
멀티 샘플 시각화 완료). 다음 = §13.1 (cost function state화 + state machine 연동),
§13.4 (prediction 연동).
