#!/usr/bin/env python3
"""Phase 1 verification — obstacle horizon GP sequence vs constant extrapolation.

For each tick, compare:
  - Our obs_arr sequence (from tick_json's `obstacles[*].s_seq / .n_seq`,
    fed by _merge_obs_sources sidecar — Phase 1 GP sequence path).
  - Reference: GP prediction msg's per-timestep (s, d) at the same time.
  - Baseline: constant vs/vd extrapolation from timestep-0 obs.

Outputs:
  - Match rate (our s_seq vs GP raw): 통과 기준 95%+ (within 1 cm).
  - Per-tick max diff: time-series.
  - GP-vs-constant baseline diff: shows whether the case actually exercises
    Phase 1 (i.e., GP differs from constant). If GP ≈ constant for the run,
    Phase 1 가 활성이어도 차이 안 보임 — sim 시나리오를 동적으로 만들어야.
  - obs_horizon_source 분포: tanchor_seq vs other.

Usage:
  python3 analyze_phase1_obs_horizon.py <bag>
"""
import sys, json, math
from collections import Counter
import numpy as np
import rosbag


if len(sys.argv) < 2:
    print('usage: analyze_phase1_obs_horizon.py <bag>')
    sys.exit(1)

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

ticks = []         # list of dict (parsed tick_json)
pred_msgs = []     # list of (t, [Obstacle, ...])
ego_states = []    # list of (t, ego_s, ego_v)

t0 = None
for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if t0 is None:
        t0 = ts
    if topic == '/mpc_auto/debug/tick_json':
        try:
            ticks.append(json.loads(msg.data))
            ticks[-1]['_t'] = ts
        except Exception:
            pass
    elif topic == '/opponent_prediction/obstacles':
        pred_msgs.append((ts, list(msg.obstacles)))
    elif topic == '/car_state/odom_frenet':
        ego_states.append((ts,
                           float(msg.pose.pose.position.x),
                           float(msg.twist.twist.linear.x)))
bag.close()

dur = (ego_states[-1][0] - ego_states[0][0]) if ego_states else 0
print(f'=== {bag_path.split("/")[-1]}  dur={dur:.1f}s ===')
print(f'  ticks={len(ticks)}  pred_msgs={len(pred_msgs)}  ego={len(ego_states)}')

# ---- obs_horizon_source 분포 ----
print('\n=== obs_horizon_source 분포 ===')
src_hist = Counter(t.get('obs_horizon_source', 'missing') for t in ticks)
for k, v in src_hist.most_common():
    pct = 100 * v / max(len(ticks), 1)
    print(f'  {k!r}: {v}  ({pct:.1f}%)')

# ---- Phase 1 active 비율 ----
phase1_active = sum(1 for t in ticks
                    if 'tanchor_seq' in (t.get('obs_horizon_source') or ''))
phase1_pct = 100 * phase1_active / max(len(ticks), 1)
print(f'\nPhase 1 (sequence sidecar) active: {phase1_active}/{len(ticks)}  ({phase1_pct:.1f}%)')


def near(arr, t):
    if not arr:
        return None
    lo, hi = 0, len(arr) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if arr[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return arr[lo]


# ---- our s_seq vs GP raw 매칭 ----
print('\n=== our obs s_seq vs GP raw prediction (Phase 1 sequence verify) ===')
match_diffs = []   # 각 tick 의 max |our_s − gp_s| (downsampled grid 위)
n_compared = 0
for tk in ticks:
    obs = tk.get('obstacles') or []
    if not obs:
        continue
    o0 = obs[0]   # slot 0 만 비교
    s_seq = o0.get('s_seq')
    n_seq = o0.get('n_seq')
    if not s_seq or not n_seq:
        continue
    # 같은 시점의 GP prediction
    pm = near(pred_msgs, tk['_t'])
    if pm is None or not pm[1]:
        continue
    pred_obs = sorted(pm[1], key=lambda p: int(getattr(p, 'id', 0) or 0))
    # tick_json 의 s_seq 는 obs_arr 의 down-sample. GP 의 i-th timestep 과 매칭
    # 어렵지만 단순화: GP 의 timestep idx 가 0, M/6, 2M/6, ... 와 비슷.
    M_pred = len(pred_obs)
    if M_pred == 0:
        continue
    diffs = []
    for k_our, s_our in enumerate(s_seq):
        # our s_seq down-sample idx: 0, step, 2*step, ...
        # tick_json side: step = max(N_p1//6, 1) → 6 ~ 7 points
        # GP side: 비례
        idx_pred = int(round(k_our * (M_pred - 1) / max(len(s_seq) - 1, 1)))
        idx_pred = max(0, min(idx_pred, M_pred - 1))
        gp_s = float(pred_obs[idx_pred].s_center)
        # Wrap difference (track length 가능)
        d = abs(s_our - gp_s)
        diffs.append(d)
    if diffs:
        match_diffs.append(max(diffs))
        n_compared += 1

if match_diffs:
    arr = np.array(match_diffs)
    print(f'  compared ticks: {n_compared}')
    print(f'  max diff (m):  mean={arr.mean():.3f}  p50={np.median(arr):.3f}  '
          f'p95={np.percentile(arr, 95):.3f}  max={arr.max():.3f}')
    within_1cm = int((arr < 0.01).sum())
    within_5cm = int((arr < 0.05).sum())
    pct1 = 100 * within_1cm / len(arr)
    pct5 = 100 * within_5cm / len(arr)
    print(f'  within 1cm: {within_1cm}/{len(arr)} ({pct1:.1f}%)')
    print(f'  within 5cm: {within_5cm}/{len(arr)} ({pct5:.1f}%)')
    print(f'  통과 기준 (95%+ within 1cm): {"PASS" if pct1 >= 95 else "FAIL"}')
else:
    print('  비교 가능한 tick 없음 (obstacle 없거나 sidecar 미부착).')

# ---- GP-vs-constant baseline 차이 ----
# constant 외삽 = obs_seq[0] + vs * dT * k. 이것과 GP 의 실제 timestep 차이.
print('\n=== GP sequence 가 constant 외삽과 얼마나 다른가 (시나리오 동적성) ===')
print('  → 차이 작으면 sim 이 등속 직선이라 Phase 1 효과 안 보임.')
print('  → 차이 크면 GP 가 곡률 / 가속 반영 — Phase 1 의 가치 드러남.')
gp_const_diffs = []
for ts, obstacles in pred_msgs:
    if len(obstacles) < 2:
        continue
    pred_obs = sorted(obstacles, key=lambda p: int(getattr(p, 'id', 0) or 0))
    s_first = float(pred_obs[0].s_center)
    vs_first = float(pred_obs[0].vs)
    # GP timestep 의 dT 모르므로 "GP sequence 의 인접 timestep 간 ds 의 변동"
    # = constant 외삽이라면 일정해야 하는 양. 변동 = 동적성.
    ds_arr = []
    for i in range(1, len(pred_obs)):
        ds_arr.append(float(pred_obs[i].s_center) - float(pred_obs[i-1].s_center))
    if len(ds_arr) >= 3:
        ds_mean = np.mean(ds_arr)
        ds_std = np.std(ds_arr)
        gp_const_diffs.append((ds_mean, ds_std))
if gp_const_diffs:
    arr = np.array(gp_const_diffs)
    print(f'  GP timestep ds: mean={arr[:,0].mean():.3f}m  std={arr[:,1].mean():.4f}m')
    print(f'  std/mean ratio: {arr[:,1].mean()/max(abs(arr[:,0].mean()), 1e-3):.3f}')
    print('  → ratio < 0.05 면 거의 등속 직선, Phase 1 효과 미미.')
    print('  → ratio > 0.1  면 동적 / 곡선 — Phase 1 의미 있는 차이.')
else:
    print('  prediction sequence 충분치 않음.')

# ---- NLP solve_ms 회귀 ----
print('\n=== NLP solve_ms 회귀 검사 ===')
solve_ms = [t.get('solve_ms') for t in ticks if t.get('solve_ms') is not None]
if solve_ms:
    arr = np.array(solve_ms)
    print(f'  count={len(arr)}  mean={arr.mean():.1f}ms  p50={np.median(arr):.1f}  '
          f'p95={np.percentile(arr,95):.1f}  p99={np.percentile(arr,99):.1f}  max={arr.max():.1f}')

# ---- Publish validation 회귀 ----
print('\n=== publish validation 회귀 검사 ===')
status_seq = [(t['_t'], t.get('status'), t.get('ipopt_status')) for t in ticks]
fail = sum(1 for _, st, _ in status_seq if st == 'PUBLISH_VALIDATE_FAIL')
print(f'  PUBLISH_VALIDATE_FAIL: {fail}/{len(status_seq)}  '
      f'(Phase 0 baseline: 0 expected)')

# ---- 첫 / 마지막 5 tick 의 obs sequence sample ----
print('\n=== sample obs sequences (first 3 + last 3) ===')
for i in (list(range(min(3, len(ticks)))) + list(range(max(len(ticks)-3, 0), len(ticks)))):
    tk = ticks[i]
    obs = tk.get('obstacles') or []
    if not obs:
        continue
    o = obs[0]
    print(f'  t={tk["_t"]-t0:6.2f}s  src={tk.get("obs_horizon_source")!r:<24s}  '
          f's0={o.get("s0"):.2f}  sN={o.get("sN"):.2f}  midN_s={o.get("midN_s"):.2f}  '
          f's_seq={o.get("s_seq")}')

print('\n=== DONE ===')
