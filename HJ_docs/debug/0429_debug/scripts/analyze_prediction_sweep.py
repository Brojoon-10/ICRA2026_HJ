#!/usr/bin/env python3
"""Check whether GP prediction's d_center sweeps within a single sequence
(across timesteps in the same publish), and whether infeasibility events
correlate with high lateral sweep amplitude.

If prediction's d_center varies, e.g., 0.10 → -0.20 → +0.25 across the
timesteps of a single publish, then the NLP sees an obstacle that "sweeps"
the corridor laterally — there may be no feasible ego trajectory at all.
That would be a true infeasibility (not a side-toggle artifact).
"""
import sys, json
import numpy as np
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

# /opponent_prediction/obstacles publishes — collect each publish's full
# sequence of (timestep_idx, d_center) for the matched opponent.
pred_publishes = []   # list of (t_publish, [(idx, s, d), ...])
infeas_times = []     # times of MPC tick_json infeasibility
ticks = []            # tick_json full

for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if topic == '/opponent_prediction/obstacles':
        seq = []
        for o in msg.obstacles:
            seq.append((int(getattr(o, 'id', 0) or 0),
                        float(o.s_center),
                        float(o.d_center)))
        # sort by timestep idx
        seq.sort(key=lambda r: r[0])
        pred_publishes.append((ts, seq))
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data)
            d['_t'] = ts
            ticks.append(d)
            if d.get('ipopt_status') == 'Infeasible_Problem_Detected':
                infeas_times.append(ts)
        except Exception:
            pass
bag.close()

t0 = pred_publishes[0][0] if pred_publishes else 0
print(f'=== {bag_path.split("/")[-1]} ===')
print(f'  pred_publishes={len(pred_publishes)}  ticks={len(ticks)}  infeas={len(infeas_times)}')

# ---- Per-publish d sweep ----
print('\n=== prediction sequence d sweep (within ONE publish) ===')
sweeps = []
for ts, seq in pred_publishes:
    if len(seq) < 2:
        continue
    ds = [d for _, _, d in seq]
    d_min, d_max = min(ds), max(ds)
    sweep = d_max - d_min
    d_first = ds[0]
    sweeps.append((ts, sweep, d_first, d_min, d_max, len(seq)))

if sweeps:
    arr = np.array([s[1] for s in sweeps])
    print(f'  sweep amplitude (m): mean={arr.mean():.3f}  p50={np.median(arr):.3f}  '
          f'p95={np.percentile(arr,95):.3f}  p99={np.percentile(arr,99):.3f}  max={arr.max():.3f}')
    print(f'  sweeps > 0.10m:  {(arr > 0.10).sum()}/{len(arr)}  ({100*(arr>0.10).mean():.1f}%)')
    print(f'  sweeps > 0.20m:  {(arr > 0.20).sum()}/{len(arr)}  ({100*(arr>0.20).mean():.1f}%)')
    print(f'  sweeps > 0.30m:  {(arr > 0.30).sum()}/{len(arr)}  ({100*(arr>0.30).mean():.1f}%)')

# ---- d_center across consecutive publishes (same timestep idx 0) ----
print('\n=== d_center timestep 0 across publishes (tracking-like raw input) ===')
d0_seq = [s[2] for s in sweeps]
if len(d0_seq) >= 2:
    diffs = [abs(d0_seq[i] - d0_seq[i-1]) for i in range(1, len(d0_seq))]
    arr = np.array(diffs)
    print(f'  per-publish |d0[t] - d0[t-1]| (m): mean={arr.mean():.4f}  p95={np.percentile(arr,95):.4f}  max={arr.max():.4f}')
    print(f'  → 작으면 (~0.001m) tracking smooth, GP sequence sweep 만 문제.')
    print(f'  → 크면 GP timestep 0 자체가 noisy.')

# ---- 무엇이 inside-publish sweep 의 정체? ----
print('\n=== sample publishes (max sweep) ===')
sweeps_sorted = sorted(sweeps, key=lambda r: -r[1])[:5]
for ts, sweep, df, dn, dx, L in sweeps_sorted:
    print(f'  t={ts-t0:6.2f}s  sweep={sweep:.3f}m  d_first={df:+.3f}  range=[{dn:+.3f}, {dx:+.3f}]  N={L}')

# ---- Infeas correlation ----
print('\n=== Infeasibility 시점 주변 sweep ===')
def find_nearest_pub(t_target):
    if not pred_publishes: return None
    diffs = [(abs(p[0] - t_target), p) for p in pred_publishes]
    return min(diffs, key=lambda x: x[0])[1]

if infeas_times:
    print(f'  {len(infeas_times)} infeasibility events.')
    sweeps_at_infeas = []
    for it in infeas_times:
        pub = find_nearest_pub(it)
        if pub is None:
            continue
        ds = [d for _, _, d in pub[1]]
        if len(ds) < 2:
            continue
        sweep = max(ds) - min(ds)
        sweeps_at_infeas.append(sweep)
    if sweeps_at_infeas:
        arr = np.array(sweeps_at_infeas)
        print(f'  sweep at infeas times: mean={arr.mean():.3f}  p50={np.median(arr):.3f}  max={arr.max():.3f}')

    overall = np.array([s[1] for s in sweeps])
    print(f'  overall  sweep: mean={overall.mean():.3f}  p50={np.median(overall):.3f}  max={overall.max():.3f}')
    if sweeps_at_infeas:
        ratio = arr.mean() / max(overall.mean(), 1e-3)
        print(f'  ratio infeas / overall: {ratio:.2f}x  (>1.5x 면 infeas 가 sweep 큰 시점에 집중)')

print('\n=== DONE ===')
