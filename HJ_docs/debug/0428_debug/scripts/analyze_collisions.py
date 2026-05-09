#!/usr/bin/env python3
"""Stage 1 통합 bag 의 충돌 53건 원인 분석.

각 collision_marker 발행 시점의:
- ego 상태 (s, n, v, psi)
- obstacle 위치 (s, n, v_s)
- side decision + ego_n 모순 여부
- mpc_mode, alpha_ramp
- trajectory.n_min/n_max (obstacle 영역과 overlap?)
"""
import json
import sys
import math
from collections import Counter
import rosbag

bag_path = sys.argv[1] if len(sys.argv) > 1 else None
if bag_path is None:
    print('usage: analyze_collisions.py <bag>')
    sys.exit(1)

print(f'Loading {bag_path} ...')
bag = rosbag.Bag(bag_path, 'r')

ticks = []
collisions = []  # (t, ego_x, ego_y) — collision_marker 의 첫 시점만
for topic, msg, t in bag.read_messages():
    if topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data); d['_t'] = t.to_sec(); ticks.append(d)
        except:
            pass
    elif topic == '/collision_marker':
        collisions.append(t.to_sec())
bag.close()

print(f'\nticks: {len(ticks)}, collision msgs: {len(collisions)}')

# Group collisions into events (consecutive < 0.5s = same event)
events = []
cur = []
last_t = None
for ct in collisions:
    if last_t is None or (ct - last_t) > 0.5:
        if cur:
            events.append((cur[0], cur[-1], len(cur)))
        cur = [ct]
    else:
        cur.append(ct)
    last_t = ct
if cur:
    events.append((cur[0], cur[-1], len(cur)))
print(f'\nDistinct collision events: {len(events)}')
for i, (ts, te, n) in enumerate(events):
    print(f"  #{i}: t=[{ts:.2f}-{te:.2f}] dur={te-ts:.2f}s msgs={n}")

# For each collision event, find tick_json closest to event start
print('\n========== Per-event analysis (each event start) ==========')
def find_tick(t_target, ticks):
    best = None; best_d = 1e9
    for tk in ticks:
        d = abs(tk['_t'] - t_target)
        if d < best_d:
            best_d = d; best = tk
    return best

for i, (ts, te, n) in enumerate(events):
    tk = find_tick(ts, ticks)
    if tk is None:
        continue
    ego = tk.get('ego', {})
    traj = tk.get('trajectory', {})
    obs = tk.get('obstacles', [])
    obs_str = ' | '.join([f"s={o.get('s0',0):.2f},n={o.get('n0',0):+.3f},sN={o.get('sN',0):.2f}" for o in obs])
    ss = tk.get('side_scores', {})
    print(f"\n#{i} t={ts:.2f} (n={n} msgs)")
    print(f"  ego: s={ego.get('s',0):.2f} n={ego.get('n',0):+.3f} v={ego.get('v',0):.2f} psi={ego.get('psi',0):+.3f}")
    print(f"  side={tk.get('side','?')} mpc_mode={tk.get('mpc_mode','?')} alpha_ramp={tk.get('alpha_ramp',0):.2f}")
    print(f"  reason={ss.get('reason','?')} d_free_L={ss.get('d_free_L',0):+.2f} d_free_R={ss.get('d_free_R',0):+.2f}")
    print(f"  trajectory: n_min={traj.get('n_min',0):+.3f} n_max={traj.get('n_max',0):+.3f} kappa={traj.get('kappa_max',0):.2f}")
    print(f"  obstacles: {obs_str}")
    print(f"  ipopt={tk.get('ipopt_status','?')[:14]} pass={tk.get('solver_pass')} tier={tk.get('tier')}")

# Side dist when obs_in_horizon=True
oh_ticks = [t for t in ticks if t.get('obs_in_horizon')]
print(f'\n========== obs_in_horizon=True analysis (n={len(oh_ticks)}) ==========')
oh_side = Counter([t.get('side','?') for t in oh_ticks])
oh_reason = Counter([t.get('side_scores',{}).get('reason','?') for t in oh_ticks])
print(f'  side dist: {dict(oh_side)}')
print(f'  reason dist: {dict(oh_reason)}')

# Trajectory ↔ obstacle overlap when obs_in_horizon (proxy: traj n range overlaps obs.n±0.20)
overlaps = 0
for t in oh_ticks:
    traj = t.get('trajectory', {})
    obs = t.get('obstacles', [])
    n_min = traj.get('n_min', 0); n_max = traj.get('n_max', 0)
    for o in obs:
        on = o.get('n0', 0); ow = 0.20
        if (n_min - 0.10) < (on + ow) and (n_max + 0.10) > (on - ow):
            overlaps += 1
            break
print(f'  trajectory ↔ obstacle overlap (obs_in_horizon=True): {overlaps}/{len(oh_ticks)} = {100*overlaps/max(1,len(oh_ticks)):.1f}%')

# side ↔ ego_n contradiction count
contradict = 0
for t in oh_ticks:
    en = t.get('ego', {}).get('n', None)
    side = t.get('side','clear')
    if en is None: continue
    if side == 'left' and en < -0.20: contradict += 1
    if side == 'right' and en > +0.20: contradict += 1
print(f'  side ↔ ego_n contradiction (|ego_n|>0.2 against side): {contradict}/{len(oh_ticks)}')

# cost balance check
obs_dom = 0
for t in oh_ticks:
    c = t.get('cost', {})
    obs_c = c.get('obs', 0)
    others = sum([c.get(k, 0) for k in ['contour','dd','dd_rate','smooth_a','wall','cont','term']])
    if others > 0 and obs_c > 5*others:
        obs_dom += 1
print(f'  cost.obs > 5x sum_others: {obs_dom}/{len(oh_ticks)} = {100*obs_dom/max(1,len(oh_ticks)):.1f}%')
