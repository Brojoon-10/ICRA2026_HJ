#!/usr/bin/env python3
"""Continuity guard L2 점프 원인 분석.

큰 L2 (>3m) 발생 시점의:
  - 직전 vs 현재 trajectory 의 차이 (n, x, y)
  - mpc_mode transition (WITH_OBS↔NO_OBS)
  - plan transition (side 변동)
  - alpha_ramp 상태
  - obs_in_horizon 변동
"""
import json
import sys
from collections import Counter
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')
ticks = []
for topic, msg, t in bag.read_messages(topics=['/mpc_auto/debug/tick_json']):
    try:
        d = json.loads(msg.data); d['_t'] = t.to_sec(); ticks.append(d)
    except: pass
bag.close()
print(f'ticks: {len(ticks)}')

# Find ticks with high L2
big = [(i, t) for i, t in enumerate(ticks)
       if t.get('continuity_guard',{}).get('L2',0) > 3.0]
print(f'\nL2 > 3m: {len(big)} ticks')
for i, t in big[:10]:
    L2 = t.get('continuity_guard',{}).get('L2',0)
    prev = ticks[i-1] if i>0 else None
    print(f'\n--- tick {i} t={t["_t"]:.2f}, L2={L2:.3f}m ---')
    print(f'  ego_s={t.get("ego",{}).get("s"):.2f} ego_n={t.get("ego",{}).get("n"):+.3f} ego_v={t.get("ego",{}).get("v"):.2f}')
    print(f'  side={t.get("side")} mode={t.get("mpc_mode")} alpha_ramp={t.get("alpha_ramp",0):.2f}')
    print(f'  obs_in_horizon={t.get("obs_in_horizon")} mode_dwell={t.get("mode_dwell")}')
    if prev:
        print(f'  prev side={prev.get("side")} mode={prev.get("mpc_mode")} alpha_ramp={prev.get("alpha_ramp",0):.2f}')
        if prev.get('side') != t.get('side'):
            print(f'  ## SIDE FLIP')
        if prev.get('mpc_mode') != t.get('mpc_mode'):
            print(f'  ## MODE FLIP')
    traj = t.get('trajectory',{})
    print(f'  traj: n_min={traj.get("n_min",0):+.3f} n_max={traj.get("n_max",0):+.3f} kappa_max={traj.get("kappa_max",0):.2f}')

# Side flip + mode flip 빈도 분석
print('\n=== L2 > 3m ticks 의 transition 동시 발생 ===')
flip_at_L2 = 0; mode_flip_at_L2 = 0
for i, t in big:
    if i == 0: continue
    p = ticks[i-1]
    if p.get('side') != t.get('side'): flip_at_L2 += 1
    if p.get('mpc_mode') != t.get('mpc_mode'): mode_flip_at_L2 += 1
print(f'  side flip 동시 발생: {flip_at_L2}/{len(big)}')
print(f'  mode flip 동시 발생: {mode_flip_at_L2}/{len(big)}')

# alpha_ramp 분포 in big-L2 ticks
ar = [t.get('alpha_ramp', 0) for _, t in big]
if ar:
    print(f'  alpha_ramp dist in big-L2: max={max(ar):.2f} min={min(ar):.2f} avg={sum(ar)/len(ar):.2f}')

# obs_in_horizon 분포 in big-L2 ticks
oh = Counter([t.get('obs_in_horizon') for _, t in big])
print(f'  obs_in_horizon in big-L2: {dict(oh)}')

# n_obs_used 분포
nobs = Counter([t.get('n_obs_used', -1) for _, t in big])
print(f'  n_obs_used in big-L2: {dict(nobs)}')
