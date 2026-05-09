#!/usr/bin/env python3
"""Efficiency 분석 — ego_v / ref_v 의 obstacle 유무별 + side 별 분포.

사용자 큰 전제: 효율적 GB 합류. 0.80 ratio 가 진짜 80% utilization 인지,
또는 obstacle 만남 시 cap 인지.
"""
import json
import sys
import rosbag
from collections import defaultdict

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')
ticks = []
for topic, msg, t in bag.read_messages(topics=['/mpc_auto/debug/tick_json']):
    try:
        d = json.loads(msg.data); d['_t'] = t.to_sec(); ticks.append(d)
    except: pass
bag.close()
print(f'ticks: {len(ticks)}')

# Group by obs_in_horizon × side
groups = defaultdict(list)
for t in ticks:
    ev = t.get('ego',{}).get('v', 0)
    rv = t.get('ref',{}).get('v_mean', 0)
    if rv < 0.5: continue
    ratio = ev / rv
    oh = t.get('obs_in_horizon', False)
    side = t.get('side', '?')
    key = ('obs' if oh else 'no_obs', side)
    groups[key].append((ratio, ev, rv))

print('\n=== Efficiency by (obs_in_horizon, side) ===')
for key in sorted(groups.keys()):
    vals = groups[key]
    if not vals: continue
    ratios = [v[0] for v in vals]
    avg_r = sum(ratios)/len(ratios)
    avg_ev = sum(v[1] for v in vals)/len(vals)
    avg_rv = sum(v[2] for v in vals)/len(vals)
    print(f'  {key}: n={len(vals):>5}  ratio_avg={avg_r:.2f}  ego_v_avg={avg_ev:.2f}  ref_v_avg={avg_rv:.2f}')

# RACELINE (obs 없음) 시간만 — 진짜 efficiency
no_obs = [t for t in ticks if not t.get('obs_in_horizon')]
ratios = []
for t in no_obs:
    ev = t.get('ego',{}).get('v',0); rv = t.get('ref',{}).get('v_mean',0)
    if rv > 0.5:
        ratios.append(ev/rv)
if ratios:
    sa = sorted(ratios)
    print(f'\n=== RACELINE (obs 없음) efficiency ===')
    print(f'  n={len(ratios)} ratio: p10={sa[len(sa)//10]:.2f} p50={sa[len(sa)//2]:.2f} '
          f'p90={sa[9*len(sa)//10]:.2f} avg={sum(ratios)/len(ratios):.2f}')

# Vmax cap 영향 — ego_v 가 a1_vmax k_min 에 가까운지
print('\n=== Vmax cap 영향 ===')
near_cap = 0
no_obs_total = 0
for t in no_obs:
    ev = t.get('ego',{}).get('v',0)
    a1 = t.get('a1_vmax', {})
    cap_min = a1.get('cap_min', 0)
    if cap_min > 0.1:
        no_obs_total += 1
        if abs(ev - cap_min) < 0.3:
            near_cap += 1
print(f'  RACELINE 중 ego_v ≈ vmax_cap_min (within 0.3 m/s): {near_cap}/{no_obs_total}')

# ref_v 분포 — raceline 자체의 속도 평균
print('\n=== ref_v_mean 분포 ===')
rv_arr = [t.get('ref',{}).get('v_mean',0) for t in ticks]
rv_arr = sorted(rv_arr)
n = len(rv_arr)
if n > 0:
    print(f'  ref_v: p10={rv_arr[n//10]:.2f} p50={rv_arr[n//2]:.2f} '
          f'p90={rv_arr[9*n//10]:.2f} max={max(rv_arr):.2f}')

# ego_v 분포
ev_arr = sorted([t.get('ego',{}).get('v',0) for t in ticks])
n = len(ev_arr)
print(f'  ego_v: p10={ev_arr[n//10]:.2f} p50={ev_arr[n//2]:.2f} p90={ev_arr[9*n//10]:.2f} max={max(ev_arr):.2f}')
