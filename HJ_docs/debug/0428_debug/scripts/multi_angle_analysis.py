#!/usr/bin/env python3
"""다양한 관점 분석 — 사용자 호소 5대 증상 + Stage 2 효과 검증.

관점:
1. Cost balance (obs vs others, plan-별 분포)
2. Continuity guard 활성도 (L2 분포 — 부드러움 지표)
3. Plan transition 빈도 (안정성)
4. Smoothness (kappa, traj.kappa_rms)
5. Efficiency — ego_v vs ref_v 비율 (속도 유지)
6. Obstacle 만남 패턴 (side 결정 연속성)
"""
import json
import sys
from collections import Counter, defaultdict
import rosbag
import math

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')
ticks = []
for topic, msg, t in bag.read_messages(topics=['/mpc_auto/debug/tick_json']):
    try:
        d = json.loads(msg.data); d['_t'] = t.to_sec(); ticks.append(d)
    except: pass
bag.close()
print(f'ticks: {len(ticks)}')

# 1. Cost balance — obs / contour / wall / cont 평균
print('\n=== 1. Cost balance ===')
cost_keys = ['contour', 'reg', 'dd', 'dd_rate', 'smooth_a', 'progress', 'obs', 'bias', 'wall', 'cont', 'term', 'slack']
sums = defaultdict(float); n = 0
for t in ticks:
    c = t.get('cost', {})
    if not c: continue
    n += 1
    for k in cost_keys:
        sums[k] += c.get(k, 0)
if n > 0:
    print(f'  Average cost (n={n}):')
    for k in cost_keys:
        print(f'    {k:>10}: {sums[k]/n:.2f}')
    obs_avg = sums['obs']/n
    other_sum = sum(sums[k]/n for k in ['contour','dd','dd_rate','smooth_a','wall','cont','term'])
    print(f'  obs / sum_others ratio: {obs_avg/max(0.1,other_sum):.2f}x')

# 2. Continuity guard L2 분포
print('\n=== 2. Continuity guard L2 분포 ===')
L2s = [t.get('continuity_guard',{}).get('L2',0) for t in ticks]
L2s_sorted = sorted(L2s)
n = len(L2s_sorted)
if n > 0:
    print(f'  L2 dist: p50={L2s_sorted[n//2]:.3f} p95={L2s_sorted[int(n*0.95)]:.3f} '
          f'p99={L2s_sorted[int(n*0.99)]:.3f} max={max(L2s):.3f}')
    cg_applied = sum(1 for t in ticks if t.get('continuity_guard',{}).get('applied'))
    print(f'  applied: {cg_applied}/{len(ticks)} = {100*cg_applied/len(ticks):.1f}%')

# 3. Plan / side transition 빈도
print('\n=== 3. Plan / side transition ===')
side_seq = [t.get('side','?') for t in ticks]
flips = sum(1 for i in range(1,len(side_seq)) if side_seq[i]!=side_seq[i-1])
print(f'  side flips: {flips}')
side_runs = []
cur_side = None; cur_len = 0
for s in side_seq:
    if s != cur_side:
        if cur_side is not None: side_runs.append((cur_side, cur_len))
        cur_side = s; cur_len = 1
    else:
        cur_len += 1
if cur_side: side_runs.append((cur_side, cur_len))
# side run length 분포
run_by_side = defaultdict(list)
for s, l in side_runs:
    run_by_side[s].append(l)
for s, lens in run_by_side.items():
    if lens:
        print(f'  {s:>8}: {len(lens)} runs, avg_len={sum(lens)/len(lens):.1f}, max_len={max(lens)}')

# 4. Smoothness — kappa, jitter
print('\n=== 4. Smoothness ===')
kmax = [t.get('trajectory',{}).get('kappa_max',0) for t in ticks]
krms = [t.get('trajectory',{}).get('kappa_rms',0) for t in ticks]
jit  = [t.get('jitter_rms_m',0) for t in ticks]
def stats(arr, label):
    if not arr: return
    sa = sorted(arr)
    print(f'  {label}: p50={sa[len(sa)//2]:.3f} p95={sa[int(len(sa)*0.95)]:.3f} '
          f'p99={sa[int(len(sa)*0.99)]:.3f} max={max(arr):.3f}')
stats(kmax, 'kappa_max')
stats(krms, 'kappa_rms')
stats(jit,  'jitter_rms_m')

# 5. Efficiency — ego_v / ref_v
print('\n=== 5. Efficiency (ego_v / ref_v) ===')
eff = []
for t in ticks:
    ev = t.get('ego',{}).get('v', 0)
    rv = t.get('ref',{}).get('v_mean', 0)
    if rv > 0.5:
        eff.append(ev/rv)
if eff:
    sa = sorted(eff)
    print(f'  ratio dist: p50={sa[len(sa)//2]:.2f} p95={sa[int(len(sa)*0.95)]:.2f} '
          f'avg={sum(eff)/len(eff):.2f}')

# 6. Obstacle handling - side consistency in obs window
print('\n=== 6. Obstacle 만남 패턴 ===')
obs_ticks = [t for t in ticks if t.get('obs_in_horizon')]
print(f'  obs_in_horizon: {len(obs_ticks)}/{len(ticks)} = {100*len(obs_ticks)/max(1,len(ticks)):.1f}%')
if obs_ticks:
    obs_side = Counter([t.get('side','?') for t in obs_ticks])
    print(f'  obs window side dist: {dict(obs_side)}')
    # decider reasons
    obs_reason = Counter([t.get('side_scores',{}).get('reason','?') for t in obs_ticks])
    print(f'  reason dist: {dict(obs_reason)}')
    # plan distribution if logged (Phase 2-1+)
    obs_plan = Counter([t.get('plan_name', '?') for t in obs_ticks])  # tick_json 에 plan_name 추가됐다면
    if any(p != '?' for p in obs_plan):
        print(f'  plan dist: {dict(obs_plan)}')
    # trajectory ↔ obs overlap (proxy)
    overlap = 0
    for t in obs_ticks:
        traj = t.get('trajectory',{})
        obs = t.get('obstacles',[])
        n_min = traj.get('n_min',0); n_max = traj.get('n_max',0)
        for o in obs:
            on = o.get('n0',0); ow = 0.20
            if (n_min - 0.10) < (on + ow) and (n_max + 0.10) > (on - ow):
                overlap += 1; break
    print(f'  trajectory ↔ obs overlap: {overlap}/{len(obs_ticks)} = {100*overlap/max(1,len(obs_ticks)):.1f}%')

# 7. Status 분포 (HOLD_LAST / EMERGENCY 발생 시점)
print('\n=== 7. Status 분포 ===')
status_ctr = Counter([t.get('status','?') for t in ticks])
print(f'  {dict(status_ctr)}')
