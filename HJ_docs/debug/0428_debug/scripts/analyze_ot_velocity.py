#!/usr/bin/env python3
"""OT 전후 ego_v 회복 + plan transition 시 부드러움 분석."""
import json
import sys
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')
ticks = []
for topic, msg, t in bag.read_messages(topics=['/mpc_auto/debug/tick_json']):
    try:
        d = json.loads(msg.data); d['_t'] = t.to_sec(); ticks.append(d)
    except: pass
bag.close()

# Plan transition 시점들
transitions = []
for i in range(1, len(ticks)):
    p = ticks[i-1].get('side','?'); c = ticks[i].get('side','?')
    if p != c:
        transitions.append((i, ticks[i]['_t'], p, c))
print(f'\n=== Plan transitions: {len(transitions)} ===')
for idx, t, p, c in transitions[:20]:
    print(f'  t={t:.2f} {p} → {c}')

# Plan 별 ego_v / ref_v 평균 + max|Δn| (transition 시 jitter)
from collections import defaultdict
plan_v = defaultdict(list)
for tk in ticks:
    side = tk.get('side','?')
    ev = tk.get('ego',{}).get('v',0)
    rv = tk.get('ref',{}).get('v_mean',0)
    if rv > 0.5: plan_v[side].append((ev, rv, ev/rv))
print(f'\n=== Per-plan ego_v / ref_v ===')
for p, vals in plan_v.items():
    if not vals: continue
    avg_ev = sum(v[0] for v in vals)/len(vals)
    avg_rv = sum(v[1] for v in vals)/len(vals)
    avg_ratio = sum(v[2] for v in vals)/len(vals)
    print(f'  {p:>8}: n={len(vals):>5}  ego_v avg={avg_ev:.2f}  ref_v avg={avg_rv:.2f}  ratio={avg_ratio:.2f}')

# OT 시 ego_v 변동: LEFT/RIGHT_PASS 후 clear 시점 5s window 의 ego_v 시계열
print(f'\n=== Post LEFT/RIGHT_PASS ego_v 회복 (3s window) ===')
for idx, t_trans, p, c in transitions:
    if p in ('left','right') and c == 'clear':
        print(f'\n  Transition #{idx} {p}→clear at t={t_trans:.2f}')
        # 5s window 의 ego_v
        window = [(tk['_t'], tk.get('ego',{}).get('v',0), tk.get('ref',{}).get('v_mean',0))
                  for tk in ticks if t_trans <= tk['_t'] < t_trans + 3.0]
        if not window: continue
        # samples
        samples = window[::max(1, len(window)//8)][:8]
        for st, ev, rv in samples:
            r = ev/rv if rv>0.5 else 0
            print(f'    t={st-t_trans:+.2f}s  ego_v={ev:.2f}  ref_v={rv:.2f}  ratio={r:.2f}')

# Side flip 빈도 (전체)
from collections import Counter
sides = Counter([tk.get('side','?') for tk in ticks])
print(f'\n=== Side dist: {dict(sides)} ===')
