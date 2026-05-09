#!/usr/bin/env python3
"""Where do infeas events happen — ego.s + obs sequence + ipopt status."""
import sys, json
import rosbag

bag = rosbag.Bag(sys.argv[1], 'r')
ticks = []
for topic, msg, t in bag.read_messages(topics=['/mpc_auto/debug/tick_json']):
    try:
        d = json.loads(msg.data)
        d['_t'] = t.to_sec()
        ticks.append(d)
    except Exception:
        pass
bag.close()

t0 = ticks[0]['_t']
fails = [d for d in ticks if d.get('status') == 'CONVERGENCE_QUINTIC']
print(f'total ticks: {len(ticks)}')
print(f'infeas events: {len(fails)}')
print()
print('각 fail event 의 ego.s/n, kappa, corridor, obs:')
print(f'{"t":>7s} {"ego.s":>7s} {"ego.n":>7s} {"v":>5s} {"side":>6s} {"plan":>11s} '
      f'{"kappa":>5s} {"corw":>4s} {"obs.s0":>7s} {"obs.n0":>7s} {"obs.n_seq":<55s} {"ipopt":>30s}')
for ev in fails:
    rel = ev['_t'] - t0
    e = ev.get('ego', {})
    traj = ev.get('trajectory', {})
    ref = ev.get('ref', {})
    obs_list = ev.get('obstacles') or []
    obs = obs_list[0] if obs_list else {}
    n_seq = obs.get('n_seq', [])
    n_seq_str = '[' + ','.join(f'{x:+.2f}' for x in n_seq) + ']'
    ipopt = ev.get('ipopt_status', '')
    print(f'{rel:7.2f} {e.get("s",0):7.2f} {e.get("n",0):+7.3f} {e.get("v",0):5.2f} '
          f'{str(ev.get("side"))[:6]:>6s} {str(ev.get("plan"))[:11]:>11s} '
          f'{traj.get("kappa_max",0):5.2f} {ref.get("corridor_width_mean",0):4.2f} '
          f'{obs.get("s0",0):7.2f} {obs.get("n0",0):+7.3f} {n_seq_str:<55s} {ipopt[:30]:>30s}')

# 같은 ego.s 위치에서 반복?
print('\n=== ego.s clusters of fails ===')
from collections import Counter
s_cluster = Counter()
for ev in fails:
    s = ev.get('ego', {}).get('s', 0)
    bucket = round(s, 0)
    s_cluster[bucket] += 1
for k, v in sorted(s_cluster.items()):
    print(f'  ego.s ~{k:.0f}: {v} fails')

# 코너 곡률 위치 분포
print('\n=== kappa_max distribution at fails ===')
ks = sorted(ev.get('trajectory',{}).get('kappa_max',0) for ev in fails)
if ks:
    print(f'  min={ks[0]:.2f}  median={ks[len(ks)//2]:.2f}  max={ks[-1]:.2f}')

# corridor width 분포
print('\n=== corridor_width at fails ===')
cw = sorted(ev.get('ref',{}).get('corridor_width_mean',0) for ev in fails)
if cw:
    print(f'  min={cw[0]:.2f}  median={cw[len(cw)//2]:.2f}  max={cw[-1]:.2f}')
