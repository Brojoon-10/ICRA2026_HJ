#!/usr/bin/env python3
"""Comprehensive long-run bag summary.

Inputs everything we record in 0429 runs and produces:
  - Lap count + per-lap collision count
  - Plan transition frequency + dwell distribution
  - SM state TRAILING ↔ GB_TRACK toggle frequency
  - MPC publish rate + gap distribution
  - Tier (0/1/2/3) usage histogram
  - HOLD_LAST streak histogram
  - ego_v vs obs_vs in TRAILING ticks (does ego match opp speed?)
  - Local_wpnts lag distribution (ego_s vs wpnt0_s)
  - Per-collision deep diagnostic (1.0s lead-up trace)

Pass / Fail判定:
  - 충돌 0 ?
  - publish gap p99 < 50ms ?
  - plan transitions/min < 5 ?
  - SM state toggles/min < 5 ?
  - HOLD_LAST streak max < 100 (i.e. < 2s of NLP fail)?
"""
import sys, json, math
from collections import defaultdict
import rosbag

if len(sys.argv) < 2:
    print('usage: long_run_summary.py <bag>'); sys.exit(1)

bag_path = sys.argv[1]
TRACK = 85.6
DT = 0.05

print(f'=== {bag_path.split("/")[-1]} ===')

bag = rosbag.Bag(bag_path, 'r')

ego = []   # (t, s, n, vs)
obs = []   # (t, [(s, n, vs, vd, size, is_static)])
plans = [] # (t, plan, ipopt, status, mpc_mode, tier)
beh = []   # (t, state, n_loc, wpnt0_s)
sm = []    # (t, state)
mpc_pub = []  # (t, n_wpnts, s_first, ot_line)
status = []   # (t, status_str)
coll_edges = []
last_coll = False
status_msgs = 0

for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.twist.twist.linear.x))
    elif topic == '/tracking/obstacles':
        obs.append((ts, [(o.s_center, o.d_center, o.vs, o.vd,
                          o.size, o.is_static) for o in msg.obstacles]))
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data)
            plans.append((ts, d.get('plan'), d.get('ipopt_status'),
                          d.get('status'), d.get('mpc_mode'), d.get('tier')))
        except Exception:
            pass
    elif topic == '/behavior_strategy':
        n_loc = len(msg.local_wpnts) if msg.local_wpnts else 0
        s0 = float(msg.local_wpnts[0].s_m) if n_loc > 0 else float('nan')
        beh.append((ts, str(msg.state), n_loc, s0))
    elif topic == '/state_machine':
        sm.append((ts, str(msg.data)))
    elif topic == '/planner/mpc/wpnts':
        n = len(msg.wpnts)
        s0 = float(msg.wpnts[0].s_m) if n > 0 else float('nan')
        ot_line = getattr(msg, 'ot_line', '')
        mpc_pub.append((ts, n, s0, ot_line))
    elif topic == '/mpc_auto/status':
        status.append((ts, msg.data))
        status_msgs += 1
    elif topic == '/opponent_collision':
        if bool(msg.data) and not last_coll:
            coll_edges.append(ts)
        last_coll = bool(msg.data)

bag.close()

if not ego:
    print('NO EGO MSGS'); sys.exit(1)

t0, t1 = ego[0][0], ego[-1][0]
dur = t1 - t0
print(f'duration: {dur:.1f}s ({dur/60:.2f} min)')
print(f'  ego: {len(ego)}  obs: {len(obs)}  plans: {len(plans)}  beh: {len(beh)}  sm: {len(sm)}  mpc_pub: {len(mpc_pub)}  status: {status_msgs}  coll edges: {len(coll_edges)}')

def near(arr, t):
    if not arr: return None
    lo, hi = 0, len(arr)-1
    while lo < hi:
        mid = (lo+hi)//2
        if arr[mid][0] < t: lo = mid+1
        else: hi = mid
    return arr[lo]

# ---------- Lap detection ----------
laps = 0
last_s = ego[0][1]
for ts, s, n, v in ego:
    if last_s > 60 and s < 20:
        laps += 1
    last_s = s
print(f'laps: {laps}')

# ---------- Collision events (clusters) ----------
events = []
for c in coll_edges:
    if events and c - events[-1] < 0.5: continue
    events.append(c)
print(f'\n=== COLLISIONS: {len(events)} events  /  {len(coll_edges)} raw edges ===')
for ev in events:
    e = near(ego, ev)
    o = near(obs, ev)
    p = near(plans, ev)
    s = near(sm, ev)
    bs0 = near(beh, ev)
    if e and o and o[1]:
        best = min(o[1], key=lambda x: ((x[0] - e[1]) % TRACK))
        ds = (best[0] - e[1]) % TRACK
        if ds > TRACK/2: ds -= TRACK
        plan = p[1] if p else '?'
        sm_state = s[1] if s else '?'
        print(f'  t={ev-t0:6.2f}s  plan={plan:<10}  sm={sm_state:<10}  '
              f'ego_n={e[2]:+.3f}  obs_n={best[1]:+.3f}  ds={ds:+.2f}m  '
              f'ego_v={e[3]:.2f}  obs_vs={best[2]:.2f}')

# ---------- Plan transitions ----------
transitions = []
last_p = None
for ts, p, *_ in plans:
    if p and p != last_p:
        transitions.append(ts - t0)
        last_p = p
print(f'\nplan transitions: {len(transitions)} ({len(transitions)/(dur/60):.2f}/min)')

# ---------- SM state transitions ----------
sm_trans = []
last_s = None
for ts, st in sm:
    if st != last_s:
        sm_trans.append(ts - t0)
        last_s = st
print(f'SM state transitions: {len(sm_trans)} ({len(sm_trans)/(dur/60):.2f}/min)')

# ---------- MPC publish rate + gaps ----------
gaps = [(mpc_pub[i][0] - mpc_pub[i-1][0]) for i in range(1, len(mpc_pub))]
if gaps:
    g = sorted(gaps)
    print(f'\nMPC publish: {len(mpc_pub)} msgs  ({len(mpc_pub)/dur:.1f} Hz avg)')
    print(f'  gap p50={g[len(g)//2]*1000:.1f}ms  p95={g[int(len(g)*0.95)]*1000:.1f}ms  p99={g[int(len(g)*0.99)]*1000:.1f}ms  max={g[-1]*1000:.1f}ms')

# ---------- Tier histogram ----------
tier_hist = defaultdict(int)
status_hist = defaultdict(int)
for _, _, _, st, _, ti in plans:
    tier_hist[ti] += 1
    status_hist[st] += 1
print('\ntier histogram:')
for k in sorted(tier_hist.keys(), key=lambda x: (x is None, x)):
    print(f'  tier={k}: {tier_hist[k]}  ({100*tier_hist[k]/max(len(plans),1):.1f}%)')
print('\nstatus histogram:')
for k, v in sorted(status_hist.items(), key=lambda kv: -kv[1])[:8]:
    print(f'  {k}: {v}  ({100*v/max(len(plans),1):.1f}%)')

# ---------- HOLD_LAST streak ----------
ms = [int(s.split('streak=')[1].split()[0])
      for _, s in status if 'HOLD_LAST streak=' in s]
if ms:
    ms.sort()
    print(f'\nHOLD_LAST streak: count={len(ms)}  p50={ms[len(ms)//2]}  p95={ms[int(len(ms)*0.95)]}  max={ms[-1]}')

# ---------- TRAILING ego_v vs obs_vs ----------
trail_egos, trail_obss = [], []
for ts, st, n_loc, ws in beh:
    if st != 'TRAILING': continue
    e = near(ego, ts)
    o = near(obs, ts)
    if e and o and o[1]:
        best = min(o[1], key=lambda x: ((x[0] - e[1]) % TRACK))
        trail_egos.append(e[3])
        trail_obss.append(best[2])
if trail_egos:
    diffs = [a-b for a, b in zip(trail_egos, trail_obss)]
    diffs.sort()
    print(f'\nTRAILING tick: {len(trail_egos)} ({100*len(trail_egos)/max(len(beh),1):.1f}% of beh)')
    print(f'  ego_v - obs_vs: p50={diffs[len(diffs)//2]:+.2f}  p95={diffs[int(len(diffs)*0.95)]:+.2f}  max={max(diffs):+.2f}')

# ---------- Local wpnt lag ----------
lags = []
for ts, st, n_loc, ws in beh:
    if ws != ws: continue
    e = near(ego, ts)
    if e:
        diff = (ws - e[1]) % TRACK
        if diff > TRACK/2: diff -= TRACK
        lags.append(diff)
if lags:
    lags.sort()
    big = sum(1 for l in lags if abs(l) > 0.5)
    print(f'\nlocal_wpnts lag (ws - ego_s):')
    print(f'  p50={lags[len(lags)//2]:+.2f}  p95={lags[int(len(lags)*0.95)]:+.2f}  p99={lags[int(len(lags)*0.99)]:+.2f}')
    print(f'  |lag|>0.5m: {big} ({100*big/len(lags):.1f}%)')

# ---------- Pass / Fail判定 ----------
print('\n=== PASS / FAIL JUDGMENT ===')
gates = []
gates.append(('충돌 0',                       len(events) == 0,                       f'{len(events)} events'))
if gaps:
    gates.append(('publish gap p99 < 50ms',   g[int(len(g)*0.99)] * 1000 < 50,        f'p99={g[int(len(g)*0.99)]*1000:.1f}ms'))
gates.append(('plan trans/min < 5',           len(transitions)/(dur/60) < 5,          f'{len(transitions)/(dur/60):.2f}'))
gates.append(('SM trans/min < 5',             len(sm_trans)/(dur/60) < 5,             f'{len(sm_trans)/(dur/60):.2f}'))
if ms:
    gates.append(('HOLD_LAST streak max < 100', ms[-1] < 100,                        f'max={ms[-1]}'))
for name, ok, val in gates:
    sym = '✓' if ok else '✗'
    print(f'  [{sym}] {name:<30}  {val}')
overall = all(ok for _, ok, _ in gates)
print(f'\nOVERALL: {"PASS" if overall else "FAIL"}')
