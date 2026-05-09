#!/usr/bin/env python3
"""User-supplied bag deep analysis.
Focus:
  - Collision events vs ego_v / plan / vx_mps / opponent.vs around the time
  - Plan transition sequence
  - TRAILING state freq and behavior (does ego decelerate to obs.vs?)
  - state_machine/state value sequence
  - obstacles_in_interest count (proxy = num obstacles in /tracking/obstacles)
"""
import sys, json
from collections import defaultdict
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

ego = []      # (t, s, n, vs)
obs = []      # (t, s, n, vs)
plans = []    # (t, plan_name, jitter, mpc_mode_echo, ipopt_status, solve_ms)
sm_state = [] # (t, state_str)
behavior = [] # (t, state_int, n_trail_targets, n_local_wpnts, target_v_first)
coll = []     # collision rising edges
last_coll = False

for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x, msg.pose.pose.position.y,
                    msg.twist.twist.linear.x))
    elif topic == '/tracking/obstacles':
        for o in msg.obstacles:
            obs.append((ts, o.s_center, o.d_center, o.vs))
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data)
            plans.append((ts, d.get('plan'),
                          float(d.get('jitter_rms_m', 0.0)),
                          d.get('mpc_mode'),
                          d.get('ipopt_status'),
                          float(d.get('solve_ms', 0.0))))
        except Exception:
            pass
    elif topic == '/state_machine':
        sm_state.append((ts, msg.data))
    elif topic == '/behavior_strategy':
        n_loc = len(msg.local_wpnts) if msg.local_wpnts else 0
        n_trail = len(msg.trailing_targets) if msg.trailing_targets else 0
        v0 = float(msg.local_wpnts[0].vx_mps) if n_loc > 0 else float('nan')
        behavior.append((ts, str(msg.state), n_trail, n_loc, v0))
    elif topic == '/opponent_collision':
        if bool(msg.data) and not last_coll:
            coll.append(ts)
        last_coll = bool(msg.data)

bag.close()

t0 = ego[0][0]
t1 = ego[-1][0]
dur = t1 - t0
print('duration: %.2fs  ego: %d  obs: %d  plans: %d  sm: %d  beh: %d  coll: %d' %
      (dur, len(ego), len(obs), len(plans), len(sm_state), len(behavior), len(coll)))

# ---------- Plan transitions ----------
trans = []
last_p = None
for ts, p, *_ in plans:
    if p and p != last_p:
        trans.append((ts - t0, p))
        last_p = p
print('\n=== Plan transitions: %d ===' % len(trans))
for tr, p in trans[:50]:
    print('  t=%6.2fs -> %s' % (tr, p))
if len(trans) > 50:
    print('  ... (%d more)' % (len(trans) - 50))

# ---------- SM state transitions ----------
sm_trans = []
last_s = None
for ts, s in sm_state:
    if s != last_s:
        sm_trans.append((ts - t0, s))
        last_s = s
print('\n=== SM state transitions: %d ===' % len(sm_trans))
for tr, s in sm_trans[:40]:
    print('  t=%6.2fs -> %s' % (tr, s))

# ---------- TRAILING behavior ----------
# For each tick where SM state is TRAILING, compare ego_v to obs.vs (closest by time).
def near(arr, t):
    if not arr: return None
    lo, hi = 0, len(arr) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if arr[mid][0] < t: lo = mid + 1
        else: hi = mid
    return arr[lo]

print('\n=== TRAILING tick samples (vx_mps from behavior, ego_v from odom, obs_vs from tracking) ===')
print('%6s %4s %8s %8s %8s %8s %s' % ('t_rel', 'st', 'ego_v', 'cmd_v0', 'obs_vs', 'd_obs_v', 'plan'))
trailing_ticks = 0
for ts, st_int, n_trail, n_loc, v0 in behavior:
    if st_int != 'TRAILING':
        continue
    e = near(ego, ts)
    o = near(obs, ts)
    p = near(plans, ts)
    if e is None: continue
    ego_v = e[3]
    obs_vs = o[3] if o is not None else float('nan')
    plan_name = p[1] if p else '?'
    delta = ego_v - obs_vs if obs_vs == obs_vs else float('nan')
    if trailing_ticks < 30:
        print('  %6.2f %-9s %8.2f %8.2f %8.2f %+8.2f %s' %
              (ts - t0, st_int, ego_v, v0, obs_vs, delta, plan_name))
    trailing_ticks += 1
print('  total TRAILING ticks: %d' % trailing_ticks)

# ---------- Collision detail ----------
print('\n=== Collisions (rising edges, clustered to single event if <0.5s) ===')
clusters = []
for ct in coll:
    if clusters and ct - clusters[-1] < 0.5:
        continue
    clusters.append(ct)
print('  events: %d (raw edges %d)' % (len(clusters), len(coll)))
print('%6s %-10s %8s %8s %8s %8s %8s %s' %
      ('t_rel', 'plan', 'ego_n', 'obs_n', 'ds', 'ego_v', 'obs_vs', 'sm_state'))
for ct in clusters:
    e = near(ego, ct)
    o = near(obs, ct)
    p = near(plans, ct)
    s = near(sm_state, ct)
    if not e or not o: continue
    ds = (o[1] - e[1])
    while ds > 42.8: ds -= 85.6
    while ds < -42.8: ds += 85.6
    pname = p[1] if p else '?'
    sm = s[1] if s else '?'
    print('  %6.2f %-10s %+8.3f %+8.3f %+8.2f %8.2f %8.2f %s' %
          (ct - t0, pname, e[2], o[2], ds, e[3], o[3], sm))

# ---------- ego_v / cmd_v / obs_vs aggregate ----------
print('\n=== Aggregate ===')
print('  TRAILING ticks fraction: %.1f%%' % (100.0*trailing_ticks/max(len(behavior),1)))
trailing_egos = []
for ts, st_int, n_trail, n_loc, v0 in behavior:
    if st_int != 'TRAILING': continue
    e = near(ego, ts)
    if e: trailing_egos.append(e[3])
if trailing_egos:
    s = sorted(trailing_egos)
    print('  TRAILING ego_v: min=%.2f med=%.2f max=%.2f' %
          (s[0], s[len(s)//2], s[-1]))
trailing_cmds = [v0 for _, st, _, _, v0 in behavior if st == 'TRAILING' and v0 == v0]
if trailing_cmds:
    s = sorted(trailing_cmds)
    print('  TRAILING cmd_v0: min=%.2f med=%.2f max=%.2f' %
          (s[0], s[len(s)//2], s[-1]))
