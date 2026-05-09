#!/usr/bin/env python3
"""Analyze the GB <-> recovery toggle right after spawn.

User report: spawn 직후 recovery 경로 계산되는데 잠깐 GB <-> recovery
왔다갔다하다 recovery로 안착. 원인 찾기.

Walks the FIRST 10s of the bag tick-by-tick, paired across:
  /state_machine_mpc/debug
  /mpc_auto/debug/tick_json
  /mpc_auto/status
  /planner/mpc/wpnts
  /local_waypoints
  /behavior_strategy
  /car_state/odom_frenet
"""
import sys, json, math
import rosbag
from collections import Counter

if len(sys.argv) < 2:
    print('usage: analyze_spawn_toggle.py <bag>'); sys.exit(1)

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

mpc_dbg = []
mpc_tick = []
mpc_status = []
mpc_pub = []
local_wpnts = []
beh = []
ego = []
sm = []

t0 = None
for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if t0 is None: t0 = ts
    if topic == '/state_machine_mpc/debug':
        try:
            d = json.loads(msg.data); mpc_dbg.append((ts, d))
        except Exception: pass
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            mpc_tick.append((ts, json.loads(msg.data)))
        except Exception: pass
    elif topic == '/mpc_auto/status':
        mpc_status.append((ts, msg.data))
    elif topic == '/planner/mpc/wpnts':
        n = len(msg.wpnts)
        s0 = float(msg.wpnts[0].s_m) if n>0 else float('nan')
        ot = getattr(msg, 'ot_line', '')
        mpc_pub.append((ts, ot, n, s0))
    elif topic == '/local_waypoints':
        n = len(msg.wpnts)
        s0 = float(msg.wpnts[0].s_m) if n>0 else float('nan')
        vx0 = float(msg.wpnts[0].vx_mps) if n>0 else float('nan')
        local_wpnts.append((ts, n, s0, vx0))
    elif topic == '/behavior_strategy':
        n = len(msg.local_wpnts) if msg.local_wpnts else 0
        s0 = float(msg.local_wpnts[0].s_m) if n>0 else float('nan')
        beh.append((ts, str(msg.state), n, s0))
    elif topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x, msg.twist.twist.linear.x))
    elif topic == '/state_machine':
        sm.append((ts, str(msg.data)))
bag.close()

dur = (ego[-1][0] - ego[0][0]) if ego else 0
print(f'=== {bag_path.split("/")[-1]}  dur={dur:.1f}s ===')
print(f'  mpc_dbg={len(mpc_dbg)}  tick={len(mpc_tick)}  pub={len(mpc_pub)}  '
      f'local={len(local_wpnts)}  beh={len(beh)}  sm={len(sm)}')

# --- pretty print first ~10s of MPC SM debug (path_source decisions) ---
print('\n=== state_machine_mpc/debug timeline (first 10s) ===')
print(f'{"t":>6s}  {"path_src":<8s}  {"sm_state":<10s}  {"mpc_mode":<10s}  '
      f'{"ot_age":>6s}  {"rc_age":>6s}  {"n_wp":>4s}  {"trail":>5s}')
for ts, d in mpc_dbg:
    rel = ts - t0
    if rel > 10.0: break
    ps = str(d.get('path_source'))
    ss = str(d.get('sm_state'))
    me = str(d.get('mpc_mode_echo'))
    ot = d.get('ot_age_ms', -1)
    rc = d.get('rc_age_ms', -1)
    nw = d.get('n_wpnts', -1)
    tr = d.get('trail_targets_n', -1)
    print(f'{rel:6.2f}  {ps:<8s}  {ss:<10s}  {me:<10s}  '
          f'{ot:>6}  {rc:>6}  {nw:>4}  {tr:>5}')

# --- MPC tick timeline (first 10s) ---
print('\n=== MPC tick timeline (first 10s) ===')
print(f'{"t":>6s}  {"plan":<10s}  {"mode":<8s}  {"tier":>4s}  {"status":<22s}  '
      f'{"ipopt":<28s}  {"solve":>6s}')
for ts, d in mpc_tick:
    rel = ts - t0
    if rel > 10.0: break
    plan = d.get('plan'); mode = d.get('mpc_mode'); tier = d.get('tier')
    st = d.get('status'); ip = d.get('ipopt_status', '')
    sm_ = d.get('solve_ms', 0)
    print(f'{rel:6.2f}  {str(plan):<10s}  {str(mode):<8s}  {str(tier):>4s}  '
          f'{str(st):<22s}  {str(ip)[:28]:<28s}  {sm_:>6.1f}')

# --- MPC status messages (first 10s) ---
print('\n=== MPC status messages (first 10s) ===')
for ts, txt in mpc_status:
    rel = ts - t0
    if rel > 10.0: break
    print(f'  t={rel:6.2f}s  "{txt[:130]}"')

# --- /planner/mpc/wpnts publish timeline (first 10s) ---
print('\n=== /planner/mpc/wpnts publish timeline (first 10s) ===')
print(f'{"t":>6s}  {"ot_line":<10s}  {"n":>4s}  {"s0":>7s}  {"gap_ms":>7s}')
prev_t = None
for ts, ot, n, s0 in mpc_pub:
    rel = ts - t0
    if rel > 10.0: break
    gap = (ts - prev_t)*1000 if prev_t else 0
    print(f'{rel:6.2f}  {str(ot):<10s}  {n:>4d}  {s0:>7.2f}  {gap:>7.1f}')
    prev_t = ts

# --- path_source transitions in first 10s ---
print('\n=== path_source TRANSITIONS (first 10s) ===')
last_ps = None
for ts, d in mpc_dbg:
    rel = ts - t0
    if rel > 10.0: break
    cur = d.get('path_source')
    if cur != last_ps:
        ot = d.get('ot_age_ms', -1)
        rc = d.get('rc_age_ms', -1)
        ss = d.get('sm_state')
        me = d.get('mpc_mode_echo')
        nw = d.get('n_wpnts', -1)
        print(f'  t={rel:6.2f}s  {last_ps!r:>10s} -> {cur!r:<10s}  '
              f'sm={ss!r:<10s}  mode={me!r:<8s}  '
              f'ot_age={ot}ms  rc_age={rc}ms  n_wp={nw}')
        last_ps = cur

# --- print full MPC SM debug field set (first msg) for any extra fields ---
print('\n=== mpc_dbg first msg full keys/values ===')
if mpc_dbg:
    for k, v in mpc_dbg[0][1].items():
        print(f'  {k}: {v}')

print('\n=== DONE ===')
