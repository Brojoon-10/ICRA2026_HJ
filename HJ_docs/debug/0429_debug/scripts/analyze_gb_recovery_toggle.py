#!/usr/bin/env python3
"""Analyze why /local_waypoints toggles between GB and recovery,
and why recovery sometimes appears LATE (GB first, recovery later).

Looks at:
  /state_machine_mpc/debug      — per-tick MPC SM decision (source of local_wpnts)
  /state_machine                 — outer SM state
  /behavior_strategy             — final strategy with local_wpnts
  /mpc_auto/debug/tick_json      — MPC tick decision
  /mpc_auto/status               — MPC status messages
  /planner/mpc/wpnts             — MPC OTWpntArray (ot_line: avoid/recover)
  /local_waypoints               — final published wpnts (controller input)
  /car_state/odom_frenet         — ego s

The MPC SM (3d_mpc_state_machine_node) is the one that picks which path
goes into /local_waypoints. We want to see exactly which decision branch
fired each tick and why.
"""
import sys, json
from collections import Counter, defaultdict
import rosbag

if len(sys.argv) < 2:
    print('usage: analyze_gb_recovery_toggle.py <bag>'); sys.exit(1)

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

mpc_dbg = []     # (t, dict)
sm_main = []     # (t, str)
beh = []         # (t, state, src, n_loc, s0)
mpc_pub = []     # (t, ot_line, n, s0)
local_wpnts = [] # (t, n, s0, vx0)
mpc_tick = []    # (t, plan, ipopt, status, mpc_mode, tier)
mpc_status = []  # (t, str)
ego = []         # (t, s)

t0 = None
for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if t0 is None:
        t0 = ts

    if topic == '/state_machine_mpc/debug':
        try:
            d = json.loads(msg.data)
            mpc_dbg.append((ts, d))
        except Exception:
            pass
    elif topic == '/state_machine':
        sm_main.append((ts, str(msg.data)))
    elif topic == '/behavior_strategy':
        n_loc = len(msg.local_wpnts) if msg.local_wpnts else 0
        s0 = float(msg.local_wpnts[0].s_m) if n_loc > 0 else float('nan')
        src = getattr(msg, 'local_wpnts_source', '?')
        beh.append((ts, str(msg.state), str(src), n_loc, s0))
    elif topic == '/planner/mpc/wpnts':
        n = len(msg.wpnts)
        s0 = float(msg.wpnts[0].s_m) if n > 0 else float('nan')
        ot = getattr(msg, 'ot_line', '')
        mpc_pub.append((ts, ot, n, s0))
    elif topic == '/local_waypoints':
        n = len(msg.wpnts)
        if n > 0:
            local_wpnts.append((ts, n, float(msg.wpnts[0].s_m),
                                float(msg.wpnts[0].vx_mps)))
        else:
            local_wpnts.append((ts, 0, float('nan'), float('nan')))
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data)
            mpc_tick.append((ts, d.get('plan'), d.get('ipopt_status'),
                             d.get('status'), d.get('mpc_mode'),
                             d.get('tier')))
        except Exception:
            pass
    elif topic == '/mpc_auto/status':
        mpc_status.append((ts, msg.data))
    elif topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x))

bag.close()

dur = (ego[-1][0] - ego[0][0]) if ego else 0
print(f'=== {bag_path.split("/")[-1]}  dur={dur:.1f}s ===')
print(f'  mpc_dbg={len(mpc_dbg)}  sm={len(sm_main)}  beh={len(beh)}  '
      f'mpc_pub={len(mpc_pub)}  local={len(local_wpnts)}  '
      f'mpc_tick={len(mpc_tick)}  status={len(mpc_status)}')

# ---- MPC SM debug fields ----
if mpc_dbg:
    print('\n=== MPC SM debug field set (sample) ===')
    s = mpc_dbg[0][1]
    print(' ', sorted(list(s.keys())))

# ---- Source of local_wpnts each tick (from behavior_strategy) ----
print('\n=== beh.local_wpnts_source histogram ===')
src_hist = Counter(s for _, _, s, _, _ in beh)
for k, v in src_hist.most_common():
    print(f'  {k}: {v}  ({100*v/max(len(beh),1):.1f}%)')

# ---- ot_line histogram on /planner/mpc/wpnts ----
print('\n=== /planner/mpc/wpnts ot_line histogram ===')
ot_hist = Counter(ot for _, ot, _, _ in mpc_pub)
for k, v in ot_hist.most_common():
    print(f'  {k!r}: {v}')

# ---- mpc_mode histogram from MPC tick_json ----
print('\n=== MPC tick.mpc_mode histogram ===')
mode_hist = Counter(m for _, _, _, _, m, _ in mpc_tick)
for k, v in mode_hist.most_common():
    print(f'  {k!r}: {v}')

# ---- Plan / status / tier histograms ----
print('\n=== MPC tick.plan ===')
plan_hist = Counter(p for _, p, *_ in mpc_tick)
for k, v in plan_hist.most_common():
    print(f'  {k!r}: {v}')

print('\n=== MPC tick.status ===')
status_hist = Counter(s for _, _, _, s, _, _ in mpc_tick)
for k, v in status_hist.most_common(8):
    print(f'  {k!r}: {v}')

print('\n=== MPC tick.tier ===')
tier_hist = Counter(t for _, _, _, _, _, t in mpc_tick)
for k, v in sorted(tier_hist.items(), key=lambda kv: (kv[0] is None, kv[0])):
    print(f'  {k}: {v}')

# ---- local_wpnts_source transitions (the toggle the user is seeing) ----
print('\n=== beh.local_wpnts_source TRANSITIONS ===')
trans = []
last = None
for ts, st, src, n, s0 in beh:
    if src != last:
        trans.append((ts - t0, last, src, st, s0))
        last = src
print(f'  total transitions: {len(trans)}')
for tr in trans[:80]:
    rel, prev, cur, st, s0 = tr
    print(f'  t={rel:6.2f}s  {prev!r:>22} -> {cur!r:<22}  state={st:<10}  s0={s0:.2f}')

# ---- For each transition, find what the MPC SM debug said in the same tick ----
def near_ts(arr, t, max_gap=0.2):
    if not arr: return None
    lo, hi = 0, len(arr)-1
    while lo < hi:
        mid = (lo+hi)//2
        if arr[mid][0] < t: lo = mid+1
        else: hi = mid
    if abs(arr[lo][0] - t) > max_gap: return None
    return arr[lo]

print('\n=== Transition deep-dive: MPC SM decision at each toggle ===')
for tr in trans[:40]:
    rel, prev, cur, st, s0 = tr
    abs_t = rel + t0
    md = near_ts(mpc_dbg, abs_t, 0.3)
    mt = near_ts(mpc_tick, abs_t, 0.3)
    print(f'\n  t={rel:6.2f}s  {prev} -> {cur}')
    if md:
        d = md[1]
        keys = ['decision', 'reason', 'src_choice', 'have_mpc_path',
                'mpc_path_age_s', 'mpc_age_ms', 'mpc_freshness_ms',
                'avoid_active', 'recover_active', 'gb_fallback',
                'ot_line', 'mpc_state', 'pub_state',
                'mpc_avail', 'recovery_avail', 'horizon_clear',
                'force_gb', 'force_recover']
        info = {k: d.get(k) for k in keys if k in d}
        if info:
            print('    SM dbg:', info)
        else:
            print('    SM dbg keys:', list(d.keys())[:12])
    else:
        print('    SM dbg: no msg within 0.3s')
    if mt:
        _, p, ip, st_, mm, ti = mt
        print(f'    MPC tick: plan={p}  status={st_}  ipopt={ip}  mode={mm}  tier={ti}')

# ---- Recovery LATE detection ----
# Pattern: ot_line=='recover' should appear quickly when MPC enters
# recovery, but if /planner/mpc/wpnts publishes 'avoid' or nothing
# while SM picks GB tracking, then later switches to 'recover',
# that's the late-recovery the user describes.
print('\n=== Looking for "GB published then recovery shows up later" pattern ===')
# Walk through and find sequences where local_wpnts_source==GB tracking
# then flips to RECOVERY-from-MPC. Measure latency.
late_events = []
for i in range(1, len(trans)):
    prev_rel, prev_from, prev_to, prev_st, _ = trans[i-1]
    cur_rel, cur_from, cur_to, cur_st, _ = trans[i]
    # interesting: previous = GB tracking, current = something MPC-recovery-ish
    p_from = str(prev_from)
    p_to = str(prev_to)
    c_to = str(cur_to)
    if 'GB' in p_to.upper() or 'GLOBAL' in p_to.upper() or 'TRACK' in p_to.upper():
        if 'RECOVER' in c_to.upper() or 'MPC' in c_to.upper():
            gap = cur_rel - prev_rel
            late_events.append((prev_rel, p_to, c_to, gap, prev_st))
print(f'  late events (GB->recovery within session): {len(late_events)}')
for ev in late_events[:30]:
    rel, p_to, c_to, gap, st = ev
    print(f'    t={rel:6.2f}s  {p_to} -> {c_to}  gap={gap*1000:.0f}ms  state={st}')

# ---- /planner/mpc/wpnts ot_line transitions vs local_wpnts source ----
print('\n=== /planner/mpc/wpnts ot_line transitions ===')
ot_trans = []
last_ot = None
for ts, ot, n, s0 in mpc_pub:
    if ot != last_ot:
        ot_trans.append((ts - t0, last_ot, ot))
        last_ot = ot
print(f'  total ot_line transitions: {len(ot_trans)}')
for tr in ot_trans[:60]:
    print(f'  t={tr[0]:6.2f}s  {tr[1]!r} -> {tr[2]!r}')

# ---- MPC publish gaps near each toggle ----
print('\n=== MPC publish gaps (>=200ms) ===')
big_gaps = []
for i in range(1, len(mpc_pub)):
    gap = mpc_pub[i][0] - mpc_pub[i-1][0]
    if gap >= 0.2:
        big_gaps.append((mpc_pub[i-1][0]-t0, gap, mpc_pub[i-1][1], mpc_pub[i][1]))
for g in big_gaps[:30]:
    print(f'  t={g[0]:6.2f}s  gap={g[1]*1000:.0f}ms  {g[2]!r} -> {g[3]!r}')

# ---- For each ot_line transition, find what status/tier was active
print('\n=== ot_line transition deep-dive (avoid<->recover<->none) ===')
for tr in ot_trans[:30]:
    rel, prev, cur = tr
    abs_t = rel + t0
    mt = near_ts(mpc_tick, abs_t, 0.3)
    ms = near_ts(mpc_status, abs_t, 0.3)
    md = near_ts(mpc_dbg, abs_t, 0.3)
    line = f'  t={rel:6.2f}s  {prev!r}->{cur!r}'
    if mt:
        _, p, ip, st_, mm, ti = mt
        line += f'  mpc(plan={p} mode={mm} tier={ti} status={st_})'
    if ms:
        line += f'  status="{ms[1][:80]}"'
    print(line)

print('\n=== DONE ===')
