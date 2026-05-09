#!/usr/bin/env python3
"""Trace MPC publish (planner/mpc/wpnts) timing + first wpnt s_m vs ego_s.

Goal: distinguish whether the bug is
  (a) MPC stops publishing for some duration -> SM uses cached avoidance
  (b) MPC keeps publishing but first wpnt is at a different lap position
  (c) Something else (callback drops, msg corruption)
"""
import sys
import json
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

ego = []      # (t, s)
mpc_pub = []  # (t_record, t_header, n_wpnts, s_m_first, ot_line)
beh = []      # (t, state, wpnts0_s)
plans = []    # (t, plan, ipopt_status, status, mpc_mode_echo, tier)
status = []   # (t, status_str)

for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x))
    elif topic == '/planner/mpc/wpnts':
        n = len(msg.wpnts)
        s0 = float(msg.wpnts[0].s_m) if n > 0 else float('nan')
        ot_line = getattr(msg, 'ot_line', '')
        t_hdr = msg.header.stamp.to_sec() if msg.header else ts
        mpc_pub.append((ts, t_hdr, n, s0, ot_line))
    elif topic == '/behavior_strategy':
        n = len(msg.local_wpnts) if msg.local_wpnts else 0
        s0 = float(msg.local_wpnts[0].s_m) if n > 0 else float('nan')
        beh.append((ts, str(msg.state), s0))
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data)
            plans.append((ts, d.get('plan'),
                          d.get('ipopt_status'),
                          d.get('status'),
                          d.get('mpc_mode'),
                          d.get('tier')))
        except Exception:
            pass
    elif topic == '/mpc_auto/status':
        status.append((ts, msg.data))
bag.close()

t0 = ego[0][0]

# ---------- MPC publish gaps ----------
print('=== MPC publish (/planner/mpc/wpnts) ===')
print('total: %d in %.1fs (avg %.1f Hz)' %
      (len(mpc_pub), (mpc_pub[-1][0]-mpc_pub[0][0]),
       len(mpc_pub) / max(mpc_pub[-1][0]-mpc_pub[0][0], 1e-3)))
gaps = [(mpc_pub[i][0]-mpc_pub[i-1][0]) for i in range(1, len(mpc_pub))]
gaps.sort()
print('  gap p50=%.3fs p90=%.3fs p99=%.3fs max=%.3fs' %
      (gaps[len(gaps)//2], gaps[int(len(gaps)*0.9)],
       gaps[int(len(gaps)*0.99)], gaps[-1]))

print('\nlongest publish gaps (>0.1s):')
gaps_idx = [(mpc_pub[i][0]-mpc_pub[i-1][0], i) for i in range(1, len(mpc_pub))]
for g, i in sorted(gaps_idx, reverse=True)[:10]:
    if g <= 0.1: break
    t_prev = mpc_pub[i-1][0] - t0
    t_cur = mpc_pub[i][0] - t0
    s_prev = mpc_pub[i-1][3]
    s_cur = mpc_pub[i][3]
    print('  gap=%.3fs  t_prev=%6.2fs  t_cur=%6.2fs  s_prev=%6.2f -> s_cur=%6.2f' %
          (g, t_prev, t_cur, s_prev, s_cur))

# ---------- Status during gaps ----------
print('\n=== Status during big gaps (look for EMERGENCY/HOLD_LAST) ===')
for g, i in sorted(gaps_idx, reverse=True)[:5]:
    if g <= 0.1: break
    t_a = mpc_pub[i-1][0]
    t_b = mpc_pub[i][0]
    print('\ngap %.3fs (t=%.2f to %.2f):' % (g, t_a-t0, t_b-t0))
    for ts, s in status:
        if t_a <= ts <= t_b:
            print('  t=%6.2fs status=%s' % (ts-t0, s))
    for ts, p, ipopt, st, mm, tier in plans:
        if t_a <= ts <= t_b:
            print('  t=%6.2fs plan=%s ipopt=%s status=%s tier=%s' %
                  (ts-t0, p, ipopt, st, tier))

# ---------- MPC first wpnt s_m vs ego_s aligned ----------
def near(arr, t):
    lo, hi = 0, len(arr)-1
    while lo < hi:
        mid = (lo+hi)//2
        if arr[mid][0] < t: lo = mid+1
        else: hi = mid
    return arr[lo]

print('\n=== MPC publish first wpnt s_m vs ego_s ===')
TRACK = 85.6
big_diffs = 0
samples = []
for ts, t_hdr, n, s0, ot_line in mpc_pub:
    if s0 != s0: continue
    e = near(ego, ts)
    es = e[1]
    diff = (s0 - es) % TRACK
    if diff > 0.5*TRACK: diff -= TRACK
    samples.append((ts-t0, es, s0, diff, ot_line))
    if abs(diff) > 0.3:
        big_diffs += 1
print('total mpc samples: %d' % len(mpc_pub))
print('|s_m_first - ego_s| > 0.3m: %d  (%.1f%%)' %
      (big_diffs, 100.0*big_diffs/max(len(mpc_pub),1)))

print('\nworst |s_m_first - ego_s| samples:')
samples_sorted = sorted(samples, key=lambda x: -abs(x[3]))
for ts, es, s0, diff, ot_line in samples_sorted[:20]:
    print('  t=%6.2fs  ego_s=%6.2f  mpc_s0=%6.2f  diff=%+7.2f  ot_line=%s' %
          (ts, es, s0, diff, ot_line))

# ---------- Behavior_strategy timing relative to MPC publish ----------
print('\n=== beh.local_wpnts[0].s_m chain ===')
print('Sample where beh.s0 != mpc.s0 (mpc fresh but beh different):')
mismatches = 0
for ts, st, bs0 in beh:
    if bs0 != bs0: continue
    # Find latest mpc_pub before this ts
    candidates = [(t, s0) for t, _, _, s0, _ in mpc_pub if t < ts and s0 == s0]
    if not candidates: continue
    last_mpc_t, last_mpc_s0 = candidates[-1]
    if abs(last_mpc_s0 - bs0) > 0.5:
        if mismatches < 20:
            print('  t=%6.2fs  beh.s0=%6.2f  last_mpc.s0=%6.2f (%.2fs ago) state=%s' %
                  (ts-t0, bs0, last_mpc_s0, ts-last_mpc_t, st))
        mismatches += 1
print('total mismatches (>0.5m): %d / %d' % (mismatches, len(beh)))
