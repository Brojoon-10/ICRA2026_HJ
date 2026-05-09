#!/usr/bin/env python3
"""Walk MPC tick_json + status + local_waypoints + mpc_pub side-by-side.

Goal: understand why local_waypoints visually toggles between GB-shape
and recovery-shape, and why recovery appears LATE after GB.

Hypothesis to test:
  H1 — NLP success (tier=0) and cache_follow (tier=2) alternate;
       both have ot_line='recover' but the path geometry differs
       (NLP near raceline vs cache curve). So visual toggle = tier toggle.
  H2 — When MPC publish stalls (>200ms gap), the SM falls back to GB
       (raceline slice) for local_waypoints, then when MPC publish
       resumes (cache or NLP), local_waypoints jumps to recovery shape.
"""
import sys, json, math
import rosbag

if len(sys.argv) < 2:
    print('usage: analyze_tier_timeline.py <bag>'); sys.exit(1)

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

mpc_tick = []     # (t, dict)
mpc_status = []   # (t, str)
mpc_pub = []      # (t, ot_line, n, s0)
local_wpnts = []  # (t, n, s0, vx0, vx_mean)
beh = []          # (t, state, n_loc, s0, vx0)
ego = []          # (t, s, vs)
mpc_dbg = []      # (t, dict)

t0 = None
for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if t0 is None: t0 = ts
    if topic == '/mpc_auto/debug/tick_json':
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
        if n>0:
            s0 = float(msg.wpnts[0].s_m)
            vx0 = float(msg.wpnts[0].vx_mps)
            vxm = sum(w.vx_mps for w in msg.wpnts)/n
            local_wpnts.append((ts, n, s0, vx0, vxm))
        else:
            local_wpnts.append((ts, 0, float('nan'), float('nan'), float('nan')))
    elif topic == '/behavior_strategy':
        n = len(msg.local_wpnts) if msg.local_wpnts else 0
        s0 = float(msg.local_wpnts[0].s_m) if n>0 else float('nan')
        vx0 = float(msg.local_wpnts[0].vx_mps) if n>0 else float('nan')
        beh.append((ts, str(msg.state), n, s0, vx0))
    elif topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x,
                    msg.twist.twist.linear.x))
    elif topic == '/state_machine_mpc/debug':
        try: mpc_dbg.append((ts, json.loads(msg.data)))
        except Exception: pass

bag.close()

print(f'=== timeline of MPC ticks (every 5th) ===')
print(f'{"t":>7s}  {"plan":<10s}  {"mode":<8s}  {"tier":>4s}  '
      f'{"status":<22s}  {"ipopt":<28s}  {"solve_ms":>8s}')
for i, (ts, d) in enumerate(mpc_tick):
    if i % 5 != 0: continue
    rel = ts - t0
    plan = d.get('plan')
    mode = d.get('mpc_mode')
    tier = d.get('tier')
    st = d.get('status')
    ip = d.get('ipopt_status', '')
    sm = d.get('solve_ms', 0)
    print(f'{rel:7.2f}  {str(plan):<10s}  {str(mode):<8s}  {str(tier):>4s}  '
          f'{str(st):<22s}  {str(ip)[:28]:<28s}  {sm:>8.1f}')

# ---- Tier transitions (this is the toggle the user sees) ----
print('\n=== tier transitions ===')
trans = []
last_tier = None
last_status = None
for ts, d in mpc_tick:
    ti = d.get('tier')
    st = d.get('status')
    if ti != last_tier or st != last_status:
        trans.append((ts - t0, last_tier, ti, last_status, st,
                      d.get('ipopt_status'), d.get('solve_ms', 0)))
        last_tier = ti
        last_status = st
print(f'  total tier/status transitions: {len(trans)}')
for tr in trans[:80]:
    rel, pt, ct, ps, cs, ip, sm = tr
    print(f'  t={rel:6.2f}s  tier {pt}->{ct}  status {ps}->{cs}  '
          f'ipopt={str(ip)[:24]:<24s}  solve={sm:.1f}ms')

# ---- For each tier transition (NLP-success -> cache or vice versa),
#      check if any visible cause exists in MPC status messages. ----
print('\n=== status messages near each tier transition ===')
def near_status(t, gap=0.3):
    out = []
    for sts, txt in mpc_status:
        if abs(sts - t) <= gap:
            out.append((sts - t, txt))
    return out

for tr in trans[:40]:
    rel = tr[0]
    abs_t = rel + t0
    near = near_status(abs_t, 0.3)
    if near:
        print(f'  t={rel:6.2f}s  tier {tr[1]}->{tr[2]}  status {tr[3]}->{tr[4]}')
        for off, txt in near[:3]:
            print(f'      ({off*1000:+.0f}ms) {txt[:120]}')

# ---- During publish gaps, what does local_waypoints look like? ----
print('\n=== Big publish gaps deep-dive ===')
for i in range(1, len(mpc_pub)):
    gap = mpc_pub[i][0] - mpc_pub[i-1][0]
    if gap < 0.2: continue
    t_a = mpc_pub[i-1][0]
    t_b = mpc_pub[i][0]
    print(f'\n  gap {gap*1000:.0f}ms  from t={t_a-t0:6.2f}s to t={t_b-t0:6.2f}s')
    # local_waypoints msgs published in this gap
    inside = [(lt, n, s0, vx0, vxm) for lt, n, s0, vx0, vxm in local_wpnts
              if t_a < lt < t_b]
    print(f'    local_waypoints during gap: {len(inside)} msgs')
    for lt, n, s0, vx0, vxm in inside[:10]:
        print(f'      t={lt-t0:6.2f}s  n={n}  s0={s0:.2f}  vx0={vx0:.2f}  vx_mean={vxm:.2f}')
    # behavior state during gap
    beh_inside = [(bt, st) for bt, st, *_ in beh if t_a < bt < t_b]
    states = set(s for _, s in beh_inside)
    print(f'    beh.state during gap: {states}')
    # MPC status messages during gap
    stat_inside = [(st, txt) for st, txt in mpc_status if t_a < st < t_b]
    print(f'    MPC status during gap: {len(stat_inside)} msgs')
    for st, txt in stat_inside[:8]:
        print(f'      t={st-t0:6.2f}s  "{txt[:100]}"')
    # MPC tick during gap (the solver was doing what?)
    tick_inside = [(tt, d) for tt, d in mpc_tick if t_a < tt < t_b]
    print(f'    MPC tick during gap: {len(tick_inside)} ticks')
    for tt, d in tick_inside[:8]:
        print(f'      t={tt-t0:6.2f}s  tier={d.get("tier")}  status={d.get("status")}'
              f'  ipopt={d.get("ipopt_status")}  solve={d.get("solve_ms",0):.1f}ms')

# ---- Compare local_waypoints first-vx with raceline GB vx around toggle ----
# When local_waypoints first wpnt has GB-raceline-shaped vx_mps profile,
# SM is publishing GB; when it has recovery-curve-shape, SM is using MPC.
print('\n=== local_waypoints stats summary ===')
if local_wpnts:
    s_arr = [s0 for _, n, s0, vx0, vxm in local_wpnts if n>0]
    vx0_arr = [vx0 for _, n, s0, vx0, vxm in local_wpnts if n>0]
    print(f'  msgs={len(local_wpnts)}  s0 range=[{min(s_arr):.1f}, {max(s_arr):.1f}]')
    print(f'  vx0 stats: min={min(vx0_arr):.2f}  max={max(vx0_arr):.2f}  mean={sum(vx0_arr)/len(vx0_arr):.2f}')

# ---- mpc_dbg.path_source histogram (if it tells us GB vs MPC) ----
print('\n=== state_machine_mpc/debug.path_source ===')
from collections import Counter
ps = Counter(d.get('path_source') for _, d in mpc_dbg)
for k, v in ps.most_common():
    print(f'  {k!r}: {v}')
ms = Counter(d.get('mpc_mode_echo') for _, d in mpc_dbg)
print('mpc_mode_echo:')
for k, v in ms.most_common(): print(f'  {k!r}: {v}')

# ---- path_source transitions in MPC SM (this is what the user actually sees) ----
print('\n=== MPC SM path_source transitions (THIS is what feeds /local_waypoints) ===')
ps_trans = []
last_ps = None
for ts, d in mpc_dbg:
    cur = d.get('path_source')
    if cur != last_ps:
        ps_trans.append((ts - t0, last_ps, cur, d.get('sm_state'),
                         d.get('mpc_mode_echo'), d.get('ot_age_ms', -1),
                         d.get('rc_age_ms', -1), d.get('n_wpnts', -1)))
        last_ps = cur
print(f'  total path_source transitions: {len(ps_trans)}')
for tr in ps_trans[:80]:
    rel, pp, cp, st, mm, ot_age, rc_age, nw = tr
    print(f'  t={rel:6.2f}s  {pp!r:>12s} -> {cp!r:<14s}  '
          f'sm={st!r:<10s}  mode={mm!r:<10s}  '
          f'ot_age={ot_age}ms  rc_age={rc_age}ms  n_wpnts={nw}')

print('\n=== DONE ===')
