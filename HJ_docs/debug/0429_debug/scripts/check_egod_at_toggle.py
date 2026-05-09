#!/usr/bin/env python3
"""Cross-correlate path_source toggles with ego d (lateral) and v_cur."""
import sys, json, math
import rosbag

bag = rosbag.Bag(sys.argv[1], 'r')
mpc_dbg, ego, gb = [], [], []
t0 = None
for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if t0 is None: t0 = ts
    if topic == '/state_machine_mpc/debug':
        try: mpc_dbg.append((ts, json.loads(msg.data)))
        except Exception: pass
    elif topic == '/car_state/odom_frenet':
        # x = s, y = d, twist.linear.x = vs
        ego.append((ts, msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.twist.twist.linear.x))
    elif topic == '/global_waypoints_scaled':
        # only first message; build s -> vx lookup
        if not gb:
            for w in msg.wpnts:
                gb.append((float(w.s_m), float(w.vx_mps)))
bag.close()

def near(arr, t):
    if not arr: return None
    lo, hi = 0, len(arr)-1
    while lo < hi:
        m = (lo+hi)//2
        if arr[m][0] < t: lo = m+1
        else: hi = m
    return arr[lo]

def vtarget_at(s):
    if not gb: return None
    # binary search
    lo, hi = 0, len(gb)-1
    while lo < hi:
        m = (lo+hi)//2
        if gb[m][0] < s: lo = m+1
        else: hi = m
    return gb[lo][1]

# Find path_source transitions and dump ego d + speed ratio
last_ps = None
print(f'{"t":>7s}  {"trans":<24s}  {"sm":<10s}  {"|n|":>6s}  '
      f'{"v_cur":>6s}  {"v_tgt":>6s}  {"ratio":>6s}  '
      f'{"thr_used":>9s}  {"close_n":>8s}  {"close_h":>8s}')
for ts, d in mpc_dbg:
    rel = ts - t0
    if rel > 12.0: break
    cur = d.get('path_source')
    if cur != last_ps:
        e = near(ego, ts)
        if e:
            n_abs = abs(e[2])
            vs = e[3]
            vt = vtarget_at(e[1]) or 0.0
            ratio = (vs / vt) if vt > 0.05 else 0.0
            thr = 0.2 if ratio < 0.5 else 0.5  # tight vs loose
            close_n = '<thr' if n_abs < thr else 'ABOVE'
            # heading-check uses |n| < deg2rad(20) ≈ 0.349
            close_h = '<0.349' if n_abs < math.radians(20) else 'ABOVE'
            print(f'{rel:7.2f}  {str(last_ps)+" -> "+str(cur):<24s}  '
                  f'{str(d.get("sm_state")):<10s}  '
                  f'{n_abs:>6.3f}  {vs:>6.2f}  {vt:>6.2f}  {ratio:>6.2f}  '
                  f'{thr:>9.2f}  {close_n:>8s}  {close_h:>8s}')
        last_ps = cur

# Also dump ego d at every 0.2s for first 12s
print('\n=== ego d, vs over first 12s (every 0.2s) ===')
print(f'{"t":>6s}  {"|n|":>6s}  {"vs":>6s}  {"v_tgt":>6s}  {"ratio":>6s}')
last_print = -1
for ts, s, n, vs in ego:
    rel = ts - t0
    if rel > 12.0: break
    if rel < last_print + 0.2: continue
    last_print = rel
    vt = vtarget_at(s) or 0.0
    ratio = (vs / vt) if vt > 0.05 else 0.0
    print(f'{rel:6.2f}  {abs(n):>6.3f}  {vs:>6.2f}  {vt:>6.2f}  {ratio:>6.2f}')
