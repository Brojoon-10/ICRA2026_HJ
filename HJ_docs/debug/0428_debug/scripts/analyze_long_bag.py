#!/usr/bin/env python3
"""Long-bag analysis (10+ min). Per-lap breakdown, time-windowed metrics.
"""
import sys
import json
from collections import defaultdict

import rosbag

bag_path = sys.argv[1]
TRACK_LEN = 85.6
WINDOW_S = 60.0  # bin size for time-windowed stats

bag = rosbag.Bag(bag_path, 'r')
ego, obs, plans, coll_times = [], [], [], []
last_coll = False
for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x))
    elif topic == '/tracking/obstacles_truth':
        for o in msg.obstacles:
            obs.append((ts, o.s_center, o.d_center))
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data)
            if d.get('plan'):
                plans.append((ts, d['plan']))
        except Exception:
            pass
    elif topic == '/opponent_collision':
        if bool(msg.data) and not last_coll:
            coll_times.append(ts)
        last_coll = bool(msg.data)
bag.close()

if not ego:
    print('NO EGO MSGS'); sys.exit(1)

t0, t1 = ego[0][0], ego[-1][0]
dur = t1 - t0
print('duration: %.1fs (%.2f min)  ego: %d  obs: %d  plans: %d  coll: %d' %
      (dur, dur/60.0, len(ego), len(obs), len(plans), len(coll_times)))

# Plan transitions
transitions = []
last = None
for ts, p in plans:
    if p != last:
        transitions.append(ts - t0); last = p
print('total transitions: %d  (%.2f /min)' %
      (len(transitions), len(transitions) / max(dur/60.0, 1e-3)))

# Lap detection — ego_s wraps from ~85 to ~0
laps = []
last_s = ego[0][1]
lap_start_t = ego[0][0]
for ts, s, n, vs in ego:
    if last_s > 60 and s < 20:   # wrap
        laps.append((lap_start_t - t0, ts - t0))
        lap_start_t = ts
    last_s = s
laps.append((lap_start_t - t0, t1 - t0))   # final partial
print('laps detected: %d' % len(laps))

# Collision count + lateral min per minute window
nbins = int(dur / WINDOW_S) + 1
bin_coll = [0] * nbins
bin_trans = [0] * nbins
bin_lat_min = [float('inf')] * nbins
for ct in coll_times:
    b = int((ct - t0) / WINDOW_S)
    if 0 <= b < nbins:
        bin_coll[b] += 1
for tr in transitions:
    b = int(tr / WINDOW_S)
    if 0 <= b < nbins:
        bin_trans[b] += 1

def near_ego_s(target_t):
    lo, hi = 0, len(ego) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if ego[mid][0] < target_t: lo = mid + 1
        else: hi = mid
    return ego[lo]

for ot, os, on in obs:
    e = near_ego_s(ot)
    s_e, n_e = e[1], e[2]
    ds = os - s_e
    while ds > 0.5 * TRACK_LEN: ds -= TRACK_LEN
    while ds < -0.5 * TRACK_LEN: ds += TRACK_LEN
    if abs(ds) < 1.0:
        dn = abs(n_e - on)
        b = int((ot - t0) / WINDOW_S)
        if 0 <= b < nbins and dn < bin_lat_min[b]:
            bin_lat_min[b] = dn

print('\n=== Per 60s window ===')
print('%4s %5s %5s %8s' % ('bin', 'coll', 'trans', 'lat_min'))
for i in range(nbins):
    lat_str = '%dmm' % int(bin_lat_min[i] * 1000) if bin_lat_min[i] < float('inf') else '-'
    print('%4d %5d %5d %8s' % (i, bin_coll[i], bin_trans[i], lat_str))

print('\n=== Aggregate ===')
print('  collisions per minute:    %.2f' % (len(coll_times) / max(dur/60.0, 1e-3)))
print('  transitions per minute:   %.2f' % (len(transitions) / max(dur/60.0, 1e-3)))
valid_lat = [b for b in bin_lat_min if b < float('inf')]
if valid_lat:
    sl = sorted(valid_lat)
    print('  lat_min: min=%dmm  med=%dmm  max=%dmm' %
          (int(min(valid_lat)*1000),
           int(sl[len(sl)//2]*1000),
           int(max(valid_lat)*1000)))
