#!/usr/bin/env python3
"""Aggregate metrics across multiple short trial bags.

Usage:  python3 aggregate_trials.py <bag_glob>
        e.g. python3 aggregate_trials.py 'trial_*'
"""
import glob
import json
import os
import sys
from collections import defaultdict

import rosbag

pattern = sys.argv[1]
bagdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'bags')
bags = sorted(glob.glob(os.path.join(bagdir, pattern + '.bag')))
if not bags:
    print('no bags match', pattern)
    sys.exit(1)
print('found %d bags' % len(bags))

TRACK_LEN = 85.6

def analyse_bag(path):
    bag = rosbag.Bag(path, 'r')
    ego, obs, plans = [], [], []
    coll_edges = 0
    last_coll = False
    for topic, msg, t in bag.read_messages():
        ts = t.to_sec()
        if topic == '/car_state/odom_frenet':
            ego.append((ts, msg.pose.pose.position.x, msg.pose.pose.position.y))
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
                coll_edges += 1
            last_coll = bool(msg.data)
    bag.close()
    if not ego:
        return None
    t0, t1 = ego[0][0], ego[-1][0]
    dur = t1 - t0
    transitions = 0
    last = None
    for _, p in plans:
        if p != last:
            transitions += 1
            last = p
    # Lateral min when ego close to obs
    def near_ego(target_t):
        lo, hi = 0, len(ego) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if ego[mid][0] < target_t: lo = mid + 1
            else: hi = mid
        return ego[lo]
    lat_min_close = float('inf')
    for t_o, s_o, n_o in obs:
        e = near_ego(t_o)
        s_e, n_e = e[1], e[2]
        ds = s_o - s_e
        while ds > 0.5 * TRACK_LEN: ds -= TRACK_LEN
        while ds < -0.5 * TRACK_LEN: ds += TRACK_LEN
        if abs(ds) < 1.0:
            dn = abs(n_e - n_o)
            if dn < lat_min_close:
                lat_min_close = dn
    return {
        'duration': dur,
        'transitions': transitions,
        'trans_per_min': transitions / max(dur / 60.0, 1e-3),
        'collisions': coll_edges,
        'lat_min_mm': int(lat_min_close * 1000) if lat_min_close < float('inf') else -1,
    }

results = []
for b in bags:
    r = analyse_bag(b)
    if r is None: continue
    r['bag'] = os.path.basename(b)
    results.append(r)

print('\n%-25s %6s %6s %5s %8s' % ('bag', 'dur', 'trans', 'coll', 'lat_min'))
for r in results:
    print('%-25s %6.1f %6d %5d %6dmm' %
          (r['bag'], r['duration'], r['transitions'], r['collisions'], r['lat_min_mm']))

total_coll = sum(r['collisions'] for r in results)
total_trans = sum(r['transitions'] for r in results)
total_dur = sum(r['duration'] for r in results)
lat_mins = [r['lat_min_mm'] for r in results if r['lat_min_mm'] >= 0]
print('\n=== Aggregate (%d bags, %.0fs) ===' % (len(results), total_dur))
print('  total collisions:   %d  (%.1f / 60s)' % (total_coll, total_coll / max(total_dur/60, 1e-3)))
print('  total transitions:  %d  (%.1f /min)' % (total_trans, total_trans / max(total_dur/60, 1e-3)))
if lat_mins:
    print('  lat_min:  min=%dmm  med=%dmm  max=%dmm' %
          (min(lat_mins), sorted(lat_mins)[len(lat_mins)//2], max(lat_mins)))
