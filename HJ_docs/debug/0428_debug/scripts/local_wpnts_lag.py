#!/usr/bin/env python3
"""Compare ego.cur_s with behavior_strategy.local_wpnts[0].s_m to spot
stale-path or wrong-nearest cases that produce local_wpnts starting far
from ego (e.g. at the obstacle position).
"""
import sys
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

ego = []   # (t, s)
beh = []   # (t, state, wpnts0_s, n_loc)
for topic, msg, t in bag.read_messages(topics=[
        '/car_state/odom_frenet', '/behavior_strategy']):
    ts = t.to_sec()
    if topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x))
    elif topic == '/behavior_strategy':
        n = len(msg.local_wpnts) if msg.local_wpnts else 0
        s0 = float(msg.local_wpnts[0].s_m) if n > 0 else float('nan')
        beh.append((ts, str(msg.state), s0, n))
bag.close()

t0 = ego[0][0]
TRACK = 85.6

def near(arr, t):
    lo, hi = 0, len(arr)-1
    while lo < hi:
        mid = (lo+hi)//2
        if arr[mid][0] < t: lo = mid+1
        else: hi = mid
    return arr[lo]

big_lags = []
for ts, st, s0, n in beh:
    if s0 != s0: continue
    e = near(ego, ts)
    ego_s = e[1]
    lag = (s0 - ego_s) % TRACK
    if lag > 0.5 * TRACK: lag -= TRACK
    if abs(lag) > 0.5:
        big_lags.append((ts - t0, st, ego_s, s0, lag, n))

print('total beh msgs: %d' % len(beh))
print('msgs with |lag|>0.5m: %d' % len(big_lags))
print('\n%6s %-9s %8s %8s %8s %4s' % ('t_rel', 'state', 'ego_s', 'wpnt0_s', 'lag_m', 'nloc'))
for tr, st, es, ws, lag, n in big_lags[:40]:
    print('%6.2f %-9s %8.2f %8.2f %+8.2f %4d' %
          (tr, st, es, ws, lag, n))

# Histogram
import numpy as np
lags = np.array([(s0-near(ego,ts)[1])%TRACK for ts,_,s0,_ in beh if s0==s0])
lags = np.where(lags > 0.5*TRACK, lags-TRACK, lags)
print('\nlag histogram (m):')
print('  p50=%.2f  p90=%.2f  p95=%.2f  p99=%.2f  max=%.2f' %
      (np.percentile(lags, 50),
       np.percentile(lags, 90),
       np.percentile(lags, 95),
       np.percentile(lags, 99),
       np.max(lags)))
print('  min=%.2f' % np.min(lags))
