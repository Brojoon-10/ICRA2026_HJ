#!/usr/bin/env python3
"""Analyze a specific time window in a bag — ego/obs trajectory, plan, scores
to understand why a collision happened."""
import sys
import json
import rosbag

bag_path = sys.argv[1]
t_target = float(sys.argv[2])
window = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

bag = rosbag.Bag(bag_path, 'r')
t_start_abs = None
events = []

for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if t_start_abs is None:
        t_start_abs = ts
    t_rel = ts - t_start_abs
    if abs(t_rel - t_target) > window:
        continue
    if topic == '/car_state/odom_frenet':
        events.append((t_rel, 'EGO',
                       'n=%+.3f s=%.2f vs=%.2f' %
                       (msg.pose.pose.position.y,
                        msg.pose.pose.position.x,
                        msg.twist.twist.linear.x)))
    elif topic == '/tracking/obstacles_truth':
        for o in msg.obstacles:
            events.append((t_rel, 'OBS',
                           'n=%+.3f s=%.2f vs=%.2f' %
                           (o.d_center, o.s_center, o.vs)))
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data)
            events.append((t_rel, 'TICK',
                           'plan=%s side=%s d_free_L=%.3f d_free_R=%.3f obs_in_h=%s' %
                           (d.get('plan'), d.get('side'),
                            d.get('side_scores', {}).get('d_free_L', 0),
                            d.get('side_scores', {}).get('d_free_R', 0),
                            d.get('obs_in_horizon'))))
        except Exception:
            pass
    elif topic == '/opponent_collision':
        if bool(msg.data):
            events.append((t_rel, 'COLL', 'TRUE'))

bag.close()

# Sort by time, downsample EGO/OBS to once every 0.10s for readability
events.sort(key=lambda e: e[0])
last_print = {'EGO': -1e9, 'OBS': -1e9, 'TICK': -1e9}
PRINT_INTERVAL = 0.10
for t_rel, kind, info in events:
    if kind == 'COLL':
        print('  t=%6.2f  COLL  %s' % (t_rel, info))
        continue
    if t_rel - last_print[kind] >= PRINT_INTERVAL:
        last_print[kind] = t_rel
        print('  t=%6.2f  %s   %s' % (t_rel, kind, info))
