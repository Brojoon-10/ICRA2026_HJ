#!/usr/bin/env python3
"""For each collision rising edge, show the active plan and ego/obs state."""
import sys, json
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')
ego, obs, plans, coll = [], [], [], []
last = False
for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x))
    elif topic == '/tracking/obstacles_truth':
        for o in msg.obstacles:
            obs.append((ts, o.s_center, o.d_center, o.vs))
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data)
            if d.get('plan'):
                plans.append((ts, d['plan'], d.get('side_scores', {})))
        except Exception:
            pass
    elif topic == '/opponent_collision':
        if bool(msg.data) and not last:
            coll.append(ts)
        last = bool(msg.data)
bag.close()

t0 = ego[0][0]

def near(arr, t):
    lo, hi = 0, len(arr) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if arr[mid][0] < t: lo = mid + 1
        else: hi = mid
    return arr[lo]

# Skip clusters: collisions within 0.5s of each other treated as single event
clusters = []
for ct in coll:
    if clusters and ct - clusters[-1] < 0.5:
        continue
    clusters.append(ct)
print('total collision events: %d (%d edges, %d clustered)' %
      (len(clusters), len(coll), len(clusters)))

print('%-7s %-10s %-9s %-9s %-12s %s' %
      ('t_rel', 'plan', 'ego_n', 'obs_n', 'ds', 'extra'))
plan_count = {}
for ct in clusters:
    e = near(ego, ct)
    o = near(obs, ct)
    p = near(plans, ct)
    s_e, n_e = e[1], e[2]
    s_o, n_o, vs_o = o[1], o[2], o[3]
    ds = s_o - s_e
    while ds > 42.8: ds -= 85.6
    while ds < -42.8: ds += 85.6
    plan_name = p[1] if p else '?'
    plan_count[plan_name] = plan_count.get(plan_name, 0) + 1
    side_scores = p[2] if p else {}
    extra = 'd_free_L=%.2f d_free_R=%.2f' % (
        side_scores.get('d_free_L', 0), side_scores.get('d_free_R', 0))
    print('%6.1fs %-10s %+.3f    %+.3f    %+.2f    %s' %
          (ct - t0, plan_name, n_e, n_o, ds, extra))

print('\n=== Collision distribution by plan ===')
for p, c in sorted(plan_count.items(), key=lambda kv: -kv[1]):
    print('  %-12s %d' % (p, c))
