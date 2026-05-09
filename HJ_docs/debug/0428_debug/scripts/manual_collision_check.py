#!/usr/bin/env python3
"""collision_detector 우회: ego ↔ obstacle frenet 거리 직접 계산.

진짜 충돌 정의:
  - frenet_dist = hypot(ds, dd) where ds = ego_s - obs_s, dd = ego_d - obs_d
  - threshold = ego_half + obs_half + safety_margin = 0.15 + 0.20 + 0.05 = 0.40m
  - frenet_dist < 0.40m 시 충돌 (ego body 가 obstacle body 와 겹침)
"""
import sys
import math
import rosbag
from collections import Counter

bag_path = sys.argv[1]
threshold = float(sys.argv[2]) if len(sys.argv) > 2 else 0.40

bag = rosbag.Bag(bag_path, 'r')

# ego s, d 시계열 추출
print(f'\nLoading ego trajectory from {bag_path} ...')
ego_traj = []  # (t, s, d, v)
for topic, msg, t in bag.read_messages(topics=['/car_state/odom_frenet']):
    ego_traj.append((t.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x))

# obstacle 시계열 (truth) — id 별
obs_traj = {}  # id → list of (t, s, d, vs, is_static)
for topic, msg, t in bag.read_messages(topics=['/tracking/obstacles_truth']):
    for o in msg.obstacles:
        oid = getattr(o, 'id', 0)
        if oid not in obs_traj:
            obs_traj[oid] = []
        obs_traj[oid].append((t.to_sec(), o.s_center, o.d_center, o.vs, o.is_static))

bag.close()
print(f'ego ticks: {len(ego_traj)}, obstacle ids: {list(obs_traj.keys())}')

# Track length (lap-wrap 처리용) — 추정
track_length = 85.0  # gazebo_wall_2

def signed_ds(s1, s2, tl=track_length):
    ds = (s1 - s2) % tl
    if ds > 0.5 * tl: ds -= tl
    return ds

# ego_traj 의 각 tick 에 대해 가장 가까운 obs sample 찾고 거리 계산
print(f'\nComputing ego ↔ obstacle distance per tick (threshold={threshold}m for collision)...')
collisions = []
near_misses = []  # within 0.6m
min_dist_overall = 1e9

for et, es, ed, ev in ego_traj:
    for oid, otr in obs_traj.items():
        # find closest obs sample in time
        best = min(otr, key=lambda x: abs(x[0] - et))
        ot, os_, od, ovs, ois = best
        if abs(ot - et) > 0.2:  # time mismatch > 200ms — skip
            continue
        ds = signed_ds(es, os_)
        dd = ed - od
        d = math.hypot(ds, dd)
        if d < min_dist_overall:
            min_dist_overall = d
        if d < threshold:
            collisions.append((et, oid, ds, dd, d, ois))
        elif d < 0.6:
            near_misses.append((et, oid, ds, dd, d, ois))

print(f'\nMin frenet distance overall: {min_dist_overall:.3f}m')
print(f'Collisions (dist < {threshold}m): {len(collisions)}')
print(f'Near-misses (0.4 ≤ dist < 0.6m): {len(near_misses)}')

# Group collisions into events (consecutive within 0.5s)
def group_events(arr):
    if not arr: return []
    arr_sorted = sorted(arr, key=lambda x: x[0])
    events = []
    cur = [arr_sorted[0]]
    for x in arr_sorted[1:]:
        if x[0] - cur[-1][0] < 0.5:
            cur.append(x)
        else:
            events.append(cur); cur = [x]
    events.append(cur)
    return events

ce = group_events(collisions)
nme = group_events(near_misses)
print(f'\nCollision events: {len(ce)}, Near-miss events: {len(nme)}')

# Show collision events
for i, ev in enumerate(ce[:8]):
    ts = ev[0][0]; te = ev[-1][0]
    samples = ev
    print(f'\n  Collision Event #{i}: t=[{ts:.2f}-{te:.2f}] dur={te-ts:.2f}s, msgs={len(ev)}')
    for s in samples[:3]:
        et, oid, ds, dd, d, ois = s
        print(f'    t={et:.2f} obs_id={oid} ds={ds:+.3f} dd={dd:+.3f} dist={d:.3f}m static={ois}')

# Near-miss events sample
for i, ev in enumerate(nme[:5]):
    ts = ev[0][0]; te = ev[-1][0]
    print(f'\n  Near-Miss Event #{i}: t=[{ts:.2f}-{te:.2f}] dur={te-ts:.2f}s, msgs={len(ev)}')
    s = ev[0]
    et, oid, ds, dd, d, ois = s
    print(f'    sample: ds={ds:+.3f} dd={dd:+.3f} dist={d:.3f}m')
