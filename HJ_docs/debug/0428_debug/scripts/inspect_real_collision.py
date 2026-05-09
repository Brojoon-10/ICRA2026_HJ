#!/usr/bin/env python3
"""Real collision detection 검증 — /opponent_collision + /opponent_dist 시계열."""
import sys
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

# /opponent_dist 시계열 (1초 간격 sample)
print('\n=== /opponent_dist 시계열 (5초 간격) ===')
last_log = -1e9
dist_unique = set()
for topic, msg, t in bag.read_messages(topics=['/opponent_dist']):
    dist_unique.add(round(msg.data, 2))
    if t.to_sec() - last_log < 5:
        continue
    last_log = t.to_sec()
    print(f'  t={t.to_sec():.2f}  dist={msg.data:.3f}')
print(f'\n  unique values total: {len(dist_unique)}')
print(f'  values: {sorted(dist_unique)[:20]}')

# /opponent_collision 시계열 (True/False 변화 시점)
print('\n=== /opponent_collision True 발생 시점 ===')
prev = None
n_true = 0
n_total = 0
for topic, msg, t in bag.read_messages(topics=['/opponent_collision']):
    n_total += 1
    if msg.data:
        n_true += 1
    if msg.data != prev:
        print(f'  t={t.to_sec():.2f}  collision={msg.data}')
        prev = msg.data
print(f'\n  total msgs: {n_total}, True: {n_true}')

# /collision_marker 와 /opponent_collision 의 시점 비교
print('\n=== /collision_marker 발생 시점 vs /opponent_collision ===')
n_cm = 0
cm_first = None
cm_last = None
for topic, msg, t in bag.read_messages(topics=['/collision_marker']):
    n_cm += 1
    if cm_first is None: cm_first = t.to_sec()
    cm_last = t.to_sec()
print(f'  collision_marker total: {n_cm}, first={cm_first}, last={cm_last}')

# /tracking/obstacles_truth count + 변화
print('\n=== /tracking/obstacles_truth — sample 시계열 (10초 간격) ===')
last_log = -1e9
n_total = 0
for topic, msg, t in bag.read_messages(topics=['/tracking/obstacles_truth']):
    n_total += 1
    if t.to_sec() - last_log < 10:
        continue
    last_log = t.to_sec()
    if msg.obstacles:
        o = msg.obstacles[0]
        print(f'  t={t.to_sec():.2f}  id={o.id} s={o.s_center:.2f} d={o.d_center:+.3f} '
              f'is_static={o.is_static} vs={o.vs:.3f}')
    else:
        print(f'  t={t.to_sec():.2f}  EMPTY')
print(f'  total msgs: {n_total}')

# ego_s 시계열 (10s 간격)
print('\n=== ego_s 시계열 (10초 간격) ===')
last_log = -1e9
for topic, msg, t in bag.read_messages(topics=['/car_state/odom_frenet']):
    if t.to_sec() - last_log < 10:
        continue
    last_log = t.to_sec()
    print(f'  t={t.to_sec():.2f}  s={msg.pose.pose.position.x:.2f} v={msg.twist.twist.linear.x:.2f}')

bag.close()
