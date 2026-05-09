#!/usr/bin/env python3
"""Sim 환경 검증: obstacle 동적/정적 + collision_marker 의 정확한 정보."""
import sys
import rosbag

bag_path = sys.argv[1] if len(sys.argv) > 1 else None
if bag_path is None:
    print('usage: inspect_obs_collision.py <bag>')
    sys.exit(1)

bag = rosbag.Bag(bag_path, 'r')

# 1. tracking_obstacles_truth — is_static / vs / vd 확인 (5 msgs sample)
print(f'\n=== tracking/obstacles_truth (5 sample msgs) ===')
n = 0
for topic, msg, t in bag.read_messages(topics=['/tracking/obstacles_truth']):
    if n >= 5: break
    print(f'  t={t.to_sec():.2f}:')
    for o in msg.obstacles:
        print(f'    id={getattr(o,"id","?")} s={getattr(o,"s_center",0):.2f} d={getattr(o,"d_center",0):+.3f} '
              f'is_static={getattr(o,"is_static","?")} '
              f'vs={getattr(o,"vs",0):.3f} vd={getattr(o,"vd",0):.3f} '
              f'size={getattr(o,"size",0):.3f}')
    n += 1

# 2. tracking/obstacles (live tracking) — 같은 정보
print(f'\n=== tracking/obstacles (5 sample msgs) ===')
n = 0
for topic, msg, t in bag.read_messages(topics=['/tracking/obstacles']):
    if n >= 5: break
    print(f'  t={t.to_sec():.2f}:')
    for o in msg.obstacles:
        print(f'    id={getattr(o,"id","?")} s={getattr(o,"s_center",0):.2f} d={getattr(o,"d_center",0):+.3f} '
              f'is_static={getattr(o,"is_static","?")} '
              f'vs={getattr(o,"vs",0):.3f} vd={getattr(o,"vd",0):.3f}')
    n += 1

# 3. collision_marker 의 정확한 정보
print(f'\n=== collision_marker (전체 + namespace/id/text) ===')
n = 0
for topic, msg, t in bag.read_messages(topics=['/collision_marker']):
    if n >= 10: break
    if hasattr(msg, 'markers'):
        for m in msg.markers[:3]:
            print(f'  t={t.to_sec():.2f} ns={getattr(m,"ns","?")} id={getattr(m,"id","?")} '
                  f'type={getattr(m,"type","?")} action={getattr(m,"action","?")} '
                  f'text="{getattr(m,"text","")}"')
    else:
        print(f'  t={t.to_sec():.2f} ns={getattr(msg,"ns","?")} id={getattr(msg,"id","?")} '
              f'type={getattr(msg,"type","?")} action={getattr(msg,"action","?")} '
              f'text="{getattr(msg,"text","")}"')
    n += 1

# 4. obstacle position 시계열 — 동적이면 s 가 lap 따라 이동, 정적이면 고정
print(f'\n=== obstacle s 시계열 (10s 간격) ===')
last_log = -100
for topic, msg, t in bag.read_messages(topics=['/tracking/obstacles_truth']):
    if t.to_sec() - last_log < 10:
        continue
    last_log = t.to_sec()
    if not msg.obstacles:
        continue
    o0 = msg.obstacles[0]
    print(f'  t={t.to_sec():.2f}  id={getattr(o0,"id","?")} s={o0.s_center:.2f} d={o0.d_center:+.3f} '
          f'is_static={o0.is_static} vs={o0.vs:.3f}')

bag.close()
