#!/usr/bin/env python3
"""Inspect obstacle source — is it really 2 obstacles, or prediction snapshots?"""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-07-40-34.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()

# Sample at the side flip ticks
sample_t = [2.008, 2.229, 2.467, 3.140, 3.430, 4.084, 5.096, 5.457]

print("=== obs_tag, n_obs_used, n_obs_raw, all obstacles ===\n")
for st in sample_t:
    for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
        rel = t.to_sec() - t0
        if abs(rel - st) > 0.020:
            continue
        try:
            d = json.loads(msg.data)
        except Exception:
            continue
        print(f"t={rel:.3f} tick={d.get('tick')}")
        print(f"  obs_tag={d.get('obs_tag')}  n_obs_used={d.get('n_obs_used')}  n_obs_raw={d.get('n_obs_raw')}")
        obs = d.get('obstacles', [])
        for i, ob in enumerate(obs):
            print(f"    [{i}] s0={ob.get('s0'):.3f} n0={ob.get('n0'):.3f} sN={ob.get('sN'):.3f} nN={ob.get('nN'):.3f}")
        print()
        break

# Also count raw prediction msgs around 4.0s
print("\n=== /opponent_prediction/obstacles raw at t≈4.0 ===")
for topic, msg, t in b.read_messages(
        topics=["/opponent_prediction/obstacles"],
        start_time=__import__('rospy').Time(t0 + 3.95),
        end_time=__import__('rospy').Time(t0 + 4.10)):
    rel = t.to_sec() - t0
    n = len(msg.obstacles) if hasattr(msg, 'obstacles') else 0
    print(f"  t={rel:.3f} obstacles count={n}")
    if n > 0:
        for i, ob in enumerate(msg.obstacles[:25]):
            sid = getattr(ob, 'id', None)
            sst = ob.header.stamp.to_sec() - t0 if hasattr(ob, 'header') else None
            sc = float(getattr(ob, 's_center', 0.0))
            dc = float(getattr(ob, 'd_center', 0.0))
            vs = float(getattr(ob, 'vs', 0.0) or 0.0)
            vd = float(getattr(ob, 'vd', 0.0) or 0.0)
            sz = float(getattr(ob, 'size', 0.0) or 0.0)
            stt = bool(getattr(ob, 'is_static', False))
            print(f"    [{i}] id={sid} stamp_rel={sst} s={sc:.2f} d={dc:.3f} vs={vs:.2f} vd={vd:.3f} size={sz:.2f} static={stt}")
    break

# Same for tracking obstacles
print("\n=== /perception/obstacles raw at t≈4.0 ===")
for topic, msg, t in b.read_messages(
        topics=["/perception/obstacles"],
        start_time=__import__('rospy').Time(t0 + 3.95),
        end_time=__import__('rospy').Time(t0 + 4.10)):
    rel = t.to_sec() - t0
    n = len(msg.obstacles) if hasattr(msg, 'obstacles') else 0
    print(f"  t={rel:.3f} obstacles count={n}")
    for i, ob in enumerate(msg.obstacles[:5]):
        sid = getattr(ob, 'id', None)
        sc = float(getattr(ob, 's_center', 0.0))
        dc = float(getattr(ob, 'd_center', 0.0))
        sz = float(getattr(ob, 'size', 0.0) or 0.0)
        stt = bool(getattr(ob, 'is_static', False))
        print(f"    [{i}] id={sid} s={sc:.2f} d={dc:.3f} size={sz:.2f} static={stt}")
    break

b.close()
