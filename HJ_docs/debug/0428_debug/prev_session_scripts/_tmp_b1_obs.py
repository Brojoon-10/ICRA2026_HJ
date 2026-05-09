#!/usr/bin/env python3
"""Bag 1: how many obstacles total? Check tracking + tick_json."""
import rosbag, json
BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-13-12-51.bag"
b = rosbag.Bag(BAG); t0 = b.get_start_time()

# tracking obstacles unique IDs
print("=== /tracking/obstacles unique IDs ===")
id_data = {}
for topic, msg, t in b.read_messages(topics=["/tracking/obstacles"]):
    rel = t.to_sec() - t0
    for o in msg.obstacles:
        oid = getattr(o, 'id', -1)
        if oid not in id_data:
            id_data[oid] = []
        id_data[oid].append((rel, float(o.s_center), float(o.d_center), bool(getattr(o, 'is_static', True))))
for oid, samples in sorted(id_data.items()):
    s_arr = [x[1] for x in samples]
    d_arr = [x[2] for x in samples]
    static_first = samples[0][3]
    print(f"  id={oid}: count={len(samples)}, s=[{min(s_arr):.2f},{max(s_arr):.2f}], "
          f"d=[{min(d_arr):+.3f},{max(d_arr):+.3f}], is_static={static_first}, "
          f"first_t={samples[0][0]:.2f}, last_t={samples[-1][0]:.2f}")

# n_obs distribution from tick_json
print("\n=== tick_json n_obs distribution ===")
counts = {}
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    try: d = json.loads(msg.data)
    except: continue
    key = (d.get('n_obs_raw',0), d.get('n_obs_used',0))
    counts[key] = counts.get(key, 0) + 1
for k, c in sorted(counts.items()):
    print(f"  raw={k[0]} used={k[1]}: {c} ticks")
b.close()
