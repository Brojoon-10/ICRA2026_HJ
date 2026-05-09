#!/usr/bin/env python3
import rosbag, json
BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-05-00-55.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()

# Time window expanded
WIN_LO, WIN_HI = 25.0, 30.0
tier_count = {}
status_count = {}
ipopt_count = {}
prev_status = None
transitions = []
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    if not (WIN_LO <= rel <= WIN_HI): continue
    try: d = json.loads(msg.data)
    except: continue
    tier = d.get('tier'); status = d.get('status'); ipopt = d.get('ipopt_status')
    tier_count[tier] = tier_count.get(tier, 0) + 1
    status_count[status] = status_count.get(status, 0) + 1
    ipopt_count[ipopt] = ipopt_count.get(ipopt, 0) + 1
    if status != prev_status:
        transitions.append((rel, tier, status, ipopt, d.get('fail_streak'),
                           d.get('ego', {}).get('n'), d.get('ego', {}).get('v')))
        prev_status = status
print(f"=== window {WIN_LO}-{WIN_HI}s ===")
print(f"tier counts: {sorted(tier_count.items())}")
print(f"status counts: {sorted(status_count.items())}")
print(f"ipopt counts: {sorted(ipopt_count.items())}")
print(f"\n=== status transitions ===")
for tr in transitions:
    print(f"  t={tr[0]:.3f}  tier={tr[1]}  status={tr[2]:30}  ipopt={tr[3]:30}  fs={tr[4]}  ego_n={tr[5]}  v={tr[6]}")
b.close()
