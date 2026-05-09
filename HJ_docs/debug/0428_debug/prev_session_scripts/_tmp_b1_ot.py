#!/usr/bin/env python3
"""Bag 1 first 4s: OT attempt before ego got stuck."""
import rosbag, json
BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-13-12-51.bag"
b = rosbag.Bag(BAG); t0 = b.get_start_time()
recs = []
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    try: d = json.loads(msg.data)
    except: continue
    recs.append((rel, d))

print("=== first 4s timeline (OT attempt) ===")
last_t = -1.0
for rel, d in recs:
    if rel > 4.5: break
    if rel - last_t < 0.15: continue
    last_t = rel
    ego = d.get('ego', {})
    si = d.get('solver_input', {})
    sc = d.get('side_scores', {})
    sph = d.get('solver_pass_hist', [])
    pass1_ok = sph[0]['ok'] if sph else None
    pass2_status = sph[1]['status'] if len(sph) > 1 else None
    obs = d.get('obstacles') or []
    obs_str = ','.join([f"({o.get('s0',0):.2f},{o.get('n0',0):+.3f})" for o in obs])
    print(f"t={rel:5.3f} ego(s={ego.get('s',0):6.2f},n={ego.get('n',0):+.3f},v={ego.get('v',0):.2f}) "
          f"side={d.get('side'):6}/se={si.get('side_effective')} tier={d.get('tier')} "
          f"pass1_ok={pass1_ok} pass2={str(pass2_status)[:25]:25} "
          f"nub_0={si.get('nub_0'):+.3f} nlb_0={si.get('nlb_0'):+.3f} "
          f"reason={sc.get('reason')} obs=[{obs_str}]")
b.close()
