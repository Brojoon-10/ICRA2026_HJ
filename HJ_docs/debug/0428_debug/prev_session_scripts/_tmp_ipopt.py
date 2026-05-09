#!/usr/bin/env python3
"""IPOPT status + infeas info at NLP failure ticks."""
import rosbag, json
BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-10-55-57.bag"
b = rosbag.Bag(BAG); t0 = b.get_start_time()
recs = []
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    try: d = json.loads(msg.data)
    except: continue
    recs.append((rel, d))

# IPOPT status / infeas at each failure
print("=== ipopt_status / infeas / pass_history at tier>=1 ===")
for rel, d in recs:
    tier = d.get('tier', 0)
    if tier < 1: continue
    ipopt = d.get('ipopt_status')
    si = d.get('solver_input') or {}
    sf = d.get('solver_infeas') or {}
    sp = d.get('solver_pass')
    sph = d.get('solver_pass_hist')
    side_eff = si.get('side_effective')
    nlb_min = si.get('nlb_min'); nub_max = si.get('nub_max')
    nlb_0 = si.get('nlb_0'); nub_0 = si.get('nub_0')
    cw = si.get('corridor_min_width')
    vmin = si.get('vmax_min'); vmax = si.get('vmax_max')
    n0 = si.get('n0'); v0 = si.get('v0')
    print(f"t={rel:.3f} tier={tier} pass={sp}/{sph} ipopt={ipopt!r:30}")
    print(f"   side_eff={side_eff} v0={v0} vmax_min={vmin} vmax_max={vmax}")
    print(f"   n0={n0} nub_0={nub_0} nlb_0={nlb_0} corridor_min_w={cw}")
    if sf:
        print(f"   infeas: {sf}")
b.close()
