#!/usr/bin/env python3
"""Analyze bag where tier-0 succeeded but trajectory punches through wall."""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-04-10-10.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()

print("=== tick_json scan: tier=0 with concerning n_traj_max / margin_min / ego_n ===\n")
hits = []
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    try:
        d = json.loads(msg.data)
    except Exception:
        continue
    tier = d.get('tier')
    if tier != 0:
        continue
    ego = d.get('ego', {})
    ego_n = ego.get('n')
    traj_stats = d.get('trajectory', {})
    si = d.get('solver_input', {})
    nub_max = si.get('nub_max')
    nlb_min = si.get('nlb_min')
    n_max = traj_stats.get('n_max')
    n_min = traj_stats.get('n_min')
    a4 = d.get('a4_wall_ramp', {})

    # Identify cases where trajectory is past corridor
    bad = False
    if n_max is not None and nub_max is not None and n_max > nub_max + 0.02:
        bad = True
    if n_min is not None and nlb_min is not None and n_min < nlb_min - 0.02:
        bad = True
    if ego_n is not None and abs(ego_n) > 0.30:
        bad = True

    if bad:
        hits.append({
            't': rel,
            'tick': d.get('tick'),
            'ego_n': ego_n,
            'mode': d.get('mpc_mode'),
            'side': d.get('side'),
            'n_max': n_max,
            'n_min': n_min,
            'nub_max': nub_max,
            'nlb_min': nlb_min,
            'slack_max': d.get('slack_max'),
            'wall_ramp_active': a4.get('active'),
            'wall_ramp_K_entry': a4.get('K_entry'),
            'a1_vmax': d.get('a1_vmax', {}),
            'cost': d.get('cost', {}),
        })

if not hits:
    print("No tier-0 tick exceeded corridor by >2cm or had |ego_n|>0.30.")
else:
    print(f"Found {len(hits)} tier-0 ticks with wall concerns.\n")
    # Print first 5 and around
    for h in hits[:8]:
        print(f"t={h['t']:.3f} tick={h['tick']} mode={h['mode']} side={h['side']}")
        print(f"  ego_n={h['ego_n']}  n_traj=[{h['n_min']},{h['n_max']}]  corridor=[{h['nlb_min']},{h['nub_max']}]")
        print(f"  slack_max={h['slack_max']}  wall_ramp_active={h['wall_ramp_active']} K_entry={h['wall_ramp_K_entry']}")
        print(f"  a1_vmax: k0={h['a1_vmax'].get('k0')} k_end={h['a1_vmax'].get('k_end')} mono={h['a1_vmax'].get('monotone_dec')}")
        c = h['cost']
        wall_c = c.get('wall', '?'); slack_c = c.get('slack', '?'); contour_c = c.get('contour', '?')
        print(f"  cost: contour={contour_c} wall={wall_c} slack={slack_c}")
        print()

# Also: scan n_traj across all ticks to find peak excursion
print("\n=== max n_traj excursion over corridor (any tier) ===")
peak = 0.0; peak_rec = None
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    try:
        d = json.loads(msg.data)
    except Exception:
        continue
    si = d.get('solver_input', {})
    traj = d.get('trajectory', {})
    n_max = traj.get('n_max'); n_min = traj.get('n_min')
    nub_max = si.get('nub_max'); nlb_min = si.get('nlb_min')
    over = 0.0
    if n_max is not None and nub_max is not None:
        over = max(over, n_max - nub_max)
    if n_min is not None and nlb_min is not None:
        over = max(over, nlb_min - n_min)
    if over > peak:
        peak = over
        peak_rec = (rel, d.get('tick'), d.get('tier'), d.get('mpc_mode'),
                    n_min, n_max, nlb_min, nub_max,
                    d.get('a4_wall_ramp', {}).get('active'),
                    d.get('cost', {}).get('wall'),
                    d.get('cost', {}).get('slack'))
print(f"Peak n_traj corridor overshoot: {peak:.3f} m")
if peak_rec:
    print(f"  at t={peak_rec[0]:.3f} tick={peak_rec[1]} tier={peak_rec[2]} mode={peak_rec[3]}")
    print(f"  n_traj=[{peak_rec[4]},{peak_rec[5]}] corridor=[{peak_rec[6]},{peak_rec[7]}]")
    print(f"  wall_ramp_active={peak_rec[8]}  wall_cost={peak_rec[9]} slack_cost={peak_rec[10]}")

b.close()
