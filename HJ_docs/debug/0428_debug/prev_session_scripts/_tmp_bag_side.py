#!/usr/bin/env python3
"""Analyze SideDecider behavior in 2026-04-27-07-40-34.bag from 2s."""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-07-40-34.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()

print(f"bag: duration={b.get_end_time()-t0:.1f}s\n")

# Find first OT scenario (side != clear) from 2s
WIN_LO, WIN_HI = 2.0, 25.0

prev_side = None
flips = []
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    if not (WIN_LO <= rel <= WIN_HI):
        continue
    try:
        d = json.loads(msg.data)
    except Exception:
        continue
    side = d.get("side")
    sc = d.get("side_scores", {}) or {}
    ego = d.get("ego", {}) or {}
    obs_list = d.get("obstacles", []) or []
    if not isinstance(sc, dict):
        sc = {}
    if side != prev_side:
        flips.append({
            't': rel,
            'tick': d.get('tick'),
            'side': side,
            'reason': sc.get('reason'),
            'ego_n': ego.get('n'),
            'ego_v': ego.get('v'),
            'n_o': sc.get('n_o'),
            'd_free_L': sc.get('d_free_L'),
            'd_free_R': sc.get('d_free_R'),
            'can_pass_L': sc.get('can_pass_L'),
            'can_pass_R': sc.get('can_pass_R'),
            'dv': sc.get('dv'),
            'mpc_mode': d.get('mpc_mode'),
            'n_obs_used': d.get('n_obs_used'),
            'obstacles': obs_list[:3],   # first few
        })
        prev_side = side

print(f"=== side flips in window {WIN_LO}-{WIN_HI}s ({len(flips)} total) ===\n")
for f in flips[:15]:
    print(f"t={f['t']:.3f} tick={f['tick']} → side={f['side']!r} reason={f['reason']!r}")
    print(f"  ego: n={f['ego_n']} v={f['ego_v']}")
    print(f"  obstacle n_o={f['n_o']} d_free_L={f['d_free_L']} d_free_R={f['d_free_R']}")
    print(f"  can_pass_L={f['can_pass_L']} can_pass_R={f['can_pass_R']} dv={f['dv']}")
    print(f"  mode={f['mpc_mode']} n_obs={f['n_obs_used']}")
    if f['obstacles']:
        for ob in f['obstacles']:
            print(f"    obs: s0={ob.get('s0')} n0={ob.get('n0')} sN={ob.get('sN')} nN={ob.get('nN')}")
    print()

# Also check the ref_slice d_left/d_right at each flip — need solver_input
print("\n=== solver_input nlb/nub at flips ===")
for f in flips[:8]:
    # Find tick_json msg at exact time
    for topic, msg, t in b.read_messages(
            topics=["/mpc_auto/debug/tick_json"],
            start_time=__import__('rospy').Time(t0 + f['t'] - 0.005),
            end_time=__import__('rospy').Time(t0 + f['t'] + 0.005)):
        try:
            d = json.loads(msg.data)
        except Exception:
            continue
        if d.get('tick') != f['tick']:
            continue
        si = d.get('solver_input', {})
        ref = d.get('ref', {})
        print(f"t={f['t']:.3f} side={f['side']!r}")
        print(f"  solver_input: nlb_min={si.get('nlb_min')} nub_max={si.get('nub_max')}")
        print(f"  ref: dL_min={ref.get('dL_min')} dR_min={ref.get('dR_min')}")
        break

b.close()
