#!/usr/bin/env python3
"""Deeper inspection of tier-0 wall-punch case."""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-04-10-10.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()

print("=== inspect tick at t=3.752 (peak overshoot 0.056m past corridor) ===\n")
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    if not (3.7 <= rel <= 3.8):
        continue
    try:
        d = json.loads(msg.data)
    except Exception:
        continue
    print(f"--- t={rel:.3f}  tick={d.get('tick')} tier={d.get('tier')} ipopt={d.get('ipopt_status')} ---")
    print(f"  ego: {d.get('ego')}")
    print(f"  side: {d.get('side')}, side_scores: {d.get('side_scores')}")
    print(f"  trajectory: {d.get('trajectory')}")
    si = d.get('solver_input', {})
    print(f"  solver_input: pass={si.get('pass')} side_eff={si.get('side_effective')} "
          f"n0={si.get('n0')} n0c={si.get('n0_clamped')} mu0={si.get('mu0')} v0={si.get('v0')} "
          f"de0={si.get('de0_locked')} de0_act={si.get('de0_active')} "
          f"nlb_min={si.get('nlb_min')} nub_max={si.get('nub_max')}")
    print(f"  solver_infeas: {d.get('solver_infeas')}")
    print(f"  cost: {d.get('cost')}")
    print(f"  weights snapshot: {d.get('weights')}")
    print(f"  ego_outside_corridor: {d.get('ego_outside_corridor')}")
    a1 = d.get('a1_vmax', {})
    a4 = d.get('a4_wall_ramp', {})
    print(f"  a1_vmax: {a1}")
    print(f"  a4_wall_ramp: {a4}")
    print(f"  slack_max: {d.get('slack_max')}")
    print(f"  warm_used: {d.get('warm_used')}")
    print()

# Also check first early-failure tick (t=0.95)
print("\n=== inspect tick at t=0.95 (ego_n=0.92 with slack_max=99999) ===\n")
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    if not (0.94 <= rel <= 0.96):
        continue
    try:
        d = json.loads(msg.data)
    except Exception:
        continue
    print(f"--- t={rel:.3f}  tick={d.get('tick')} tier={d.get('tier')} ipopt={d.get('ipopt_status')} ---")
    print(f"  ego: {d.get('ego')}")
    print(f"  trajectory: {d.get('trajectory')}")
    si = d.get('solver_input', {})
    print(f"  solver_input: pass={si.get('pass')} n0={si.get('n0')} n0c={si.get('n0_clamped')} "
          f"mu0={si.get('mu0')} v0={si.get('v0')} "
          f"nlb_min={si.get('nlb_min')} nub_max={si.get('nub_max')}")
    print(f"  solver_infeas: {d.get('solver_infeas')}")
    print(f"  cost: {d.get('cost')}")
    print(f"  weights: {d.get('weights')}")
    print(f"  slack_max: {d.get('slack_max')}")
    print(f"  iter: {d.get('iter')}")
    print(f"  ego_outside_corridor: {d.get('ego_outside_corridor')}")
    a4 = d.get('a4_wall_ramp', {})
    print(f"  a4_wall_ramp: {a4}")
    break

b.close()
