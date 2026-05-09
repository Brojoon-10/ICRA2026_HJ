#!/usr/bin/env python3
"""Inspect obstacle propagation and ego state at the failure moment."""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-25-21-17-45.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()

for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    if 49.65 <= rel <= 50.0:
        try:
            d = json.loads(msg.data)
        except Exception:
            continue
        ipopt = d.get("ipopt_status", "")
        if "Solve_Succeeded" in ipopt and rel < 49.8:
            continue   # focus on fails + nearby
        print(f"=== t={rel:.3f} ipopt={ipopt} solve_ms={d.get('solve_ms')} ===")
        ego = d.get("ego", {})
        print(f"  ego: n0={ego.get('n0')}  mu0={ego.get('mu0')}  v0={ego.get('v0')}  s={ego.get('s')}")
        print(f"  ego_outside_corridor: {d.get('ego_outside_corridor')}")
        ref = d.get("ref", {})
        if isinstance(ref, dict):
            print(f"  ref: dL[0]={ref.get('dL0')}  dR[0]={ref.get('dR0')}  kappa0={ref.get('kappa0')}")
            print(f"       dL_min={ref.get('dL_min')}  dR_min={ref.get('dR_min')}")
        obs = d.get("obstacles", [])
        if isinstance(obs, list) and obs:
            for i, ob in enumerate(obs):
                print(f"  obs[{i}]: {ob}")
        else:
            print(f"  obs: (none / {type(obs).__name__})")
        sin = d.get("solver_input", {})
        if isinstance(sin, dict):
            print(f"  solver_input keys: {list(sin.keys())[:10]}")
            for k in ('n0', 'mu0', 'v0', 'de0', 'side', 'bias_scale'):
                if k in sin:
                    print(f"    {k}: {sin[k]}")
            # if 'corridor' in sin: dump head
            if 'nlb' in sin:
                v = sin['nlb']
                if isinstance(v, list):
                    print(f"    nlb (lower-corridor) [head]: {[round(x,3) for x in v[:6]]}  [tail]: {[round(x,3) for x in v[-3:]]}")
            if 'nub' in sin:
                v = sin['nub']
                if isinstance(v, list):
                    print(f"    nub (upper-corridor) [head]: {[round(x,3) for x in v[:6]]}  [tail]: {[round(x,3) for x in v[-3:]]}")
        ssh = d.get("solver_pass_hist", None)
        if ssh is not None:
            print(f"  solver_pass_hist: {ssh}")
        infeas = d.get("solver_infeas", None)
        if infeas is not None:
            print(f"  solver_infeas: {infeas}")
        cost = d.get("cost", {})
        if isinstance(cost, dict):
            print(f"  cost: {cost}")
        print()
b.close()
