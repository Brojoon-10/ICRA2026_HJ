#!/usr/bin/env python3
"""Analyze 2026-04-27-04-31-34.bag — tier-0 wall-punch trajectories."""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-04-31-34.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()

# Find tier-0 ticks where trajectory MARGIN is negative (n past d_L or d_R)
print("=== tier-0 ticks with n_traj past actual wall (margin < 0) ===\n")
hits = []
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    try:
        d = json.loads(msg.data)
    except Exception:
        continue
    if d.get('tier') != 0:
        continue
    traj = d.get('trajectory', {})
    mLmin = traj.get('margin_L_min')
    mRmin = traj.get('margin_R_min')
    si = d.get('solver_input', {})
    a4 = d.get('a4_wall_ramp', {})
    a1 = d.get('a1_vmax', {})
    ego = d.get('ego', {})
    if mLmin is None and mRmin is None: continue
    bad = (mLmin is not None and mLmin < 0) or (mRmin is not None and mRmin < 0)
    if bad:
        hits.append({
            't': rel, 'tick': d.get('tick'),
            'mode': d.get('mpc_mode'),
            'side': d.get('side'),
            'ego': ego,
            'mL': mLmin, 'mR': mRmin,
            'n_traj': (traj.get('n_min'), traj.get('n_max'), traj.get('n_end')),
            'corridor': (si.get('nlb_min'), si.get('nub_max')),
            'v0': si.get('v0'),
            'mu0': si.get('mu0'),
            'de0': si.get('de0_locked'),
            'wall_ramp': (a4.get('active'), a4.get('K_entry')),
            'a1': (a1.get('k0'), a1.get('k_end'), a1.get('monotone_dec')),
            'slack_max': d.get('slack_max'),
            'iter': d.get('iter'),
            'cost': d.get('cost', {}),
            'ipopt': d.get('ipopt_status'),
        })

if not hits:
    print("None! tier-0 trajectories all stay inside actual walls.")
else:
    print(f"Found {len(hits)} tier-0 wall-punch ticks.\n")
    # Print first 5 cases representative
    for h in hits[:5]:
        c = h['cost']
        print(f"t={h['t']:.3f} tick={h['tick']} mode={h['mode']} side={h['side']}")
        print(f"  ego: n={h['ego'].get('n')} v={h['ego'].get('v')} psi={h['ego'].get('psi')}")
        print(f"  margin_L_min={h['mL']:.3f}  margin_R_min={h['mR']:.3f}  n_traj={h['n_traj']}")
        print(f"  corridor=[{h['corridor'][0]:.3f},{h['corridor'][1]:.3f}]")
        print(f"  v0={h['v0']:.3f} mu0={h['mu0']:.3f} de0={h['de0']}")
        print(f"  wall_ramp_active={h['wall_ramp'][0]} K_entry={h['wall_ramp'][1]}")
        print(f"  a1_vmax: k0={h['a1'][0]} k_end={h['a1'][1]} mono={h['a1'][2]}")
        print(f"  slack_max={h['slack_max']:.4f} iter={h['iter']} ipopt={h['ipopt']}")
        print(f"  cost: contour={c.get('contour')} wall={c.get('wall')} slack={c.get('slack')}")
        print()

    # Distribution stats
    import statistics
    mLs = [h['mL'] for h in hits if h['mL'] is not None]
    mRs = [h['mR'] for h in hits if h['mR'] is not None]
    v0s = [h['v0'] for h in hits if h['v0'] is not None]
    sl = [h['slack_max'] for h in hits if h['slack_max'] is not None]
    if mLs:
        print(f"\nmargin_L_min stats over {len(mLs)} hits: min={min(mLs):.3f} median={statistics.median(mLs):.3f}")
    if mRs:
        print(f"margin_R_min stats over {len(mRs)} hits: min={min(mRs):.3f} median={statistics.median(mRs):.3f}")
    if v0s:
        print(f"v0 stats: min={min(v0s):.3f} max={max(v0s):.3f} median={statistics.median(v0s):.3f}")
    if sl:
        print(f"slack_max stats: min={min(sl):.3f} max={max(sl):.3f} median={statistics.median(sl):.3f}")

b.close()
