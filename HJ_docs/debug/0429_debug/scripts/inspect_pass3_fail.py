#!/usr/bin/env python3
"""For each Infeasible event, print full pass history + obs info."""
import sys, json
import rosbag

bag = rosbag.Bag(sys.argv[1], 'r')
ticks = []
for topic, msg, t in bag.read_messages(topics=['/mpc_auto/debug/tick_json']):
    try:
        d = json.loads(msg.data)
        d['_t'] = t.to_sec()
        ticks.append(d)
    except Exception:
        pass
bag.close()

t0 = ticks[0]['_t']
failing = [d for d in ticks if d.get('ipopt_status') == 'Infeasible_Problem_Detected']
print(f'total infeas events: {len(failing)}')

for i, ev in enumerate(failing[:8]):
    rel = ev['_t'] - t0
    plan_name = ev.get('plan')
    side = ev.get('side')
    print(f'\n--- event #{i+1}  t={rel:6.2f}s  plan={plan_name}  side={side} ---')
    hist = ev.get('solver_pass_hist') or []
    for p in hist:
        ps = p.get('pass')
        st = p.get('status')
        wo = p.get('worst')
        print(f'  Pass {ps}: status={st}  worst={wo}')
    si = ev.get('solver_infeas') or {}
    print(f'  final infeas: worst={si.get("worst")}  worst_val={si.get("worst_val")}  slack_peak={si.get("slack_peak")}')
    print(f'  ego: {ev.get("ego")}')
    obs = ev.get('obstacles') or []
    if obs:
        o = obs[0]
        ns = o.get('n_seq')
        ss = o.get('s_seq')
        print(f'  obs s_seq: {ss}')
        print(f'  obs n_seq: {ns}')
    traj = ev.get('trajectory') or {}
    ref = ev.get('ref') or {}
    print(f'  trajectory n_min={traj.get("n_min")} n_max={traj.get("n_max")} '
          f'kappa_max={traj.get("kappa_max")} margin_L={traj.get("margin_L_min")} '
          f'margin_R={traj.get("margin_R_min")}')
    print(f'  ref corridor_width_mean={ref.get("corridor_width_mean")} v_mean={ref.get("v_mean")}')
    weights = ev.get('weights') or {}
    print(f'  weights: w_obs={weights.get("w_obs")} w_side_bias={weights.get("w_side_bias")} '
          f'q_n={weights.get("q_n")} q_n_term={weights.get("q_n_term")}')
