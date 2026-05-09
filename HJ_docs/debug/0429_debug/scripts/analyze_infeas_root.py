#!/usr/bin/env python3
"""For each Infeasible_Problem_Detected event, dump the solver state at
that tick: pass history, infeas info, ego/obs geometry, side decision
just before, etc. Goal — pinpoint the actual NLP failure cause.
"""
import sys, json
import numpy as np
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

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
print(f'=== {bag_path.split("/")[-1]} ===  ticks={len(ticks)}')

infeas_ticks = [t for t in ticks if t.get('ipopt_status') == 'Infeasible_Problem_Detected']
print(f'  infeas events: {len(infeas_ticks)}')

print('\n=== infeas event detail ===')
for i, ev in enumerate(infeas_ticks):
    rel = ev['_t'] - t0
    print(f'\n--- event #{i+1}  t={rel:6.2f}s  tier={ev.get("tier")}  status={ev.get("status")} ---')
    print(f'  plan        : {ev.get("plan")}')
    print(f'  side        : {ev.get("side")}')
    print(f'  side_scores : {ev.get("side_scores")}')
    print(f'  mpc_mode    : {ev.get("mpc_mode")}')
    print(f'  ego         : {ev.get("ego")}')
    print(f'  u0          : {ev.get("u0")}')
    print(f'  solver_pass : {ev.get("solver_pass")}  hist={ev.get("solver_pass_hist")}')
    print(f'  solver_infeas: {ev.get("solver_infeas")}')
    si = ev.get('solver_input', {})
    if si:
        # solver_input 에 어떤 필드 있는지 sample 확인
        keys = list(si.keys())[:10]
        print(f'  solver_input keys (sample): {keys}')
        # 핵심 필드만
        for k in ['v0', 'n0', 'mu0', 'delta0', 'ref_v_target', 'corridor_width_min',
                  'd_left_min', 'd_right_min', 'wall_safe', 'side_sign', 'q_n',
                  'q_mu', 'q_v', 'w_obs', 'sigma_n_obs', 'obs_count',
                  'obs_s_min', 'obs_s_max', 'obs_n_min', 'obs_n_max']:
            if k in si:
                v = si[k]
                if isinstance(v, list) and len(v) > 5:
                    v = f'[{v[0]:.3f}..{v[-1]:.3f}, n={len(v)}]'
                print(f'    {k}: {v}')

    obs = ev.get('obstacles') or []
    print(f'  n_obs_used  : {ev.get("n_obs_used")}')
    for j, o in enumerate(obs[:2]):
        print(f'  obs[{j}]    : s0={o.get("s0"):.2f}  n0={o.get("n0"):+.3f}  '
              f'sN={o.get("sN"):.2f}  nN={o.get("nN"):+.3f}  midN_s={o.get("midN_s"):.2f}')
        if 'n_seq' in o:
            print(f'    n_seq    : {o.get("n_seq")}')

    print(f'  obs_in_horizon: {ev.get("obs_in_horizon")}  min_obs_ds={ev.get("min_obs_ds")}')
    print(f'  ttc_min       : {ev.get("ttc_min")}')
    print(f'  trajectory    : {ev.get("trajectory")}')
    print(f'  ref           : {ev.get("ref")}')
    pred_var = ev.get('pred_variance', {}) or {}
    print(f'  pred_variance : {pred_var}')
    print(f'  ego_outside_corridor: {ev.get("ego_outside_corridor")}')
    print(f'  q_n_near_boost: {ev.get("q_n_near_boost_applied")}')
    print(f'  weights       : {ev.get("weights")}')

    # 직전 tick 의 plan/side 비교 — toggle 직후인지
    idx_in_all = ticks.index(ev)
    if idx_in_all > 0:
        prev = ticks[idx_in_all - 1]
        prev_rel = prev['_t'] - t0
        print(f'  PREV tick (t={prev_rel:6.2f}, Δ={1000*(ev["_t"]-prev["_t"]):.0f}ms): '
              f'plan={prev.get("plan")}  side={prev.get("side")}  status={prev.get("status")}')
print('\n=== DONE ===')
