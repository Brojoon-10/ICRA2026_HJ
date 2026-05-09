#!/usr/bin/env python3
"""Full solver input + trajectory probe for each fail event."""
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
fails = [d for d in ticks if d.get('status') == 'CONVERGENCE_QUINTIC']
print(f'fails: {len(fails)}')

for i, ev in enumerate(fails):
    rel = ev['_t'] - t0
    e = ev.get('ego', {})
    print(f'\n========== fail #{i+1}  t={rel:.2f}s  ego.s={e.get("s"):.2f}  ego.n={e.get("n"):+.3f}  v={e.get("v"):.2f}  ==========')
    print(f'  side={ev.get("side")}  plan={ev.get("plan")}')
    print(f'  ipopt_status={ev.get("ipopt_status")}')

    # solver_input
    si = ev.get('solver_input', {}) or {}
    print(f'  solver_input keys: {list(si.keys())[:30]}')
    for k in ['v0', 'n0', 'n0_clamped', 'mu0', 'de0_active',
              'nlb_min', 'nub_max', 'side_effective',
              'ref_v_target', 'wall_safe', 'inflation', 'q_n', 'gamma']:
        if k in si:
            v = si[k]
            print(f'    {k}: {v}')

    # solver_infeas
    sinf = ev.get('solver_infeas', {}) or {}
    print(f'  solver_infeas: {sinf}')

    # solver_pass_hist
    hist = ev.get('solver_pass_hist') or []
    print(f'  pass history:')
    for p in hist:
        print(f'    Pass {p.get("pass")}: {p.get("status")} worst={p.get("worst")}')

    # Pass 3 success or fail
    print(f'  final solver_pass: {ev.get("solver_pass")}')

    # weights at the time
    w = ev.get('weights', {})
    print(f'  weights: q_n={w.get("q_n")} q_n_term={w.get("q_n_term")} '
          f'q_n_ramp={w.get("q_n_ramp")} gamma={w.get("gamma_progress")} '
          f'w_obs={w.get("w_obs")} w_side_bias={w.get("w_side_bias")} '
          f'w_slack={w.get("w_slack")} w_wall_buf={w.get("w_wall_buf")}')

    # trajectory
    traj = ev.get('trajectory', {})
    print(f'  trajectory: n_min={traj.get("n_min")}  n_max={traj.get("n_max")}  '
          f'kappa_max={traj.get("kappa_max")} margin_L={traj.get("margin_L_min")} margin_R={traj.get("margin_R_min")}')

    # ref
    r = ev.get('ref', {})
    print(f'  ref: corridor_width_mean={r.get("corridor_width_mean")} v_mean={r.get("v_mean")}')

    # obs
    obs_list = ev.get('obstacles') or []
    for j, o in enumerate(obs_list):
        s_seq = o.get('s_seq')
        n_seq = o.get('n_seq')
        print(f'  obs[{j}]: s_seq={s_seq}')
        print(f'  obs[{j}]: n_seq={n_seq}')
        print(f'  obs[{j}]: w={o.get("w")}')
