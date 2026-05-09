#!/usr/bin/env python3
"""Pre-collision analysis: why did t=519.83 collision happen?
Window: t=518.0 to 521.4 (covers approach + collision + first fail)."""
import json
from collections import Counter

with open('/tmp/bag1_ticks.json') as f: B1 = json.load(f)

window = [t for t in B1 if 518.0 <= t['_t_bag'] <= 521.5]
print(f'Pre-collision window 518.0-521.5: {len(window)} ticks')

# Show full evolution: ego_s, ego_n, ego_v, side, pass, ipopt, obstacles
print('\n========== TIMELINE (every tick) ==========')
print(f'{"t":>7} {"ego_s":>7} {"ego_n":>7} {"ego_v":>5} {"side":>5} {"pass":>4} {"tier":>4} '
      f'{"ipopt":>14} {"reason":>15} {"d_free_L":>8} {"d_free_R":>8} {"obs1":>30} {"obs2":>30}')
for t in window:
    ego = t.get('ego',{})
    ss = t.get('side_scores',{})
    obs = t.get('obstacles', [])
    obs_str = []
    for o in obs[:2]:
        obs_str.append(f"s={o.get('s0',0):.2f},n={o.get('n0',0):+.3f},sN={o.get('sN',0):.2f}")
    while len(obs_str) < 2: obs_str.append('-')
    coll_marker = ' <COLL' if abs(t['_t_bag']-519.826)<0.05 else ''
    print(f"{t['_t_bag']:>7.3f} {ego.get('s',0):>7.2f} {ego.get('n',0):>+7.3f} {ego.get('v',0):>5.2f} "
          f"{t.get('side','?'):>5} {t.get('solver_pass',-1):>4} {t.get('tier',-1):>4} "
          f"{(t.get('ipopt_status') or '?')[:14]:>14} "
          f"{(ss.get('reason') or '?')[:15]:>15} "
          f"{ss.get('d_free_L',0):>+8.3f} {ss.get('d_free_R',0):>+8.3f} "
          f"{obs_str[0]:>30} {obs_str[1]:>30}{coll_marker}")

# Specifically zoom in on the moment ego_s ≈ 9.4-9.5 (collision zone)
print('\n========== COLLISION ZONE (ego_s 8-10) ==========')
print('Looking for the tick where ego is just before/at obstacle id100 (s=9.8 d=-0.16):')
coll_zone = [t for t in window if 8.0 <= t.get('ego',{}).get('s',0) <= 10.5]
for t in coll_zone:
    ego = t.get('ego',{})
    si = t.get('solver_input',{})
    traj = t.get('trajectory',{})
    sf = t.get('solver_infeas',{})
    sph = t.get('solver_pass_hist', [])
    sph_str = '; '.join([f"P{p['pass']}={p['status'][:5]}({'ok' if p.get('ok') else 'fail'},{p.get('worst','')})" for p in sph])
    print(f"\nt={t['_t_bag']:.3f} ego_s={ego.get('s'):.2f} ego_n={ego.get('n'):+.3f} ego_v={ego.get('v'):.2f}")
    print(f"  side={t.get('side')} pass={t.get('solver_pass')} tier={t.get('tier')} "
          f"ipopt={t.get('ipopt_status')} sph=[{sph_str}]")
    print(f"  solver_input: n0={si.get('n0',0):+.3f} v0={si.get('v0',0):.2f} "
          f"vmax_min={si.get('vmax_min',0):.2f} corridor_min={si.get('corridor_min_width',0):.3f} "
          f"nlb=[{si.get('nlb_min',0):+.3f},{si.get('nlb_0',0):+.3f}] nub=[{si.get('nub_0',0):+.3f},{si.get('nub_max',0):+.3f}]")
    print(f"  obstacles({len(t.get('obstacles',[]))}):", [
        f"s={o.get('s0',0):.2f},n={o.get('n0',0):+.3f},sN={o.get('sN',0):.2f},nN={o.get('nN',0):+.3f}"
        for o in t.get('obstacles',[])])
    print(f"  trajectory: n_min={traj.get('n_min',0):+.3f} n_max={traj.get('n_max',0):+.3f} n_end={traj.get('n_end',0):+.3f} "
          f"margin_L={traj.get('margin_L_min',0):.3f} margin_R={traj.get('margin_R_min',0):.3f}")
    if sf:
        print(f"  solver_infeas: worst={sf.get('worst')} val={sf.get('worst_val',0):.4f} k={sf.get('worst_k',0)} slack_peak={sf.get('slack_peak',0):.3f}")
    ss = t.get('side_scores',{})
    print(f"  side_scores: d_free_L={ss.get('d_free_L',0):+.3f} d_free_R={ss.get('d_free_R',0):+.3f} "
          f"reason={ss.get('reason')} dv={ss.get('dv',0):.2f} v_obs={ss.get('v_obs',0):.2f} "
          f"can_pass_L_corr={ss.get('can_pass_L_corr')} can_pass_R_corr={ss.get('can_pass_R_corr')}")

# Side decision flow leading to the collision
print('\n========== SIDE DECISION FLOW (518.0-521.4) ==========')
side_runs = []
last_side = None; run_start = None; run_ticks = []
for t in window:
    s = t.get('side','?')
    if s != last_side:
        if run_ticks:
            side_runs.append((last_side, run_start, run_ticks[-1]['_t_bag'], len(run_ticks), run_ticks))
        last_side = s; run_start = t['_t_bag']; run_ticks = []
    run_ticks.append(t)
if run_ticks: side_runs.append((last_side, run_start, run_ticks[-1]['_t_bag'], len(run_ticks), run_ticks))
print(f'{len(side_runs)} side runs:')
for s, ts, te, n, _ in side_runs:
    print(f'  {s:>5}: t=[{ts:.3f}-{te:.3f}] dur={te-ts:.2f}s ticks={n}')

# What did the published trajectory look like at t=519.5-519.85 (just before collision)?
print('\n========== JUST BEFORE COLLISION (t=519.5-519.83) ==========')
near_coll = [t for t in window if 519.5 <= t['_t_bag'] <= 519.85]
for t in near_coll:
    ego = t.get('ego',{})
    traj = t.get('trajectory',{})
    si = t.get('solver_input',{})
    obs = t.get('obstacles',[])
    print(f"\nt={t['_t_bag']:.3f} ego s={ego.get('s'):.3f} n={ego.get('n'):+.3f} v={ego.get('v'):.2f} psi={ego.get('psi'):+.4f}")
    print(f"  side={t.get('side')} pass={t.get('solver_pass')} tier={t.get('tier')} "
          f"ipopt={t.get('ipopt_status','?')[:14]}")
    print(f"  trajectory: n_min={traj.get('n_min',0):+.3f} n_max={traj.get('n_max',0):+.3f} n_end={traj.get('n_end',0):+.3f}")
    print(f"  solver_input: nlb=[{si.get('nlb_min',0):+.3f},{si.get('nlb_0',0):+.3f}] nub=[{si.get('nub_0',0):+.3f},{si.get('nub_max',0):+.3f}]")
    print(f"  obstacles: ", [f"s={o.get('s0',0):.2f},n={o.get('n0',0):+.3f}" for o in obs])
    ss = t.get('side_scores',{})
    print(f"  decider: side={t.get('side')}, reason={ss.get('reason')}, dv={ss.get('dv',0):.2f}")

# Was there a tick where the published trajectory n_max went into the obstacle's n region?
print('\n========== TRAJECTORY ↔ OBSTACLE OVERLAP CHECK ==========')
print('For each tick, check if trajectory n range overlaps with any obstacle n+/-w region')
for t in window:
    traj = t.get('trajectory',{})
    obs = t.get('obstacles',[])
    n_min = traj.get('n_min',0); n_max = traj.get('n_max',0)
    overlap = False
    for o in obs:
        on = o.get('n0',0); ow = 0.20  # half_w
        if (n_min - 0.10) < (on + ow) and (n_max + 0.10) > (on - ow):
            overlap_with = o
            overlap = True
            break
    if overlap:
        ego = t.get('ego',{})
        print(f"  t={t['_t_bag']:.3f} ego_s={ego.get('s'):.2f} side={t.get('side')} "
              f"traj_n=[{n_min:+.3f},{n_max:+.3f}] OVERLAP with obs s={overlap_with.get('s0',0):.2f} n={overlap_with.get('n0',0):+.3f}")

# Cost analysis in the collision zone
print('\n========== COST ANALYSIS IN COLLISION ZONE ==========')
for t in coll_zone[:8]:
    c = t.get('cost',{})
    if not c: continue
    obs_c = c.get('obs',0)
    others = sum([c.get(k,0) for k in ['contour','dd','dd_rate','smooth_a','wall','cont','term']])
    bias = c.get('bias',0)
    slack = c.get('slack',0)
    print(f"  t={t['_t_bag']:.3f} obs={obs_c:.0f} others={others:.0f} bias={bias:.1f} slack={slack:.2f} ratio={obs_c/max(1,others):.1f}x")
