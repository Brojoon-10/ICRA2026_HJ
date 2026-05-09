#!/usr/bin/env python3
"""Deep-dive into the two bags — focus on causal chains."""
import json
from collections import Counter

with open('/tmp/bag1_ticks.json') as f: B1 = json.load(f)
with open('/tmp/bag2_ticks.json') as f: B2 = json.load(f)

def section(name): print(f'\n========== {name} ==========')

# 1) Where do kappa explosions happen?
section('Kappa spikes (>5.0) — where?')
for name, ticks in [('bag1',B1), ('bag2',B2)]:
    spikes = [(t['_t_bag'], t.get('trajectory',{}).get('kappa_max',0),
               t.get('side'), t.get('mpc_mode'),
               t.get('solver_pass'), t.get('tier'),
               t.get('ipopt_status'),
               t.get('n_obs_used'),
               t.get('jitter_rms_m'),
               t.get('continuity_guard',{}).get('L2'))
              for t in ticks if t.get('trajectory',{}).get('kappa_max',0) > 5.0]
    print(f'\n--- {name}: {len(spikes)} spike ticks (kappa_max>5)')
    # bucketed by side & pass
    by_pass = Counter([(s[4], s[5], s[6]) for s in spikes])
    print('   (pass,tier,ipopt) ->', dict(by_pass))
    # show top 10 ranks
    spikes.sort(key=lambda x: -x[1])
    for s in spikes[:10]:
        print(f"   t={s[0]:.2f} kmax={s[1]:.1f} side={s[2]} mode={s[3]} pass={s[4]} tier={s[5]}"
              f" ipopt={s[6]} n_obs={s[7]} jitter={s[8]:.3f} cgL2={s[9]}")

# 2) NO_OBS + obs_in_horizon=True — the violation cases
section('NO_OBS but obstacle in horizon (rule violation candidates)')
for name, ticks in [('bag1',B1), ('bag2',B2)]:
    bad = [t for t in ticks if (not (t.get('mpc_mode')=='WITH_OBS')) and t.get('obs_in_horizon')]
    print(f'\n--- {name}: {len(bad)} ticks with NO_OBS+obs_in_horizon=True')
    for t in bad[:20]:
        traj = t.get('trajectory',{})
        print(f"   t={t['_t_bag']:.2f} side={t.get('side')} mode={t.get('mpc_mode')} "
              f"ego_n={t.get('ego',{}).get('n',0):+.2f} "
              f"min_obs_ds={t.get('min_obs_ds',0):.2f} "
              f"n_min={traj.get('n_min',0):+.2f} n_max={traj.get('n_max',0):+.2f} "
              f"k_max={traj.get('kappa_max',0):.1f} "
              f"alpha_ramp={t.get('alpha_ramp',-1):.2f}")

# 3) Multi-obstacle decision detail (bag1 only — bag2 has none)
section('Bag1: multi-obstacle decision detail — the failure modes')
multi = [t for t in B1 if t.get('n_obs_used',0)>=2]
# Group by the *pair* of obstacles seen (rough s1, s2)
def obs_pair_key(t):
    obs = t.get('obstacles',[])
    if len(obs)<2: return None
    s1, s2 = sorted([o.get('s0',0) for o in obs])
    return (round(s1), round(s2))
pair_runs = []
prev_key = None; start_t = None; run_ticks=[]
for t in multi:
    k = obs_pair_key(t)
    if k != prev_key:
        if prev_key is not None and run_ticks:
            pair_runs.append((prev_key, start_t, run_ticks[-1]['_t_bag'], len(run_ticks), run_ticks))
        start_t = t['_t_bag']; run_ticks=[]
    run_ticks.append(t)
    prev_key = k
if prev_key is not None and run_ticks:
    pair_runs.append((prev_key, start_t, run_ticks[-1]['_t_bag'], len(run_ticks), run_ticks))

print(f'multi-obs pair runs: {len(pair_runs)}')
for k,ts,te,nt,rticks in pair_runs[:8]:
    print(f'\n  pair s={k}, t=[{ts:.2f}-{te:.2f}] {nt} ticks')
    side_seq = [t.get('side','?')[:1] for t in rticks]
    pass_seq = [t.get('solver_pass',0) for t in rticks]
    reason_ctr = Counter([t.get('side_scores',{}).get('reason') for t in rticks])
    pass_ctr = Counter(pass_seq)
    side_ctr = Counter([t.get('side','?') for t in rticks])
    ego_n_arr = [t.get('ego',{}).get('n',0) for t in rticks]
    en_min = min(ego_n_arr); en_max = max(ego_n_arr)
    print(f'    side dist: {dict(side_ctr)}')
    print(f'    pass dist: {dict(pass_ctr)}')
    print(f'    reason   : {dict(reason_ctr)}')
    print(f'    ego_n range: [{en_min:+.2f}, {en_max:+.2f}]   side seq: {"".join(side_seq[:60])}')
    # Show first 10 ticks in detail
    for t in rticks[:6]:
        ss = t.get('side_scores',{})
        print(f"      t={t['_t_bag']:.2f} side={t.get('side'):>5} pass={t.get('solver_pass')} "
              f"ego_n={t.get('ego',{}).get('n',0):+.2f} n_obs[0]_n={t['obstacles'][0].get('n0',0):+.3f} "
              f"d_free_L={ss.get('d_free_L',0):+.2f} d_free_R={ss.get('d_free_R',0):+.2f} "
              f"reason={ss.get('reason')} ipopt={t.get('ipopt_status','?')[:10]}")

# 4) Pass1<->Pass2 alternations — locate them
section('Pass1<->Pass2 alternation events')
for name, ticks in [('bag1',B1), ('bag2',B2)]:
    print(f'\n--- {name}')
    last = None; events=[]
    for t in ticks:
        p = t.get('solver_pass',-1)
        if last is not None and p != last and (last in (1,2) and p in (1,2)):
            events.append(t)
        last = p
    print(f'   {len(events)} alternation events')
    for t in events[:12]:
        print(f"   t={t['_t_bag']:.2f} pass={t.get('solver_pass')} side={t.get('side')} "
              f"reason={t.get('side_scores',{}).get('reason')} "
              f"ipopt={t.get('ipopt_status','?')[:14]} "
              f"n_obs={t.get('n_obs_used')}")

# 5) Continuity guard intensity (when does it apply heavy?)
section('Continuity guard L2 distribution')
for name, ticks in [('bag1',B1), ('bag2',B2)]:
    L2s = [(t.get('continuity_guard',{}).get('L2',0), t) for t in ticks]
    L2s_sorted = sorted(L2s, key=lambda x: -x[0])
    print(f'\n--- {name}  top 10 L2:')
    for l, t in L2s_sorted[:10]:
        print(f"   L2={l:.3f} t={t['_t_bag']:.2f} side={t.get('side')} pass={t.get('solver_pass')} "
              f"k_max={t.get('trajectory',{}).get('kappa_max',0):.1f} "
              f"applied={t.get('continuity_guard',{}).get('applied')}")

# 6) HOLD_LAST chains — how long?
section('HOLD_LAST chain durations')
for name, ticks in [('bag1',B1), ('bag2',B2)]:
    chains=[]; cur=[]
    for t in ticks:
        if t.get('tier')==1:
            cur.append(t)
        else:
            if cur: chains.append(cur); cur=[]
    if cur: chains.append(cur)
    print(f'\n--- {name}: {len(chains)} HOLD_LAST chains')
    for c in chains[:8]:
        if not c: continue
        ts0 = c[0]['_t_bag']; tsN = c[-1]['_t_bag']
        print(f"   t=[{ts0:.2f}-{tsN:.2f}] dur={tsN-ts0:.2f}s ticks={len(c)} "
              f"first ipopt={c[0].get('ipopt_status','?')[:14]} side={c[0].get('side')}")

# 7) Ratio cost.obs vs everything else
section('Cost balance — obs vs others (per tick max ratio)')
for name, ticks in [('bag1',B1), ('bag2',B2)]:
    obs_dom_count = 0
    for t in ticks:
        c = t.get('cost',{})
        obs = c.get('obs',0)
        others = sum([c.get(k,0) for k in ['contour','dd','dd_rate','smooth_a','wall','cont','term']])
        if others > 0 and obs > 10*others:
            obs_dom_count += 1
    print(f'   {name}: ticks where obs_cost > 10x sum_other_costs = {obs_dom_count}/{len(ticks)} = {100*obs_dom_count/len(ticks):.1f}%')

# 8) Obstacle separation — how close are the 2 obstacles?
section('Bag1: obstacle pair separation')
for t in B1:
    if t.get('n_obs_used',0)>=2:
        obs = t['obstacles']
        if len(obs)>=2:
            s1, s2 = sorted([o.get('s0',0) for o in obs])
            n1, n2 = [o.get('n0',0) for o in obs]
            ds = abs(s2-s1)
            dn = abs(n1-n2)
            t['_obs_ds'] = ds; t['_obs_dn'] = dn
ds_arr = [t.get('_obs_ds') for t in B1 if t.get('_obs_ds') is not None]
dn_arr = [t.get('_obs_dn') for t in B1 if t.get('_obs_dn') is not None]
if ds_arr:
    ds_arr.sort(); dn_arr.sort()
    print(f'   ds: p10={ds_arr[len(ds_arr)//10]:.2f} p50={ds_arr[len(ds_arr)//2]:.2f} p90={ds_arr[9*len(ds_arr)//10]:.2f}')
    print(f'   dn: p10={dn_arr[len(dn_arr)//10]:.2f} p50={dn_arr[len(dn_arr)//2]:.2f} p90={dn_arr[9*len(dn_arr)//10]:.2f}')

# 9) Decider's reason transitions
section('Decider reason transitions (which reasons cluster?)')
for name, ticks in [('bag1',B1), ('bag2',B2)]:
    seq = [t.get('side_scores',{}).get('reason','?') for t in ticks]
    transitions = Counter()
    for i in range(1,len(seq)):
        if seq[i]!=seq[i-1]:
            transitions[(seq[i-1], seq[i])] += 1
    print(f'\n--- {name}:')
    for k,v in transitions.most_common(15):
        print(f'   {k[0]:>20s} -> {k[1]:<20s}  {v}')
