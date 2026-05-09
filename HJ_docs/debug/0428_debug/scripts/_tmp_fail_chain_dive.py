#!/usr/bin/env python3
"""Deep dive into bag1's 51-tick HOLD_LAST chain (t=521.39-525.01).
Goal: identify the EXACT IPOPT failure cause via tick_json fields.
No guesses — only what tick_json explicitly reports."""
import json
from collections import Counter

with open('/tmp/bag1_ticks.json') as f: B1 = json.load(f)

# Find the failure chain — all consecutive ticks with tier=1 starting from t≈521.39
# Plus context: the success tick immediately before, and the recovery tick after
chain_window = [t for t in B1 if 521.0 <= t['_t_bag'] <= 525.5]
print(f'Window 521.0-525.5: {len(chain_window)} ticks')

# 1) The success tick just before the chain
success_before = None
fail_chain = []
for i, t in enumerate(chain_window):
    if t.get('tier') == 1:
        if success_before is None:
            success_before = chain_window[i-1] if i>0 else None
        fail_chain.append(t)
    elif fail_chain:
        # found recovery tick
        recovery = t
        break
else:
    recovery = None

print(f'\n=== ANCHOR TICKS ===')
def dump_anchor(name, t):
    if t is None: print(f'{name}: None'); return
    si = t.get('solver_input', {})
    sf = t.get('solver_infeas', {})
    cost = t.get('cost', {})
    print(f'\n{name}: t={t["_t_bag"]:.3f} tier={t.get("tier")} pass={t.get("solver_pass")} '
          f'ipopt={t.get("ipopt_status")} iter={t.get("iter")}')
    print(f'  side={t.get("side")} mode={t.get("mpc_mode")} mode_dwell={t.get("mode_dwell")} '
          f'alpha_ramp={t.get("alpha_ramp")}')
    print(f'  ego: s={t.get("ego",{}).get("s"):.2f} n={t.get("ego",{}).get("n"):+.3f} '
          f'v={t.get("ego",{}).get("v"):.3f} psi={t.get("ego",{}).get("psi"):+.4f}')
    print(f'  solver_input: n0={si.get("n0"):.3f} mu0={si.get("mu0"):+.4f} v0={si.get("v0"):.3f} '
          f'de0_locked={si.get("de0_locked"):+.4f} de0_active={si.get("de0_active")}')
    print(f'    nlb=[{si.get("nlb_min"):+.3f},{si.get("nlb_0"):+.3f}] nub=[{si.get("nub_0"):+.3f},{si.get("nub_max"):+.3f}]')
    print(f'    corridor_min_width={si.get("corridor_min_width"):.3f} nlb_min_k={si.get("nlb_min_k")}')
    print(f'    vmax: min={si.get("vmax_min"):.3f} max={si.get("vmax_max"):.3f}')
    print(f'    kappa_abs_max={si.get("kappa_abs_max"):.3f}')
    print(f'    dL_min={si.get("dL_min"):.3f} dR_min={si.get("dR_min"):.3f}  n_obs_active={si.get("n_obs_active")}')
    print(f'  solver_infeas: {sf}')
    print(f'  obstacles ({len(t.get("obstacles",[]))}): ' +
          ' | '.join([f"id?:s={o.get('s0',0):.2f},n={o.get('n0',0):+.3f},sN={o.get('sN',0):.2f},nN={o.get('nN',0):+.3f},w={o.get('w',0):.0f}"
                      for o in t.get("obstacles",[])]))
    ss = t.get('side_scores',{})
    print(f'  side_scores: d_free_L={ss.get("d_free_L",0):+.3f} d_free_R={ss.get("d_free_R",0):+.3f} '
          f'reason={ss.get("reason")} dv={ss.get("dv",0):.2f} v_obs={ss.get("v_obs",0):.2f}')
    print(f'  cost: ' + ' '.join([f'{k}={v:.1f}' for k,v in cost.items()]))
    sph = t.get('solver_pass_hist', [])
    print(f'  solver_pass_hist: {sph}')
    if 'a1_vmax' in t: print(f'  a1_vmax: {t.get("a1_vmax")}')
    print(f'  warm_used={t.get("warm_used")} fail_streak={t.get("fail_streak")} ipopt_iter={t.get("iter")}')
    print(f'  obs_in_horizon={t.get("obs_in_horizon")} min_obs_ds={t.get("min_obs_ds")} ttc_min={t.get("ttc_min")}')

dump_anchor('SUCCESS just before chain', success_before)
print(f'\n  === FAIL CHAIN ({len(fail_chain)} ticks) ===')
dump_anchor('FIRST FAIL (t=521.39 ish)', fail_chain[0] if fail_chain else None)
mid = fail_chain[len(fail_chain)//2] if fail_chain else None
dump_anchor('MID FAIL', mid)
dump_anchor('LAST FAIL (just before recovery)', fail_chain[-1] if fail_chain else None)
dump_anchor('RECOVERY tick', recovery)

# 2) What changed between success_before and first_fail?
print('\n\n========== DIFF: success_before vs first_fail ==========')
if success_before and fail_chain:
    sb = success_before; ff = fail_chain[0]
    keys = ['side', 'mpc_mode', 'mode_dwell', 'alpha_ramp', 'n_obs_used', 'fail_streak', 'warm_used',
            'min_obs_ds', 'ttc_min', 'obs_in_horizon']
    for k in keys:
        a = sb.get(k); b = ff.get(k)
        if a != b:
            print(f'  {k}: {a} → {b}')
    sb_si = sb.get('solver_input',{}); ff_si = ff.get('solver_input',{})
    for k in ['n0','mu0','v0','de0_locked','de0_active','nlb_0','nub_0','nlb_min','nub_max',
              'corridor_min_width','nlb_min_k','vmax_min','vmax_max','kappa_abs_max',
              'dL_min','dR_min','n_obs_active']:
        a = sb_si.get(k); b = ff_si.get(k)
        try:
            if abs(float(a)-float(b)) > 1e-6 or (type(a)!=type(b)):
                print(f'  solver_input.{k}: {a} → {b}')
        except:
            if a != b: print(f'  solver_input.{k}: {a} → {b}')

# 3) Solver pass history breakdown across the chain
print('\n\n========== SOLVER_PASS_HIST BREAKDOWN (every 5th tick) ==========')
for i, t in enumerate(fail_chain[::5]):
    sph = t.get('solver_pass_hist', [])
    sph_summary = '; '.join([f"P{p['pass']}={p['status']}({'ok' if p.get('ok') else 'fail'})" for p in sph])
    print(f'  t={t["_t_bag"]:.2f} (#{i*5}): {sph_summary}')

# 4) IPOPT status and iter distribution across chain
print('\n\n========== IPOPT STATUS + ITER ACROSS CHAIN ==========')
ipopt_ctr = Counter()
iter_arr = []
for t in fail_chain:
    sph = t.get('solver_pass_hist', [])
    for p in sph:
        ipopt_ctr[p.get('status','?')] += 1
    iter_arr.append(t.get('iter', -1))
print(f'  ipopt status across all passes in chain: {dict(ipopt_ctr)}')
print(f'  iter distribution: min={min(iter_arr)} max={max(iter_arr)} mean={sum(iter_arr)/len(iter_arr):.1f}')

# 5) Obstacle dynamics during chain
print('\n\n========== OBSTACLE EVOLUTION (every 5th tick) ==========')
for i, t in enumerate(fail_chain[::5]):
    obs_str = ' | '.join([f"s={o.get('s0',0):.2f},n={o.get('n0',0):+.3f},sN={o.get('sN',0):.2f},w={o.get('w',0):.0f}"
                          for o in t.get('obstacles',[])])
    print(f'  t={t["_t_bag"]:.2f} ego_s={t["ego"]["s"]:.2f} n_obs={t.get("n_obs_used")} obs=[{obs_str}]')

# 6) Cost evolution (does any cost spike or stay stable?)
print('\n\n========== COST EVOLUTION (every 5th tick, before & after recovery) ==========')
sample_ticks = [success_before] + fail_chain[::5] + ([recovery] if recovery else [])
for t in sample_ticks:
    if t is None: continue
    c = t.get('cost', {})
    label = 'SUCC' if t.get('tier')==0 and t.get('solver_pass')!=2 else \
            ('PASS2' if t.get('solver_pass')==2 else \
             ('FAIL/HOLD' if t.get('tier')==1 else 'OTHER'))
    print(f'  t={t["_t_bag"]:.2f} {label:8s}: ' + ' '.join([f'{k}={v:.0f}' for k,v in c.items() if v!=0]))

# 7) The critical fields: did anything in solver_input cross a threshold?
print('\n\n========== SOLVER_INPUT INSPECTION ==========')
print('  Looking for unusual values: vmax_min<v0, corridor inverted, kappa explosion, etc.')
for i, t in enumerate(fail_chain[::3]):
    si = t.get('solver_input',{})
    v0 = si.get('v0',0); vmax_min = si.get('vmax_min',0)
    corridor_min = si.get('corridor_min_width',0)
    n0 = si.get('n0',0); nlb_0 = si.get('nlb_0',0); nub_0 = si.get('nub_0',0)
    de0 = si.get('de0_locked',0); de0_act = si.get('de0_active',0)
    flags = []
    if vmax_min < v0 - 0.1: flags.append(f'vmax<v0 ({vmax_min:.2f}<{v0:.2f})')
    if corridor_min < 0.5: flags.append(f'corridor narrow ({corridor_min:.2f})')
    if n0 < nlb_0 - 0.05 or n0 > nub_0 + 0.05: flags.append(f'n0 outside corridor (n0={n0:.3f} in [{nlb_0:.3f},{nub_0:.3f}])')
    print(f"  t={t['_t_bag']:.2f} v0={v0:.2f} vmax_min={vmax_min:.2f} corridor_min={corridor_min:.3f} "
          f"n0={n0:+.3f} ∈[{nlb_0:+.3f},{nub_0:+.3f}] de0={de0:+.3f}(active={de0_act}) "
          f"FLAGS={flags}")

# 8) Were there OTHER fail ticks in the bag with similar patterns?
print('\n\n========== ALL FAIL TICKS IN BAG1 — IPOPT STATUS DISTRIBUTION ==========')
all_fails = [t for t in B1 if t.get('tier') == 1]
print(f'  Total fail ticks: {len(all_fails)}')
all_pass_status = Counter()
for t in all_fails:
    for p in t.get('solver_pass_hist',[]):
        all_pass_status[p.get('status','?')] += 1
print(f'  All pass statuses across fail ticks: {dict(all_pass_status)}')

# 9) What does success_before say about the prior solution that got cached?
if success_before:
    traj = success_before.get('trajectory',{})
    print(f'\n\n=== success_before trajectory snapshot (cached → HOLD_LAST will replay this) ===')
    print(f'  len={traj.get("len")} n_min={traj.get("n_min"):+.3f} n_max={traj.get("n_max"):+.3f} '
          f'n_end={traj.get("n_end"):+.3f}')
    print(f'  margin_L_min={traj.get("margin_L_min"):.3f} margin_R_min={traj.get("margin_R_min"):.3f}')
    print(f'  kappa_rms={traj.get("kappa_rms"):.2f} kappa_max={traj.get("kappa_max"):.2f}')

# 10) Pre-chain ramp — were there warning signs in the previous several ticks?
print(f'\n\n========== PRE-CHAIN RAMP (10 ticks before 1st fail) ==========')
pre_idx = [i for i,t in enumerate(B1) if t['_t_bag'] >= fail_chain[0]['_t_bag']][0] if fail_chain else 0
for i in range(max(0, pre_idx-10), pre_idx+1):
    t = B1[i]
    si = t.get('solver_input',{})
    print(f"  t={t['_t_bag']:.2f} tier={t.get('tier')} pass={t.get('solver_pass')} "
          f"ipopt={t.get('ipopt_status','?')[:14]} side={t.get('side')} "
          f"v0={si.get('v0',0):.2f} vmax_min={si.get('vmax_min',0):.2f} "
          f"corridor_min={si.get('corridor_min_width',0):.3f} n_obs={t.get('n_obs_used')}")
