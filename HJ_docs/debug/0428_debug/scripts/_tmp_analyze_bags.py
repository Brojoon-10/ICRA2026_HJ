#!/usr/bin/env python3
"""Parse /mpc_auto/debug/tick_json from two bags and emit a structural diagnosis."""
import os, sys, json, subprocess
from collections import Counter, defaultdict

BAGS = [
    ('bag1', '/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-13-09-22.bag'),
    ('bag2', '/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-13-12-51.bag'),
]
TOPIC = '/mpc_auto/debug/tick_json'

def extract(bag):
    """Return list of dicts in time order."""
    import rosbag
    msgs = []
    bag_h = rosbag.Bag(bag, 'r')
    for topic, msg, t in bag_h.read_messages(topics=[TOPIC]):
        try:
            d = json.loads(msg.data)
            d['_t_bag'] = t.to_sec()
            msgs.append(d)
        except Exception as e:
            pass
    bag_h.close()
    return msgs

def summarize(name, ticks):
    n = len(ticks)
    print(f'\n========== {name}  (n={n}) ==========')
    if n == 0:
        return
    t0 = ticks[0]['_t_bag']; tN = ticks[-1]['_t_bag']
    print(f'duration: {tN-t0:.2f}s, rate: {n/(tN-t0):.1f} Hz')

    tier_ctr = Counter([t.get('tier',-1) for t in ticks])
    status_ctr = Counter([t.get('status','?') for t in ticks])
    ipopt_ctr = Counter([t.get('ipopt_status','?') for t in ticks])
    pass_ctr  = Counter([t.get('solver_pass',-1) for t in ticks])
    side_ctr  = Counter([t.get('side','?') for t in ticks])
    mode_ctr  = Counter([t.get('mpc_mode','?') for t in ticks])
    nobs_ctr  = Counter([t.get('n_obs_used',-1) for t in ticks])
    nobs_seen = Counter([t.get('side_scores',{}).get('n_obs_seen',-1) for t in ticks])
    reason_ctr = Counter([t.get('side_scores',{}).get('reason','?') for t in ticks])

    print('-- tier distribution:', dict(tier_ctr))
    print('-- status:', dict(status_ctr))
    print('-- ipopt_status:', dict(ipopt_ctr))
    print('-- solver_pass:', dict(pass_ctr))
    print('-- side:', dict(side_ctr))
    print('-- mpc_mode:', dict(mode_ctr))
    print('-- n_obs_used:', dict(nobs_ctr))
    print('-- n_obs_seen (decider):', dict(nobs_seen))
    print('-- decider.reason:', dict(reason_ctr))

    # tier 3 events?
    t3 = [t for t in ticks if t.get('tier',-1) == 3]
    if t3:
        print(f'\n!!! TIER 3 events: {len(t3)} ticks')
        for t in t3[:6]:
            print(f"   t={t['_t_bag']:.2f} status={t.get('status')} ipopt={t.get('ipopt_status')} "
                  f"side={t.get('side')} obs_in_h={t.get('obs_in_horizon')} "
                  f"min_obs_ds={t.get('min_obs_ds'):.2f}")

    # tier 1 / hold_last
    t1 = [t for t in ticks if t.get('tier',-1) == 1]
    if t1:
        print(f'-- TIER 1 (HOLD_LAST): {len(t1)} ticks  ({100*len(t1)/n:.1f}%)')

    # tier 2
    t2 = [t for t in ticks if t.get('tier',-1) == 2]
    if t2:
        print(f'-- TIER 2 (geometric): {len(t2)} ticks ({100*len(t2)/n:.1f}%)')

    # solver pass2 fallback
    p2 = [t for t in ticks if t.get('solver_pass',-1) == 2]
    if p2:
        print(f'-- SOLVER PASS=2 (TRAIL fallback): {len(p2)} ticks ({100*len(p2)/n:.1f}%)')

    # side flip
    flips = []
    for i in range(1,n):
        a = ticks[i-1].get('side','?'); b = ticks[i].get('side','?')
        if a != b and a != '?' and b != '?':
            flips.append((ticks[i]['_t_bag'], a, b, ticks[i].get('mpc_mode'),
                          ticks[i].get('n_obs_used'), ticks[i].get('ego',{}).get('n', None)))
    print(f'\n-- side flips: {len(flips)}')
    for f in flips[:30]:
        print(f"   t={f[0]:.2f} {f[1]:>5}->{f[2]:<5} mode={f[3]} n_obs={f[4]} ego_n={f[5]}")

    # mode flip
    mflips = []
    for i in range(1,n):
        a = ticks[i-1].get('mpc_mode','?'); b = ticks[i].get('mpc_mode','?')
        if a != b:
            mflips.append((ticks[i]['_t_bag'], a, b, ticks[i].get('side'),
                           ticks[i].get('min_obs_ds'),
                           ticks[i].get('mode_dwell')))
    print(f'\n-- mpc_mode flips: {len(mflips)}')
    for m in mflips[:30]:
        print(f"   t={m[0]:.2f} {m[1]}->{m[2]} side={m[3]} min_obs_ds={m[4]} dwell={m[5]}")

    # pass 1<->2 oscillation: consecutive same/diff pass
    p1_runs = 0; p2_runs = 0
    last_p = None
    p_alt = 0
    for t in ticks:
        p = t.get('solver_pass', -1)
        if last_p is not None and p != last_p and (last_p in (1,2) and p in (1,2)):
            p_alt += 1
        last_p = p
    print(f'-- pass 1<->2 alternations: {p_alt}')

    # jitter / kappa stats
    jit = [t.get('jitter_rms_m',0) for t in ticks if 'jitter_rms_m' in t]
    if jit:
        jit_sorted = sorted(jit)
        p50 = jit_sorted[len(jit_sorted)//2]
        p95 = jit_sorted[int(len(jit_sorted)*0.95)]
        p99 = jit_sorted[int(len(jit_sorted)*0.99)]
        print(f'-- jitter_rms_m: p50={p50:.3f} p95={p95:.3f} p99={p99:.3f} max={max(jit):.3f}')

    kappa_max_arr = [t.get('trajectory',{}).get('kappa_max',0) for t in ticks]
    if kappa_max_arr:
        ksorted = sorted(kappa_max_arr)
        p95 = ksorted[int(len(ksorted)*0.95)]
        p99 = ksorted[int(len(ksorted)*0.99)]
        print(f'-- traj.kappa_max: p95={p95:.3f} p99={p99:.3f} max={max(kappa_max_arr):.3f}')

    kappa_rms_arr = [t.get('trajectory',{}).get('kappa_rms',0) for t in ticks]
    if kappa_rms_arr:
        ksorted = sorted(kappa_rms_arr)
        p95 = ksorted[int(len(ksorted)*0.95)]
        print(f'-- traj.kappa_rms: p95={p95:.3f} max={max(kappa_rms_arr):.3f}')

    cg_applied = sum(1 for t in ticks if t.get('continuity_guard',{}).get('applied'))
    print(f'-- continuity_guard.applied: {cg_applied} ({100*cg_applied/n:.1f}%)')

    # solve_ms
    sm = [t.get('solve_ms',0) for t in ticks]
    smv = sorted(sm)
    print(f'-- solve_ms: p50={smv[len(smv)//2]:.1f} p95={smv[int(len(smv)*0.95)]:.1f} p99={smv[int(len(smv)*0.99)]:.1f} max={max(sm):.1f}')

    # cost breakdown averages (focus on multi-obstacle ticks)
    cost_keys = ['contour','reg','dd','dd_rate','smooth_a','progress','obs','bias','wall','cont','term','slack']
    obs_ticks = [t for t in ticks if t.get('n_obs_used',0) >= 1]
    print(f'\n-- cost breakdown means (n_obs>=1, n={len(obs_ticks)}):')
    if obs_ticks:
        for k in cost_keys:
            vals = [t.get('cost',{}).get(k,0) for t in obs_ticks]
            if vals:
                print(f'   {k:>10s}: mean={sum(vals)/len(vals):>9.2f} max={max(vals):>9.2f}')

    # Multi-obstacle (n_obs>=2) inspection
    multi = [t for t in ticks if t.get('n_obs_used',0) >= 2]
    print(f'\n-- MULTI-OBSTACLE ticks (n_obs_used>=2): {len(multi)} ({100*len(multi)/n:.1f}%)')
    if multi:
        # examine side decisions during multi
        m_side = Counter([t.get('side','?') for t in multi])
        m_pass = Counter([t.get('solver_pass',-1) for t in multi])
        m_tier = Counter([t.get('tier',-1) for t in multi])
        m_reason = Counter([t.get('side_scores',{}).get('reason','?') for t in multi])
        print(f'   side dist: {dict(m_side)}')
        print(f'   pass dist: {dict(m_pass)}')
        print(f'   tier dist: {dict(m_tier)}')
        print(f'   reason   : {dict(m_reason)}')

        # Details of first multi-obstacle window
        t_start = multi[0]['_t_bag']
        t_end = multi[-1]['_t_bag']
        print(f'   multi-obs time window: t={t_start:.2f} to {t_end:.2f} ({t_end-t_start:.2f}s span)')

        # Sample some ticks
        print('   sample ticks:')
        for t in multi[::max(1,len(multi)//10)][:12]:
            obs_summary = '|'.join([f"s={o.get('s0',0):.1f},n={o.get('n0',0):.2f},w={o.get('w',0):.0f}" for o in t.get('obstacles',[])])
            ss = t.get('side_scores',{})
            print(f"   t={t['_t_bag']:.2f} side={t.get('side'):>5} pass={t.get('solver_pass')} tier={t.get('tier')} "
                  f"ego_n={t.get('ego',{}).get('n',0):>+.2f} "
                  f"d_free_L={ss.get('d_free_L',0):>+.2f} d_free_R={ss.get('d_free_R',0):>+.2f} "
                  f"reason={ss.get('reason')} obs=[{obs_summary}]")

    # Continuity guard activations vs side flips
    cg_at_flip = 0
    for i in range(1,n):
        if ticks[i-1].get('side','?') != ticks[i].get('side','?'):
            if ticks[i].get('continuity_guard',{}).get('applied'):
                cg_at_flip += 1
    print(f'-- continuity_guard.applied at side flip: {cg_at_flip}/{len(flips)}')

    # ego_n vs side mismatch (commit ego_n is one direction, side picks other)
    contradict = 0
    for t in ticks:
        en = t.get('ego',{}).get('n', None)
        side = t.get('side','clear')
        if en is None: continue
        if side == 'left' and en < -0.20: contradict += 1
        if side == 'right' and en > +0.20: contradict += 1
    print(f'-- side vs ego_n contradiction (|ego_n|>0.2 against side): {contradict}')

    # Obs in horizon vs mpc_mode
    oh_with = sum(1 for t in ticks if t.get('obs_in_horizon') and t.get('mpc_mode')=='WITH_OBS')
    oh_no   = sum(1 for t in ticks if t.get('obs_in_horizon') and t.get('mpc_mode')=='NO_OBS')
    no_with = sum(1 for t in ticks if not t.get('obs_in_horizon') and t.get('mpc_mode')=='WITH_OBS')
    no_no   = sum(1 for t in ticks if not t.get('obs_in_horizon') and t.get('mpc_mode')=='NO_OBS')
    print(f'-- obs_in_horizon × mpc_mode:  WO+with={oh_with}  WO+NoO={oh_no}  NoO+WO={no_with}  NoO+NoO={no_no}')

    # specific anomalies — ipopt failures details
    fails = [t for t in ticks if t.get('ipopt_status','') not in ('Solve_Succeeded','Solved_To_Acceptable_Level')]
    if fails:
        print(f'\n-- IPOPT non-success: {len(fails)} ({100*len(fails)/n:.1f}%)')
        f_status = Counter([t.get('ipopt_status') for t in fails])
        print(f'   {dict(f_status)}')
        for t in fails[:8]:
            si = t.get('solver_input',{})
            print(f"   t={t['_t_bag']:.2f} ipopt={t.get('ipopt_status')} pass={t.get('solver_pass')} "
                  f"corridor_min={si.get('corridor_min_width', None):.3f} "
                  f"vmax_min={si.get('vmax_min', None):.2f} "
                  f"side={t.get('side')} n_obs={t.get('n_obs_used')} "
                  f"obs_in_h={t.get('obs_in_horizon')}")


def main():
    out_paths = []
    for name, bag in BAGS:
        try:
            ticks = extract(bag)
        except Exception as e:
            print(f'ERROR loading {bag}: {e}')
            continue
        out_path = f'/tmp/{name}_ticks.json'
        with open(out_path, 'w') as f:
            json.dump(ticks, f)
        out_paths.append(out_path)
        summarize(name, ticks)
    print('\nSaved:', out_paths)

if __name__ == '__main__':
    main()
