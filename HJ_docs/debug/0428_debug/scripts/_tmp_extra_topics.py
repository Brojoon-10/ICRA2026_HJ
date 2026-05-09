#!/usr/bin/env python3
"""Pull additional topics: published wpnts shape, predictor stability, tracking truth, SM output, collisions."""
import json
from collections import Counter, defaultdict
import rosbag

BAGS = [
    ('bag1', '/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-13-09-22.bag'),
    ('bag2', '/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-13-12-51.bag'),
]

def section(t): print(f'\n========== {t} ==========')

for name, path in BAGS:
    section(f'{name}: {path}')
    bag = rosbag.Bag(path, 'r')
    
    # Published wpnts: shape + smoothness verification
    n_msgs = 0
    wp_sizes = []
    wp_kappa_estim = []  # consecutive-point dx,dy → ds + heading change → kappa
    wp_ds_mins = []  # min ds within each msg
    wp_n_jumps = []  # max |Δn| within msg
    ot_line_seq = []
    ot_line_t = []
    last_xy0 = None
    inter_msg_jumps = []
    for topic, msg, t in bag.read_messages(topics=['/planner/mpc/wpnts']):
        ts = t.to_sec()
        wpnts = msg.wpnts
        wp_sizes.append(len(wpnts))
        ot_line_seq.append(msg.ot_line)
        ot_line_t.append(ts)
        # In-message kappa estimate
        kappas = []
        ds_arr = []
        n_arr = []
        for i in range(1, len(wpnts)):
            dx = wpnts[i].x_m - wpnts[i-1].x_m
            dy = wpnts[i].y_m - wpnts[i-1].y_m
            ds = (dx*dx+dy*dy)**0.5
            ds_arr.append(ds)
            n_arr.append(wpnts[i].d_m)
            if i >= 2:
                ax = wpnts[i-1].x_m - wpnts[i-2].x_m
                ay = wpnts[i-1].y_m - wpnts[i-2].y_m
                # heading change / ds → kappa estimate
                import math
                a1 = math.atan2(ay, ax) if (ax*ax+ay*ay)>1e-6 else 0
                a2 = math.atan2(dy, dx) if ds>1e-6 else 0
                dpsi = a2 - a1
                while dpsi > math.pi: dpsi -= 2*math.pi
                while dpsi < -math.pi: dpsi += 2*math.pi
                if ds > 1e-6:
                    kappas.append(abs(dpsi)/ds)
        if kappas:
            wp_kappa_estim.append(max(kappas))
        if ds_arr:
            wp_ds_mins.append(min(ds_arr))
        if n_arr:
            wp_n_jumps.append(max(n_arr) - min(n_arr))
        # Inter-message jump (first wpnt vs last published first wpnt)
        if last_xy0 is not None and wpnts:
            d0 = ((wpnts[0].x_m - last_xy0[0])**2 + (wpnts[0].y_m - last_xy0[1])**2)**0.5
            inter_msg_jumps.append((ts, d0))
        if wpnts:
            last_xy0 = (wpnts[0].x_m, wpnts[0].y_m)
        n_msgs += 1
    
    print(f'mpc_wpnts msgs: {n_msgs}, avg_len: {sum(wp_sizes)/max(1,len(wp_sizes)):.1f}')
    if wp_kappa_estim:
        ks = sorted(wp_kappa_estim)
        print(f'  in-msg kappa estim p50={ks[len(ks)//2]:.2f} p95={ks[int(len(ks)*0.95)]:.2f} p99={ks[int(len(ks)*0.99)]:.2f} max={max(ks):.2f}')
    if wp_ds_mins:
        ds = sorted(wp_ds_mins)
        print(f'  in-msg min(ds) p10={ds[len(ds)//10]:.4f} p50={ds[len(ds)//2]:.4f}')
    if wp_n_jumps:
        nj = sorted(wp_n_jumps)
        print(f'  in-msg n_range p50={nj[len(nj)//2]:.3f} p95={nj[int(len(nj)*0.95)]:.3f} max={max(nj):.3f}')
    if inter_msg_jumps:
        big_jumps = [(ts, d) for ts, d in inter_msg_jumps if d > 0.3]
        print(f'  inter-msg first-wpnt jumps >0.3m: {len(big_jumps)}')
        for ts, d in big_jumps[:10]:
            print(f'    t={ts:.2f} jump={d:.3f}m')
    # ot_line distribution
    ol_ctr = Counter(ot_line_seq)
    print(f'  ot_line dist: {dict(ol_ctr)}')
    # ot_line transitions
    ol_transitions = Counter()
    for i in range(1, len(ot_line_seq)):
        if ot_line_seq[i] != ot_line_seq[i-1]:
            ol_transitions[(ot_line_seq[i-1], ot_line_seq[i])] += 1
    print(f'  ot_line transitions: {dict(ol_transitions)}')
    
    # Predictor obstacles (sub-sample)
    print('\n  Predictor stability:')
    pred_pairs = []
    pred_count = 0
    for topic, msg, t in bag.read_messages(topics=['/opponent_prediction/obstacles']):
        if pred_count % 50 != 0:
            pred_count += 1; continue
        pred_count += 1
        obs = msg.obstacles
        if len(obs) >= 1:
            entries = [(o.s_start if hasattr(o,'s_start') else o.s_center if hasattr(o,'s_center') else 0,
                        o.d_center if hasattr(o,'d_center') else 0,
                        o.id if hasattr(o,'id') else 0,
                        o.is_static if hasattr(o,'is_static') else False,
                        o.is_actually_a_gap if hasattr(o,'is_actually_a_gap') else False)
                       for o in obs]
            pred_pairs.append((t.to_sec(), entries))
    print(f'  sampled predictor states: {len(pred_pairs)} (every 50th of {pred_count})')
    # Show 6 samples
    for ts, entries in pred_pairs[::max(1,len(pred_pairs)//8)][:8]:
        descr = ' | '.join([f'id={e[2]} s={e[0]:.1f} d={e[1]:.2f} static={e[3]}' for e in entries])
        print(f'    t={ts:.2f}: {descr}')

    # Truth (tracking ground truth)
    print('\n  Tracking truth check:')
    tru_pairs = []
    tru_count = 0
    for topic, msg, t in bag.read_messages(topics=['/tracking/obstacles_truth']):
        if tru_count % 20 != 0:
            tru_count += 1; continue
        tru_count += 1
        obs = msg.obstacles
        entries = [(getattr(o,'s_start',0) or getattr(o,'s_center',0),
                    getattr(o,'d_center',0),
                    getattr(o,'id',0),
                    getattr(o,'is_static',False))
                   for o in obs]
        tru_pairs.append((t.to_sec(), entries))
    print(f'  sampled truth states: {len(tru_pairs)} (every 20th of {tru_count})')
    for ts, entries in tru_pairs[::max(1,len(tru_pairs)//6)][:6]:
        descr = ' | '.join([f'id={e[2]} s={e[0]:.1f} d={e[1]:.2f} static={e[3]}' for e in entries])
        print(f'    t={ts:.2f}: {descr}')
    
    # Behavior strategy from SM
    print('\n  Behavior strategy from state machine:')
    bs_seq = []
    for topic, msg, t in bag.read_messages(topics=['/behavior_strategy']):
        try:
            state = getattr(msg, 'state_machine_state', None) or getattr(msg, 'state', None)
            bs_seq.append((t.to_sec(), state))
        except Exception as e:
            pass
    print(f'  state msgs: {len(bs_seq)}')
    if bs_seq:
        bsc = Counter([s for _, s in bs_seq])
        print(f'  state dist: {dict(bsc)}')
        # transitions
        trans = Counter()
        for i in range(1,len(bs_seq)):
            if bs_seq[i][1] != bs_seq[i-1][1]:
                trans[(bs_seq[i-1][1], bs_seq[i][1])] += 1
        print(f'  state transitions: {dict(trans)}')

    # Collision markers
    coll_count = 0
    coll_first = None
    for topic, msg, t in bag.read_messages(topics=['/collision_marker']):
        coll_count += 1
        if coll_first is None:
            coll_first = t.to_sec()
    print(f'\n  collision_marker msgs: {coll_count}, first at t={coll_first}')
    
    bag.close()
