#!/usr/bin/env python3
"""OT (overtake) 끝나고 GB (raceline) 합류 거동 분석.

사용자 명시: "OT 끝나고 GB 붙는 거동도 중요. 문제 있을 것"

검증 항목:
1. Overtake 시점 (ds 음수→양수 전환) 식별
2. Overtake 후 ego_n 의 시계열 (raceline 0 으로 복귀 시간)
3. GB 합류 시 jitter / oscillation
4. ego_v 가 raceline ref_v 로 회복하는 시간
5. plan transition (left_pass/right_pass → raceline)
"""
import json
import sys
import math
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')
ticks = []
ego_traj = []
obs_traj = []
for topic, msg, t in bag.read_messages():
    if topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data); d['_t'] = t.to_sec(); ticks.append(d)
        except: pass
    elif topic == '/car_state/odom_frenet':
        ego_traj.append((t.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x))
    elif topic == '/tracking/obstacles_truth':
        for o in msg.obstacles:
            obs_traj.append((t.to_sec(), o.s_center, o.d_center, o.vs))
bag.close()

TL = 85.0
def signed_ds(a, b, tl=TL):
    ds = (a - b) % tl
    if ds > 0.5*tl: ds -= tl
    return ds

# overtake 시점 식별
def find_overtakes(ego_traj, obs_traj):
    """ego_s ↔ obs_s 의 signed_ds 시계열 → overtake 시점 (ds 음수→양수 전환)."""
    # interleave: 각 ego sample 에 closest obs sample 매칭
    ot_events = []  # (t, ego_s, obs_s)
    j = 0; prev_ds = None
    for et, es, en, ev in ego_traj:
        while j < len(obs_traj) - 1 and obs_traj[j+1][0] < et: j += 1
        os = obs_traj[j][1]
        ds = signed_ds(es, os)
        if prev_ds is not None and prev_ds < -0.5 and ds > 0.5:
            ot_events.append((et, es, os))
        prev_ds = ds
    return ot_events

ots = find_overtakes(ego_traj, obs_traj)
print(f'\n=== Overtake events: {len(ots)} ===')
for t, es, os_ in ots[:10]:
    print(f'  t={t:.2f} ego_s={es:.2f} obs_s={os_:.2f}')

# 각 overtake 후 5초 동안의 ego_n + plan + ego_v 시계열
print(f'\n=== Post-overtake recovery (5s window after each OT) ===')
for i, (ot_t, ot_es, ot_os) in enumerate(ots[:5]):
    print(f'\n  --- OT #{i} t={ot_t:.2f} ego_s={ot_es:.2f} ---')
    # post window
    post_ego = [(et, es, en, ev) for et, es, en, ev in ego_traj
                if ot_t <= et < ot_t + 5.0]
    if not post_ego: continue
    # ego_n 시계열
    ns = [e[2] for e in post_ego]
    ts_rel = [e[0] - ot_t for e in post_ego]
    print(f'    ego_n: start={ns[0]:+.3f}, end (5s)={ns[-1]:+.3f}, max|ego_n|={max(abs(n) for n in ns):.3f}')
    # ego_n=0 까지 도달 시간 (|ego_n| < 0.05)
    converged_t = None
    for tr, n in zip(ts_rel, ns):
        if abs(n) < 0.05:
            converged_t = tr; break
    if converged_t is not None:
        print(f'    raceline 복귀 (|n|<0.05): {converged_t:.2f}s')
    else:
        print(f'    raceline 복귀 미달 (5s 안)')
    # plan transition
    post_ticks = [tk for tk in ticks if ot_t <= tk['_t'] < ot_t + 5.0]
    plan_seq = [tk.get('side','?') for tk in post_ticks]
    if plan_seq:
        # consecutive groups
        groups = []
        cur_p = None; cur_n = 0
        for p in plan_seq:
            if p != cur_p:
                if cur_p is not None: groups.append((cur_p, cur_n))
                cur_p = p; cur_n = 1
            else: cur_n += 1
        if cur_p: groups.append((cur_p, cur_n))
        print(f'    plan/side seq: {groups[:5]}')

# overtake 후 jitter (ego_n 의 변동률) — recovery 동작의 부드러움
print(f'\n=== Post-OT ego_n 부드러움 ===')
for i, (ot_t, _, _) in enumerate(ots[:3]):
    post_ego = [(et, es, en, ev) for et, es, en, ev in ego_traj
                if ot_t <= et < ot_t + 3.0]
    if len(post_ego) < 10: continue
    ns = [e[2] for e in post_ego]
    # pairwise diff
    diffs = [abs(ns[k+1] - ns[k]) for k in range(len(ns)-1)]
    print(f'  OT #{i}: max |Δego_n/sample|={max(diffs):.4f}m, avg={sum(diffs)/len(diffs):.4f}m')

# overall: side='right' or 'left' → 'clear' 전환 후의 ego_n 추세
print(f'\n=== plan transition (left/right → raceline/clear) 회수 ===')
trans_count = 0; trans_t_list = []
for k in range(1, len(ticks)):
    p = ticks[k-1].get('side','?'); c = ticks[k].get('side','?')
    if p in ('left','right') and c == 'clear':
        trans_count += 1
        trans_t_list.append(ticks[k]['_t'])
print(f'  count: {trans_count}')
print(f'  first few: {[f"{t:.1f}" for t in trans_t_list[:5]]}')
