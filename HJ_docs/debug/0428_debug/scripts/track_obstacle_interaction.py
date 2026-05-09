#!/usr/bin/env python3
"""ego ↔ obstacle 상호작용 추적.

질문:
1. Ego 가 obstacle 만나는 빈도?
2. 추월 (overtake) 이벤트 — ego_s 가 obstacle_s 를 추월하는 시점
3. 접근 (approach) — ego ↔ obstacle 거리 < N m
4. 회피 (avoid) — trajectory n 이 obstacle 옆으로 변동
5. 추월 성공률
"""
import sys
import math
import rosbag
from collections import defaultdict

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

# ego 시계열 (s, n, v)
ego_traj = []
for topic, msg, t in bag.read_messages(topics=['/car_state/odom_frenet']):
    ego_traj.append((t.to_sec(), msg.pose.pose.position.x,
                     msg.pose.pose.position.y, msg.twist.twist.linear.x))

# obstacle 시계열 (id 별)
obs_traj = defaultdict(list)
for topic, msg, t in bag.read_messages(topics=['/tracking/obstacles_truth']):
    for o in msg.obstacles:
        oid = getattr(o, 'id', 0)
        obs_traj[oid].append((t.to_sec(), o.s_center, o.d_center, o.vs))

bag.close()
print(f'Ego ticks: {len(ego_traj)}, obstacle IDs: {list(obs_traj.keys())}')

# Track length
TL = 85.0

def signed_ds(a, b, tl=TL):
    ds = (a - b) % tl
    if ds > 0.5 * tl: ds -= tl
    return ds

# Ego 거동
if ego_traj:
    s_arr = [e[1] for e in ego_traj]
    v_arr = [e[3] for e in ego_traj]
    print(f'\n=== Ego 거동 ===')
    print(f'  s range: [{min(s_arr):.2f}, {max(s_arr):.2f}]')
    print(f'  v: avg={sum(v_arr)/len(v_arr):.2f} max={max(v_arr):.2f} min={min(v_arr):.2f}')
    # lap progress (s wrap 회수)
    laps = 0
    for i in range(1, len(s_arr)):
        if s_arr[i-1] > 60 and s_arr[i] < 25: laps += 1
    print(f'  laps completed: ~{laps}')

# Ego ↔ obstacle 상호작용
for oid, otr in obs_traj.items():
    print(f'\n=== Obstacle id={oid} 상호작용 ===')
    if not otr: continue
    o_s_arr = [o[1] for o in otr]
    print(f'  obs s range: [{min(o_s_arr):.2f}, {max(o_s_arr):.2f}]')
    o_v_arr = [o[3] for o in otr]
    print(f'  obs vs: avg={sum(o_v_arr)/len(o_v_arr):.2f} max={max(o_v_arr):.2f}')

    # ego 와 매 시점 매칭 → signed_ds 계산
    interaction = []  # (t, ego_s, obs_s, ds, ego_v, obs_vs)
    j = 0
    for et, es, en, ev in ego_traj:
        # find closest obstacle sample in time
        while j < len(otr) - 1 and otr[j+1][0] < et:
            j += 1
        os, od, ovs = otr[j][1], otr[j][2], otr[j][3]
        ds = signed_ds(es, os)  # ego - obs. 음수 = ego 가 obs 뒤
        interaction.append((et, es, os, ds, en, od, ev, ovs))

    # Distance distribution
    ds_arr = [i[3] for i in interaction]
    abs_ds = [abs(d) for d in ds_arr]
    abs_sorted = sorted(abs_ds)
    print(f'  |ego_s - obs_s| dist: p10={abs_sorted[len(abs_sorted)//10]:.2f} '
          f'p50={abs_sorted[len(abs_sorted)//2]:.2f} '
          f'p90={abs_sorted[9*len(abs_sorted)//10]:.2f} min={min(abs_ds):.2f}')

    # Approach events: |ds| < 5m
    approach = [i for i in interaction if abs(i[3]) < 5.0]
    print(f'  Approach events (|ds|<5m): {len(approach)} / {len(interaction)} ticks ({100*len(approach)/len(interaction):.1f}%)')

    # Close approach events: |ds| < 2m
    very_close = [i for i in interaction if abs(i[3]) < 2.0]
    print(f'  Very close (|ds|<2m): {len(very_close)} / {len(interaction)} ticks ({100*len(very_close)/len(interaction):.1f}%)')

    # Overtake events: ds 부호가 - → + 로 변하는 시점 (ego 가 obs 추월)
    overtakes = 0
    overtake_times = []
    for i in range(1, len(interaction)):
        d_prev = interaction[i-1][3]
        d_curr = interaction[i][3]
        if d_prev < -0.5 and d_curr > 0.5:  # 분명한 추월
            overtakes += 1
            overtake_times.append(interaction[i][0])
    print(f'  Overtake events (ds: -0.5→+0.5): {overtakes}')
    if overtake_times[:5]:
        print(f'    first 5: {[f"{t:.1f}" for t in overtake_times[:5]]}')

    # 추월 시 lateral 거리 (en - od)
    if approach:
        en_during_close = []
        for i in very_close:
            ds = i[3]; en = i[4]; od = i[5]
            if abs(ds) < 1.0:  # 옆 통과 시점
                en_during_close.append((ds, en, od, abs(en - od)))
        print(f'  Lateral dist (en-od) when |ds|<1m: {len(en_during_close)} samples')
        if en_during_close:
            lats = [e[3] for e in en_during_close]
            print(f'    avg={sum(lats)/len(lats):.3f}m min={min(lats):.3f}m max={max(lats):.3f}m')

    # close call (충돌 직전): |ds|<0.5m AND |en-od|<0.4m (ego_half + obs_half + small margin)
    close_calls = []
    for i in interaction:
        ds = i[3]; en = i[4]; od = i[5]
        if abs(ds) < 0.5 and abs(en - od) < 0.4:
            close_calls.append((i[0], ds, en - od))
    print(f'  Close calls (|ds|<0.5 + |dn|<0.4): {len(close_calls)}')
    for cc in close_calls[:5]:
        print(f'    t={cc[0]:.1f} ds={cc[1]:+.2f} dn={cc[2]:+.2f}')

# Summary: 추월이 진짜로 일어났나
print(f'\n=== SUMMARY ===')
total_overtakes = sum(
    sum(1 for i in range(1, len(otr))
        if next((-signed_ds(e[1], otr[i-1][1]) for e in ego_traj if abs(e[0]-otr[i-1][0])<0.1), 1e9) < -0.5
        and next((-signed_ds(e[1], otr[i][1]) for e in ego_traj if abs(e[0]-otr[i][0])<0.1), -1e9) > 0.5)
    for oid, otr in obs_traj.items()
)
