#!/usr/bin/env python3
"""d_free_L / d_free_R 시계열 진동 양상 분석.

사용자 질문: d_free 가 어떻게 진동? 양상 알고 싶음.
"""
import json
import sys
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')
ticks = []
for topic, msg, t in bag.read_messages(topics=['/mpc_auto/debug/tick_json']):
    try:
        d = json.loads(msg.data); d['_t'] = t.to_sec(); ticks.append(d)
    except: pass
bag.close()

# obs_in_horizon=True 시점 의 d_free 시계열
obs_ticks = [(t['_t'],
              t.get('side_scores', {}).get('d_free_L', 0),
              t.get('side_scores', {}).get('d_free_R', 0),
              t.get('side', '?'),
              t.get('side_scores', {}).get('reason', '?'),
              t.get('ego', {}).get('s', 0),
              t.get('ego', {}).get('n', 0),
              [o.get('s0',0) for o in t.get('obstacles',[])][:1])
             for t in ticks if t.get('obs_in_horizon')]
print(f'\nobs_in_horizon=True ticks: {len(obs_ticks)}')

# 시계열 sample (1초 간격)
print('\n=== d_free 시계열 (1초 간격) ===')
print(f'{"t":>9} {"d_free_L":>10} {"d_free_R":>10} {"side":>6} {"ego_s":>7} {"ego_n":>7} {"obs_s":>7} {"reason":>20}')
last_log = -1e9
for t, dL, dR, side, reason, es, en, os_list in obs_ticks:
    if t - last_log < 1.0:
        continue
    last_log = t
    os_str = f'{os_list[0]:.2f}' if os_list else '?'
    print(f'{t:>9.2f} {dL:>+10.4f} {dR:>+10.4f} {side:>6} {es:>7.2f} {en:>+7.3f} {os_str:>7} {reason[:20]:>20}')

# d_free_L 의 매 tick 변화 (Δd_free)
print('\n=== d_free 변화 (인접 ticks Δ) ===')
diffs_L = []
diffs_R = []
for i in range(1, len(obs_ticks)):
    t_curr = obs_ticks[i][0]; t_prev = obs_ticks[i-1][0]
    if t_curr - t_prev < 0.2:  # 인접 sample (50ms 정도)
        dL_curr = obs_ticks[i][1]; dL_prev = obs_ticks[i-1][1]
        dR_curr = obs_ticks[i][2]; dR_prev = obs_ticks[i-1][2]
        diffs_L.append(abs(dL_curr - dL_prev))
        diffs_R.append(abs(dR_curr - dR_prev))
if diffs_L:
    diffs_L_s = sorted(diffs_L)
    print(f'  d_free_L |Δ|: p50={diffs_L_s[len(diffs_L_s)//2]:.4f} p95={diffs_L_s[int(len(diffs_L_s)*0.95)]:.4f} max={max(diffs_L):.4f}')
if diffs_R:
    diffs_R_s = sorted(diffs_R)
    print(f'  d_free_R |Δ|: p50={diffs_R_s[len(diffs_R_s)//2]:.4f} p95={diffs_R_s[int(len(diffs_R_s)*0.95)]:.4f} max={max(diffs_R):.4f}')

# Sign 변동 (LEFT 가능 → 불가능 → 가능 등)
print('\n=== d_free 부호 변동 (positive ↔ negative) ===')
sign_flips_L = 0; sign_flips_R = 0
for i in range(1, len(obs_ticks)):
    if obs_ticks[i-1][1] > 0 and obs_ticks[i][1] < 0: sign_flips_L += 1
    elif obs_ticks[i-1][1] < 0 and obs_ticks[i][1] > 0: sign_flips_L += 1
    if obs_ticks[i-1][2] > 0 and obs_ticks[i][2] < 0: sign_flips_R += 1
    elif obs_ticks[i-1][2] < 0 and obs_ticks[i][2] > 0: sign_flips_R += 1
print(f'  d_free_L sign flips: {sign_flips_L}')
print(f'  d_free_R sign flips: {sign_flips_R}')

# Plan flip 시점의 d_free
print('\n=== Plan flip 시점의 d_free 양상 ===')
flip_count = 0
for i in range(1, len(ticks)):
    p = ticks[i-1].get('side','?'); c = ticks[i].get('side','?')
    if p != c and p in ('left','right','trail') and c in ('left','right','trail'):
        if flip_count < 8:
            ss = ticks[i].get('side_scores',{})
            ss_prev = ticks[i-1].get('side_scores',{})
            print(f'  t={ticks[i]["_t"]:.2f} {p}→{c}  '
                  f'd_free_L: {ss_prev.get("d_free_L",0):+.4f}→{ss.get("d_free_L",0):+.4f}  '
                  f'd_free_R: {ss_prev.get("d_free_R",0):+.4f}→{ss.get("d_free_R",0):+.4f}')
        flip_count += 1
print(f'  total flip events: {flip_count}')
