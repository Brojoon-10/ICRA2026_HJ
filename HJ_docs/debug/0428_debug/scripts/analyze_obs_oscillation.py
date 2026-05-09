#!/usr/bin/env python3
"""obstacle 의 lateral 진동 분석 — d_free 진동의 진짜 근원 추정.

가설: obstacle_publisher 의 osc_amplitude=0.25, n_waves_per_lap=1.5 → obstacle
자체가 lateral 으로 진동. 이게 d_free_L/R 변동의 원인.
"""
import sys
import math
import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')
obs_traj = []
for topic, msg, t in bag.read_messages(topics=['/tracking/obstacles_truth']):
    for o in msg.obstacles:
        obs_traj.append((t.to_sec(), o.s_center, o.d_center, o.vs))
bag.close()

print(f'obstacle samples: {len(obs_traj)}')
if not obs_traj: sys.exit()

# obs.d (lateral position) 시계열 — 진동 분석
print(f'\n=== obstacle d_center (lateral) 시계열 ===')
print(f'{"t":>9} {"s":>7} {"d":>7} {"vs":>6}')
last_log = -1e9
d_arr = []
s_arr = []
for t, s, d, vs in obs_traj:
    d_arr.append(d)
    s_arr.append(s)
    if t - last_log < 1.0: continue
    last_log = t
    print(f'{t:>9.2f} {s:>7.2f} {d:>+7.4f} {vs:>6.2f}')

# d_arr statistics
d_min = min(d_arr); d_max = max(d_arr)
print(f'\n  d range: [{d_min:.4f}, {d_max:.4f}] amplitude={d_max-d_min:.4f}m')
# pairwise diff (인접 sample, 변동률)
diffs = [abs(d_arr[i+1]-d_arr[i]) for i in range(len(d_arr)-1)]
diffs_sorted = sorted(diffs)
print(f'  |Δd|/sample: p50={diffs_sorted[len(diffs_sorted)//2]:.4f} p95={diffs_sorted[int(len(diffs_sorted)*0.95)]:.4f} max={max(diffs):.4f}')

# d 의 부호 flip (+ ↔ -) 회수
sign_flips = 0
for i in range(1, len(d_arr)):
    if d_arr[i-1] > 0 and d_arr[i] < 0: sign_flips += 1
    elif d_arr[i-1] < 0 and d_arr[i] > 0: sign_flips += 1
print(f'  d sign flips: {sign_flips}')

# Spatial frequency (d 가 s 따라 어떻게 변동)
# Lap 안의 d 변동 — s 정렬해서 d 시계열
sorted_pairs = sorted(zip(s_arr, d_arr), key=lambda x: x[0])
ss, dd = zip(*sorted_pairs[:200])
print(f'\n=== d(s) 패턴 (s 정렬, 첫 200 sample) ===')
for i in range(0, len(ss), max(1, len(ss)//10)):
    print(f'  s={ss[i]:>6.2f}  d={dd[i]:>+.4f}')
