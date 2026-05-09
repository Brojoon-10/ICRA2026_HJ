#!/usr/bin/env python3
"""S1-5 라이브 검증 — bag 의 tick_json 분석.

검증 항목:
1. tier=1 (HOLD_LAST) 발생 시 fail_streak 분포 — cap 3 이내인지
2. tier=-1 (EMERGENCY_NO_TRAJ) 발생 횟수 + 이전 fail_streak
3. inter-msg first wpnt jump (publish 된 wpnts 토픽 기준) — 0.3m 초과 빈도
4. 충돌 발생 (collision_marker) 횟수
5. obs_in_horizon 분포 + tier 매핑 정상성
"""
import json
import sys
import math
from collections import Counter
import rosbag

bag_path = sys.argv[1] if len(sys.argv) > 1 else None
if bag_path is None:
    print('usage: validate_s1_5.py <bag>')
    sys.exit(1)

print(f'Loading {bag_path} ...')
bag = rosbag.Bag(bag_path, 'r')

ticks = []
wpnts_msgs = []
coll = 0
for topic, msg, t in bag.read_messages():
    if topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data); d['_t'] = t.to_sec(); ticks.append(d)
        except:
            pass
    elif topic == '/planner/mpc/wpnts':
        wpnts_msgs.append((t.to_sec(), msg.wpnts, getattr(msg, 'ot_line', '?')))
    elif topic == '/collision_marker':
        coll += 1
bag.close()

print(f'\nticks: {len(ticks)}, wpnts_msgs: {len(wpnts_msgs)}, collision: {coll}')

# 1. Tier distribution
t_ctr = Counter([t.get('tier', -99) for t in ticks])
print(f'\nTier dist: {dict(t_ctr)}')

# 2. fail_streak when tier=1
hl_streaks = [t.get('fail_streak', 0) for t in ticks if t.get('tier') == 1]
if hl_streaks:
    print(f'\nHOLD_LAST(tier=1): n={len(hl_streaks)}, fail_streak max={max(hl_streaks)}, '
          f'mean={sum(hl_streaks)/len(hl_streaks):.2f}, dist={dict(Counter(hl_streaks))}')
    # Cap=3 위반 (>3) 찾기
    over = [s for s in hl_streaks if s > 3]
    print(f'  fail_streak > 3 (cap 위반): {len(over)} ticks')

# 3. EMERGENCY_NO_TRAJ
em = [t for t in ticks if t.get('tier') == -1 or t.get('status', '') == 'EMERGENCY_NO_TRAJ']
print(f'\nEMERGENCY_NO_TRAJ: {len(em)} ticks')
for t in em[:5]:
    print(f"  t={t['_t']:.2f} fail_streak={t.get('fail_streak')} ipopt={t.get('ipopt_status')}")

# 4. Inter-msg first-wpnt jump
jumps = []
last_xy0 = None
for ts, wpnts, ol in wpnts_msgs:
    if not wpnts:
        last_xy0 = None
        continue
    if last_xy0 is not None:
        d = math.hypot(wpnts[0].x_m - last_xy0[0], wpnts[0].y_m - last_xy0[1])
        jumps.append((ts, d, ol))
    last_xy0 = (wpnts[0].x_m, wpnts[0].y_m)
big = [(ts, d, ol) for ts, d, ol in jumps if d > 0.3]
print(f'\nInter-msg jumps > 0.3m: {len(big)} / total {len(jumps)}')
for ts, d, ol in big[:8]:
    print(f"  t={ts:.2f} jump={d:.3f}m ot_line={ol}")

# 5. obs_in_horizon vs tier
oh_t = Counter([(t.get('obs_in_horizon'), t.get('tier')) for t in ticks])
print(f'\nobs_in_horizon × tier: {dict(oh_t)}')

# 6. ipopt status dist
ip_ctr = Counter([t.get('ipopt_status', '?') for t in ticks])
print(f'\nipopt_status dist: {dict(ip_ctr)}')

# 7. solver_pass dist
pass_ctr = Counter([t.get('solver_pass', -1) for t in ticks])
print(f'\nsolver_pass dist: {dict(pass_ctr)}')

# 8. side dist
side_ctr = Counter([t.get('side', '?') for t in ticks])
print(f'\nside dist: {dict(side_ctr)}')

# 9. kappa stats
kmax = [t.get('trajectory', {}).get('kappa_max', 0) for t in ticks]
if kmax:
    sk = sorted(kmax)
    print(f'\nkappa_max p50={sk[len(sk)//2]:.2f} p95={sk[int(len(sk)*0.95)]:.2f} '
          f'p99={sk[int(len(sk)*0.99)]:.2f} max={max(kmax):.2f}')

# 10. solve_ms
sm = [t.get('solve_ms', 0) for t in ticks]
sms = sorted(sm)
if sms:
    print(f'solve_ms p50={sms[len(sms)//2]:.1f} p95={sms[int(len(sms)*0.95)]:.1f} '
          f'p99={sms[int(len(sms)*0.99)]:.1f}')
