#!/usr/bin/env python3
"""Analyze post-filter MPC bag.

Metrics:
  - Plan transitions (count, sequence, dwell distribution)
  - Real collision events from /opponent_collision (Bool, edge-detected)
  - Lateral min between ego and obstacle (proper s-wrap, ds threshold)
  - Adaptive gap_lat indicators if pred_variance present in tick_json
"""
import sys
import json
from collections import defaultdict

import rosbag

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path, 'r')

ego = []                # [(t, s, n, vs)]
obs = []                # [(t, s, n)]
plan_msgs = []          # [(t, plan_name, jitter_rms, cont_L2, cont_applied)]
collision_edges = []    # [t] when /opponent_collision toggles to True
last_collision = False

for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if topic == '/car_state/odom_frenet':
        ego.append((ts, msg.pose.pose.position.x, msg.pose.pose.position.y,
                    msg.twist.twist.linear.x))
    elif topic == '/tracking/obstacles_truth':
        for o in msg.obstacles:
            obs.append((ts, o.s_center, o.d_center))
    elif topic == '/mpc_auto/debug/tick_json':
        try:
            d = json.loads(msg.data)
            if 'plan' in d and d['plan'] is not None:
                jit = float(d.get('jitter_rms_m', 0.0))
                cg = d.get('continuity_guard', {}) or {}
                plan_msgs.append((ts, d['plan'], jit,
                                  float(cg.get('L2', 0.0)),
                                  bool(cg.get('applied', False))))
        except Exception:
            pass
    elif topic == '/opponent_collision':
        if bool(msg.data) and not last_collision:
            collision_edges.append(ts)
        last_collision = bool(msg.data)

bag.close()

if not ego:
    print('NO EGO MSGS')
    sys.exit(1)
print('ego samples: %d  obs samples: %d  plan msgs: %d  coll edges: %d' %
      (len(ego), len(obs), len(plan_msgs), len(collision_edges)))

t0 = ego[0][0]
t1 = ego[-1][0]
print('duration: %.1fs' % (t1 - t0))

# Plan transitions
plan_seq = plan_msgs   # list of (ts, plan, jitter, cont_L2, cont_applied)
transitions = []
last = None
for ts, p, _, _, _ in plan_seq:
    if p != last:
        transitions.append((ts - t0, p, ts))
        last = p

print('\n=== Plan transitions: %d (over %.0fs, %.1f/min) ===' %
      (len(transitions), t1 - t0, len(transitions) / max((t1 - t0) / 60.0, 1e-3)))
for ts_rel, p, _ in transitions[:30]:
    print('  t=%6.2fs -> %s' % (ts_rel, p))
if len(transitions) > 30:
    print('  ... (%d more)' % (len(transitions) - 30))

durations = defaultdict(list)
for i in range(len(transitions) - 1):
    durations[transitions[i][1]].append(transitions[i+1][0] - transitions[i][0])
if transitions:
    durations[transitions[-1][1]].append((t1 - t0) - transitions[-1][0])

# Trajectory smoothness around plan transitions (jitter_rms in 0.5s window).
# Higher jitter near transition → controller-disruptive trajectory step.
jitter_at_transition = []
for ts_rel, _, ts_abs in transitions[1:]:   # skip first (no prev plan)
    samples = [j for ts2, _, j, _, _ in plan_seq
               if abs(ts2 - ts_abs) < 0.25]
    if samples:
        jitter_at_transition.append(max(samples))

# Continuity-guard activation rate.
cont_applied_count = sum(1 for _, _, _, _, ca in plan_seq if ca)
all_jitter = [j for _, _, j, _, _ in plan_seq]
all_L2 = [L for _, _, _, L, _ in plan_seq]
print('\n=== Plan dwell times ===')
for p, ds in sorted(durations.items()):
    if ds:
        med = sorted(ds)[len(ds)//2]
        print('  %12s  count=%3d  med=%5.2fs  max=%5.2fs  total=%5.1fs' %
              (p, len(ds), med, max(ds), sum(ds)))

# Lateral min (only when ego near obstacle, with proper s-wrap)
TRACK_LEN = 85.6
def nearest_ego(target_t):
    if not ego: return None
    lo, hi = 0, len(ego) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if ego[mid][0] < target_t: lo = mid + 1
        else: hi = mid
    return ego[lo]

lateral_min_close = float('inf')
lateral_min_at = None
for t_o, s_o, n_o in obs:
    e = nearest_ego(t_o)
    if e is None: continue
    _, s_e, n_e, _ = e
    ds = s_o - s_e
    while ds > 0.5 * TRACK_LEN: ds -= TRACK_LEN
    while ds < -0.5 * TRACK_LEN: ds += TRACK_LEN
    if abs(ds) < 1.0:
        dn = abs(n_e - n_o)
        if dn < lateral_min_close:
            lateral_min_close = dn
            lateral_min_at = (t_o - t0, ds, n_e, n_o)

print('\n=== Trajectory smoothness ===')
if all_jitter:
    sj = sorted(all_jitter)
    print('  jitter_rms_m  p50=%.4f  p95=%.4f  p99=%.4f  max=%.4f' %
          (sj[len(sj)//2], sj[int(len(sj)*0.95)], sj[int(len(sj)*0.99)], sj[-1]))
if jitter_at_transition:
    sjt = sorted(jitter_at_transition)
    print('  jitter at transition (±0.25s):  p50=%.4f  max=%.4f' %
          (sjt[len(sjt)//2], sjt[-1]))
print('  continuity-guard applied: %d / %d ticks (%.1f%%)' %
      (cont_applied_count, len(plan_seq),
       100.0 * cont_applied_count / max(len(plan_seq), 1)))
if all_L2:
    sl = sorted(all_L2)
    print('  cont_L2_m     p50=%.4f  p95=%.4f  max=%.4f' %
          (sl[len(sl)//2], sl[int(len(sl)*0.95)], sl[-1]))

print('\n=== Lateral safety ===')
print('  collisions (real, /opponent_collision rising edges): %d' % len(collision_edges))
for ts in collision_edges[:5]:
    print('     coll at t=%.2fs' % (ts - t0))
print('  lateral min when |ds|<1m: %.0f mm' % (lateral_min_close * 1000))
if lateral_min_at:
    t_, ds_, ne_, no_ = lateral_min_at
    print('     at t=%.2fs ds=%+.2fm n_ego=%+.3f n_obs=%+.3f' %
          (t_, ds_, ne_, no_))
