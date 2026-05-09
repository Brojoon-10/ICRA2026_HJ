#!/usr/bin/env python3
"""Encounter analyzer (Phase 2 prototype, offline bag analysis).

For each tick where SM is in TRAILING/OVERTAKE, compute the encounter
metrics that the future StrategicPlanner (Layer C) will need:

  encounter_k       — argmin over horizon of |ego_traj[k] − obs_traj[k]|
  encounter_t       — encounter_k * dT
  encounter_ds, dn  — frenet distance at encounter
  pass_margin_L, R  — lateral room past obstacle on each side, with safety buf
  closure_rate      — ego.vs - obs.vs at encounter
  obs_lateral_drift — sign(median(vd[k] for k in horizon))

Bag must contain:
  /car_state/odom_frenet
  /tracking/obstacles
  /opponent_prediction/obstacles
  /planner/mpc/wpnts            (ego horizon estimate)
  /behavior_strategy            (state for filtering)
  /opponent_collision           (rising-edge marker)

The analyzer prints per-collision deep diagnostics: 1.0s leading up to
each collision, every 0.1s sample with the encounter_summary that the
new StrategicPlanner *would have computed*. This tells us whether Layer
B / C would have caught the collision (and chosen BRAKE_HARD).
"""
import sys
import json
import math
from collections import deque, defaultdict

import rosbag

if len(sys.argv) < 2:
    print("usage: encounter_trace.py <bag>", file=sys.stderr)
    sys.exit(1)
BAG = sys.argv[1]
TRACK = 85.6
EGO_WIDTH = 0.4
OBS_RADIUS = 0.2
SAFETY_BUF = 0.3   # baseline gap_lat
DT = 0.05
N_MPC = 40

bag = rosbag.Bag(BAG, 'r')

ego_state = []   # (t, s, n, vs)
obs_track = []   # (t, [(s, n, vs, vd, size, is_static)])
mpc_path  = []   # (t, [(x, y, s, d, vx) for k])
beh       = []   # (t, state_str)
coll      = []   # rising edges
last_coll = False

for topic, msg, t in bag.read_messages(topics=[
        '/car_state/odom_frenet',
        '/tracking/obstacles',
        '/planner/mpc/wpnts',
        '/behavior_strategy',
        '/opponent_collision']):
    ts = t.to_sec()
    if topic == '/car_state/odom_frenet':
        ego_state.append((ts, msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.twist.twist.linear.x))
    elif topic == '/tracking/obstacles':
        obs_track.append((ts, [(o.s_center, o.d_center, o.vs, o.vd,
                                o.size, o.is_static)
                               for o in msg.obstacles]))
    elif topic == '/planner/mpc/wpnts':
        # Sample first N_MPC waypoints; (s, d, vx) approximates ego horizon
        wps = [(w.x_m, w.y_m, w.s_m, w.d_m, w.vx_mps)
               for w in msg.wpnts[:N_MPC]]
        mpc_path.append((ts, wps))
    elif topic == '/behavior_strategy':
        beh.append((ts, str(msg.state)))
    elif topic == '/opponent_collision':
        if bool(msg.data) and not last_coll:
            coll.append(ts)
        last_coll = bool(msg.data)

bag.close()

t0 = ego_state[0][0]


def near_idx(arr, t):
    lo, hi = 0, len(arr) - 1
    while lo < hi:
        m = (lo + hi) // 2
        if arr[m][0] < t:
            lo = m + 1
        else:
            hi = m
    return lo


def encounter_summary(ts, ego_traj_xy, obs_state):
    """Approximate encounter analyzer.

    ego_traj_xy: list of (x, y, s, d, vx) for k=0..N-1 from /planner/mpc/wpnts
    obs_state: (s, n, vs, vd, size, is_static)

    Predicts obs_traj as constant-vs/vd from current state (placeholder
    until full GP timestep handling is wired up; same approximation MPC
    is currently using).
    """
    if not ego_traj_xy or obs_state is None:
        return None
    s_o, n_o, vs_o, vd_o, sz_o, _ = obs_state
    # Build obs predicted (s, n) per horizon step assuming constant vs/vd
    obs_pred = [(s_o + vs_o * k * DT, n_o + vd_o * k * DT)
                for k in range(len(ego_traj_xy))]
    # Compute distance per timestep using frenet (s, d)
    best_k = -1
    best_dist = 1e9
    for k, (xe, ye, se, de, ve) in enumerate(ego_traj_xy):
        sp, np_ = obs_pred[k]
        ds = (sp - se) % TRACK
        if ds > 0.5 * TRACK:
            ds -= TRACK
        dn = np_ - de
        dist = math.hypot(ds, dn)
        if dist < best_dist:
            best_dist = dist
            best_k = k
    if best_k < 0:
        return None
    enc = {
        'k': best_k,
        't': best_k * DT,
        'ds': (obs_pred[best_k][0] - ego_traj_xy[best_k][2]) % TRACK,
        'dn': obs_pred[best_k][1] - ego_traj_xy[best_k][3],
        'distance': best_dist,
        'obs_n_at_enc': obs_pred[best_k][1],
        'ego_n_at_enc': ego_traj_xy[best_k][3],
        'ego_v_at_enc': ego_traj_xy[best_k][4],
        'closure': (ego_traj_xy[best_k][4] - vs_o),
        'obs_vd_med': vd_o,
    }
    if enc['ds'] > 0.5 * TRACK:
        enc['ds'] -= TRACK
    return enc


# Cluster collision edges into events
events = []
for c in coll:
    if events and c - events[-1] < 0.5:
        continue
    events.append(c)

print('=== Bag summary ===')
print('  duration: %.1fs   ego: %d   obs: %d   mpc: %d   beh: %d   coll edges: %d  events: %d' %
      (ego_state[-1][0] - t0, len(ego_state), len(obs_track),
       len(mpc_path), len(beh), len(coll), len(events)))

print('\n=== Per-collision deep diagnostics ===')
for ev_ts in events:
    print('\n--- Collision at t=%.2fs (rel) ---' % (ev_ts - t0))
    print('  %-7s %-9s %8s %8s %8s %8s %8s %8s  %s' %
          ('t_lead', 'sm_state', 'ego_n', 'obs_n', 'enc_t', 'enc_ds',
           'enc_dn', 'closure', 'note'))
    # Walk back 1.0s before collision in 0.1s steps
    for lead in range(10, 0, -1):
        ts = ev_ts - lead * 0.1
        ego = ego_state[near_idx(ego_state, ts)]
        ot = obs_track[near_idx(obs_track, ts)]
        sm = beh[near_idx(beh, ts)]
        mp = mpc_path[near_idx(mpc_path, ts)] if mpc_path else None
        # Choose closest obstacle by current s gap
        if ot[1]:
            best = min(ot[1], key=lambda o: ((o[0] - ego[1]) % TRACK))
        else:
            best = None
        enc = encounter_summary(ts, mp[1] if mp else [], best) if best else None
        note = ''
        if best and best[5]:
            note = '(static)'
        if enc is None:
            print('  -%4.1fs %-9s %8.3f %8s %8s %8s %8s %8s  %s' %
                  (lead * 0.1, sm[1], ego[2], '-', '-', '-', '-', '-', note))
        else:
            print('  -%4.1fs %-9s %8.3f %+8.3f %8.2f %+8.2f %+8.2f %+8.2f  %s' %
                  (lead * 0.1, sm[1], ego[2], enc['obs_n_at_enc'],
                   enc['t'], enc['ds'], enc['dn'], enc['closure'], note))


# Stats: how often does encounter_dist drop below 0.4m (sub-vehicle-width)
# during TRAILING / OVERTAKE state, and is that strongly correlated with
# the actual collisions later?
print('\n=== Phase-1 close-call statistics ===')
n_close = 0
n_total = 0
for ts, st in beh:
    if st not in ('TRAILING', 'OVERTAKE'):
        continue
    n_total += 1
    ego = ego_state[near_idx(ego_state, ts)]
    ot = obs_track[near_idx(obs_track, ts)]
    mp = mpc_path[near_idx(mpc_path, ts)] if mpc_path else None
    if not ot[1] or not mp:
        continue
    best = min(ot[1], key=lambda o: ((o[0] - ego[1]) % TRACK))
    enc = encounter_summary(ts, mp[1], best)
    if enc and enc['distance'] < 0.4:
        n_close += 1
print('  TRAILING/OVERTAKE ticks: %d   dist<0.4m: %d (%.1f%%)' %
      (n_total, n_close, 100.0 * n_close / max(n_total, 1)))
