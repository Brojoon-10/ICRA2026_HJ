#!/usr/bin/env python3
"""Analyze 25s+ window in 2026-04-27-05-00-55.bag.
- What path is the controller actually following?
- What does the cyan marker represent?
- Is recovery being recomputed every tick?
"""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-27-05-00-55.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()
WIN_LO, WIN_HI = 25.0, 28.0

print("=== topic msg counts in window ===")
counts = {}
for topic, msg, t in b.read_messages(topics=[
    "/planner/mpc/wpnts",
    "/planner/recovery/wpnts",
    "/planner/avoidance/otwpnts",
    "/behavior_strategy",
    "/state_machine_mpc/debug",
    "/state_machine",
    "/mpc_auto/debug/tick_json",
]):
    rel = t.to_sec() - t0
    if not (WIN_LO <= rel <= WIN_HI): continue
    counts[topic] = counts.get(topic, 0) + 1
for k, v in sorted(counts.items()):
    print(f"  {k}: {v}")

print("\n=== /state_machine_mpc/debug — sm_state, path_source, mpc_mode ===")
prev = None
for topic, msg, t in b.read_messages(topics=["/state_machine_mpc/debug"]):
    rel = t.to_sec() - t0
    if not (WIN_LO <= rel <= WIN_HI): continue
    try:
        d = json.loads(msg.data)
    except Exception:
        continue
    key = (d.get("sm_state"), d.get("path_source"), d.get("mpc_mode_echo"),
           round(d.get("ot_age_ms") or 0, 0), round(d.get("rc_age_ms") or 0, 0))
    if key != prev:
        print(f"  t={rel:.3f}  state={d.get('sm_state')!r:20}  src={d.get('path_source')!r:10}  "
              f"mode={d.get('mpc_mode_echo')!r:8}  ot_age={d.get('ot_age_ms')}  rc_age={d.get('rc_age_ms')}")
        prev = key

print("\n=== /behavior_strategy state + local_wpnts (n,d) ===")
prev = None
for topic, msg, t in b.read_messages(topics=["/behavior_strategy"]):
    rel = t.to_sec() - t0
    if not (WIN_LO <= rel <= WIN_HI): continue
    wp_list = msg.local_wpnts if isinstance(msg.local_wpnts, list) else (
        msg.local_wpnts.wpnts if hasattr(msg.local_wpnts, 'wpnts') else [])
    n = len(wp_list)
    df = wp_list[0].d_m if n > 0 else 0.0
    dl = wp_list[-1].d_m if n > 0 else 0.0
    fs = wp_list[0].s_m if n > 0 else 0.0
    ls = wp_list[-1].s_m if n > 0 else 0.0
    st = msg.state.data if hasattr(msg.state, 'data') else str(msg.state)
    nt = len(msg.trailing_targets) if msg.trailing_targets else 0
    key = (st, n, round(df, 2), round(dl, 2), nt)
    if key != prev:
        print(f"  t={rel:.3f}  state={st!r:18}  n_wp={n:3d}  d=[{df:+.3f},{dl:+.3f}]  s={fs:.2f}..{ls:.2f}  trail={nt}")
        prev = key

print("\n=== /mpc_auto/debug/tick_json — tier history ===")
prev = None
for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    if not (WIN_LO <= rel <= WIN_HI): continue
    try:
        d = json.loads(msg.data)
    except Exception:
        continue
    tr = d.get('trajectory', {})
    si = d.get('solver_input', {})
    a4 = d.get('a4_wall_ramp', {})
    key = (d.get("tier"), d.get("ipopt_status"), d.get("mpc_mode"))
    if key != prev:
        print(f"  t={rel:.3f}  tier={d.get('tier')}  ipopt={d.get('ipopt_status')}  mode={d.get('mpc_mode')}  "
              f"solve_ms={d.get('solve_ms')}  ego_n={d.get('ego', {}).get('n')}  v0={si.get('v0')}  "
              f"n_traj=[{tr.get('n_min')},{tr.get('n_max')}]  margin_L_min={tr.get('margin_L_min')}  "
              f"margin_R_min={tr.get('margin_R_min')}  fs={d.get('fail_streak')}  slk_max={d.get('slack_max')}  "
              f"wall_ramp_active={a4.get('active')}")
        prev = key

print("\n=== /planner/mpc/wpnts publish events (count + ot_line tag) ===")
ot_lines = []
for topic, msg, t in b.read_messages(topics=["/planner/mpc/wpnts"]):
    rel = t.to_sec() - t0
    if not (WIN_LO <= rel <= WIN_HI): continue
    ot_lines.append((rel, msg.ot_line, len(msg.wpnts)))
print(f"total mpc/wpnts msgs: {len(ot_lines)}")
prev_line = None
for rel, line, n in ot_lines[:20]:
    if line != prev_line:
        print(f"  t={rel:.3f} ot_line={line!r} n_wpnts={n}")
        prev_line = line

print("\n=== /planner/recovery/wpnts publishers (if any) ===")
rec_count = 0
for topic, msg, t in b.read_messages(topics=["/planner/recovery/wpnts"]):
    rel = t.to_sec() - t0
    if not (WIN_LO <= rel <= WIN_HI): continue
    rec_count += 1
print(f"  /planner/recovery/wpnts msgs in window: {rec_count}")

b.close()
