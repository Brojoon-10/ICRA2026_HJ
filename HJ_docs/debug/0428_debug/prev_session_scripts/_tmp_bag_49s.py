#!/usr/bin/env python3
"""Analyze /state_machine + /behavior_strategy around t=49s."""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-25-21-17-45.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()
WIN_LO, WIN_HI = 47.0, 51.5
sm_states = []
sm_debug = []
beh = []
mpc_msgs = []
force_trail = []

for topic, msg, t in b.read_messages(topics=[
        "/state_machine_mpc/debug",
        "/state_machine",
        "/behavior_strategy",
        "/planner/mpc/wpnts",
        "/opponent_prediction/force_trailing",
]):
    rel = t.to_sec() - t0
    if rel < WIN_LO or rel > WIN_HI:
        continue
    if topic == "/state_machine_mpc/debug":
        try:
            d = json.loads(msg.data)
        except Exception:
            d = {}
        sm_debug.append((rel, d))
    elif topic == "/state_machine":
        sm_states.append((rel, msg.data))
    elif topic == "/behavior_strategy":
        wp_list = msg.local_wpnts if isinstance(msg.local_wpnts, list) else (
            msg.local_wpnts.wpnts if hasattr(msg.local_wpnts, 'wpnts') else [])
        n = len(wp_list)
        first_d = wp_list[0].d_m if n > 0 else 0.0
        last_d = wp_list[-1].d_m if n > 0 else 0.0
        first_s = wp_list[0].s_m if n > 0 else 0.0
        last_s = wp_list[-1].s_m if n > 0 else 0.0
        n_trail = len(msg.trailing_targets) if msg.trailing_targets else 0
        # state field can be std_msgs/String or just str on this msg type
        st = msg.state.data if hasattr(msg.state, 'data') else str(msg.state)
        beh.append((rel, st, n, first_d, last_d, first_s, last_s, n_trail))
    elif topic == "/planner/mpc/wpnts":
        mpc_msgs.append((rel, len(msg.wpnts)))
    elif topic == "/opponent_prediction/force_trailing":
        force_trail.append((rel, bool(msg.data)))
b.close()

print(f"bag start (epoch {t0:.2f}), window {WIN_LO:.1f} ~ {WIN_HI:.1f}s\n")

print("=== /state_machine (state) ===")
prev = None
for rel, s in sm_states:
    if s != prev:
        print(f"  t={rel:6.3f}  {s}")
        prev = s

print("\n=== /behavior_strategy state + local_wpnts(n, d_first, d_last, s_first..s_last) + trail ===")
prev = None
for rel, st, n, df, dl, fs, ls, nt in beh:
    line = (f"  t={rel:6.3f}  state={st!r:20s}  n_wp={n:3d}  "
            f"d_first={df:+.3f}  d_last={dl:+.3f}  s={fs:.2f}..{ls:.2f}  trail={nt}")
    key = (st, nt, n, round(df, 2), round(dl, 2))
    if key != prev:
        print(line)
        prev = key

print("\n=== /state_machine_mpc/debug ===")
prev = None
for rel, d in sm_debug:
    sm_state = d.get("sm_state")
    src = d.get("path_source")
    mode = d.get("mpc_mode_echo")
    ot_age = d.get("ot_age_ms")
    rc_age = d.get("rc_age_ms")
    n_ego = d.get("n_ego")
    key = (sm_state, src, mode)
    if key != prev:
        print(f"  t={rel:6.3f}  st={sm_state!r:20s}  src={src!r:10s}  mode={mode!r}  "
              f"ot_age={ot_age}  rc_age={rc_age}  n_ego={n_ego}")
        prev = key

print("\n=== force_trailing ===")
prev = None
for rel, ft in force_trail:
    if ft != prev:
        print(f"  t={rel:6.3f}  force_trailing={ft}")
        prev = ft
print(f"\n/planner/mpc/wpnts msgs in window: {len(mpc_msgs)}")
