#!/usr/bin/env python3
"""Find /planner/mpc/wpnts gaps + check tick_json solve_ms around the flap."""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-25-21-17-45.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()
WIN_LO, WIN_HI = 47.0, 51.5

mpc_times = []  # (rel, ot_msg_seq)
tick = []  # (rel, solve_ms, mpc_mode, fail_streak)

for topic, msg, t in b.read_messages(topics=[
        "/planner/mpc/wpnts",
        "/mpc_auto/debug/tick_json",
        "/mpc_auto/timing_ms",
]):
    rel = t.to_sec() - t0
    if rel < WIN_LO or rel > WIN_HI:
        continue
    if topic == "/planner/mpc/wpnts":
        mpc_times.append(rel)
    elif topic == "/mpc_auto/debug/tick_json":
        try:
            d = json.loads(msg.data)
        except Exception:
            d = {}
        tick.append((rel, d.get("solve_ms"), d.get("mpc_mode"),
                     d.get("ipopt_status"), d.get("fail_streak"),
                     d.get("warm"), d.get("tier")))
    elif topic == "/mpc_auto/timing_ms":
        # also collect raw timing
        pass
b.close()

print("=== /planner/mpc/wpnts publish gaps (>40ms 만) ===")
prev = None
for rel in mpc_times:
    if prev is not None:
        gap = rel - prev
        if gap > 0.040:
            print(f"  t={rel:7.3f}  gap_from_prev={gap*1000:.1f}ms  (prev_t={prev:7.3f})")
    prev = rel

print(f"\n/planner/mpc/wpnts msgs in window: {len(mpc_times)}")
if len(mpc_times) > 1:
    avg = (mpc_times[-1] - mpc_times[0]) / max(len(mpc_times) - 1, 1)
    print(f"avg interval: {avg*1000:.1f}ms (so ~{1/avg:.1f}Hz)")

print("\n=== tick_json solve_ms 폭증 (>40ms) + 상태 ===")
for rec in tick:
    rel, s_ms, mode, status, fs, warm, tier = rec
    if s_ms is None: continue
    if float(s_ms) > 40.0:
        print(f"  t={rel:7.3f}  solve_ms={float(s_ms):.1f}  mode={mode}  ipopt={status}  tier={tier}  fail_streak={fs}  warm={warm}")

print("\n=== tick_json 48.6~50.0 (fully) ===")
for rec in tick:
    rel, s_ms, mode, status, fs, warm, tier = rec
    if 48.6 <= rel <= 50.0:
        print(f"  t={rel:7.3f}  solve_ms={s_ms}  mode={mode}  ipopt={status}  tier={tier}  fs={fs}  warm={warm}")
