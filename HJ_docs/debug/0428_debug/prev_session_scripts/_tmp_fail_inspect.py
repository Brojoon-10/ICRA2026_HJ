#!/usr/bin/env python3
"""Inspect WHY ipopt failed at t=49.805 and 49.941. Dump full tick_json."""
import rosbag
import json

BAG = "/home/nuc3/catkin_ws/src/race_stack/bag/2026-04-25-21-17-45.bag"
b = rosbag.Bag(BAG)
t0 = b.get_start_time()

PRINT_FIELDS = [
    "tick", "solve_ms", "mpc_mode", "alpha_ramp",
    "ipopt_status", "iter_count", "tier", "fail_streak",
    "warm",
    "ego_s", "ego_n", "ego_v", "ego_mu",
    "side", "side_raw", "bias_scale",
    "n_obs_used", "obs_source", "obs_freshness_ms",
    "corridor_dL_min", "corridor_dR_min",
    "margin_left_min", "margin_right_min",
    "slack_max", "slack_l2",
    "n_traj_max", "n_traj_min",
    "obs_tag",
]

for topic, msg, t in b.read_messages(topics=["/mpc_auto/debug/tick_json"]):
    rel = t.to_sec() - t0
    # Print for failures and the few ticks BEFORE/AFTER
    if 49.65 <= rel <= 50.05:
        try:
            d = json.loads(msg.data)
        except Exception:
            d = {}
        print(f"--- t={rel:.3f}s ---")
        for k in PRINT_FIELDS:
            if k in d:
                v = d[k]
                if isinstance(v, float):
                    print(f"  {k}: {v:.4f}")
                else:
                    print(f"  {k}: {v}")
        # Also dump fields we don't know about, in case they're useful
        skip = set(PRINT_FIELDS) | {"side_scores", "obs_active",
                                    "trajectory", "n_profile",
                                    "best_sample", "viz", "mode_dwell"}
        extras = sorted(set(d.keys()) - skip)
        if extras:
            print(f"  [+ extras: {', '.join(extras[:30])}]")
        # Side scores if present
        if "side_scores" in d and isinstance(d["side_scores"], dict):
            ss = d["side_scores"]
            print(f"  side_scores: {ss}")
        # Obstacle detail
        for ob_key in ("obs_n0", "obs_s_first", "obs_n_first", "obs_active",
                       "obs_dvar_max", "obs_min_ds"):
            if ob_key in d:
                print(f"  {ob_key}: {d[ob_key]}")
        print()

b.close()
