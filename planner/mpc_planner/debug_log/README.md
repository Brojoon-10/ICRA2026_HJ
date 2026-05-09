# mpc_planner debug_log

Per-tick MPC solver logger for offline analysis.

## What gets saved

`runs/<YYYYMMDD_HHMMSS>_<state>_<git_sha>/`
- `meta.json` — params snapshot, solver settings, git sha
- `ticks.csv` — 1 row/tick, always (timeline, status, curvature, solve_ms, ...)
- `ticks/tick_NNNNNN.npz` — per-tick full dump (sampled or on anomaly)

Tier B NPZ trigger is `on_anomaly` by default — saves every tick that:
- entered a fallback tier (≥1)
- had non-success IPOPT status
- `traj_kappa_max > anomaly_kappa_max` (default 3.0 rad/m)
- `slack_max > 0.1`
- ego was outside the corridor

## Enabling from launch

```xml
<param name="debug_log_enable" value="true"/>
<param name="debug_log_dir" value="$(find mpc_planner)/debug_log/runs"/>
<param name="debug_log_tier_b" value="on_anomaly"/>    <!-- never|every_n|on_anomaly|always -->
<param name="debug_log_every_n" value="10"/>
<param name="debug_log_anomaly_kappa_max" value="3.0"/>
```

## Analysis scripts (run from `analysis/`)

```bash
python3 overview.py       <run_dir>                # CDFs, timelines, status pie
python3 plot_tick.py      <run_dir> <idx|worst-kappa|worst-slack>
python3 overlay_ticks.py  <run_dir> <start> <count>  # tick-to-tick oscillation
python3 cost_decomp.py    <run_dir> <idx|worst-kappa>
python3 find_anomalies.py <run_dir> --kappa=2.0 --tier=1 --not-succeeded
```

`pandas` + `matplotlib` + `numpy` required.
