#!/usr/bin/env python3
"""
summarize_sweep.py — pretty-print comparison table from a sweep directory.

Reads <sweep_dir>/<label>/summary.json for each subdirectory and prints
condition-vs-condition tables for the key per-node and per-topic metrics.
Also writes a flat comparison CSV next to the input dir for spreadsheet use.

Usage:
  python3 summarize_sweep.py <sweep_dir>
"""
import argparse
import csv
import glob
import json
import os
import sys


def load_runs(sweep_dir):
    runs = []
    for sub in sorted(os.listdir(sweep_dir)):
        path = os.path.join(sweep_dir, sub)
        if not os.path.isdir(path):
            continue
        json_path = os.path.join(path, "summary.json")
        if not os.path.isfile(json_path):
            continue
        try:
            with open(json_path) as f:
                d = json.load(f)
            runs.append((sub, d))
        except (OSError, json.JSONDecodeError) as e:
            print(f"[WARN] skipping {json_path}: {e}", file=sys.stderr)
    return runs


def fmt(v, fmtstr):
    if v is None:
        return "  -"
    try:
        return fmtstr % v
    except (TypeError, ValueError):
        return str(v)


def print_overview(runs):
    print("")
    print("=" * 110)
    print("OVERVIEW")
    print("=" * 110)
    header = f"{'condition':22s} {'dur_s':>6s} {'ticks':>6s} {'%burst':>7s} {'throttle/min':>13s}"
    print(header)
    print("-" * 110)
    for label, d in runs:
        print(f"{label:22s} "
              f"{fmt(d.get('duration_s'),'%6.1f')} "
              f"{fmt(d.get('n_ticks_total'),'%6d')} "
              f"{fmt(d.get('pct_burst'),'%7.1f')} "
              f"{fmt(d.get('throttle_per_min'),'%13.1f')}")


def print_node_table(runs, node, metric_key, fmtstr, header_label):
    print("")
    print(f"--- {node.upper():6s}  {header_label} ---")
    cols = [r[0] for r in runs]
    col_w = max(12, max(len(c) for c in cols) + 2)
    head = " " * 18 + "".join(f"{c:>{col_w}s}" for c in cols)
    print(head)
    burst_row = "  BURST           "
    idle_row  = "  IDLE            "
    delta_row = "  DELTA (b-i)     "
    ratio_row = "  RATIO (b/i)     "
    for label, d in runs:
        nstats = (d.get("nodes") or {}).get(node)
        if not nstats:
            burst_row += f"{'(no)':>{col_w}s}"
            idle_row  += f"{'(no)':>{col_w}s}"
            delta_row += f"{'(no)':>{col_w}s}"
            ratio_row += f"{'(no)':>{col_w}s}"
            continue
        b = nstats.get("burst_" + metric_key)
        i = nstats.get("idle_" + metric_key)
        burst_row += f"{fmt(b, fmtstr):>{col_w}s}"
        idle_row  += f"{fmt(i, fmtstr):>{col_w}s}"
        if b is not None and i is not None:
            delta_row += f"{fmt(b - i, fmtstr):>{col_w}s}"
            ratio_row += (f"{(b/i):>{col_w-1}.2f}x" if i > 1e-9 else f"{'inf':>{col_w}s}")
        else:
            delta_row += f"{'-':>{col_w}s}"
            ratio_row += f"{'-':>{col_w}s}"
    print(burst_row)
    print(idle_row)
    print(delta_row)
    print(ratio_row)


def print_node_event_table(runs):
    print("")
    print(f"--- WAIT EVENTS >50ms per minute ---")
    cols = [r[0] for r in runs]
    col_w = max(12, max(len(c) for c in cols) + 2)
    head = " " * 12 + "".join(f"{c:>{col_w}s}" for c in cols)
    print(head)
    for node in ["sm", "cm", "vesc", "frenet", "rslidar"]:
        row = f"  {node:8s}  "
        for label, d in runs:
            ns = (d.get("nodes") or {}).get(node)
            v = ns.get("wait_events_per_min") if ns else None
            row += f"{fmt(v, '%' + str(col_w-2) + '.2f'):>{col_w}s}"
        print(row)


def print_topic_table(runs, topic):
    print("")
    print(f"--- TOPIC {topic}   (ia = inter-arrival, lat = stamp→arrival) ---")
    cols = [r[0] for r in runs]
    col_w = max(12, max(len(c) for c in cols) + 2)
    head = " " * 28 + "".join(f"{c:>{col_w}s}" for c in cols)
    print(head)

    rows = [
        ("hz overall",          lambda ts: ts.get("hz_overall") if ts else None,           "%.1f"),
        ("drops/min",           lambda ts: ts.get("drop_per_min") if ts else None,         "%.2f"),
        ("BURST ia mean ms",    lambda ts: (ts.get("burst") or {}).get("ia_mean_ms") if ts else None,  "%.2f"),
        ("BURST ia stdev ms",   lambda ts: (ts.get("burst") or {}).get("ia_stdev_ms") if ts else None, "%.2f"),
        ("BURST ia p95 ms",     lambda ts: (ts.get("burst") or {}).get("ia_p95_ms") if ts else None,   "%.2f"),
        ("BURST ia max ms",     lambda ts: (ts.get("burst") or {}).get("ia_max_ms") if ts else None,   "%.2f"),
        ("BURST lat mean ms",   lambda ts: (ts.get("burst") or {}).get("lat_mean_ms") if ts else None, "%.1f"),
        ("BURST lat p95 ms",    lambda ts: (ts.get("burst") or {}).get("lat_p95_ms") if ts else None,  "%.1f"),
        ("BURST lat max ms",    lambda ts: (ts.get("burst") or {}).get("lat_max_ms") if ts else None,  "%.1f"),
        ("IDLE  ia mean ms",    lambda ts: (ts.get("idle") or {}).get("ia_mean_ms") if ts else None,   "%.2f"),
        ("IDLE  ia max ms",     lambda ts: (ts.get("idle") or {}).get("ia_max_ms") if ts else None,    "%.2f"),
        ("IDLE  lat mean ms",   lambda ts: (ts.get("idle") or {}).get("lat_mean_ms") if ts else None,  "%.1f"),
        ("IDLE  lat max ms",    lambda ts: (ts.get("idle") or {}).get("lat_max_ms") if ts else None,   "%.1f"),
    ]
    for label_row, getter, fmts in rows:
        line = f"  {label_row:25s} "
        for label, d in runs:
            ts = (d.get("topics") or {}).get(topic)
            v = getter(ts)
            line += f"{fmt(v, '%' + str(col_w-2) + fmts.replace('%','')):>{col_w}s}" \
                if v is not None else f"{'-':>{col_w}s}"
        print(line)


def print_leak_table(runs):
    print("")
    print(f"--- per-CPU LEAK during BURST (burst%active - idle%active)  -- ctrl=cpu0,5  lidar=cpu12 ---")
    cols = [r[0] for r in runs]
    col_w = max(12, max(len(c) for c in cols) + 2)
    head = " " * 14 + "".join(f"{c:>{col_w}s}" for c in cols)
    print(head)
    for cpu_n in [0, 5, 12]:
        row = f"  cpu{cpu_n:>2d} leak_pp"
        for label, d in runs:
            cl = (d.get("cpu_leak") or {}).get(str(cpu_n)) or (d.get("cpu_leak") or {}).get(cpu_n)
            v = cl.get("leak_pp") if cl else None
            row += f"{fmt(v,'%' + str(col_w-2) + '.1f'):>{col_w}s}"
        print(row)


def write_csv(runs, out_path):
    """Flat CSV: one row per condition, columns are flattened key paths."""
    if not runs:
        return
    fieldnames = ["label", "duration_s", "n_ticks_total", "n_ticks_burst",
                  "pct_burst", "throttle_count_delta", "throttle_per_min"]
    nodes = ["sm", "cm", "vesc", "frenet", "rslidar", "glim"]
    node_metrics = ["burst_cpu_pct", "idle_cpu_pct",
                    "burst_wait_ms_s", "idle_wait_ms_s",
                    "burst_wait_p95_ms_s", "idle_wait_p95_ms_s",
                    "wait_events_per_min", "wait_ms_max"]
    for n in nodes:
        for m in node_metrics:
            fieldnames.append(f"{n}.{m}")
    for cpu in [0, 5, 12]:
        fieldnames.append(f"cpu{cpu}.leak_pp")
    topic_keys = ["rslidar_points", "base_odom", "vesc_odom"]
    topic_metrics = [
        ("hz_overall",        lambda ts: ts.get("hz_overall") if ts else None),
        ("drop_per_min",      lambda ts: ts.get("drop_per_min") if ts else None),
        ("burst_ia_mean_ms",  lambda ts: (ts.get("burst") or {}).get("ia_mean_ms") if ts else None),
        ("burst_ia_stdev_ms", lambda ts: (ts.get("burst") or {}).get("ia_stdev_ms") if ts else None),
        ("burst_ia_p95_ms",   lambda ts: (ts.get("burst") or {}).get("ia_p95_ms") if ts else None),
        ("burst_ia_max_ms",   lambda ts: (ts.get("burst") or {}).get("ia_max_ms") if ts else None),
        ("burst_lat_mean_ms", lambda ts: (ts.get("burst") or {}).get("lat_mean_ms") if ts else None),
        ("burst_lat_p95_ms",  lambda ts: (ts.get("burst") or {}).get("lat_p95_ms") if ts else None),
        ("burst_lat_max_ms",  lambda ts: (ts.get("burst") or {}).get("lat_max_ms") if ts else None),
        ("idle_ia_mean_ms",   lambda ts: (ts.get("idle") or {}).get("ia_mean_ms") if ts else None),
        ("idle_ia_max_ms",    lambda ts: (ts.get("idle") or {}).get("ia_max_ms") if ts else None),
        ("idle_lat_mean_ms",  lambda ts: (ts.get("idle") or {}).get("lat_mean_ms") if ts else None),
        ("idle_lat_max_ms",   lambda ts: (ts.get("idle") or {}).get("lat_max_ms") if ts else None),
    ]
    for t in topic_keys:
        for m, _ in topic_metrics:
            fieldnames.append(f"topic.{t}.{m}")

    with open(out_path, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=fieldnames)
        w.writeheader()
        for label, d in runs:
            row = {"label": label}
            for k in ["duration_s", "n_ticks_total", "n_ticks_burst", "pct_burst",
                     "throttle_count_delta", "throttle_per_min"]:
                row[k] = d.get(k)
            for n in nodes:
                ns = (d.get("nodes") or {}).get(n) or {}
                for m in node_metrics:
                    row[f"{n}.{m}"] = ns.get(m)
            for cpu in [0, 5, 12]:
                cl = (d.get("cpu_leak") or {}).get(str(cpu)) or (d.get("cpu_leak") or {}).get(cpu) or {}
                row[f"cpu{cpu}.leak_pp"] = cl.get("leak_pp")
            for t in topic_keys:
                ts = (d.get("topics") or {}).get(t)
                for m, getter in topic_metrics:
                    row[f"topic.{t}.{m}"] = getter(ts)
            w.writerow(row)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sweep_dir")
    args = ap.parse_args()

    runs = load_runs(args.sweep_dir)
    if not runs:
        print(f"[ERR] no summary.json found under {args.sweep_dir}/*/", file=sys.stderr)
        sys.exit(1)

    print_overview(runs)

    for node in ["sm", "cm", "vesc", "frenet", "rslidar"]:
        print_node_table(runs, node, "wait_ms_s", "%9.2f",  "wait ms/s (sum across threads)")
    for node in ["sm", "cm"]:
        print_node_table(runs, node, "cpu_pct",   "%9.1f",  "CPU% (% of 1 core, summed threads)")

    print_node_event_table(runs)
    print_leak_table(runs)

    for t in ["rslidar_points", "base_odom", "vesc_odom"]:
        print_topic_table(runs, t)

    csv_path = os.path.join(args.sweep_dir, "_comparison.csv")
    write_csv(runs, csv_path)
    print("")
    print(f"[INFO] flat CSV written: {csv_path}")


if __name__ == "__main__":
    main()
