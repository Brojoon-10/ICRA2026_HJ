#!/usr/bin/env python3
"""
diagnose_sweep.py — separate "isolation issue" from "inherent burst load"
                    using the same summary.json files produced by run_sweep.

For each (condition, node) cell, classify into ONE of:

  ISOLATION_BROKEN     — cpu_leak_pp on the node's assigned core > 10pp
                         (glim work is leaking into the isolated domain)
  CONTENTION_ONLY      — cpu_ratio ≈ 1 (no more work) but wait_ratio > 1.3
                         (same workload, more wait → core starvation; better
                          isolation should fix this)
  LOAD_DRIVEN_ONLY     — cpu_ratio > 1.3 and wait_ratio ≈ cpu_ratio
                         (node simply has more work during burst; isolation
                          alone won't help, kernel/IRQ batching or upstream
                          queueing is delivering more callbacks per tick)
  LOAD_DRIVEN_PLUS_CONTENTION
                       — cpu_ratio > 1.3 AND wait_ratio >> cpu_ratio
                         (both: more work AND extra contention on top)
  CLEAN                — wait_ratio < 1.3 (nothing meaningfully bad in BURST)
  INSUFFICIENT_DATA    — too few burst or idle samples to compute

This is the analysis layer that converts raw measurements into the actual
question the user asked: "isolation 설정 문제인가, 그 시점 자체의 부하 폭증인가?"

Usage:
  python3 diagnose_sweep.py <sweep_dir>
"""
import argparse
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
        except (OSError, json.JSONDecodeError):
            continue
    return runs


def safe_ratio(a, b, default=None, floor=1e-3):
    """Return a/b clamped; if b is tiny return None (insufficient signal)."""
    if a is None or b is None:
        return default
    if abs(b) < floor:
        return default
    return a / b


# Pinned-core map: which cpus the apply_isolation.sh ctrl group lives on.
# When CTRL_CORES is the default 0,5 — sm/cm/vesc/frenet should be on cpu0 or cpu5.
# rslidar lives on cpu 12 (LIDAR_MODE=e) or 1,2 / 20,21 depending.
CTRL_CPUS = [0, 5]
LIDAR_CPUS_BY_MODE = {
    "e":  [12],
    "p":  [1, 2],
    "lp": [20, 21],
    "off": [],
}


def get_leak_for_node(d, node):
    """Return the worst leak_pp across the cores this node is supposed to live on.

    For sm/cm/vesc/frenet -> CTRL_CPUS.
    For rslidar -> LIDAR cores derived from the label (best-effort).
    Glim has no isolation domain, return None.

    Returns (leak_pp, ref_cpu) or (None, None) if not applicable / no data.
    """
    if node == "glim":
        return None, None
    cpu_leak = d.get("cpu_leak") or {}
    label = d.get("label", "")
    if node == "rslidar":
        # parse label suffix to pick the relevant lidar cores
        cores = []
        for mode, cs in LIDAR_CPUS_BY_MODE.items():
            if label.endswith("lidar_" + mode):
                cores = cs
                break
        if not cores:
            return None, None
    else:
        cores = CTRL_CPUS
    worst = None
    worst_cpu = None
    for c in cores:
        entry = cpu_leak.get(str(c)) or cpu_leak.get(c)
        if not entry:
            continue
        lp = entry.get("leak_pp")
        if lp is None:
            continue
        if worst is None or lp > worst:
            worst = lp
            worst_cpu = c
    return worst, worst_cpu


def classify(d, node,
             cpu_change_thresh=1.3,
             wait_change_thresh=1.3,
             contention_excess_thresh=1.5,
             leak_thresh_pp=10.0):
    ns = (d.get("nodes") or {}).get(node)
    if not ns:
        return "NO_NODE", {}
    bcpu = ns.get("burst_cpu_pct")
    icpu = ns.get("idle_cpu_pct")
    bwait = ns.get("burst_wait_ms_s")
    iwait = ns.get("idle_wait_ms_s")

    cpu_ratio = safe_ratio(bcpu, icpu)
    wait_ratio = safe_ratio(bwait, iwait)

    leak_pp, leak_cpu = get_leak_for_node(d, node)

    info = {
        "burst_cpu_pct": bcpu, "idle_cpu_pct": icpu,
        "burst_wait_ms_s": bwait, "idle_wait_ms_s": iwait,
        "cpu_ratio": cpu_ratio, "wait_ratio": wait_ratio,
        "leak_pp": leak_pp, "leak_cpu": leak_cpu,
    }

    if cpu_ratio is None or wait_ratio is None:
        return "INSUFFICIENT_DATA", info

    if leak_pp is not None and leak_pp > leak_thresh_pp:
        return "ISOLATION_BROKEN", info

    if wait_ratio < wait_change_thresh:
        return "CLEAN", info

    # wait clearly elevated. Is it because work went up, or because of contention?
    if cpu_ratio < cpu_change_thresh:
        # Same workload, more wait → starvation/contention
        return "CONTENTION_ONLY", info

    # More work in burst. Is wait elevation explained by work alone, or extra?
    # If wait_ratio is close to cpu_ratio (within contention_excess_thresh), it's
    # purely load-driven (node is busier so naturally waits more proportionally).
    excess = wait_ratio / cpu_ratio
    if excess < contention_excess_thresh:
        return "LOAD_DRIVEN_ONLY", info
    else:
        return "LOAD_DRIVEN_PLUS_CONTENTION", info


DIAGNOSIS_GLYPH = {
    "ISOLATION_BROKEN":           "BROKEN",
    "CONTENTION_ONLY":            "contend",
    "LOAD_DRIVEN_ONLY":           "load",
    "LOAD_DRIVEN_PLUS_CONTENTION": "load+ct",
    "CLEAN":                      "clean",
    "INSUFFICIENT_DATA":          "no_data",
    "NO_NODE":                    "(no)",
}


def print_diagnosis_matrix(runs):
    nodes = ["sm", "cm", "vesc", "frenet", "rslidar"]
    cols = [r[0] for r in runs]
    col_w = max(14, max(len(c) for c in cols) + 2)

    print("")
    print("=" * 110)
    print("DIAGNOSIS — what is the cause of BURST wait spike for each (condition, node)?")
    print("=" * 110)
    print("  BROKEN  : cpu_leak_pp on isolated core > 10pp  (taskset failed)")
    print("  contend : same CPU% in burst, but more wait    (core starvation — isolation can help)")
    print("  load    : higher CPU% in burst, wait ∝ work    (more work arrives — isolation alone won't help)")
    print("  load+ct : higher CPU% AND wait > work_factor   (both effects stacked)")
    print("  clean   : wait_ratio < 1.3                     (no meaningful burst impact)")
    print("")
    print(f"  {'node':10s}" + "".join(f"{c:>{col_w}s}" for c in cols))
    for n in nodes:
        line = f"  {n:10s}"
        for label, d in runs:
            diag, _ = classify(d, n)
            line += f"{DIAGNOSIS_GLYPH.get(diag, diag):>{col_w}s}"
        print(line)


def print_detail_per_node(runs, node):
    print("")
    print("-" * 110)
    print(f"DETAIL — {node}")
    print("-" * 110)
    print(f"  {'condition':22s} {'B CPU%':>8s} {'I CPU%':>8s} {'cpu_r':>6s} "
          f"{'B wait':>10s} {'I wait':>10s} {'wait_r':>7s} "
          f"{'leak':>6s} {'verdict':>30s}")
    for label, d in runs:
        diag, info = classify(d, node)
        def f(v, fs):
            return (fs % v) if v is not None else "  -"
        print(f"  {label:22s} "
              f"{f(info.get('burst_cpu_pct'), '%8.1f')} "
              f"{f(info.get('idle_cpu_pct'),  '%8.1f')} "
              f"{f(info.get('cpu_ratio'),     '%6.2f')} "
              f"{f(info.get('burst_wait_ms_s'),'%10.1f')} "
              f"{f(info.get('idle_wait_ms_s'), '%10.1f')} "
              f"{f(info.get('wait_ratio'),    '%7.2f')} "
              f"{f(info.get('leak_pp'),       '%6.1f')} "
              f"{diag:>30s}")


def print_summary_takeaway(runs):
    print("")
    print("=" * 110)
    print("TAKEAWAY — which conditions remove burst impact for which nodes?")
    print("=" * 110)
    nodes = ["sm", "cm", "vesc", "frenet", "rslidar"]
    # for each node, find the condition with lowest wait_ratio
    for n in nodes:
        best = None
        worst = None
        for label, d in runs:
            diag, info = classify(d, n)
            wr = info.get("wait_ratio")
            if wr is None:
                continue
            if best is None or wr < best[1]:
                best = (label, wr, diag, info)
            if worst is None or wr > worst[1]:
                worst = (label, wr, diag, info)
        if best:
            bcpu = best[3].get("burst_cpu_pct")
            print(f"  {n:8s}  BEST  {best[0]:22s}  wait_ratio={best[1]:.2f}  "
                  f"B_CPU%={bcpu if bcpu is not None else '-':>5}  → {best[2]}")
        if worst:
            wcpu = worst[3].get("burst_cpu_pct")
            print(f"  {n:8s}  WORST {worst[0]:22s}  wait_ratio={worst[1]:.2f}  "
                  f"B_CPU%={wcpu if wcpu is not None else '-':>5}  → {worst[2]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sweep_dir")
    args = ap.parse_args()

    runs = load_runs(args.sweep_dir)
    if not runs:
        print(f"[ERR] no summary.json under {args.sweep_dir}/*/", file=sys.stderr)
        sys.exit(1)

    print(f"[diagnose] loaded {len(runs)} conditions from {args.sweep_dir}")
    print_diagnosis_matrix(runs)
    for n in ["sm", "cm", "vesc", "frenet", "rslidar"]:
        print_detail_per_node(runs, n)
    print_summary_takeaway(runs)


if __name__ == "__main__":
    main()
