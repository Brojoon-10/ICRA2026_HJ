#!/usr/bin/env python3
"""
measure_one.py — single-condition burst-aligned impact measurement.

Run for DURATION seconds inside container with ROS + race-stack live.
Identifies glim "burst" 50ms ticks (R-count >= BURST_R), classifies remaining
ticks as "idle" (R-count <= IDLE_R), then compares per-node CPU/wait_ns
deltas and per-topic inter-arrival jitter between the two populations.

Compared to burst_cross_impact.py:
  - covers sm + cm in addition to rslidar/vesc/frenet
  - covers topics base_odom / vesc/odom in addition to rslidar_points
  - emits one-line JSON summary on the LAST stdout line for sweep aggregation
  - reads throttle counter delta over the window

Output:
  stdout : human-readable report + final line is JSON dict
  --json-out PATH writes the JSON dict to PATH (recommended for sweeps)

Usage:
  ./measure_one.py 120
  ./measure_one.py 120 --label workers_lidar_e --json-out /tmp/x.json

No external deps. Needs rospy importable (source the workspace first).
"""
import argparse
import json
import os
import statistics as S
import sys
import threading
import time
from collections import defaultdict


def find_pid(pattern):
    try:
        out = os.popen(f"pgrep -f '{pattern}'").read().split()
        for p in out:
            try:
                return int(p)
            except ValueError:
                continue
    except Exception:
        pass
    return None


def discover_pids():
    return {
        "glim":    find_pid("__name:=glim_ros"),
        "rslidar": find_pid("rslidar_sdk_node"),
        "vesc":    find_pid("__name:=vesc"),
        "frenet":  find_pid("__name:=frenet_conversion_server"),
        "sm":      find_pid("3d_state_machine_node"),
        "cm":      find_pid("controller_manager"),
    }


def read_proc_aggregate(pid):
    """Sum jiffies and wait_ns across threads. Returns (jiffies, wait_ns, R_count)."""
    task = f"/proc/{pid}/task"
    if not os.path.isdir(task):
        return (0, 0, 0)
    j = w = r = 0
    try:
        tids = os.listdir(task)
    except FileNotFoundError:
        return (0, 0, 0)
    for tid in tids:
        try:
            with open(f"{task}/{tid}/stat") as f:
                s = f.read().split()
            j += int(s[13]) + int(s[14])
            if s[2] == 'R':
                r += 1
            with open(f"{task}/{tid}/schedstat") as f:
                ss = f.read().split()
            w += int(ss[1])
        except (FileNotFoundError, IndexError, ValueError):
            pass
    return (j, w, r)


def read_proc_stat_cpus():
    out = {}
    try:
        with open("/proc/stat") as f:
            for line in f:
                if not line.startswith("cpu"):
                    continue
                parts = line.split()
                name = parts[0]
                if name == "cpu":
                    continue
                try:
                    n = int(name[3:])
                except ValueError:
                    continue
                nums = [int(x) for x in parts[1:9]]
                active = nums[0] + nums[1] + nums[2] + nums[5] + nums[6] + nums[7]
                idle = nums[3] + nums[4]
                out[n] = (active, idle)
    except OSError:
        pass
    return out


def read_throttle_count():
    total = 0
    try:
        for n in os.listdir("/sys/devices/system/cpu"):
            if not n.startswith("cpu"):
                continue
            path = f"/sys/devices/system/cpu/{n}/thermal_throttle/core_throttle_count"
            try:
                with open(path) as f:
                    total += int(f.read().strip())
            except (OSError, ValueError):
                pass
    except OSError:
        pass
    return total


def percentile(lst, q):
    if not lst:
        return 0.0
    s = sorted(lst)
    k = max(0, min(len(s) - 1, int(round(q * (len(s) - 1)))))
    return float(s[k])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("duration", type=float)
    ap.add_argument("--label", default="run")
    ap.add_argument("--json-out", default=None)
    ap.add_argument("--sample-hz", type=float, default=20.0,
                    help="proc sampler rate (default 20Hz = 50ms ticks)")
    ap.add_argument("--burst-r", type=int, default=8,
                    help="glim thread R-count threshold for BURST ticks")
    ap.add_argument("--idle-r", type=int, default=2,
                    help="glim thread R-count threshold for IDLE ticks")
    ap.add_argument("--wait-trip-ms", type=float, default=50.0,
                    help="threshold for counting wait events")
    args = ap.parse_args()

    import rospy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry

    pids = discover_pids()
    print(f"[INFO] label={args.label}  duration={args.duration}s  sample={args.sample_hz}Hz")
    print(f"[INFO] PIDs:")
    for k, v in pids.items():
        print(f"  {k:8s} = {v}")
    missing = [k for k, v in pids.items() if v is None]
    if missing:
        print(f"[WARN] missing pids: {missing}")

    # per-node time series : list of (mono, R, jiffies, wait_ns)
    samples = {k: [] for k in pids}
    # per-CPU jiffies
    cpu_samples = []  # (mono, {cpu_n: (active, idle)})

    # ROS topic arrival buffers: (mono, wall_arrival_sec, header_stamp_sec)
    # - mono           : monotonic clock at callback fire — for inter-arrival jitter
    # - wall_arrival   : time.time() at callback fire     — same epoch as header.stamp
    # - header_stamp   : driver's stamp (sensor capture time)
    # → latency_ms = (wall_arrival - header_stamp) * 1000
    arr_rslidar = []
    arr_base_odom = []
    arr_vesc_odom = []

    rospy.init_node("hj_isolation_sweep_measure", anonymous=True,
                    disable_signals=True)

    def cb_pc(msg):
        arr_rslidar.append((time.monotonic(), time.time(), msg.header.stamp.to_sec()))

    def cb_base(msg):
        arr_base_odom.append((time.monotonic(), time.time(), msg.header.stamp.to_sec()))

    def cb_vesc(msg):
        arr_vesc_odom.append((time.monotonic(), time.time(), msg.header.stamp.to_sec()))

    rospy.Subscriber("/rslidar_points", PointCloud2, cb_pc,
                     queue_size=500, tcp_nodelay=True)
    rospy.Subscriber("/glim_ros/base_odom", Odometry, cb_base,
                     queue_size=500, tcp_nodelay=True)
    rospy.Subscriber("/vesc/odom", Odometry, cb_vesc,
                     queue_size=500, tcp_nodelay=True)
    time.sleep(0.5)  # subscriber settle

    throttle_t0 = read_throttle_count()
    t0 = time.monotonic()

    def sampler():
        while time.monotonic() - t0 < args.duration:
            m = time.monotonic()
            for name, pid in pids.items():
                if pid is None:
                    continue
                j, w, r = read_proc_aggregate(pid)
                samples[name].append((m, r, j, w))
            cpu_samples.append((m, read_proc_stat_cpus()))
            nxt = m + 1.0 / args.sample_hz
            slp = nxt - time.monotonic()
            if slp > 0:
                time.sleep(slp)

    th = threading.Thread(target=sampler, daemon=True)
    th.start()
    # main thread spins ROS — subscriber callbacks fire on background spinners
    while time.monotonic() - t0 < args.duration:
        time.sleep(0.2)
    th.join(timeout=2.0)
    throttle_t1 = read_throttle_count()

    actual_dur = time.monotonic() - t0

    # -------- classify glim ticks --------
    glim_seq = samples["glim"]
    burst_idx = set()
    idle_idx = set()
    for i, (m, r, _, _) in enumerate(glim_seq):
        if r >= args.burst_r:
            burst_idx.add(i)
        elif r <= args.idle_r:
            idle_idx.add(i)

    n_burst = len(burst_idx)
    n_idle = len(idle_idx)
    n_total = len(glim_seq)

    # -------- per-node delta rate helper --------
    def per_tick_rate(lst, idxs, field_idx):
        """field_idx: 2=jiffies, 3=wait_ns. Returns deltas per second."""
        out = []
        for i in idxs:
            if i == 0 or i >= len(lst):
                continue
            dt = lst[i][0] - lst[i - 1][0]
            if dt <= 0:
                continue
            d = lst[i][field_idx] - lst[i - 1][field_idx]
            out.append(d / dt)
        return out

    # -------- per-node aggregates --------
    node_stats = {}
    HZ = os.sysconf("SC_CLK_TCK")
    for node in ["sm", "cm", "vesc", "frenet", "rslidar", "glim"]:
        if pids[node] is None:
            node_stats[node] = None
            continue
        lst = samples[node]
        if not lst:
            node_stats[node] = None
            continue
        # CPU% over entire window (averaged across threads/cores not normalized — raw jiffies/sec converted to "% of 1 core")
        bj = per_tick_rate(lst, burst_idx, 2)
        ij = per_tick_rate(lst, idle_idx, 2)
        bw = per_tick_rate(lst, burst_idx, 3)
        iw = per_tick_rate(lst, idle_idx, 3)

        # wait events crossing trip threshold across all ticks (per-tick wait_ms)
        wait_events = 0
        wait_ms_all = []
        for i in range(1, len(lst)):
            dt = lst[i][0] - lst[i - 1][0]
            if dt <= 0:
                continue
            dw = (lst[i][3] - lst[i - 1][3]) / 1e6  # ms total summed across threads
            wait_ms_all.append(dw)
            if dw >= args.wait_trip_ms:
                wait_events += 1

        node_stats[node] = {
            "burst_cpu_pct":   100.0 * (S.mean(bj) / HZ) if bj else None,
            "idle_cpu_pct":    100.0 * (S.mean(ij) / HZ) if ij else None,
            "burst_wait_ms_s": (S.mean(bw) / 1e6) if bw else None,
            "idle_wait_ms_s":  (S.mean(iw) / 1e6) if iw else None,
            "burst_wait_p95_ms_s": (percentile(bw, 0.95) / 1e6) if bw else None,
            "idle_wait_p95_ms_s":  (percentile(iw, 0.95) / 1e6) if iw else None,
            "wait_events_over_trip": wait_events,
            "wait_events_per_min":   60.0 * wait_events / actual_dur,
            "wait_ms_max":           max(wait_ms_all) if wait_ms_all else None,
        }

    # -------- per-CPU LEAK check --------
    def cpu_active_pct(idxs, cpu_n):
        out = []
        for i in idxs:
            if i == 0 or i >= len(cpu_samples):
                continue
            a0, i0 = cpu_samples[i - 1][1].get(cpu_n, (0, 0))
            a1, i1 = cpu_samples[i][1].get(cpu_n, (0, 0))
            da = a1 - a0
            di = i1 - i0
            tot = da + di
            if tot > 0:
                out.append(100.0 * da / tot)
        return S.mean(out) if out else None

    cpu_leak = {}
    for cpu_n in [0, 5, 12]:
        b = cpu_active_pct(burst_idx, cpu_n)
        i = cpu_active_pct(idle_idx, cpu_n)
        cpu_leak[cpu_n] = {
            "burst_pct": b,
            "idle_pct": i,
            "leak_pp": (b - i) if (b is not None and i is not None) else None,
        }

    # -------- per-topic inter-arrival + latency in BURST vs IDLE --------
    # arr entries are (mono, wall_arrival_sec, header_stamp_sec)
    def topic_stats(arr, expected_hz=None):
        """expected_hz: nominal rate; drop_event = inter-arrival > 1.5 / expected_hz."""
        if len(arr) < 3:
            return None
        burst_ia = []
        idle_ia = []
        burst_lat = []
        idle_lat = []
        drop_events = 0
        drop_thresh_ms = (1500.0 / expected_hz) if expected_hz else None
        for k in range(len(arr) - 1):
            m0 = arr[k][0]
            m1 = arr[k + 1][0]
            ia = (m1 - m0) * 1000.0
            # end-to-end stamp-to-arrival latency (this sample)
            lat = (arr[k][1] - arr[k][2]) * 1000.0 if arr[k][2] > 0 else None
            tick_idx = int((m0 - t0) * args.sample_hz)
            if 0 <= tick_idx < len(glim_seq):
                if tick_idx in burst_idx:
                    burst_ia.append(ia)
                    if lat is not None: burst_lat.append(lat)
                elif tick_idx in idle_idx:
                    idle_ia.append(ia)
                    if lat is not None: idle_lat.append(lat)
            if drop_thresh_ms is not None and ia > drop_thresh_ms:
                drop_events += 1
        # last frame latency too
        last_lat = (arr[-1][1] - arr[-1][2]) * 1000.0 if arr[-1][2] > 0 else None
        if last_lat is not None:
            tick_idx = int((arr[-1][0] - t0) * args.sample_hz)
            if 0 <= tick_idx < len(glim_seq):
                if tick_idx in burst_idx: burst_lat.append(last_lat)
                elif tick_idx in idle_idx: idle_lat.append(last_lat)

        def st(ia_lst, lat_lst):
            if not ia_lst:
                return None
            d = {
                "n": len(ia_lst),
                "ia_mean_ms":  S.mean(ia_lst),
                "ia_stdev_ms": S.stdev(ia_lst) if len(ia_lst) > 1 else 0.0,
                "ia_p95_ms":   percentile(ia_lst, 0.95),
                "ia_max_ms":   max(ia_lst),
            }
            if lat_lst:
                d.update({
                    "lat_mean_ms": S.mean(lat_lst),
                    "lat_p95_ms":  percentile(lat_lst, 0.95),
                    "lat_max_ms":  max(lat_lst),
                })
            return d

        return {
            "n_total": len(arr),
            "hz_overall": (len(arr) - 1) / (arr[-1][0] - arr[0][0]) if arr[-1][0] > arr[0][0] else None,
            "drop_events": drop_events,
            "drop_per_min": 60.0 * drop_events / actual_dur if actual_dur > 0 else 0.0,
            "drop_thresh_ms": drop_thresh_ms,
            "burst": st(burst_ia, burst_lat),
            "idle":  st(idle_ia,  idle_lat),
        }

    # Expected rates (used to define "frame drop"):
    #   rslidar  = 10 Hz nominal (RS-LiDAR-M1/Helios)
    #   base_odom = 75 Hz nominal (glim_ros publishing odom at sensor rate-ish)
    #   vesc_odom = ~50 Hz nominal (vesc_driver default)
    topic_stats_out = {
        "rslidar_points": topic_stats(arr_rslidar, expected_hz=10.0),
        "base_odom":      topic_stats(arr_base_odom, expected_hz=75.0),
        "vesc_odom":      topic_stats(arr_vesc_odom, expected_hz=50.0),
    }

    summary = {
        "label": args.label,
        "duration_s": round(actual_dur, 2),
        "n_ticks_total": n_total,
        "n_ticks_burst": n_burst,
        "n_ticks_idle":  n_idle,
        "pct_burst":     100.0 * n_burst / n_total if n_total else 0.0,
        "throttle_count_delta": throttle_t1 - throttle_t0,
        "throttle_per_min":     60.0 * (throttle_t1 - throttle_t0) / actual_dur if actual_dur > 0 else 0.0,
        "nodes":     node_stats,
        "cpu_leak":  cpu_leak,
        "topics":    topic_stats_out,
    }

    # -------- human report --------
    print("")
    print("=" * 76)
    print(f"[REPORT] label={args.label}  dur={actual_dur:.1f}s")
    print(f"  ticks: total={n_total}  burst={n_burst} ({summary['pct_burst']:.1f}%)  idle={n_idle}")
    print(f"  throttle delta: {summary['throttle_count_delta']}  "
          f"({summary['throttle_per_min']:.1f}/min)")
    print("")
    print(f"  {'node':8s} {'burst CPU%':>10s} {'idle CPU%':>10s} "
          f"{'burst wait ms/s':>16s} {'idle wait ms/s':>16s} "
          f"{'>50ms ev/min':>12s} {'max ms':>8s}")
    for node in ["sm", "cm", "vesc", "frenet", "rslidar", "glim"]:
        s = node_stats.get(node)
        if s is None:
            print(f"  {node:8s}  (no pid)")
            continue
        def f(x, fmt):
            return fmt % x if x is not None else "  -"
        print(f"  {node:8s} {f(s['burst_cpu_pct'],'%10.1f')} {f(s['idle_cpu_pct'],'%10.1f')} "
              f"{f(s['burst_wait_ms_s'],'%16.2f')} {f(s['idle_wait_ms_s'],'%16.2f')} "
              f"{f(s['wait_events_per_min'],'%12.2f')} {f(s['wait_ms_max'],'%8.1f')}")

    print("")
    print(f"  per-CPU LEAK (>10pp delta means burst leaks onto pinned domain):")
    for cpu_n, d in cpu_leak.items():
        dom = {0: "ctrl", 5: "ctrl", 12: "lidar"}[cpu_n]
        b = d["burst_pct"]; i = d["idle_pct"]; lp = d["leak_pp"]
        flag = "  LEAK!" if (lp is not None and lp > 10) else ""
        print(f"    cpu{cpu_n:>2d} ({dom:5s})  burst={b if b is not None else '-':>6} "
              f"idle={i if i is not None else '-':>6}  leak_pp={lp if lp is not None else '-':>6}{flag}")

    print("")
    print(f"  per-topic (burst vs idle):")
    print(f"    inter-arrival ms: how spread-out frames are (jitter)")
    print(f"    latency ms       : stamp→callback delay (queue/processing backup)")
    print(f"    drops/min        : inter-arrival > 1.5x nominal period")
    for tname, ts in topic_stats_out.items():
        if ts is None:
            print(f"    {tname:18s}  (insufficient data)")
            continue
        b = ts["burst"]; i = ts["idle"]
        hz = ts["hz_overall"]
        thr = ts.get("drop_thresh_ms")
        thr_str = f"  drop_thr={thr:.0f}ms  drops={ts['drop_events']} ({ts['drop_per_min']:.2f}/min)" if thr else ""
        hz_str = f"  hz_overall={hz:.1f}" if hz else ""
        print(f"    {tname:18s}  n={ts['n_total']}{hz_str}{thr_str}")
        for tag, s in (("BURST", b), ("IDLE", i)):
            if not s: continue
            ia = f"ia mean={s['ia_mean_ms']:6.2f}  std={s['ia_stdev_ms']:6.2f}  p95={s['ia_p95_ms']:6.2f}  max={s['ia_max_ms']:7.2f}"
            if "lat_mean_ms" in s:
                lat = f"  lat mean={s['lat_mean_ms']:6.1f}  p95={s['lat_p95_ms']:6.1f}  max={s['lat_max_ms']:7.1f}"
            else:
                lat = "  lat (no header stamp)"
            print(f"      {tag:5s} n={s['n']:4d}  {ia}{lat}")

    if args.json_out:
        try:
            with open(args.json_out, "w") as f:
                json.dump(summary, f, indent=2)
            print(f"\n[INFO] wrote {args.json_out}")
        except OSError as e:
            print(f"[ERR] could not write {args.json_out}: {e}", file=sys.stderr)

    # last line is JSON for sweep aggregator
    print("__SUMMARY_JSON__ " + json.dumps(summary))


if __name__ == "__main__":
    main()
