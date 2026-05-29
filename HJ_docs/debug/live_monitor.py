#!/usr/bin/env python3
# Live monitor v2 — delay/wait 중심 (CPU 안 봄)
# 100ms 단위 fine-grained 샘플링 → 5s마다 max wait, p95, event count 보고
#
# 5 axes (what we watch): rslidar input, lidar-odom(glim worker), base_odom, sm, cm.
# - PIDs are re-discovered every report so node restarts are tracked automatically;
#   a node that is down simply shows blank (does not break the rest).
# - With --json-out <path>, the latest slice is written as one JSON line each
#   report so an external tool (set_pl.sh monitor) can merge it. Loose coupling:
#   the consumer treats a missing/stale file as "no data".
import os, sys, time, json, argparse
from collections import deque

INTERVAL = 5.0
SAMPLE_DT = 0.1   # 100ms 내부 샘플링
HZ = os.sysconf("SC_CLK_TCK")

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

ap = argparse.ArgumentParser()
ap.add_argument("--json-out", default=os.environ.get("LIVE_MONITOR_JSON", ""),
                help="path to write latest-slice JSON (one line, overwritten each report)")
args = ap.parse_args()

# Per-key match rule: (required_tokens, alt_script_tokens).
# A process qualifies iff its cmdline contains ALL of required_tokens AND ANY
# of alt_script_tokens. The required token (`__name:=...`) is the unambiguous
# roslaunch-assigned ROS node name. The alt list covers script variants of the
# *same logical node* across launch files — they are never up simultaneously,
# so picking whichever one is currently running is unambiguous.
# Demanding a script token (and not just `__name:=`) excludes the launch parent
# and wrapper shells (they print the node name as an argument but don't carry
# the script path) — this is what fixed the prior sm/cm 0ms bug.
NODE_MATCH = {
    # sm has two script variants: 3d_state_machine_node.py (standard headtohead
    # launch) vs fbga_state_machine_node.py (3d_headtohead_fbga.launch with the
    # C++ FBGA state machine). Both register under __name:=state_machine.
    "sm":   (["__name:=state_machine"],     ["3d_state_machine_node", "fbga_state_machine_node"]),
    "cm":   (["__name:=controller_manager"], ["controller_manager.py"]),
    "glim": (["__name:=glim_ros"],          ["glim_rosnode"]),
}
SELF_PID = os.getpid()


def _cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\0", b" ").decode("utf-8", "replace")
    except OSError:
        return ""


def find_pid(key):
    """Return the pid of the real node for `key`, or None. Matches a process iff
    its cmdline contains ALL required tokens AND ANY of the alt script tokens.
    """
    required, alts = NODE_MATCH[key]
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        p = int(pid)
        if p == SELF_PID:
            continue
        cl = _cmdline(p)
        if cl and all(t in cl for t in required) and any(t in cl for t in alts):
            return p
    return None


def discover_pids():
    return {k: find_pid(k) for k in NODE_MATCH}


pids = discover_pids()


def read_wait_ns(pid):
    """sum of wait_ns across all threads of pid"""
    if pid is None:
        return 0
    w = 0
    try:
        for tid in os.listdir(f"/proc/{pid}/task"):
            try:
                with open(f"/proc/{pid}/task/{tid}/schedstat") as f:
                    w += int(f.read().split()[1])
            except (FileNotFoundError, IndexError, ValueError):
                pass
    except FileNotFoundError:
        pass
    return w


arr_base = deque(maxlen=5000)
arr_rsl = deque(maxlen=500)

def cb_base(msg): arr_base.append(time.monotonic())
def cb_rsl(msg):  arr_rsl.append(time.monotonic())

rospy.init_node("hj_live_monitor", anonymous=True, disable_signals=True)
rospy.Subscriber("/glim_ros/base_odom", Odometry, cb_base, queue_size=1000, tcp_nodelay=True)
rospy.Subscriber("/rslidar_points", PointCloud2, cb_rsl, queue_size=200, tcp_nodelay=True)
time.sleep(0.5)


def percentile(lst, q):
    if not lst:
        return 0.0
    s = sorted(lst)
    k = max(0, min(len(s)-1, int(round(q*(len(s)-1)))))
    return s[k]


print(f"# Delay monitor — INTERVAL={INTERVAL}s, internal sample={SAMPLE_DT*1000:.0f}ms")
print(f"# PIDs: sm={pids['sm']} cm={pids['cm']} glim={pids['glim']}")
print(f"# 100ms 슬라이스마다 각 노드 thread wait_ns 증가량 측정")
print(f"# 5s 동안 그 증가량의 max / p95 / 5ms 초과 슬라이스 카운트")
print(f"# {'time':8s}  {'sm_mx':>6s} {'sm_p95':>6s} {'sm>5':>4s}  {'cm_mx':>6s} {'cm_p95':>6s} {'cm>5':>4s}  "
      f"{'gl_mx':>6s} {'gl_p95':>6s}  {'b_hz':>5s} {'b_mxg':>6s}  {'r_hz':>5s} {'r_mxg':>6s}  flags")
sys.stdout.flush()

prev_wait = {k: read_wait_ns(v) for k, v in pids.items()}
prev_t = time.monotonic()
next_report = prev_t + INTERVAL
slice_waits = {k: [] for k in pids}

# Track which pid each key currently maps to, so a restart resets the baseline
# (new pid's cumulative run_delay must not be diffed against the old pid's).
cur_pid = dict(pids)


def write_json(path, ts, stats, base_hz, base_max_gap, rsl_hz, rsl_max_gap, flags):
    """Overwrite path with one JSON line describing the latest slice. Atomic via
    temp+rename so a concurrent reader never sees a half-written file."""
    if not path:
        return
    rec = {
        "t": time.time(),  # wall clock for staleness check by consumer
        "ts": ts,
        # None => node not running (consumer shows blank)
        "sm":   None if cur_pid["sm"]   is None else round(stats["sm"][0], 1),
        "cm":   None if cur_pid["cm"]   is None else round(stats["cm"][0], 1),
        "glim": None if cur_pid["glim"] is None else round(stats["glim"][0], 1),
        "base_hz": round(base_hz, 1), "base_gap": round(base_max_gap, 1),
        "rsl_hz": round(rsl_hz, 1), "rsl_gap": round(rsl_max_gap, 1),
        "flags": flags,
    }
    try:
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            f.write(json.dumps(rec))
        os.replace(tmp, path)
    except OSError:
        pass


try:
    while True:
        time.sleep(SAMPLE_DT)
        now = time.monotonic()
        for k, pid in pids.items():
            w = read_wait_ns(pid)
            dw = w - prev_wait[k]
            # negative dw (pid wrapped/restarted) -> treat as 0 this slice
            prev_wait[k] = w
            slice_waits[k].append(max(0.0, dw) / 1e6)  # ms in this ~100ms slice
        prev_t = now

        if now >= next_report:
            cutoff = now - INTERVAL
            rb = [t for t in arr_base if t >= cutoff]
            base_hz = (len(rb)-1)/INTERVAL if len(rb)>=2 else 0.0
            base_max_gap = max(((rb[i]-rb[i-1])*1000 for i in range(1,len(rb))), default=0.0)
            rr = [t for t in arr_rsl if t >= cutoff]
            rsl_hz = (len(rr)-1)/INTERVAL if len(rr)>=2 else 0.0
            rsl_max_gap = max(((rr[i]-rr[i-1])*1000 for i in range(1,len(rr))), default=0.0)

            stats = {}
            for k in pids:
                arr = slice_waits[k]
                if not arr:
                    stats[k] = (0.0, 0.0, 0)
                    continue
                mx = max(arr)
                p95 = percentile(arr, 0.95)
                gt5 = sum(1 for x in arr if x > 5)
                stats[k] = (mx, p95, gt5)
                slice_waits[k] = []

            flags = []
            if stats['sm'][0] > 50: flags.append("SM_MAX>50")
            if stats['cm'][0] > 50: flags.append("CM_MAX>50")
            if stats['sm'][2] >= 5: flags.append("SM_FREQ_SPIKE")
            if stats['cm'][2] >= 5: flags.append("CM_FREQ_SPIKE")
            if base_max_gap > 150: flags.append("BASE_GAP")
            if base_hz < 70: flags.append("BASE_LOW_HZ")

            ts = time.strftime("%H:%M:%S")
            flag_str = " ".join(flags) if flags else "OK"
            print(f"{ts}  "
                  f"{stats['sm'][0]:>6.1f} {stats['sm'][1]:>6.1f} {stats['sm'][2]:>4d}  "
                  f"{stats['cm'][0]:>6.1f} {stats['cm'][1]:>6.1f} {stats['cm'][2]:>4d}  "
                  f"{stats['glim'][0]:>6.1f} {stats['glim'][1]:>6.1f}  "
                  f"{base_hz:>5.1f} {base_max_gap:>6.1f}  "
                  f"{rsl_hz:>5.1f} {rsl_max_gap:>6.1f}  {flag_str}")
            sys.stdout.flush()

            # Write latest slice for external consumers (set_pl.sh monitor).
            write_json(args.json_out, ts, stats, base_hz, base_max_gap,
                       rsl_hz, rsl_max_gap, flags)

            # Re-discover PIDs so node restarts are tracked. If a pid changed,
            # reset that key's baseline to the new pid's current run_delay so the
            # next slice isn't a bogus huge delta (old cumulative vs new).
            newpids = discover_pids()
            for k, np_ in newpids.items():
                if np_ != cur_pid[k]:
                    cur_pid[k] = np_
                    pids[k] = np_
                    prev_wait[k] = read_wait_ns(np_)
            next_report = now + INTERVAL
except KeyboardInterrupt:
    pass
