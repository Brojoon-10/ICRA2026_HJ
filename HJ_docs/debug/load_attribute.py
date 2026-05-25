#!/usr/bin/env python3
"""
load_attribute.py
Three-axis per-pid load attribution to pair with set_pl.sh monitor.

Each tick prints:
  - System PSI (cpu.some / io.full / mem.some, 10s window) + procs R/D
  - TOP-N by absolute CPU%       (steady heavy consumers)
  - TOP-N by |Δ CPU%|            (jumped / crashed vs rolling window)
  - TOP-N by runqueue wait ms/s  (CPU starvation victims — points at
                                  whoever else is hogging the core)
  - TOP-N by disk write KB/s

When PSI crosses threshold:
  - line is marked '*'
  - full top-15 along each axis is appended to a spike log file

Usage:
  ./load_attribute.py                     # 1s interval, 10s window
  ./load_attribute.py --interval 0.5 --window 20 --top 5
  ./load_attribute.py --csv               # also dump per-tick CSV
  ./load_attribute.py --quiet --csv       # log only, no stdout

Tail live spike log in another pane:
  tail -F ~/.set_pl_monitor/load_attr_*_spikes.log

No deps beyond stdlib. Reads /proc directly. Safe to run as user
(some /proc/<pid>/io / schedstat may be unreadable; those pids skipped).
"""
import argparse
import collections
import os
import re
import sys
import time
from datetime import datetime

PROC = '/proc'
HZ = os.sysconf(os.sysconf_names['SC_CLK_TCK'])

TH_CPU_SOME = 20.0   # cpu.some avg10 (%)
TH_IO_FULL  = 5.0    # io.full  avg10 (%)
TH_MEM_SOME = 5.0    # memory.some avg10 (%)
NOISE_CPU   = 0.05   # below this, skip pid entirely
NOISE_WAIT  = 0.10   # ms/s
NOISE_WR    = 1.0    # KB/s

NAME_RE = re.compile(r'__name:=(\S+)')


def read_text(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def read_first_line(path):
    txt = read_text(path)
    return txt.split('\n', 1)[0] if txt else None


def list_pids():
    return [int(n) for n in os.listdir(PROC) if n.isdigit()]


def proc_name(pid, cache):
    if pid in cache:
        return cache[pid]
    cmdline = read_text(f'{PROC}/{pid}/cmdline')
    name = None
    if cmdline:
        flat = cmdline.replace('\x00', ' ').strip()
        m = NAME_RE.search(flat)
        if m:
            name = m.group(1)
    if not name:
        name = read_first_line(f'{PROC}/{pid}/comm') or f'pid{pid}'
    cache[pid] = name
    return name


def read_pid_sample(pid):
    """Return cumulative counters dict, or None if pid vanished / unreadable."""
    stat = read_first_line(f'{PROC}/{pid}/stat')
    sched = read_first_line(f'{PROC}/{pid}/schedstat')
    if not stat or not sched:
        return None
    rp = stat.rfind(')')
    if rp < 0:
        return None
    rest = stat[rp + 2:].split()
    if len(rest) < 13:
        return None
    try:
        utime = int(rest[11])
        stime = int(rest[12])
    except ValueError:
        return None
    sf = sched.split()
    try:
        run_ns = int(sf[0])
        wait_ns = int(sf[1])
    except (ValueError, IndexError):
        return None
    write_b = 0
    io_txt = read_text(f'{PROC}/{pid}/io')
    if io_txt:
        for line in io_txt.splitlines():
            if line.startswith('write_bytes:'):
                try:
                    write_b = int(line.split()[1])
                except (ValueError, IndexError):
                    pass
                break
    return {
        'cpu_ticks': utime + stime,
        'wait_ns':   wait_ns,
        'run_ns':    run_ns,
        'write_b':   write_b,
    }


def parse_psi(path):
    out = {}
    txt = read_text(path)
    if not txt:
        return out
    for line in txt.splitlines():
        parts = line.split()
        if not parts:
            continue
        kind = parts[0]
        for tok in parts[1:]:
            if '=' in tok:
                k, v = tok.split('=', 1)
                try:
                    out[f'{kind}_{k}'] = float(v)
                except ValueError:
                    pass
    return out


def read_procs_rd():
    txt = read_text(f'{PROC}/stat') or ''
    r = b = 0
    for line in txt.splitlines():
        if line.startswith('procs_running'):
            try: r = int(line.split()[1])
            except: pass
        elif line.startswith('procs_blocked'):
            try: b = int(line.split()[1])
            except: pass
    return r, b


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                 description=__doc__)
    ap.add_argument('--interval', type=float, default=1.0)
    ap.add_argument('--window', type=int, default=10,
                    help='rolling samples used as Δ baseline (default 10)')
    ap.add_argument('--top', type=int, default=3)
    ap.add_argument('--logdir', default=os.path.expanduser('~/.set_pl_monitor'))
    ap.add_argument('--csv', action='store_true')
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--oneline', action='store_true',
                    help='print compact single-line per tick (good for inline / tee)')
    ap.add_argument('--name-width', type=int, default=14,
                    help='truncate process names to this width in --oneline mode')
    args = ap.parse_args()

    os.makedirs(args.logdir, exist_ok=True)
    stamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    spike_path = os.path.join(args.logdir, f'load_attr_{stamp}_spikes.log')
    csv_path   = os.path.join(args.logdir, f'load_attr_{stamp}.csv')

    print(f'[INFO] interval={args.interval}s window={args.window} top={args.top}')
    print(f'[INFO] PSI thresholds: cpu.some>{TH_CPU_SOME} io.full>{TH_IO_FULL} mem.some>{TH_MEM_SOME}')
    print(f'[INFO] spikes  -> {spike_path}')
    if args.csv:
        print(f'[INFO] csv     -> {csv_path}')

    csv_f = open(csv_path, 'w') if args.csv else None
    if csv_f:
        csv_f.write('ts,psi_cpu_some,psi_io_full,psi_mem_some,procR,procD,'
                    'axis,rank,name,pid,value\n')

    prev = {}                  # pid -> last sample
    history = {}               # pid -> deque of cpu% for Δ baseline
    name_cache = {}            # pid -> resolved name
    last_t = time.monotonic()

    try:
        while True:
            time.sleep(args.interval)
            now_t = time.monotonic()
            dt = max(1e-3, now_t - last_t)
            last_t = now_t
            ts = datetime.now().strftime('%H:%M:%S')

            psi_cpu = parse_psi(f'{PROC}/pressure/cpu')
            psi_io  = parse_psi(f'{PROC}/pressure/io')
            psi_mem = parse_psi(f'{PROC}/pressure/memory')
            procR, procD = read_procs_rd()
            pcpu = psi_cpu.get('some_avg10', 0.0)
            piof = psi_io.get('full_avg10', 0.0)
            pmem = psi_mem.get('some_avg10', 0.0)
            spike = pcpu > TH_CPU_SOME or piof > TH_IO_FULL or pmem > TH_MEM_SOME
            mark = '*' if spike else ' '

            rows = []
            cur = {}
            for pid in list_pids():
                s = read_pid_sample(pid)
                if s is None:
                    continue
                cur[pid] = s
                p = prev.get(pid)
                if p is None:
                    continue
                d_ticks = max(0, s['cpu_ticks'] - p['cpu_ticks'])
                d_wait  = max(0, s['wait_ns']   - p['wait_ns'])
                d_write = max(0, s['write_b']   - p['write_b'])
                cpu_pct    = (d_ticks / HZ) / dt * 100.0     # % of one core
                wait_ms_s  = d_wait / 1e6 / dt
                write_KB_s = d_write / 1024.0 / dt

                hist = history.setdefault(pid, collections.deque(maxlen=args.window))
                avg = sum(hist) / len(hist) if hist else cpu_pct
                hist.append(cpu_pct)
                d_cpu = cpu_pct - avg

                if (cpu_pct < NOISE_CPU and abs(d_cpu) < NOISE_CPU
                        and wait_ms_s < NOISE_WAIT and write_KB_s < NOISE_WR):
                    continue

                rows.append({
                    'name':  proc_name(pid, name_cache),
                    'pid':   pid,
                    'cpu':   cpu_pct,
                    'd_cpu': d_cpu,
                    'wait':  wait_ms_s,
                    'wr':    write_KB_s,
                })

            prev = cur
            dead = set(history) - set(cur)
            for pid in dead:
                history.pop(pid, None)
                name_cache.pop(pid, None)

            top_cpu  = sorted(rows, key=lambda r: -r['cpu'])
            top_dcpu = sorted(rows, key=lambda r: -abs(r['d_cpu']))
            top_wait = sorted(rows, key=lambda r: -r['wait'])
            top_wr   = sorted(rows, key=lambda r: -r['wr'])

            if not args.quiet:
                nw = max(4, args.name_width)
                def trunc(s):
                    return s if len(s) <= nw else s[:nw-1] + '…'
                if args.oneline:
                    # Filter zero-ish values per axis (cuts noise like "name(0KB)")
                    def topn_filt(items, key, fmt, vmin):
                        out = [fmt(trunc(it['name']), it[key])
                               for it in items[:args.top]
                               if abs(it[key]) >= vmin]
                        return ' '.join(out) if out else '-'
                    cpu_s  = topn_filt(top_cpu,  'cpu',   lambda n,v: f'{n}({v:.0f}%)',   1)
                    dcpu_s = topn_filt(top_dcpu, 'd_cpu', lambda n,v: f'{n}({v:+.0f}%)',  1)
                    wait_s = topn_filt(top_wait, 'wait',  lambda n,v: f'{n}({v:.0f}ms)',  1)
                    wr_s   = topn_filt(top_wr,   'wr',    lambda n,v: f'{n}({v:.0f}KB)',  4)
                    # Sub-row style. psi/R/D omitted (already in set_pl line)
                    print(f'{ts}{mark} attr   TOP {cpu_s} | Δ {dcpu_s} | WAIT {wait_s} | WR {wr_s}')
                else:
                    print(f'{ts}{mark} PSI cpu.some={pcpu:5.1f} io.full={piof:5.1f} '
                          f'mem.some={pmem:4.1f}  R={procR} D={procD}')
                    def show(label, items, key, fmt):
                        items = items[:args.top]
                        if not items:
                            print(f'  {label:<10}: -'); return
                        s = '  '.join(fmt(it['name'], it[key]) for it in items)
                        print(f'  {label:<10}: {s}')
                    show('TOP CPU%',  top_cpu,  'cpu',   lambda n,v: f'{n}({v:.1f}%)')
                    show('Δ CPU%',    top_dcpu, 'd_cpu', lambda n,v: f'{n}({v:+.1f}%)')
                    show('TOP wait',  top_wait, 'wait',  lambda n,v: f'{n}({v:.1f}ms/s)')
                    show('TOP write', top_wr,   'wr',    lambda n,v: f'{n}({v:.0f}KB/s)')

            if csv_f:
                base = f'{ts},{pcpu},{piof},{pmem},{procR},{procD}'
                for axis, items, key in (('cpu',   top_cpu,  'cpu'),
                                         ('dcpu',  top_dcpu, 'd_cpu'),
                                         ('wait',  top_wait, 'wait'),
                                         ('write', top_wr,   'wr')):
                    for rank, it in enumerate(items[:args.top], 1):
                        csv_f.write(f'{base},{axis},{rank},{it["name"]},'
                                    f'{it["pid"]},{it[key]:.3f}\n')
                csv_f.flush()

            if spike:
                with open(spike_path, 'a') as f:
                    f.write(f'\n=== {datetime.now().isoformat(timespec="seconds")} '
                            f'cpu.some={pcpu:.1f} io.full={piof:.1f} '
                            f'mem.some={pmem:.1f} R={procR} D={procD} ===\n')
                    def dump(label, items, key, fmt):
                        f.write(f'-- top 15 by {label} --\n')
                        for it in items[:15]:
                            f.write(f'  {fmt(it[key])}  pid={it["pid"]:>7}  {it["name"]}\n')
                    dump('CPU%',     top_cpu,  'cpu',   lambda v: f'{v:6.1f}%')
                    dump('|Δ CPU%|', top_dcpu, 'd_cpu', lambda v: f'{v:+6.1f}%')
                    dump('wait',     top_wait, 'wait',  lambda v: f'{v:7.1f}ms/s')
                    dump('write',    top_wr,   'wr',    lambda v: f'{v:9.0f}KB/s')

    except KeyboardInterrupt:
        print('\n[INFO] stopped.')
    finally:
        if csv_f:
            csv_f.close()


if __name__ == '__main__':
    main()
