#!/bin/bash
# set_pl.sh
# Configure Intel RAPL power limits + boost ratio cap + live monitor.
# Tuned for ICRA2026 NUC14RVB (Core Ultra 7 155H) running glim/ROS stack.
# See HJ_docs/cpu_throttle_debug.md for the design rationale.
#
# Subcommands:
#   show                            # current PL1/PL2/max_perf_pct/throttle counters
#   set [PL1] [PL2] [PCT]           # set values (W, W, %)
#                                   #   no args   -> PL1=64 PL2=64 PCT=70  (HJ default)
#                                   #   set 40    -> PL1=40 PL2=40 (PCT unchanged)
#                                   #   set 40 80 -> PL1=40 PL2=80 (PCT unchanged)
#                                   #   set 64 64 70 -> all three
#   pct N                           # only set max_perf_pct (boost ratio %, 1-100)
#   reset                           # back to PPD-default-ish (PL1=200, PL2=64, pct=100)
#   monitor [interval_sec]          # live throttle/temp/power until Ctrl-C; logs to file
#   install                         # install required tools (msr-tools, sysstat)
#
# Works on host or inside docker (kernel sysfs is shared); needs root for set/reset.
# Auto re-runs itself with sudo if not root and a write is needed.
# Auto-installs missing tools the first time (apt-get).

set -uo pipefail

PL_DIR="/sys/class/powercap/intel-rapl:0"
PL1_FILE="$PL_DIR/constraint_0_power_limit_uw"
PL2_FILE="$PL_DIR/constraint_1_power_limit_uw"
PSTATE_DIR="/sys/devices/system/cpu/intel_pstate"
PCT_FILE="$PSTATE_DIR/max_perf_pct"
NOTURBO_FILE="$PSTATE_DIR/no_turbo"

# HJ-tuned defaults (see HJ_docs/cpu_throttle_debug.md)
# PL2=PL1=64W: NUC14RVB cooling sustained limit. PL2 used to be 100W as
# burst headroom but pct cap (≤80) keeps package well under 64W in practice
# so the higher PL2 was decorative.
DEFAULT_PL1=64
DEFAULT_PL2=64
DEFAULT_PCT=70

# ---------- helpers ----------

need_root() {
  if [ "$(id -u)" -ne 0 ]; then
    exec sudo -E "$0" "$@"
  fi
}

ensure_tools() {
  # Auto-install tools if any required one is missing. Skips if already installed.
  local missing=()
  command -v rdmsr   >/dev/null 2>&1 || missing+=(msr-tools)
  command -v mpstat  >/dev/null 2>&1 || missing+=(sysstat)
  command -v sensors >/dev/null 2>&1 || missing+=(lm-sensors)
  if [ ${#missing[@]} -gt 0 ]; then
    echo "[INFO] missing tools: ${missing[*]} — installing via apt-get..."
    if [ "$(id -u)" -ne 0 ]; then
      sudo apt-get update -qq && sudo apt-get install -y "${missing[@]}"
    else
      apt-get update -qq && apt-get install -y "${missing[@]}"
    fi
  fi
  modprobe msr 2>/dev/null || true
}

show() {
  if [ ! -e "$PL1_FILE" ]; then
    echo "[ERR] $PL1_FILE not found. Intel RAPL not exposed?" >&2
    return 1
  fi
  pl1=$(cat "$PL1_FILE" 2>/dev/null || echo 0)
  pl2=$(cat "$PL2_FILE" 2>/dev/null || echo 0)
  tw1=$(cat "$PL_DIR/constraint_0_time_window_us" 2>/dev/null || echo "?")
  tw2=$(cat "$PL_DIR/constraint_1_time_window_us" 2>/dev/null || echo "?")
  pct=$(cat "$PCT_FILE" 2>/dev/null || echo "?")
  noturbo=$(cat "$NOTURBO_FILE" 2>/dev/null || echo "?")
  thr=$(cat /sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_count 2>/dev/null || echo n/a)
  thr_t=$(cat /sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_total_time_ms 2>/dev/null || echo n/a)
  # current package power: sample RAPL energy counter over 1s.
  # energy_uj is 0400-root on modern kernels (CVE-2020-8694), so we
  # try sudo -n (NOPASSWD) when not already root. Falls back to a
  # hint message if no sudo cached.
  pkg_c=$(($(cat /sys/class/thermal/thermal_zone6/temp 2>/dev/null || echo 0)/1000))
  read_e() {
    if [ -r "$PL_DIR/energy_uj" ]; then
      cat "$PL_DIR/energy_uj" 2>/dev/null
    else
      sudo -n cat "$PL_DIR/energy_uj" 2>/dev/null
    fi
  }
  e1=$(read_e); sleep 1; e2=$(read_e)
  if [ -n "${e1:-}" ] && [ -n "${e2:-}" ]; then
    pkg_w=$(awk -v a="$e1" -v b="$e2" 'BEGIN{d=b-a; if(d<0)d+=2^32; printf "%.1f W", d/1e6}')
  else
    pkg_w="N/A (rerun via sudo — energy_uj is 0400 root)"
  fi
  printf "PL1 (long_term):    %3d W   window=%s us\n" "$((pl1/1000000))" "$tw1"
  printf "PL2 (short_term):   %3d W   window=%s us\n" "$((pl2/1000000))" "$tw2"
  printf "max_perf_pct:       %s%%   no_turbo=%s\n" "$pct" "$noturbo"
  printf "package_power:      %s W (1s avg)   pkg_temp=%s°C\n" "$pkg_w" "$pkg_c"
  printf "package_throttle:   count=%s  total_ms=%s\n" "$thr" "$thr_t"
  printf "uptime: %s\n" "$(uptime -p 2>/dev/null || echo n/a)"
}

apply_pl() {
  local pl1_w="$1" pl2_w="$2"
  for v in "$pl1_w" "$pl2_w"; do
    case "$v" in *[!0-9]*) echo "[ERR] PL must be integer (W). got: $v" >&2; exit 2 ;; esac
    if [ "$v" -lt 5 ] || [ "$v" -gt 250 ]; then
      echo "[ERR] PL=${v}W out of sane range [5,250]" >&2; exit 2
    fi
  done
  if ! echo $((pl1_w*1000000)) > "$PL1_FILE" 2>/dev/null; then
    echo "[ERR] failed to write PL1. /sys ro-mounted (container) or no privilege?" >&2
    [ -f /.dockerenv ] && echo "      Hint: run on host: $(realpath "$0") $*" >&2
    exit 3
  fi
  echo $((pl2_w*1000000)) > "$PL2_FILE"
}

apply_pct() {
  local pct="$1"
  case "$pct" in *[!0-9]*) echo "[ERR] pct must be integer 1-100. got: $pct" >&2; exit 2 ;; esac
  if [ "$pct" -lt 1 ] || [ "$pct" -gt 100 ]; then
    echo "[ERR] pct=$pct out of range [1,100]" >&2; exit 2
  fi
  if [ ! -e "$PCT_FILE" ]; then
    echo "[WARN] $PCT_FILE not present (intel_pstate not active?). Skipping pct." >&2
    return
  fi
  # Re-enable turbo so max_perf_pct has effect
  echo 0 > "$NOTURBO_FILE" 2>/dev/null || true
  echo "$pct" > "$PCT_FILE"
}

# ---------- monitor mode ----------

monitor_loop() {
  local interval="${1:-1}"
  # Override location with SETPL_LOGDIR=<path>. Useful when monitor runs inside
  # a docker container but logs must land on a bind-mounted host path so an
  # external analyzer (or Claude on the host) can read them.
  local logdir="${SETPL_LOGDIR:-${HOME:-/tmp}/.set_pl_monitor}"
  mkdir -p "$logdir"
  local logfile="$logdir/$(date +%Y-%m-%d-%H-%M-%S).log"
  local nproc; nproc=$(nproc 2>/dev/null || echo 22)

  # live_monitor.py latest-slice JSON (loose coupling). Path is derived from this
  # script's own location so it works regardless of host vs container cwd. Both
  # set_pl.sh (here) and the auto-spawned live_monitor write/read the same path.
  # Override with LIVE_MONITOR_JSON env. Missing/stale => the 5 ROS axes show "-".
  local lm_json="${LIVE_MONITOR_JSON:-$(cd "$(dirname "$0")/../../HJ_docs/debug" 2>/dev/null && pwd)/.live_monitor_latest.json}"
  local lm_stale_s=8   # JSON older than this => treat as no data (live_monitor reports every 5s)

  echo "[INFO] monitor interval=${interval}s  Ctrl-C to stop"
  echo "[INFO] log: $logfile"
  echo "[INFO] flags: THROTTLE=clock cap | HOT=pkg>95C | IO=iowait>5% | IRQ=softirq>10% | LOAD=load1>nproc | MEM=used>90%"
  echo "[INFO] PSI:   CPU_PRESS=cpu.some>20% (10s) | IO_STALL=io.full>5% | MEM_PRESS=mem.some>5%"

  # PSI helper: $1=cpu|io|memory  $2=some|full  $3=avg10|avg60|avg300
  # Returns the value as a float string, or empty if PSI not available (kernel <4.20).
  psi() {
    awk -v k="$2" -v w="$3" '$1==k {
      for(i=2;i<=NF;i++) if(index($i,w"=")==1){
        sub(w"=","",$i); printf "%.2f", $i+0; exit
      }
    }' "/proc/pressure/$1" 2>/dev/null
  }

  # energy_uj is 0400-root on modern kernels. Use sudo -n when available.
  read_energy() {
    if [ -r "$PL_DIR/energy_uj" ]; then
      cat "$PL_DIR/energy_uj" 2>/dev/null || echo 0
    else
      sudo -n cat "$PL_DIR/energy_uj" 2>/dev/null || echo 0
    fi
  }

  # Decode MSR 0x64F (MSR_CORE_PERF_LIMIT_REASONS) on cpu0 into a short cause
  # string. Lower 16 bits = currently-active reasons; upper 16 (log/sticky) read
  # then cleared so each tick reports reasons since the previous tick.
  # Bits (Intel SDM): 0 PROCHOT, 1 Thermal, 4 ResidencyState, 5 RATL,
  # 6 VR-Therm, 7 VR-IccMax(current), 10 PL1, 11 PL2, 12 MaxTurbo, 13 TurboAtten.
  # Needs rdmsr/wrmsr (msr-tools) + root. Returns "-" if unavailable.
  read_limit_reasons() {
    command -v rdmsr >/dev/null 2>&1 || { echo "-"; return; }
    local raw
    raw=$(rdmsr -p0 0x64F 2>/dev/null) || { echo "-"; return; }
    [ -z "$raw" ] && { echo "-"; return; }
    local v=$((16#$raw))
    # Use log bits (16..29) so we catch transient throttles between samples,
    # OR active bits (0..13). Combine both into one 14-bit view.
    local act=$(( v & 0x3FFF ))
    local log=$(( (v >> 16) & 0x3FFF ))
    local r=$(( act | log ))
    # Clear sticky log bits for next tick (write back with log bits zeroed).
    wrmsr -p0 0x64F $(( v & 0xFFFF )) 2>/dev/null || true
    # MAXTURBO(12)/TURBOATTN(13)/RESIDENCY(4)/bit8 are NORMAL operating states
    # (turbo hit its allowed ceiling / EDP companion flag, not a forced clock
    # cut). On this 155H, bit8 fires together with bit12 constantly under load.
    # Exclude them so only real forced-throttle reasons (current/power/thermal)
    # surface in the alert tag.
    local r=$(( r & ~((1<<12)|(1<<13)|(1<<8)|(1<<4)) ))
    [ "$r" -eq 0 ] && { echo "-"; return; }
    local s=""
    (( r & (1<<7) ))  && s+="ICCMAX "    # VR Therm Design Current = current limit
    (( r & (1<<11) )) && s+="PL2 "
    (( r & (1<<10) )) && s+="PL1 "
    (( r & (1<<1) ))  && s+="THERMAL "
    (( r & (1<<6) ))  && s+="VRTHERM "
    (( r & (1<<5) ))  && s+="RATL "
    (( r & (1<<0) ))  && s+="PROCHOT "
    echo "${s% }"
  }

  # Read live_monitor.py's latest-slice JSON and emit a compact 5-axis string:
  #   "rsl=9.8/103  glim=4ms  base=99/21  sm=5ms  cm=2ms"
  # rsl/base = hz/max_gap_ms ; glim/sm/cm = max wait_ms in slice.
  # A node that is down shows "-" for its wait. Missing/stale file => "" (caller
  # prints nothing). Uses python3 to parse (no jq dependency).
  read_live_axes() {
    [ -f "$lm_json" ] || { echo ""; return; }
    python3 - "$lm_json" "$lm_stale_s" <<'PY' 2>/dev/null || echo ""
import json, sys, time
try:
    p, stale = sys.argv[1], float(sys.argv[2])
    d = json.load(open(p))
    if time.time() - d.get("t", 0) > stale:
        print(""); sys.exit(0)
    def w(v): return "-" if v is None else f"{v:g}ms"
    print(f"rsl={d['rsl_hz']:g}/{d['rsl_gap']:g}  glim={w(d['glim'])}  "
          f"base={d['base_hz']:g}/{d['base_gap']:g}  sm={w(d['sm'])}  cm={w(d['cm'])}")
except Exception:
    print("")
PY
  }

  # baseline counters
  local prev_thr_pkg prev_thr_core prev_e prev_t
  prev_thr_pkg=$(cat /sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_count 2>/dev/null || echo 0)
  prev_thr_core=$(awk '{s+=$1} END{print s+0}' /sys/devices/system/cpu/cpu*/thermal_throttle/core_throttle_count 2>/dev/null)
  prev_e=$(read_energy)
  prev_t=$(date +%s.%N)
  if [ "${prev_e:-0}" = "0" ]; then
    echo "[WARN] energy_uj unreadable (need sudo for pkg_W column). Run with: sudo -E $(basename "$0") monitor"
  fi

  # baseline /proc/stat (for CPU breakdown deltas)
  # cpu  user nice system idle iowait irq softirq steal guest guest_nice
  local prev_user prev_nice prev_sys prev_idle prev_iow prev_irq prev_sirq prev_steal
  read _ prev_user prev_nice prev_sys prev_idle prev_iow prev_irq prev_sirq prev_steal _ < /proc/stat

  # CSV header (PSI + loadN appended at end so old log parsers still work)
  local header="ts,PL1_W,PL2_W,pct,pkg_C,core_max_C,nvme_C,freq_avg_MHz,freq_max_MHz,thr_pkg_dt,thr_core_dt,pkg_W,usr%,sys%,iowait%,soft%,load1,mem_used%,top1,top1_pct,psi_cpu_some,psi_io_full,psi_mem_some,loadN,limit_reason"
  echo "$header" | tee "$logfile"

  # Auto-spawn per-pid attribution tool (3-axis + spike dump) in background.
  # stdout is NOT redirected → ATTR lines print inline with monitor lines.
  # CSV + spike log still go to $logdir for postmortem.
  # Disable with NO_ATTR=1.
  local attr_pid="" attr_script attr_top="${ATTR_TOP:-3}"
  attr_script="$(dirname "$0")/../../HJ_docs/debug/load_attribute.py"
  if [ "${NO_ATTR:-0}" != "1" ]; then
    if [ -x "$attr_script" ] && command -v python3 >/dev/null 2>&1; then
      python3 -u "$attr_script" --interval "$interval" --top "$attr_top" \
        --oneline --csv --logdir "$logdir" &
      attr_pid=$!
      echo "[INFO] load_attribute auto-spawned pid=$attr_pid (oneline, top=$attr_top — prints inline below)"
      echo "[INFO]   csv:     ls -t $logdir/load_attr_*.csv | head -1"
      echo "[INFO]   spikes:  ls -t $logdir/load_attr_*_spikes.log | head -1"
      echo "[INFO]   tweak:   ATTR_TOP=5 $(basename "$0") monitor   |   disable: NO_ATTR=1 ..."
    else
      echo "[WARN] load_attribute.py not found/executable at $attr_script — skipping auto-spawn"
    fi
  else
    echo "[INFO] load_attribute auto-spawn disabled (NO_ATTR=1)"
  fi

  # Auto-spawn live_monitor.py (5 ROS axes: rslidar/glim/base_odom/sm/cm).
  # live_monitor needs rospy, which needs the ROS env. But set_pl is usually run
  # via sudo (energy/MSR need root), and sudo strips ROS_PACKAGE_PATH etc, so a
  # naive spawn dies on `import rospy`. Fix: source ROS and, when running under
  # sudo, drop back to the invoking user ($SUDO_USER) whose shell has ROS set up.
  # Errors go to a log (not /dev/null) so failures are diagnosable.
  local live_pid="" live_script live_log
  live_script="$(cd "$(dirname "$0")/../../HJ_docs/debug" 2>/dev/null && pwd)/live_monitor.py"
  live_log="$logdir/live_monitor.out"
  if [ "${NO_LIVE:-0}" != "1" ]; then
    if [ -f "$live_script" ] && command -v python3 >/dev/null 2>&1; then
      rm -f "$lm_json" 2>/dev/null
      # Build a command that re-sources ROS then runs live_monitor.
      local ros_setup="${ROS_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
      [ -f "$ros_setup" ] || ros_setup="/opt/ros/noetic/setup.bash"
      # ROS_HOSTNAME ends up empty under some shells, which breaks rospy
      # networking ("invalid ROS_HOSTNAME"); pin it so topic subscriptions work.
      local inner="source /opt/ros/noetic/setup.bash; [ -f '$ros_setup' ] && source '$ros_setup'; export ROS_HOSTNAME=\${ROS_HOSTNAME:-localhost}; [ -n \"\$ROS_HOSTNAME\" ] || export ROS_HOSTNAME=localhost; exec python3 -u '$live_script' --json-out '$lm_json'"
      if [ -n "${SUDO_USER:-}" ] && [ "$SUDO_USER" != "root" ]; then
        # Re-source as the original user (their env has ROS); HOME must be theirs.
        sudo -u "$SUDO_USER" -H bash -lc "$inner" >"$live_log" 2>&1 &
      else
        bash -lc "$inner" >"$live_log" 2>&1 &
      fi
      live_pid=$!
      echo "[INFO] live_monitor auto-spawned pid=$live_pid (5 ROS axes -> cyan tail; json=$lm_json)"
      echo "[INFO]   nodes down => '-'; restarts auto-tracked; errors -> $live_log"
    else
      echo "[WARN] live_monitor.py not found at $live_script — 5-axis columns disabled"
    fi
  else
    echo "[INFO] live_monitor auto-spawn disabled (NO_LIVE=1)"
  fi
  echo

  # Single-quoted action: $attr_pid expands at trap-fire time (it is a local
  # of monitor_loop and exists when the trap actually runs). $logfile is
  # briefly unquoted so it expands NOW (at trap-set time).
  trap 'echo
if [ -n "${attr_pid:-}" ] && kill -0 "$attr_pid" 2>/dev/null; then
  kill "$attr_pid" 2>/dev/null
  wait "$attr_pid" 2>/dev/null
  echo "[INFO] load_attribute stopped (pid=$attr_pid)"
fi
if [ -n "${live_pid:-}" ] && kill -0 "$live_pid" 2>/dev/null; then
  kill "$live_pid" 2>/dev/null
  wait "$live_pid" 2>/dev/null
  echo "[INFO] live_monitor stopped (pid=$live_pid)"
fi
echo "[INFO] monitor stopped. log: '"$logfile"'"
exit 0' INT TERM

  local tick_n=0
  local -a procR_hist=()
  while true; do
    sleep "$interval"
    now_t=$(date +%s.%N)
    dt=$(awk -v a="$prev_t" -v b="$now_t" 'BEGIN{printf "%.3f", b-a}')
    prev_t=$now_t
    ts=$(date +%H:%M:%S)

    pl1=$(awk '{printf "%d", $1/1e6}' "$PL1_FILE" 2>/dev/null || echo 0)
    pl2=$(awk '{printf "%d", $1/1e6}' "$PL2_FILE" 2>/dev/null || echo 0)
    pct=$(cat "$PCT_FILE" 2>/dev/null || echo "?")

    pkg_c=$(($(cat /sys/class/thermal/thermal_zone6/temp 2>/dev/null || echo 0)/1000))
    core_max=$(sensors 2>/dev/null | awk '/^Core /{gsub(/[+°C]/,"",$3);v=$3+0;if(v>m)m=v} END{print m+0}')
    nvme=$(sensors 2>/dev/null | awk '/^Composite:/{gsub(/[+°C]/,"",$2); print $2+0; exit}')

    freqs=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq 2>/dev/null)
    favg=$(echo "$freqs" | awk 'NF{s+=$1;n++} END{if(n)printf "%d", s/n/1000; else print 0}')
    fmx=$(echo "$freqs" | awk 'NF{if($1>m)m=$1} END{printf "%d", m/1000}')

    thr_pkg=$(cat /sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_count 2>/dev/null || echo 0)
    thr_pkg_dt=$((thr_pkg - prev_thr_pkg)); prev_thr_pkg=$thr_pkg
    thr_core=$(awk '{s+=$1} END{print s+0}' /sys/devices/system/cpu/cpu*/thermal_throttle/core_throttle_count 2>/dev/null)
    thr_core_dt=$((thr_core - prev_thr_core)); prev_thr_core=$thr_core

    # MSR 0x64F decoded throttle reason (ICCMAX / PL1 / PL2 / THERMAL / ...)
    limit_reason=$(read_limit_reasons)

    # 5 ROS axes from live_monitor.py (rslidar/glim/base_odom/sm/cm). "" if down.
    live_axes=$(read_live_axes)

    # Package power from RAPL energy counter (uJ -> J -> avg W over interval)
    e=$(read_energy)
    if [ "$e" != "0" ] && [ "$prev_e" != "0" ]; then
      pkg_w=$(awk -v a=$prev_e -v b=$e -v t=$dt 'BEGIN{d=b-a; if(d<0)d+=2^32; if(t>0)printf "%.1f", d/1e6/t; else print "0.0"}')
    else
      pkg_w="?"
    fi
    prev_e=$e

    # CPU time breakdown from /proc/stat delta (system-wide, all cores aggregated)
    local cur_user cur_nice cur_sys cur_idle cur_iow cur_irq cur_sirq cur_steal
    read _ cur_user cur_nice cur_sys cur_idle cur_iow cur_irq cur_sirq cur_steal _ < /proc/stat
    local d_user=$((cur_user-prev_user)) d_nice=$((cur_nice-prev_nice)) d_sys=$((cur_sys-prev_sys))
    local d_idle=$((cur_idle-prev_idle)) d_iow=$((cur_iow-prev_iow)) d_irq=$((cur_irq-prev_irq))
    local d_sirq=$((cur_sirq-prev_sirq)) d_steal=$((cur_steal-prev_steal))
    local d_total=$((d_user+d_nice+d_sys+d_idle+d_iow+d_irq+d_sirq+d_steal))
    local p_usr p_sys p_iow p_soft
    if [ "$d_total" -gt 0 ]; then
      p_usr=$(awk -v u=$d_user -v n=$d_nice -v t=$d_total 'BEGIN{printf "%.1f", 100*(u+n)/t}')
      p_sys=$(awk -v s=$d_sys -v t=$d_total 'BEGIN{printf "%.1f", 100*s/t}')
      p_iow=$(awk -v i=$d_iow -v t=$d_total 'BEGIN{printf "%.1f", 100*i/t}')
      p_soft=$(awk -v s=$d_sirq -v t=$d_total 'BEGIN{printf "%.1f", 100*s/t}')
    else
      p_usr=0; p_sys=0; p_iow=0; p_soft=0
    fi
    prev_user=$cur_user; prev_nice=$cur_nice; prev_sys=$cur_sys; prev_idle=$cur_idle
    prev_iow=$cur_iow; prev_irq=$cur_irq; prev_sirq=$cur_sirq; prev_steal=$cur_steal

    # Load average (1 min) — kernel EMA, τ=60s. Slow to react / decay,
    # good for sustained-trend view but useless for near-real-time changes.
    local load1
    load1=$(awk '{print $1}' /proc/loadavg)

    # loadN — rolling average of /proc/stat procs_running over last N samples.
    # N defaults to 5 (≈ 5s at 1Hz interval). Sharp short-window load metric.
    # /proc/stat procs_running is an instantaneous R-state count (no averaging).
    local procR_cur
    procR_cur=$(awk '/^procs_running/{print $2}' /proc/stat)
    procR_hist+=("$procR_cur")
    if [ ${#procR_hist[@]} -gt "${MON_LOADN:-5}" ]; then
      procR_hist=("${procR_hist[@]:1}")
    fi
    local loadN
    loadN=$(printf '%s\n' "${procR_hist[@]}" | awk '{s+=$1;n++} END{if(n)printf "%.1f", s/n; else print "?"}')

    # PSI (Pressure Stall Information, 10s window) — sharper short-window signal
    # than load1, AND splits the cause into CPU / I/O / memory. Kernel ≥4.20.
    local psi_cpu_some psi_io_full psi_mem_some
    psi_cpu_some=$(psi cpu    some avg10)
    psi_io_full=$(psi  io     full avg10)
    psi_mem_some=$(psi memory some avg10)

    # Memory used %
    local mem_used
    mem_used=$(awk '/^MemTotal:/{t=$2} /^MemAvailable:/{a=$2} END{if(t)printf "%.1f", 100*(t-a)/t; else print "0.0"}' /proc/meminfo)

    # Top CPU process
    top1=$(ps -eo pcpu,comm --sort=-pcpu --no-headers 2>/dev/null | head -1)
    t1n=$(echo "$top1" | awk '{print $2}')
    t1c=$(echo "$top1" | awk '{print $1}')

    # Build flags string. Each flag = a stack-affecting issue.
    local flags=""
    [ "$thr_pkg_dt" -gt 0 ] || [ "$thr_core_dt" -gt 0 ] && flags="$flags THROTTLE"
    [ "$pkg_c" -gt 95 ] 2>/dev/null && flags="$flags HOT"
    awk -v v=$p_iow 'BEGIN{exit !(v>5.0)}' && flags="$flags IO"
    awk -v v=$p_soft 'BEGIN{exit !(v>10.0)}' && flags="$flags IRQ"
    awk -v l=$load1 -v n=$nproc 'BEGIN{exit !(l>n)}' && flags="$flags LOAD"
    awk -v v=$mem_used 'BEGIN{exit !(v>90.0)}' && flags="$flags MEM"
    # PSI-derived flags (10s window — react ~6× faster than load1)
    awk -v v=${psi_cpu_some:-0} 'BEGIN{exit !(v>20.0)}' && flags="$flags CPU_PRESS"
    awk -v v=${psi_io_full:-0}  'BEGIN{exit !(v>5.0)}'  && flags="$flags IO_STALL"
    awk -v v=${psi_mem_some:-0} 'BEGIN{exit !(v>5.0)}'  && flags="$flags MEM_PRESS"

    # Flag display: 5-char fixed width, green=OK / red=first-flag.
    # Multi-flag case shows only the first flag (truncated to 5 chars).
    # Full flag list is visible in MON_VERBOSE mode.
    local flag_short
    if [ -z "$flags" ]; then
      flag_short="OK"
      mark=$'\e[32m'"$(printf '%-5s' "$flag_short")"$'\e[0m'
    else
      flag_short=$(echo "${flags# }" | awk '{print substr($1,1,5)}')
      mark=$'\e[31m'"$(printf '%-5s' "$flag_short")"$'\e[0m'
    fi

    # Per-metric color (all → red at danger):
    #   pkg_temp : orange256, red ≥90°C
    #   W (pkg_w): yellow,    red ≥PL1
    #   loadN/1  : cyan,      red ≥nproc
    # Wrap the printed value only; printf widths inside the colored
    # string keep column alignment.
    # ANSI: 33=yellow, 36=cyan, 38;5;208=orange256, 31=red.
    local pkg_col w_col load_col load1_col
    if [ "${pkg_c:-0}" -ge 90 ] 2>/dev/null; then
      pkg_col=$'\e[31m'"$(printf '%3s' "$pkg_c")"$'\e[0m'
    else
      pkg_col=$'\e[38;5;208m'"$(printf '%3s' "$pkg_c")"$'\e[0m'
    fi
    if [ "${pkg_w:-?}" != "?" ] && awk -v w="$pkg_w" -v p="$pl1" 'BEGIN{exit !(w+0>=p+0)}'; then
      w_col=$'\e[31m'"$(printf '%5s' "$pkg_w")"$'\e[0m'
    else
      w_col=$'\e[33m'"$(printf '%5s' "${pkg_w:-?}")"$'\e[0m'
    fi
    if awk -v l="$loadN" -v n="$nproc" 'BEGIN{exit !(l+0>=n+0)}'; then
      load_col=$'\e[31m'"$(printf '%5s' "$loadN")"$'\e[0m'
    else
      load_col=$'\e[36m'"$(printf '%5s' "$loadN")"$'\e[0m'
    fi
    if awk -v l="$load1" -v n="$nproc" 'BEGIN{exit !(l+0>=n+0)}'; then
      load1_col=$'\e[31m'"$(printf '%5s' "$load1")"$'\e[0m'
    else
      load1_col=$'\e[36m'"$(printf '%5s' "$load1")"$'\e[0m'
    fi

    # CSV line — full data (parseable; loadN appended at end for backward compat)
    line="$ts,$pl1,$pl2,$pct,$pkg_c,$core_max,${nvme:-?},$favg,$fmx,$thr_pkg_dt,$thr_core_dt,${pkg_w:-?},$p_usr,$p_sys,$p_iow,$p_soft,$load1,$mem_used,${t1n:-?},${t1c:-?},${psi_cpu_some:-?},${psi_io_full:-?},${psi_mem_some:-?},$loadN,${limit_reason:--}"
    echo "$line" >> "$logfile"

    # Periodic header for stdout readability (every 20 ticks)
    if [ $((tick_n % 20)) -eq 0 ]; then
      printf "\n%-8s %-5s  %-5s %-3s %-3s %-5s %-5s %-5s %-5s %-5s %-11s  %s\n" \
        "TIME" "FLAG" "PL" "pct" "pkg" "W" "thr" "load${MON_LOADN:-5}" "load1" "mem%" "psi(c/i/m)" "TOP1"
    fi
    tick_n=$((tick_n+1))

    # Throttle reason tag (red) — only shown when a reason is present this tick.
    local rtag=""
    if [ -n "$limit_reason" ] && [ "$limit_reason" != "-" ]; then
      rtag=$' \e[31m['"$limit_reason"$']\e[0m'
    fi
    # 5 ROS axes tag (cyan) — appended when live_monitor JSON is fresh.
    local atag=""
    if [ -n "$live_axes" ]; then
      atag=$' \e[36m'"$live_axes"$'\e[0m'
    fi

    if [ "${MON_VERBOSE:-0}" = "1" ]; then
      # Original full format (includes freq, pkg_W, usr/sys/iow/soft, core°C)
      printf "%s [%s] PL=%s/%s pct=%s%%  pkg=%s°C core=%s°C  freq=%s/%s  thr=%s/%s  W=%s  usr=%s%% sys=%s%% iow=%s%% soft=%s%%  load%ss=%s load1=%s  mem=%s%%  psi=%s/%s/%s  top=%s(%s%%)%s%s\n" \
        "$ts" "$mark" "$pl1" "$pl2" "$pct" "$pkg_c" "$core_max" "$favg" "$fmx" "$thr_pkg_dt" "$thr_core_dt" "${pkg_w:-?}" "$p_usr" "$p_sys" "$p_iow" "$p_soft" "${MON_LOADN:-5}" "$loadN" "$load1" "$mem_used" "${psi_cpu_some:-?}" "${psi_io_full:-?}" "${psi_mem_some:-?}" "${t1n:-?}" "${t1c:-?}" "$rtag" "$atag"
    else
      # Compact format (default) — column widths aligned with header.
      # pkg_col / load_col already include ANSI + padding; use %s for them.
      printf "%s %s  %3s/%-2s %2s%% %s %s %5s %s %s %5s %5s/%s/%s  %s(%s%%)%s%s\n" \
        "$ts" "$mark" "$pl1" "$pl2" "$pct" "$pkg_col" "$w_col" "${thr_pkg_dt}/${thr_core_dt}" "$load_col" "$load1_col" "$mem_used" "${psi_cpu_some:-?}" "${psi_io_full:-?}" "${psi_mem_some:-?}" "${t1n:-?}" "${t1c:-?}" "$rtag" "$atag"
    fi
  done
}

# ---------- top-level dispatch ----------

cmd="${1:-}"

case "$cmd" in
  -h|--help|help)
    sed -n '2,21p' "$0" | sed 's/^# \?//'
    exit 0
    ;;
  install)
    ensure_tools
    echo "[OK] tools ready."
    exit 0
    ;;
  show|"")
    show
    exit 0
    ;;
  monitor)
    ensure_tools
    monitor_loop "${2:-1}"
    ;;
  pct)
    need_root "$@"
    ensure_tools
    apply_pct "${2:?usage: $0 pct N}"
    show
    ;;
  reset)
    need_root "$@"
    ensure_tools
    echo "=== before ==="; show; echo
    apply_pl 200 64
    apply_pct 100
    echo "=== after (reset) ==="; show
    ;;
  set)
    need_root "$@"
    ensure_tools
    PL1_W="${2:-$DEFAULT_PL1}"
    PL2_W="${3:-$PL1_W}"
    PCT_V="${4:-}"
    echo "=== before ==="; show; echo
    echo "=== writing PL1=${PL1_W}W PL2=${PL2_W}W${PCT_V:+ pct=${PCT_V}%} ==="
    apply_pl "$PL1_W" "$PL2_W"
    [ -n "$PCT_V" ] && apply_pct "$PCT_V"
    echo
    echo "=== after ==="; show
    ;;
  *)
    # First arg is a number → treat as `set <PL1>` (back-compat)
    case "$cmd" in *[!0-9]*) echo "[ERR] unknown command: $cmd" >&2; "$0" --help; exit 2 ;; esac
    need_root "$@"
    ensure_tools
    PL1_W="$cmd"
    PL2_W="${2:-$PL1_W}"
    PCT_V="${3:-}"
    echo "=== before ==="; show; echo
    echo "=== writing PL1=${PL1_W}W PL2=${PL2_W}W${PCT_V:+ pct=${PCT_V}%} ==="
    apply_pl "$PL1_W" "$PL2_W"
    [ -n "$PCT_V" ] && apply_pct "$PCT_V"
    echo
    echo "=== after ==="; show
    ;;
esac
