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
#   reset                           # back to PPD-default-ish (PL1=200, PL2=40, pct=100)
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
  printf "PL1 (long_term):    %3d W   window=%s us\n" "$((pl1/1000000))" "$tw1"
  printf "PL2 (short_term):   %3d W   window=%s us\n" "$((pl2/1000000))" "$tw2"
  printf "max_perf_pct:       %s%%   no_turbo=%s\n" "$pct" "$noturbo"
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
  local logdir="${HOME:-/tmp}/.set_pl_monitor"
  mkdir -p "$logdir"
  local logfile="$logdir/$(date +%Y-%m-%d-%H-%M-%S).log"
  local nproc; nproc=$(nproc 2>/dev/null || echo 22)

  echo "[INFO] monitor interval=${interval}s  Ctrl-C to stop"
  echo "[INFO] log: $logfile"
  echo "[INFO] flags: THROTTLE=clock cap | HOT=pkg>95C | IO=iowait>5% | IRQ=softirq>10% | LOAD=load1>nproc | MEM=used>90%"
  echo

  # baseline counters
  local prev_thr_pkg prev_thr_core prev_e prev_t
  prev_thr_pkg=$(cat /sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_count 2>/dev/null || echo 0)
  prev_thr_core=$(awk '{s+=$1} END{print s+0}' /sys/devices/system/cpu/cpu*/thermal_throttle/core_throttle_count 2>/dev/null)
  prev_e=$(cat "$PL_DIR/energy_uj" 2>/dev/null || echo 0)
  prev_t=$(date +%s.%N)

  # baseline /proc/stat (for CPU breakdown deltas)
  # cpu  user nice system idle iowait irq softirq steal guest guest_nice
  local prev_user prev_nice prev_sys prev_idle prev_iow prev_irq prev_sirq prev_steal
  read _ prev_user prev_nice prev_sys prev_idle prev_iow prev_irq prev_sirq prev_steal _ < /proc/stat

  # CSV header
  local header="ts,PL1_W,PL2_W,pct,pkg_C,core_max_C,nvme_C,freq_avg_MHz,freq_max_MHz,thr_pkg_dt,thr_core_dt,pkg_W,usr%,sys%,iowait%,soft%,load1,mem_used%,top1,top1_pct"
  echo "$header" | tee "$logfile"

  trap 'echo; echo "[INFO] monitor stopped. log: '"$logfile"'"; exit 0' INT TERM

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

    # Package power from RAPL energy counter (uJ -> J -> avg W over interval)
    e=$(cat "$PL_DIR/energy_uj" 2>/dev/null || echo 0)
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

    # Load average (1 min)
    local load1
    load1=$(awk '{print $1}' /proc/loadavg)

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
    if [ -z "$flags" ]; then
      mark=$'\e[32mOK\e[0m       '
    else
      mark=$'\e[31m'"${flags# }"$'\e[0m'
      # pad to ~8 chars for alignment
      mark=$(printf "%-30s" "$mark")
    fi

    line="$ts,$pl1,$pl2,$pct,$pkg_c,$core_max,${nvme:-?},$favg,$fmx,$thr_pkg_dt,$thr_core_dt,${pkg_w:-?},$p_usr,$p_sys,$p_iow,$p_soft,$load1,$mem_used,${t1n:-?},${t1c:-?}"
    echo "$line" >> "$logfile"
    printf "%s [%s] PL=%s/%s pct=%s%%  pkg=%s°C core=%s°C  freq=%s/%s  thr=%s/%s  W=%s  usr=%s%% sys=%s%% iow=%s%% soft=%s%%  load1=%s  mem=%s%%  top=%s(%s%%)\n" \
      "$ts" "$mark" "$pl1" "$pl2" "$pct" "$pkg_c" "$core_max" "$favg" "$fmx" "$thr_pkg_dt" "$thr_core_dt" "${pkg_w:-?}" "$p_usr" "$p_sys" "$p_iow" "$p_soft" "$load1" "$mem_used" "${t1n:-?}" "${t1c:-?}"
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
    apply_pl 200 40
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
