#!/bin/bash
# apply_isolation.sh — ROS launch 후 컨테이너 내부에서 실행
#
# 목적:
#   3d_state_machine_node 와 controller_manager 를 dedicated P-core HT pair 에 고정해서
#   glim_ros 의 ~10s 주기 burst (peak 17 cores) 가 control loop 를 starve 시키지 못하게 함.
#
#   데이터 근거 (load_attribute.py 누적):
#     - glim BURST(>1260%) 시 state_machine wait avg=251.8ms (>50ms 98.7%)
#     - glim QUIET    시 state_machine wait avg= 29.9ms (>50ms  7.9%)
#     - glim CPU% 와 sm wait 단조 비례 (>=1200% 일 때 sm 평균 232ms wait)
#     → 격리 후 sm wait>50ms 가 거의 0 으로 떨어져야 함 (검증 가능)
#
# 코어 도메인 (Core Ultra 7 155H, 22 threads, hybrid P+E+LP):
#     sm + cm        →  cpu 0, 5         (1 P-core HT pair, dedicated, 캐시 친화)
#     glim_ros       →  cpu 1-4, 6-19    (18 threads: 나머지 P+모든 E, peak 17 들어옴)
#     cam ffmpeg     →  cpu 20, 21       (LP E-cores, 카메라 녹화 전용 — user 기존)
#
# 사용 (컨테이너 내부에서):
#     sudo bash stack_master/scripts/apply_isolation.sh
#     SM_CORES="0,1" sudo -E bash ... apply_isolation.sh   # 코어 변경
#     RT_PRIO="80"   sudo -E bash ...                       # RT 우선순위 변경
#     NO_RT=1   sudo -E bash ...                            # SCHED_FIFO 생략
#     NO_GOV=1  sudo -E bash ...                            # cpufreq governor 변경 생략
#
# 검증:
#     monitor 에서 state_machine wait>50ms 누적이 거의 안 증가해야 함.
#     원래 분당 ~3건 → 격리 후 거의 0건 기대.

set -uo pipefail

SM_CORES="${SM_CORES:-0,5}"
CM_CORES="${CM_CORES:-$SM_CORES}"
GLIM_CORES="${GLIM_CORES:-1-4,6-19}"
RT_PRIO="${RT_PRIO:-50}"

die() { echo "[ERR] $*" >&2; exit 1; }

# ---------- pre-check ----------

SM=$(pgrep -f 3d_state_machine_node | head -1)
CM=$(pgrep -f controller_manager    | head -1)
GP=$(pgrep -f glim_rosnode          | head -1)

echo "[info] sm=$SM  cm=$CM  glim=$GP"
[ -z "$SM" ] && die "state_machine pid not found — ROS launch 안 떴거나 노드 이름 다름?"
[ -z "$CM" ] && die "controller_manager pid not found"
[ -z "$GP" ] && die "glim_rosnode pid not found"

# ---------- 1. CPU affinity (taskset) ----------

echo ""
echo "[1/4] CPU affinity"
taskset -cp "$SM_CORES" "$SM" > /dev/null && echo "  sm  ($SM) → cpu $SM_CORES"
taskset -cp "$CM_CORES" "$CM" > /dev/null && echo "  cm  ($CM) → cpu $CM_CORES"

# glim 의 모든 thread 에 적용 (보통 51개)
n_threads=0
for tid_dir in /proc/"$GP"/task/*/; do
  t=$(basename "$tid_dir")
  if taskset -cp "$GLIM_CORES" "$t" >/dev/null 2>&1; then
    n_threads=$((n_threads+1))
  fi
done
echo "  glim ($GP, $n_threads threads) → cpu $GLIM_CORES"

# ---------- 2. RT priority (SCHED_FIFO) ----------

if [ "${NO_RT:-0}" != "1" ]; then
  echo ""
  echo "[2/4] RT priority (SCHED_FIFO, prio=$RT_PRIO)"
  if chrt -f -p "$RT_PRIO" "$SM" 2>/dev/null; then
    echo "  sm  ($SM) → SCHED_FIFO $RT_PRIO"
  else
    echo "  [WARN] chrt 실패 — CAP_SYS_NICE 또는 root 필요"
    echo "         일회 해결: setcap cap_sys_nice+ep /usr/bin/chrt"
  fi
  chrt -f -p "$RT_PRIO" "$CM" 2>/dev/null && echo "  cm  ($CM) → SCHED_FIFO $RT_PRIO"
else
  echo ""
  echo "[2/4] RT priority skipped (NO_RT=1)"
fi

# ---------- 3. cpufreq governor ----------

if [ "${NO_GOV:-0}" != "1" ]; then
  echo ""
  echo "[3/4] cpufreq governor → performance"
  changed=0
  for c in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    cur=$(cat "$c" 2>/dev/null)
    if [ "$cur" != "performance" ]; then
      echo performance > "$c" 2>/dev/null && changed=$((changed+1))
    fi
  done
  echo "  $changed cores → performance"
else
  echo ""
  echo "[3/4] governor skipped (NO_GOV=1)"
fi

# ---------- 4. verify ----------

echo ""
echo "[4/4] verify"
printf "  sm:   affinity=%s\n" "$(taskset -p $SM 2>/dev/null | awk '{print $NF}')"
printf "  sm:   sched=%s\n"    "$(chrt -p $SM 2>&1 | tr '\n' ' ' | sed 's/  */ /g')"
printf "  cm:   affinity=%s\n" "$(taskset -p $CM 2>/dev/null | awk '{print $NF}')"
printf "  cm:   sched=%s\n"    "$(chrt -p $CM 2>&1 | tr '\n' ' ' | sed 's/  */ /g')"
printf "  glim: affinity=%s (sample, main thread)\n" "$(taskset -p $GP 2>/dev/null | awk '{print $NF}')"
printf "  cpu0 governor=%s\n"  "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"

echo ""
echo "DONE. monitor 로 state_machine wait>50ms 추세 확인."
echo "      이전 분당 ~3건 → 격리 후 거의 0건 기대."
