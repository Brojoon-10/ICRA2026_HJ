#!/usr/bin/env bash
# ELRS / joy / VESC / launch debug logger
#
# Captures every low-rate signal needed to diagnose ELRS link instability,
# spurious launch fires (RB/A noise -> jerk=512 current burst), and
# autodrive -> VESC command flow, all timestamped together.
#
# Run inside the icra2026 docker container with ROS sourced:
#   docker exec -it icra2026 bash
#   source /opt/ros/noetic/setup.bash
#   source $HOME/catkin_ws/devel/setup.bash
#   bash $HOME/catkin_ws/src/race_stack/stack_master/scripts/elrs_debug.sh [duration_sec]
#
# Output (timestamped, per-session subdirectory under HJ_docs):
#   HJ_docs/debug/elrs_debug/<YYYYMMDD_HHMMSS>/
#     elrs_debug_<TS>.bag       full rosbag (raw, replayable)
#     csv/<topic>.csv           per-topic CSV via `rostopic echo -p`
#     usb_kernel.log            host kernel cp210x / ttyUSB events
#                               (journalctl tail; may be empty in container)
#     elrs_status.csv           10Hz poll of ELRS USB/node connection state:
#                               /dev/ELRS present, ttyUSB target, sysfs
#                               runtime_status, connected_duration,
#                               /elrs_joy_node alive
#     udev_events.log           event-driven udev monitor (kernel + udev),
#                               ms-precision timestamps for every USB
#                               add/remove. Catches disconnect events that
#                               the 100ms sysfs poll might still bracket.
#     manifest.txt              session metadata (start/end/topics)
#
# Usage:
#   ./elrs_debug.sh             # log until Ctrl+C
#   ./elrs_debug.sh 300         # auto-stop after 300 seconds

set -u

TS=$(date +%Y%m%d_%H%M%S)

# Resolve repo root: container path first, then host path
if [ -d "$HOME/catkin_ws/src/race_stack/HJ_docs" ]; then
    REPO="$HOME/catkin_ws/src/race_stack"
elif [ -d "$HOME/icra2026_ws/ICRA2026_HJ/HJ_docs" ]; then
    REPO="$HOME/icra2026_ws/ICRA2026_HJ"
else
    echo "[elrs_debug] ERROR: cannot find HJ_docs under \$HOME"
    exit 1
fi

OUTDIR="${REPO}/HJ_docs/debug/elrs_debug/${TS}"
CSVDIR="${OUTDIR}/csv"
mkdir -p "${CSVDIR}"

# Topics to record (every debug-useful low-rate topic)
TOPICS=(
    # ── ELRS link health ──
    /elrs_joy_node/debug_channels                       # raw 16 CRSF channels (link integrity)
    /joy                                                 # post-debounce buttons/axes
    /joy/set_feedback                                    # rumble (rare)
    /rosout                                              # all node log lines (catches CRSF connect/loss/stats, LB/RB transitions, launch FIRE)

    # ── launch state (jerk=512 current injection path) ──
    /launch_controller/debug                             # armed/active/mode/phase JSON
    /launch_controller/abort                             # external abort events

    # ── autodrive controller -> mux ──
    /vesc/high_level/ackermann_cmd_mux/input/nav_1       # controller intent BEFORE mux

    # ── mux -> VESC driver ──
    /vesc/low_level/ackermann_cmd_mux/output             # final AckermannDriveStamped to driver (jerk/accel/speed)

    # ── VESC driver outputs (ground truth of what was commanded) ──
    /vesc/commands/motor/current                         # direct current injection (THE crash channel)
    /vesc/commands/motor/speed                           # speed PID setpoint
    /vesc/commands/motor/brake                           # brake current
    /vesc/commands/motor/duty_cycle                      # duty cycle override
    /vesc/commands/motor/position                        # position mode (rare)
    /vesc/commands/servo/position                        # steering servo PWM
    /vesc/sensors/servo_position_command                 # echo of servo cmd

    # ── VESC telemetry (what actually happened on the motor) ──
    /vesc/sensors/core                                   # bus voltage, motor current, ERPM, temp
    /vesc/sensors/imu                                    # onboard IMU
    /vesc/sensors/imu/raw                                # raw IMU
    /vesc/odom                                           # wheel-odom velocity
    /vesc/filtered_eprm                                  # filtered ERPM
)

MANIFEST="${OUTDIR}/manifest.txt"
{
    echo "elrs_debug log session"
    echo "  started_at  : $(date -Iseconds)"
    echo "  host        : $(hostname)"
    echo "  user        : $(whoami)"
    echo "  ros_master  : ${ROS_MASTER_URI:-unset}"
    echo "  ros_ip      : ${ROS_IP:-unset}"
    echo "  output_dir  : ${OUTDIR}"
    echo ""
    echo "topics (${#TOPICS[@]}):"
    for T in "${TOPICS[@]}"; do echo "  ${T}"; done
} > "${MANIFEST}"

echo "[elrs_debug] output: ${OUTDIR}"
echo "[elrs_debug] starting rosbag + ${#TOPICS[@]} CSV streams + status poll + kernel tail"

# ── rosbag (raw, replayable) ──
BAG_PATH="${OUTDIR}/elrs_debug_${TS}.bag"
rosbag record -O "${BAG_PATH}" "${TOPICS[@]}" __name:=elrs_debug_recorder \
    > "${OUTDIR}/rosbag.stderr" 2>&1 &
BAG_PID=$!

# ── per-topic CSV (rostopic echo -p, pandas-friendly) ──
CSV_PIDS=()
for T in "${TOPICS[@]}"; do
    NAME=$(echo "${T#/}" | tr '/' '_')
    rostopic echo -p "${T}" > "${CSVDIR}/${NAME}.csv" 2>/dev/null &
    CSV_PIDS+=($!)
done

# ── ELRS connection-state watchdog (10 Hz, 100ms period) ──
# Polls USB / udev / sysfs / ROS-node state every 100ms and writes a row each
# tick. Bracketed against udev_events.log (event-driven, ms-precision) so a
# disconnect that lasts <1s still shows up in both: the poll captures the
# state-flip window, udev pinpoints the exact moment.
STATUS_CSV="${OUTDIR}/elrs_status.csv"
STATUS_PERIOD_S=0.1
(
    echo "ts_epoch,ts_iso,dev_elrs_present,tty_target,sysfs_path,sysfs_runtime_status,sysfs_power_control,sysfs_connected_dur_s,sysfs_speed_mbps,sysfs_bMaxPower,node_alive,joy_pubs"
    TICK=0
    while true; do
        T_EPOCH=$(date +%s.%N)
        T_ISO=$(date -Iseconds)

        # /dev/ELRS udev symlink (set by 99-elrs.rules on VID:PID 10c4:ea60)
        if [ -L /dev/ELRS ]; then
            DEV_PRESENT=1
            TTY_TARGET=$(readlink /dev/ELRS 2>/dev/null || echo "")
        elif [ -e /dev/ELRS ]; then
            DEV_PRESENT=1
            TTY_TARGET="(non-symlink)"
        else
            DEV_PRESENT=0
            TTY_TARGET=""
        fi

        # Resolve sysfs path for the CP2102 (VID 10c4 PID ea60). Dynamic because
        # disconnect can change the USB device number (3-4 -> 3-x).
        SYSFS_PATH=""
        for ID_FILE in /sys/bus/usb/devices/*/idProduct; do
            [ -f "${ID_FILE}" ] || continue
            PID=$(cat "${ID_FILE}" 2>/dev/null || echo "")
            VID=$(cat "$(dirname "${ID_FILE}")/idVendor" 2>/dev/null || echo "")
            if [ "${VID}" = "10c4" ] && [ "${PID}" = "ea60" ]; then
                SYSFS_PATH=$(dirname "${ID_FILE}")
                break
            fi
        done

        if [ -n "${SYSFS_PATH}" ]; then
            RT_STATUS=$(cat "${SYSFS_PATH}/power/runtime_status" 2>/dev/null || echo "")
            PW_CTRL=$(cat "${SYSFS_PATH}/power/control" 2>/dev/null || echo "")
            # connected_duration is in milliseconds
            CONN_MS=$(cat "${SYSFS_PATH}/power/connected_duration" 2>/dev/null || echo "")
            if [ -n "${CONN_MS}" ]; then
                CONN_S=$(awk "BEGIN{printf \"%.1f\", ${CONN_MS}/1000}")
            else
                CONN_S=""
            fi
            SPEED=$(cat "${SYSFS_PATH}/speed" 2>/dev/null || echo "")
            BMAXP=$(cat "${SYSFS_PATH}/bMaxPower" 2>/dev/null || echo "")
        else
            RT_STATUS=""
            PW_CTRL=""
            CONN_S=""
            SPEED=""
            BMAXP=""
        fi

        # ROS node liveness (rosnode list is slow; check only every ~10 ticks = 1s)
        if [ $((TICK % 10)) -eq 0 ]; then
            if rosnode list 2>/dev/null | grep -q "^/elrs_joy_node$"; then
                NODE_ALIVE_CACHED=1
            else
                NODE_ALIVE_CACHED=0
            fi
            JOY_PUBS_CACHED=$(rostopic info /joy 2>/dev/null | awk '/Publishers:/{flag=1; next} /Subscribers:/{flag=0} flag && /\*/{c++} END{print c+0}')
        fi
        TICK=$((TICK + 1))

        echo "${T_EPOCH},${T_ISO},${DEV_PRESENT},${TTY_TARGET},${SYSFS_PATH},${RT_STATUS},${PW_CTRL},${CONN_S},${SPEED},${BMAXP},${NODE_ALIVE_CACHED:-0},${JOY_PUBS_CACHED:-0}"
        sleep "${STATUS_PERIOD_S}"
    done
) > "${STATUS_CSV}" 2>/dev/null &
STATUS_PID=$!

# ── udev event-driven monitor (ms precision, catches every USB add/remove) ──
# `udevadm monitor --kernel --udev --property` prints every kernel uevent
# AND every udev-processed event with sub-ms timestamps. Filter to tty/usb
# subsystems via subsequent grep — udevadm itself filters by --subsystem-match,
# but we want both tty (ttyUSB) and usb device-level events.
UDEV_LOG="${OUTDIR}/udev_events.log"
UDEV_PID=""
if command -v udevadm >/dev/null 2>&1; then
    ( stdbuf -oL udevadm monitor --kernel --udev --property 2>/dev/null \
        | stdbuf -oL grep --line-buffered -E "^(KERNEL|UDEV)|SUBSYSTEM=(tty|usb)|DEVNAME=/dev/(tty|ELRS)|ID_VENDOR_ID=10c4|ID_MODEL_ID=ea60" \
        > "${UDEV_LOG}" ) &
    UDEV_PID=$!
fi

# ── host kernel USB events (cp210x disconnect / ttyUSB) ──
# Works only if journalctl is accessible (i.e. run on host, or container has /run/log/journal mounted).
# Inside container without that, file will simply stay empty.
KLOG="${OUTDIR}/usb_kernel.log"
KLOG_PID=""
if command -v journalctl >/dev/null 2>&1; then
    ( journalctl -kf --no-pager 2>/dev/null \
        | grep --line-buffered -E "cp210x|ttyUSB|usb 3-|disconnect" \
        > "${KLOG}" ) &
    KLOG_PID=$!
fi

cleanup() {
    # idempotent: disable further INT/TERM so cleanup body runs only once
    trap '' INT TERM
    echo ""
    echo "[elrs_debug] stopping..."
    [ -n "${KLOG_PID}" ] && kill "${KLOG_PID}" 2>/dev/null || true
    [ -n "${UDEV_PID}" ] && kill "${UDEV_PID}" 2>/dev/null || true
    kill "${STATUS_PID}" 2>/dev/null || true
    for P in "${CSV_PIDS[@]}"; do kill "${P}" 2>/dev/null || true; done

    # Clean rosbag shutdown so the .bag is finalized (not .bag.active)
    if command -v rosnode >/dev/null 2>&1; then
        rosnode kill /elrs_debug_recorder >/dev/null 2>&1 || true
    fi
    # Give rosbag a moment to flush
    for _ in 1 2 3 4 5; do
        if ! kill -0 "${BAG_PID}" 2>/dev/null; then break; fi
        sleep 0.2
    done
    kill "${BAG_PID}" 2>/dev/null || true
    wait 2>/dev/null || true

    {
        echo ""
        echo "  finished_at : $(date -Iseconds)"
        if [ -f "${BAG_PATH}" ]; then
            echo "  bag_size    : $(du -h "${BAG_PATH}" | cut -f1)"
        elif [ -f "${BAG_PATH}.active" ]; then
            echo "  bag_size    : $(du -h "${BAG_PATH}.active" | cut -f1) (still .active - rosbag didn't finalize)"
        fi
        echo "  csv_files   : $(ls -1 "${CSVDIR}" 2>/dev/null | wc -l)"
        if [ -f "${STATUS_CSV}" ]; then
            echo "  status_rows : $(($(wc -l < "${STATUS_CSV}") - 1))"
        fi
        if [ -f "${UDEV_LOG}" ]; then
            echo "  udev_lines  : $(wc -l < "${UDEV_LOG}")"
        fi
        if [ -f "${KLOG}" ]; then
            echo "  kernel_lines: $(wc -l < "${KLOG}")"
        fi
    } >> "${MANIFEST}"

    echo "[elrs_debug] done."
    echo "  bag      : ${BAG_PATH}"
    echo "  csv dir  : ${CSVDIR}/"
    echo "  status   : ${STATUS_CSV}"
    echo "  udev     : ${UDEV_LOG}"
    echo "  kernel   : ${KLOG}"
    echo "  manifest : ${MANIFEST}"
    exit 0
}
trap cleanup INT TERM

DURATION="${1:-}"
if [ -n "${DURATION}" ]; then
    echo "[elrs_debug] auto-stop after ${DURATION}s (Ctrl+C to stop early)"
    sleep "${DURATION}" &
    SLEEP_PID=$!
    wait "${SLEEP_PID}" 2>/dev/null || true
    cleanup
else
    echo "[elrs_debug] press Ctrl+C to stop"
    wait "${BAG_PID}"
    cleanup
fi
