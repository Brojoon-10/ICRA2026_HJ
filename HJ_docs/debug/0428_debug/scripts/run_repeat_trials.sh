#!/bin/bash
# Run N short bags with the same setup, then aggregate metrics.
#
# Sim variance is large per single bag — single-run comparisons mislead.
# This script records 5x60s bags back-to-back so we can average.
#
# Usage:  bash run_repeat_trials.sh <prefix> <count> <duration_s>
PREFIX="${1:-trial}"
COUNT="${2:-5}"
DUR_S="${3:-60}"

BAGDIR="$(dirname "$0")/../bags"
mkdir -p "$BAGDIR"
cd "$BAGDIR" || exit 1

for i in $(seq 1 "$COUNT"); do
    NAME="${PREFIX}_run${i}"
    echo "=== Trial ${i}/${COUNT}: ${NAME} (${DUR_S}s) ==="
    rm -f "${NAME}.bag" "${NAME}.bag.active"
    rosbag record -O "${NAME}" --duration="${DUR_S}" \
        /tracking/obstacles_truth /car_state/odom_frenet \
        /mpc_auto/best_trajectory_observation /mpc_auto/status \
        /mpc_auto/timing_ms /mpc_auto/debug/tick_json \
        /opponent_prediction/obstacles /opponent_collision \
        /planner/mpc/wpnts > /dev/null 2>&1
    sleep 1
done
echo "=== ALL TRIALS DONE ==="
