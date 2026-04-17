#!/bin/bash
### HJ : host-side wrapper — calls run_on_container.sh via docker exec
set -e

VEHICLE_NAME="${1:-rc_car_10th}"
RESOLUTION="${2:---fast}"

DOCKER_SCRIPT_DIR="/home/unicorn/catkin_ws/src/race_stack/planner/3d_gb_optimizer/fast_ggv_gen"

## IY : forward TUNING + TUNING_NAME env vars into the container (default OFF)
#       Usage from host:
#         ./run.sh rc_car_10th                                     # tuning OFF
#         TUNING=1 ./run.sh rc_car_10th                            # tuning ON (default name)
#         TUNING=1 ./run.sh rc_car_10th --full                     # ON + full resolution
#         TUNING=1 TUNING_NAME=rc_car_10th ./run.sh rc_car_10th_v2 # base v2, shared tuning
# docker exec icra2026 bash "$DOCKER_SCRIPT_DIR/run_on_container.sh" "$VEHICLE_NAME" "$RESOLUTION"
docker exec \
    -e TUNING="${TUNING:-0}" \
    -e TUNING_NAME="${TUNING_NAME:-}" \
    icra2026 \
    bash "$DOCKER_SCRIPT_DIR/run_on_container.sh" "$VEHICLE_NAME" "$RESOLUTION"
## IY : end
