#!/bin/bash
# Transmitter: /dev/ttyACM1
# Source ESP-IDF env if not already loaded.
if [ -z "$IDF_PATH" ]; then
    for cand in \
        "$(dirname "$0")/../esp-idf/export.sh" \
        "$HOME/esp/esp-idf/export.sh" \
        "$HOME/esp_ws/esp-idf/export.sh"; do
        if [ -f "$cand" ]; then
            source "$cand" > /dev/null 2>&1
            break
        fi
    done
fi
if ! command -v idf.py > /dev/null 2>&1; then
    echo "[idf.sh] ERROR: idf.py not found. Install ESP-IDF v5.3.x and source export.sh." >&2
    exit 1
fi
ESPPORT=/dev/ttyACM1 idf.py "$@"
