#!/bin/bash
### IY : 딸깍 FWBW 속도 프로파일 실행 래퍼
# Usage:
#   ./run_fwbw.sh --track <smoothed.csv> --raceline <raceline.csv>
#   ./run_fwbw.sh --track eng_0404_v2_3d_smoothed.csv --raceline eng_0406.csv
#   ./run_fwbw.sh --map-dir /path/to/map_folder   (map-dir 모드)
#
# Options (모두 optional, 기본값 있음):
#   --track      smoothed track CSV filename (data/smoothed_track_data/ 기준)
#   --raceline   racing line CSV filename    (data/global_racing_lines/ 기준)
#   --map-dir    map-dir 모드: 이 폴더 안의 CSV를 직접 사용
#   --vehicle    vehicle name (default: rc_car_10th)
#   --params     params YAML  (default: params_rc_car_10th.yml)
#   --model      FBGA model   (default: lookup)
#   --n-laps     closed-loop laps (default: 3)
#   --spacing    resample spacing [m] (default: 0.1)
#   --no-build   FBGA 빌드 스킵

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FBGA_ROOT="$SCRIPT_DIR"
GLOBAL_LINE_DIR="$(cd "$SCRIPT_DIR/../../planner/3d_gb_optimizer/global_line" 2>/dev/null && pwd || echo "")"

# ─── 기본값 ──────────────────────────────────────────────────────────────
SKIP_BUILD=false
MAP_DIR=""
FWBW_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build)  SKIP_BUILD=true; shift ;;
        --map-dir)   MAP_DIR="$2"; shift 2 ;;
        *)           FWBW_ARGS+=("$1"); shift ;;
    esac
done

# ─── 1. FBGA 바이너리 확인 / 빌드 ────────────────────────────────────────
RUNNER_BIN="$FBGA_ROOT/bin/GIGI_test_unicorn.exe"

if [[ ! -f "$RUNNER_BIN" ]]; then
    echo "[FBGA] 바이너리 없음 → 빌드 시작..."
    SKIP_BUILD=false
fi

if [[ "$SKIP_BUILD" == false && ! -f "$RUNNER_BIN" ]]; then
    echo "[FBGA] third_party 의존성 설치..."
    cd "$FBGA_ROOT"
    if [[ -f third_party.sh ]]; then
        bash third_party.sh
    fi
    echo "[FBGA] 빌드 중..."
    bash build.sh
    cd - > /dev/null

    if [[ ! -f "$RUNNER_BIN" ]]; then
        echo "ERROR: 빌드 후에도 바이너리가 없음: $RUNNER_BIN"
        exit 1
    fi
    echo "[FBGA] 빌드 완료: $RUNNER_BIN"
else
    echo "[FBGA] 바이너리 확인: $RUNNER_BIN"
fi

# ─── 2. global_line 디렉토리 확인 ────────────────────────────────────────
if [[ -z "$GLOBAL_LINE_DIR" || ! -d "$GLOBAL_LINE_DIR" ]]; then
    # fallback: FBGA가 f110_utils/libs/ 아래라고 가정
    REPO_ROOT="$(cd "$FBGA_ROOT/../../../" && pwd)"
    GLOBAL_LINE_DIR="$REPO_ROOT/planner/3d_gb_optimizer/global_line"
fi

if [[ ! -d "$GLOBAL_LINE_DIR" ]]; then
    echo "ERROR: global_line 디렉토리를 찾을 수 없음"
    echo "  시도한 경로: $GLOBAL_LINE_DIR"
    exit 1
fi

RUN_FWBW="$GLOBAL_LINE_DIR/global_racing_line/run_fwbw.py"
if [[ ! -f "$RUN_FWBW" ]]; then
    echo "ERROR: run_fwbw.py 없음: $RUN_FWBW"
    exit 1
fi

echo "[FBGA] global_line: $GLOBAL_LINE_DIR"
echo "[FBGA] run_fwbw.py: $RUN_FWBW"

# ─── 3. Python 의존성 체크 ────────────────────────────────────────────────
python3 -c "import numpy, pandas, yaml, scipy, matplotlib" 2>/dev/null || {
    echo "[FBGA] Python 의존성 설치 중..."
    pip install numpy pandas pyyaml scipy matplotlib
}

# ─── 4. run_fwbw.py 실행 ─────────────────────────────────────────────────
export FBGA_ROOT="$FBGA_ROOT"

echo ""
echo "======================================================"
echo " FWBW Velocity Profiler"
echo "  FBGA_ROOT   : $FBGA_ROOT"
echo "  run_fwbw.py : $RUN_FWBW"
echo "  args        : ${FWBW_ARGS[*]}"
echo "======================================================"
echo ""

cd "$GLOBAL_LINE_DIR"
python3 global_racing_line/run_fwbw.py "${FWBW_ARGS[@]}"
