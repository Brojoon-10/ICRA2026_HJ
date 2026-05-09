#!/usr/bin/env bash
# ### HJ : sampling_planner_state_node 슬라이더를 CLI 로 조작 + 디버그 로그 미러.
#
# 이 스크립트는 Claude / 사람 모두 rqt_reconfigure 없이 dynamic_reconfigure 파라미터를
# 변경하고, 변경이 events.jsonl 에 즉시 기록되는지 tick.csv tail 로 확인할 수 있게 한다.
#
# 사용법
# ------
#   set     <node>  <key>=<value> [<key>=<value> ...]
#   tail    [<session_dir>]                 # tick.csv 마지막 N 줄 (default 30)
#   events  [<session_dir>]                 # events.jsonl tail -F (실시간)
#   latest                                  # 가장 최신 session_dir 출력
#   show    [<session_dir>]                 # params_snapshot.yaml 보기
#
# 예시
# ----
#   ./dbg_set.sh set /sampling_planner/overtake prediction_weight=8000 side_weight=200
#   ./dbg_set.sh events
#   ./dbg_set.sh tail
#
# DEBUG_LOG_DIR (env) 으로 base_dir 오버라이드 가능. 기본은 패키지 안 debug_log/.

set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DBG_BASE="${DEBUG_LOG_DIR:-$PKG_DIR/debug_log}"

latest_session() {
    ls -dt "$DBG_BASE"/*_session 2>/dev/null | head -n1
}

cmd="${1:-help}"
shift || true

case "$cmd" in
    set)
        node="${1:?usage: set <node> key=val [...]}"
        shift
        if [ $# -eq 0 ]; then
            echo "no key=value pairs given" >&2
            exit 2
        fi
        kv_csv=""
        for kv in "$@"; do
            k="${kv%%=*}"
            v="${kv#*=}"
            if [ -z "$kv_csv" ]; then
                kv_csv="$k:$v"
            else
                kv_csv="$kv_csv,$k:$v"
            fi
        done
        # dynparam set takes "{k1: v1, k2: v2}" YAML mapping.
        ymap="{ "
        first=1
        for kv in "$@"; do
            k="${kv%%=*}"
            v="${kv#*=}"
            if [ $first -eq 0 ]; then
                ymap="$ymap, "
            fi
            ymap="$ymap$k: $v"
            first=0
        done
        ymap="$ymap }"
        echo "[dbg_set] rosrun dynamic_reconfigure dynparam set $node '$ymap'"
        rosrun dynamic_reconfigure dynparam set "$node" "$ymap"
        ;;

    tail)
        sess="${1:-$(latest_session)}"
        if [ -z "$sess" ] || [ ! -d "$sess" ]; then
            echo "no session dir found under $DBG_BASE" >&2
            exit 1
        fi
        n="${TAIL_N:-30}"
        echo "[dbg_set] tail -n $n $sess/tick.csv"
        tail -n "$n" "$sess/tick.csv"
        ;;

    events)
        sess="${1:-$(latest_session)}"
        if [ -z "$sess" ] || [ ! -d "$sess" ]; then
            echo "no session dir found under $DBG_BASE" >&2
            exit 1
        fi
        echo "[dbg_set] tail -F $sess/events.jsonl  (Ctrl-C to stop)"
        tail -F "$sess/events.jsonl"
        ;;

    latest)
        latest_session
        ;;

    show)
        sess="${1:-$(latest_session)}"
        if [ -z "$sess" ] || [ ! -d "$sess" ]; then
            echo "no session dir found under $DBG_BASE" >&2
            exit 1
        fi
        echo "==== meta.yaml ===="
        cat "$sess/meta.yaml"
        echo
        echo "==== params_snapshot.yaml ===="
        cat "$sess/params_snapshot.yaml"
        ;;

    help|*)
        sed -n '2,20p' "$0"
        ;;
esac
