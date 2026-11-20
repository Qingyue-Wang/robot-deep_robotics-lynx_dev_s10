#!/usr/bin/env bash
# YOLO 检测网页预览的启动/停止/状态助手。
#   bash scripts/yolo_web.sh start [port]
#   bash scripts/yolo_web.sh stop
#   bash scripts/yolo_web.sh status
set -o pipefail
cd "$(dirname "$0")/.."

PORT="${2:-8890}"
LOG="/tmp/yolo_web_${PORT}.log"
PIDFILE="/tmp/yolo_web_${PORT}.pid"
PATTERN="yolo_web_view.py --port ${PORT}"

_pid() {
    if [[ -f "$PIDFILE" ]]; then
        local p; p=$(cat "$PIDFILE")
        if [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null; then echo "$p"; return 0; fi
    fi
    # 兜底：按端口找监听者
    ss -ltnpH "sport = :${PORT}" 2>/dev/null | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | head -1
}

case "${1:-status}" in
start)
    p=$(_pid)
    if [[ -n "$p" ]]; then echo "[web] 已在运行 pid=$p http://0.0.0.0:${PORT}/"; exit 0; fi
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
    setsid nohup /usr/bin/python3 scripts/yolo_web_view.py --port "$PORT" --max-fps "${FPS:-4}" \
        >"$LOG" 2>&1 </dev/null &
    echo "$!" >"$PIDFILE"
    for _ in $(seq 1 15); do
        sleep 1
        if curl -s --max-time 2 "http://127.0.0.1:${PORT}/stats" >/dev/null 2>&1; then
            echo "[web] 就绪 http://0.0.0.0:${PORT}/ （日志 $LOG）"
            curl -s --max-time 2 "http://127.0.0.1:${PORT}/stats" | head -c 300; echo
            exit 0
        fi
    done
    echo "[web] 没起来，看日志：$LOG"; tail -5 "$LOG"; exit 1
    ;;
stop)
    p=$(_pid)
    if [[ -z "$p" ]]; then echo "[web] 没在跑"; rm -f "$PIDFILE"; exit 0; fi
    kill "$p" 2>/dev/null
    for _ in $(seq 1 10); do sleep 0.5; kill -0 "$p" 2>/dev/null || break; done
    kill -9 "$p" 2>/dev/null
    rm -f "$PIDFILE"
    echo "[web] 已停止 pid=$p"
    ;;
status)
    p=$(_pid)
    if [[ -z "$p" ]]; then echo "[web] 没在跑"; exit 1; fi
    echo "[web] pid=$p http://0.0.0.0:${PORT}/"
    curl -s --max-time 3 "http://127.0.0.1:${PORT}/stats" | head -c 600; echo
    ;;
*)
    echo "用法: $0 {start|stop|status} [port]"; exit 2 ;;
esac
