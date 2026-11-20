#!/usr/bin/env bash
# 起/确认 YOLO 检测旁路服务（scripts/yolo_server.py）。
#
# 为什么要旁路：本机系统 python 的 numpy 是 2.x，torch 是 numpy1.x 编译的，
# 直接 import torch 在系统环境里跑推理会报 "Numpy is not available"，所以在
# ~/yolo_env（numpy 1.26）里单起一个进程，grasp_geometry 通过 HTTP 调它。
#
# 幂等：已经在跑（/health 正常）就直接退出；权重不存在也不报错（grasp_geometry
# 会自动退回 VLM）。
set -uo pipefail

WEIGHTS="${YOLO_WEIGHTS:-$HOME/grasp_labels/runs/finetune/weights/best.pt}"
PYTHON="${YOLO_PYTHON:-$HOME/yolo_env/bin/python}"
SCRIPT="${YOLO_SERVER_SCRIPT:-$(cd "$(dirname "$0")" && pwd)/yolo_server.py}"
PORT="${YOLO_PORT:-8899}"
LOG="${YOLO_LOG:-/tmp/yolo_server_${PORT}.log}"

if curl -s --max-time 2 "http://127.0.0.1:${PORT}/health" >/dev/null 2>&1; then
    echo "[yolo] 旁路服务已在 ${PORT}"
    exit 0
fi

if [[ ! -x "$PYTHON" ]]; then
    echo "[yolo] 跳过：找不到解释器 $PYTHON（重建：python3 -m venv --system-site-packages ~/yolo_env && ~/yolo_env/bin/pip install 'numpy<2'）"
    exit 0
fi
if [[ ! -f "$WEIGHTS" ]]; then
    echo "[yolo] 跳过：找不到权重 $WEIGHTS（grasp_geometry 会自动退回 VLM）"
    exit 0
fi

setsid nohup "$PYTHON" "$SCRIPT" --weights "$WEIGHTS" --port "$PORT" --no-half \
    >"$LOG" 2>&1 </dev/null &
echo "[yolo] 启动中：$PYTHON $SCRIPT --weights $WEIGHTS --port $PORT（日志 $LOG）"

for _ in $(seq 1 30); do
    if curl -s --max-time 2 "http://127.0.0.1:${PORT}/health" >/dev/null 2>&1; then
        echo "[yolo] 就绪 http://127.0.0.1:${PORT}"
        exit 0
    fi
    sleep 1
done

echo "[yolo] 30 s 内没起来，看日志：$LOG"
exit 1
