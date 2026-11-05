#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
set +u; source /opt/ros/humble/setup.bash; source /home/nvidia/drdds_build_ws/install/setup.bash; source /home/nvidia/rslidar_ws/install/setup.bash; set -u
export PYTHONPATH="$ROOT:$ROOT/rbnx-build/codegen/proto_gen:$(rbnx path robonix-api):${PYTHONPATH:-}"
exec "$ROOT/.venv/bin/python" -m robot_outdoor.hardware
