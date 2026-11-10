#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
set +u
source "${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
source "${RSLIDAR_WS:-$HOME/rslidar_ws}/install/setup.bash"
set -u
export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"
exec "${LYNX_PYTHON:-python3}" -m lynx_lidar.main
