#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
set +u
source "${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
source "${DRDDS_SETUP:-$HOME/drdds_build_ws/install/setup.bash}"
set -u
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$(rbnx path robonix-api):${PYTHONPATH:-}"
exec "${LYNX_PYTHON:-python3}" -m lynx_chassis.main
