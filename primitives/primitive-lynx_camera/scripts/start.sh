#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_camera runtime — RTSP RGB stream and snapshot.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

set +u
source "${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
set -u

export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$(rbnx path robonix-api):${PYTHONPATH:-}"

exec "${LYNX_CAMERA_PYTHON:-${LYNX_PYTHON:-python3}}" -m lynx_camera.main
