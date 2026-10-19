#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_camera runtime — declares /DEPTH_IMAGE as camera/depth + depth_snapshot.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

cleanup() {
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"

exec python3 -m lynx_camera.main
