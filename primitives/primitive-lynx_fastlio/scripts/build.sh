#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_fastlio build — codegen only (nav_msgs/Odometry is a standard type).
# FAST-LIO itself is a colcon workspace outside the package; see
# scripts/setup_fastlio.sh and FASTLIO_WS in scripts/start.sh.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

if ! ros2 pkg prefix fork_fast_lio >/dev/null 2>&1; then
  echo "[lynx_fastlio/build] WARN: fork_fast_lio is not on the ROS 2 package path." >&2
  echo "[lynx_fastlio/build]       Run scripts/setup_fastlio.sh (or set FASTLIO_WS) before boot." >&2
fi

echo "[lynx_fastlio/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[lynx_fastlio/build] done."
