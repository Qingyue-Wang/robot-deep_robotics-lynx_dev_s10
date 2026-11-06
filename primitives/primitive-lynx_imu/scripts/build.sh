#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_imu build — codegen only (sensor_msgs/Imu is a standard ROS 2 type).
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[lynx_imu/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[lynx_imu/build] done."
