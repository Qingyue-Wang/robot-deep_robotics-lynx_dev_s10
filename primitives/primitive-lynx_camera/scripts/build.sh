#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_camera build — codegen (camera_mcp for the snapshot tool).
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[lynx_camera/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[lynx_camera/build] done."
