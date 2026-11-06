#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_health build — codegen (health_pb2 for the state/stream gRPC handlers).
# The drdds message package is prebuilt on the Jetson and sourced by start.sh.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[lynx_health/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[lynx_health/build] done."
