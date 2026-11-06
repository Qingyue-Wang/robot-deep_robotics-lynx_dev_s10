#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_health runtime — aggregates drdds battery/CPU telemetry into health/*.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# drdds is the S10's custom ROS 2 message package, prebuilt on the Jetson.
DRDDS_INSTALL="${LYNX_DRDDS_INSTALL:-/home/nvidia/drdds_build_ws/install}"

set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$DRDDS_INSTALL/setup.bash" ]; then
  source "$DRDDS_INSTALL/setup.bash"
fi
set -u

cleanup() {
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# proto_gen carries health_pb2 for the gRPC handlers (also injected by boot).
export PYTHONPATH="$(rbnx path robonix-api):$PKG:$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"

exec python3 -m lynx_health.main
