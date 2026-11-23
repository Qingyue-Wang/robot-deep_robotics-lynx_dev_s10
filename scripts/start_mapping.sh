#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Start the mapping-only profile: chassis + both Airy lidars + FAST-LIO +
# mapping (RTAB-Map). No nav2, arm, RealSense or grasp stack, and no nav
# extras (merged scan / drive-direction), so the Jetson's CPU and DDS
# bandwidth stay with mapping.
#
# Stop with the same profile:
#   ROBONIX_MANIFEST=$PWD/robonix_manifest.mapping.yaml ./stop.sh
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")/.." && pwd)"

export ROBONIX_MANIFEST="$DEPLOY_DIR/robonix_manifest.mapping.yaml"
# Default off: drive_direction is useless without nav2. lynx_scan_nav.py stays
# on (LYNX_SCAN_MERGER) because the mapping profile consumes its merged
# front+rear 3D cloud (/scanner/cloud_merged) and merged 2D scan
# (/scanner/scan_nav). Set LYNX_NAV_EXTRAS=1 to also start drive_direction.
export LYNX_NAV_EXTRAS="${LYNX_NAV_EXTRAS:-0}"
export LYNX_SCAN_MERGER="${LYNX_SCAN_MERGER:-1}"

exec "$DEPLOY_DIR/start.sh" "$@"
