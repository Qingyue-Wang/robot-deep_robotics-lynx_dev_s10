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
# Default off: neither lynx_scan_nav nor drive_direction is useful without
# nav2. Set LYNX_NAV_EXTRAS=1 to keep the merged /scanner/scan_nav overlay
# in the mapping web UI (costs ~1 % CPU and the rear cloud subscription).
export LYNX_NAV_EXTRAS="${LYNX_NAV_EXTRAS:-0}"

exec "$DEPLOY_DIR/start.sh" "$@"
