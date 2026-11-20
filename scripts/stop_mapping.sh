#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Stop the mapping-only profile started by scripts/start_mapping.sh.
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")/.." && pwd)"

export ROBONIX_MANIFEST="$DEPLOY_DIR/robonix_manifest.mapping.yaml"

exec "$DEPLOY_DIR/stop.sh" "$@"
