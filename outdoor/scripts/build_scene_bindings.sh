#!/usr/bin/env bash
# One generated contracts module must contain both Scene's and consumed RPCs.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGET="${1:?Scene package directory required}"
PY="${2:-$ROOT/.venv/bin/python}"
NAV="$ROOT/packages/service-navigation-outdoor-rbnx/capabilities"
mkdir -p "$ROOT/rbnx-build"
STAGE="$(mktemp -d "$ROOT/rbnx-build/scene-codegen.XXXXXX")"
trap 'rm -rf "$STAGE"' EXIT
mkdir -p "$STAGE/capabilities/service/navigation_outdoor" "$STAGE/capabilities/lib"
cp "$TARGET/package_manifest.yaml" "$STAGE/package_manifest.yaml"
if [[ -d "$TARGET/capabilities" ]]; then
  cp -a "$TARGET/capabilities/." "$STAGE/capabilities/"
fi
cp "$NAV/service/navigation_outdoor/"*.toml "$STAGE/capabilities/service/navigation_outdoor/"
cp -a "$NAV/lib/outdoor_execution" "$STAGE/capabilities/lib/"
mkdir -p "$STAGE/capabilities/service/localization"
cp "$ROOT/packages/service-localization-outdoor-rbnx/capabilities/driver.v1.toml" "$STAGE/capabilities/service/localization/driver.v1.toml"
rbnx codegen -p "$STAGE" --mcp --python "$PY" --out-dir "$TARGET/rbnx-build/codegen"
