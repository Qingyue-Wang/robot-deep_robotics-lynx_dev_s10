#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/rbnx-build"
bash "$ROOT/scripts/build_scene_bindings.sh" "$ROOT"
mkdir -p "$ROOT/rbnx-build"
touch "$ROOT/rbnx-build/.rbnx-built"
