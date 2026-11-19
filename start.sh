#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.nx.yaml}"
LOG_DIR="$DEPLOY_DIR/logs"
mkdir -p "$LOG_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"

if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

# Fast DDS transport profile: the front Airy PointCloud2 is ~1.08 MB per frame
# (41.7k × 26 B at 10 Hz). The default 512 KB SHM segment cannot hold it, so
# every cloud fragmented over UDP and each subscriber dropped ~25% of frames
# (measured 7.7 Hz instead of 10 Hz). config/fastdds_shm.xml raises the SHM
# segment to 16 MB and keeps UDPv4 for the robot's native DDS domain. All
# processes started below inherit this through rbnx/soma. .env can override.
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "$DEPLOY_DIR/config/fastdds_shm.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$DEPLOY_DIR/config/fastdds_shm.xml"
fi

# Pilot 的部署级行为规则：抓取默认趴着、不要站起来（见 pilot_soul.md）。
# rbnx boot 会把它传给 robonix-pilot；pilot 在每个任务回合开始时读取。
# .env 里可以覆盖，没设时回落到 ~/.robonix/SOUL.md。
export ROBONIX_PILOT_SOUL="${ROBONIX_PILOT_SOUL:-$DEPLOY_DIR/pilot_soul.md}"

# The S10 publishes its native ROS 2 topics (drdds + nav_msgs) over the default
# RMW (Fast DDS). Unlike the Lite3 deploy (which pins rmw_zenoh_cpp and starts
# rmw_zenohd), we must NOT set RMW_IMPLEMENTATION here: a zenoh RMW would make
# the S10's topics invisible to this host. ROS_DOMAIN_ID is set in the manifest.
# The commented-out RMW lines in .env are intentionally left disabled.

[[ -f "/opt/ros/humble/setup.bash" ]] || {
  echo "missing /opt/ros/humble/setup.bash; install ROS 2 Humble" >&2
  exit 1
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

set -u

command -v rbnx >/dev/null 2>&1 || {
  echo "rbnx not found; install Robonix and check PATH" >&2
  exit 1
}

# Return this deployment to a known clean state before boot (also reaps
# packages orphaned by an earlier partial bring-up).
"$DEPLOY_DIR/stop.sh"

# A failed boot can leave ROS children hung (alive, publishing nothing) and
# orphaned /dev/shm/fastrtps_* segments behind; stop.sh only matches by cwd and
# misses them. Without this, the next boot hangs 30 s per provider and reports
# "no Image / no JointState within 30.0s" — see scripts/preboot_clean.sh.
if [[ -x "$DEPLOY_DIR/scripts/preboot_clean.sh" ]]; then
  "$DEPLOY_DIR/scripts/preboot_clean.sh" || true
fi

stack_started=0

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  # `rbnx boot` handles Ctrl-C itself: it tears the children down and removes
  # state.json. Calling `rbnx shutdown` again then just prints
  # "no boot state ... has `rbnx boot` been run from this manifest?".
  # Only run the explicit shutdown when a boot actually left state behind
  # (e.g. the boot process crashed).
  if [[ "$stack_started" == "1" && -f "$DEPLOY_DIR/rbnx-boot/state.json" ]]; then
    rbnx shutdown -f "$MANIFEST" || true
  fi

  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

stack_started=1

# `rbnx boot` 会在构建标记（<pkg>/rbnx-build/.rbnx-built）比 codegen 输入旧时，
# inline 重跑一遍 codegen（日志 `⚠ <pkg>: not built — building inline`）。
# 用 ROBONIX_SKIP_INLINE_BUILD=1 可以跳过这一步（把标记刷成当前时间）。
if [[ "${ROBONIX_SKIP_INLINE_BUILD:-0}" == "1" && -x "$DEPLOY_DIR/scripts/build_status.sh" ]]; then
  echo "[boot] ROBONIX_SKIP_INLINE_BUILD=1：跳过 inline codegen（已刷新构建标记）"
  bash "$DEPLOY_DIR/scripts/build_status.sh" --touch | sed -n '2,99p'
fi

rbnx boot --no-update-check -f "$MANIFEST" "$@"
