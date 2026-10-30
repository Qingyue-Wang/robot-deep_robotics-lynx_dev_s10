#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
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
rbnx boot --no-update-check -f "$MANIFEST" "$@"
