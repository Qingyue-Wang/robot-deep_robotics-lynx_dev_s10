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

  # Stop the odom guard before the stack (it is a plain ROS node, not part of
  # `rbnx boot`, so it does not participate in rbnx's own teardown).
  if [[ -n "${ODOM_GUARD_PID:-}" ]]; then
    kill "$ODOM_GUARD_PID" 2>/dev/null || true
    wait "$ODOM_GUARD_PID" 2>/dev/null || true
  fi
  rm -f "${ODOM_GUARD_PIDFILE:-$LOG_DIR/odom_guard.pid}"

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

# ── odom safety gate ─────────────────────────────────────────────────────
# The nav2 service publishes its final velocity on /cmd_vel_service (see
# `velocity_output_topic` in the manifest); this node forwards it to /cmd_vel
# only while /odom is healthy, and stops the base + cancels the navigation
# goal when odom goes stale or jumps. Runs on the stack's ROS domain (read
# from the manifest). Failure is logged in logs/odom_guard.log; navigation
# then gets no velocity at all, which is fail-safe.
ODOM_GUARD_PIDFILE="$LOG_DIR/odom_guard.pid"
if [[ -f "$DEPLOY_DIR/config/odom_guard.yaml" ]]; then
  # A guard left over from a crashed previous run would keep zeroing /cmd_vel.
  if [[ -f "$ODOM_GUARD_PIDFILE" ]]; then
    stale_pid="$(cat "$ODOM_GUARD_PIDFILE" 2>/dev/null || true)"
    if [[ -n "$stale_pid" ]] && kill -0 "$stale_pid" 2>/dev/null; then
      echo "[start] killing stale odom guard pid=$stale_pid"
      kill "$stale_pid" 2>/dev/null || true
    fi
    rm -f "$ODOM_GUARD_PIDFILE"
  fi

  # Only deployments that run nav2 need the gate: without nav2 nothing publishes
  # /cmd_vel_service, and in a chassis-only deployment the always-on zero pump
  # would fight manual /cmd_vel driving.
  if python3 - "$MANIFEST" <<'PY'
import sys
import yaml
try:
    data = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
except Exception:
    raise SystemExit(1)
services = data.get("service") or []
raise SystemExit(0 if any(s.get("name") == "nav2" for s in services) else 1)
PY
  then
    python3 "$DEPLOY_DIR/scripts/odom_guard.py" \
      --manifest "$MANIFEST" \
      --config "$DEPLOY_DIR/config/odom_guard.yaml" \
      >>"$LOG_DIR/odom_guard.log" 2>&1 &
    ODOM_GUARD_PID=$!
    echo "$ODOM_GUARD_PID" > "$ODOM_GUARD_PIDFILE"
    echo "[start] odom guard pid=$ODOM_GUARD_PID (log: $LOG_DIR/odom_guard.log)"
  else
    echo "[start] manifest has no nav2 service; odom guard not started"
  fi
else
  echo "[start] WARNING: config/odom_guard.yaml missing; odom guard disabled" >&2
fi

stack_started=1
rbnx boot --no-update-check -f "$MANIFEST" "$@"
