#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_fastlio runtime — launches FAST-LIO (RoboSense Airy fork) with the S10
# parameter file, then runs the primitive that republishes its IMU-body
# odometry as odom -> base_link and declares robonix/primitive/chassis/odom.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
FASTLIO_WS="${FASTLIO_WS:-$HOME/fastlio_ws}"

# ROS setup scripts may reference unset variables; disable nounset temporarily.
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$FASTLIO_WS/install/setup.bash" ]; then
  source "$FASTLIO_WS/install/setup.bash"
fi
set -u

if ! ros2 pkg prefix fork_fast_lio >/dev/null 2>&1; then
  echo "[lynx_fastlio] fork_fast_lio not found (FASTLIO_WS=$FASTLIO_WS)." >&2
  echo "[lynx_fastlio] Run $PKG/scripts/setup_fastlio.sh first, or point FASTLIO_WS at the workspace." >&2
  exit 2
fi

# Kill leftovers from an unclean previous run. A stale fastlio_mapping keeps a
# RELIABLE subscription on /airy_201/imu; if that process is stuck, Fast DDS
# flow control stalls the driver's IMU writer for every reader, so the new node
# never receives IMU and this primitive times out after 60 s with no /Odometry.
# A normal shutdown removes these via the trap below, but rbnx can kill the
# process group before the trap runs.
kill_stale() {
  local pattern="$1"
  pkill -TERM -f -- "$pattern" 2>/dev/null || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    pgrep -f -- "$pattern" >/dev/null 2>&1 || return 0
    sleep 0.2
  done
  pkill -KILL -f -- "$pattern" 2>/dev/null || true
}
kill_stale "$FASTLIO_WS/install/fork_fast_lio/lib/fork_fast_lio/fastlio_mapping"
kill_stale "ros2 launch fork_fast_lio mapping.launch.py"
kill_stale "python3 -m lynx_fastlio.main"

LAUNCH_PID=""

cleanup() {
  if [ -n "$LAUNCH_PID" ]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# The launch file accepts a config directory + filename; keep the S10's
# parameters in this package so the mount/extrinsic stay next to the wrapper
# that composes them (config/s10_mount.yaml).
ros2 launch fork_fast_lio mapping.launch.py \
  config_path:="$PKG/config" \
  config_file:=robosense_airy_s10.yaml \
  rviz:=false &
LAUNCH_PID=$!

export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"

python3 -m lynx_fastlio.main
