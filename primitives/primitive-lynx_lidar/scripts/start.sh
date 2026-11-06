#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_lidar runtime — starts the RoboSense Airy driver (rslidar_sdk), then
# declares the configured lidar stream (default /airy_201/points, the front lidar)
# as robonix/primitive/lidar/lidar3d.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
RSLIDAR_WS="${RSLIDAR_WS:-$HOME/rslidar_ws}"

# ROS setup scripts may reference unset variables; disable nounset temporarily.
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$RSLIDAR_WS/install/setup.bash" ]; then
  source "$RSLIDAR_WS/install/setup.bash"
fi
set -u

RSLIDAR_NODE="$RSLIDAR_WS/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node"
RSLIDAR_PID=""
TF_PID=""

# Kill a driver left behind by an unclean previous run. A stale instance owns
# the Airy's UDP ports (6691/7781/6681); the replacement then starts but never
# receives packets, so /airy_201/points stays silent and every consumer
# (FAST-LIO included) times out even though this primitive may briefly report
# ACTIVE from the dying instance's last clouds.
kill_stale() {
  local pattern="$1"
  pkill -TERM -f -- "$pattern" 2>/dev/null || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    pgrep -f -- "$pattern" >/dev/null 2>&1 || return 0
    sleep 0.2
  done
  pkill -KILL -f -- "$pattern" 2>/dev/null || true
}
kill_stale "$RSLIDAR_NODE"

cleanup() {
  if [ -n "$RSLIDAR_PID" ]; then
    kill "$RSLIDAR_PID" 2>/dev/null || true
    wait "$RSLIDAR_PID" 2>/dev/null || true
  fi
  if [ -n "$TF_PID" ]; then
    kill "$TF_PID" 2>/dev/null || true
    wait "$TF_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Start the Airy driver. The node resolves config.yaml via its compile-time
# PROJECT_PATH (~/rslidar_ws/src/rslidar_sdk/config/config.yaml), so it needs no
# cwd or config_path argument — just the ROS + workspace environment above.
#
# Publish the front lidar extrinsic. rslidar stamps the cloud in frame `airy_201`,
# which the S10's native TF tree does not carry; without it rtabmap's ICP
# odometry drops every scan ("TF of received scan cloud ... is not set").
# Translation/rotation is the S10 URDF front lidar_joint origin: the Airy is
# mounted VERTICALLY on the head with its top facing forward, so its native
# frame (Z = up/out of the emitting face) is rotated +90 deg about Y into
# base_link (lidar "up" -> robot "forward"). This makes the 360-deg mechanical
# scan a VERTICAL disc, which projects to a thin left-right strip in the ground
# plane — see rtabmap_params.yaml Grid/NormalsSegmentation note.
if command -v ros2 >/dev/null 2>&1; then
  ros2 run tf2_ros static_transform_publisher \
    --x 0.22341 --y 0.0 --z -0.0001 \
    --roll 3.14159 --pitch -1.5708 --yaw 0 \
    --frame-id base_link --child-frame-id airy_201 \
    --ros-args -r __node:=lynx_lidar_tf_front &
  TF_PID=$!
fi

if [ -x "$RSLIDAR_NODE" ]; then
  "$RSLIDAR_NODE" &
  RSLIDAR_PID=$!
else
  echo "[lynx_lidar] rslidar_sdk_node not found at $RSLIDAR_NODE — assuming the driver is started externally" >&2
fi

export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"

python3 -m lynx_lidar.main
