#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_imu runtime — publishes the base_link -> imu_link extrinsic, then runs
# the primitive which re-stamps /IMU and declares it as robonix/primitive/imu/imu.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# ROS setup scripts may reference unset variables; disable nounset temporarily.
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

TF_PID=""

cleanup() {
  if [ -n "$TF_PID" ]; then
    kill "$TF_PID" 2>/dev/null || true
    wait "$TF_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Publish the IMU extrinsic. The IMU is now the front Airy's built-in IMU, so
# it lives at the lidar frame (airy_201): same translation/rotation as the
# lynx_lidar front extrinsic (vertical mount, roll=pi, pitch=-pi/2). main.py
# re-stamps the stream's frame_id to imu_link; this TF resolves it to base_link
# for rtabmap's ICP odometry.
if command -v ros2 >/dev/null 2>&1; then
  ros2 run tf2_ros static_transform_publisher \
    --x 0.22341 --y 0.0 --z -0.0001 \
    --roll 3.14159 --pitch -1.5708 --yaw 0 \
    --frame-id base_link --child-frame-id imu_link \
    --ros-args -r __node:=lynx_imu_tf &
  TF_PID=$!
fi

export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"

python3 -m lynx_imu.main
