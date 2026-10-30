#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# One-time FAST-LIO workspace setup for the Lynx S10.
#
# RuanJY/robosense_fast_lio (the reference implementation) is ROS 1 catkin and
# cannot run under ROS 2 Humble. This script builds a ROS 2 Humble FAST-LIO
# port with RoboSense Airy support:
#
#   skorbiz/fast_lio_fork_ros2_robosense @ ROS2
#
# It has no livox_ros_driver2 dependency (the message definitions are bundled)
# and supports RoboSense Airy as `lidar_type: 5`.
#
#   FASTLIO_WS=~/fastlio_ws ./scripts/setup_fastlio.sh
#
# Override FASTLIO_REPO / FASTLIO_REF to test another port; the wrapper only
# needs a package named `fork_fast_lio` that publishes /Odometry.
set -euo pipefail

PKG="$(cd "$(dirname "$0")/.." && pwd)"
FASTLIO_WS="${FASTLIO_WS:-$HOME/fastlio_ws}"
FASTLIO_REPO="${FASTLIO_REPO:-https://gh-proxy.org/https://github.com/skorbiz/fast_lio_fork_ros2_robosense.git}"
FASTLIO_REF="${FASTLIO_REF:-886053c7e52805d0ad1c008258ea6f4eab51fc60}"
SRC_DIR="$FASTLIO_WS/src/fast_lio_fork_ros2"

set +u
source /opt/ros/humble/setup.bash
set -u

for pkg in pcl_ros pcl_conversions; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "[setup_fastlio] missing $pkg — install it first:" >&2
    echo "  sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions libeigen3-dev libpcl-dev" >&2
    exit 2
  fi
done

mkdir -p "$FASTLIO_WS/src"
if [ -d "$SRC_DIR/.git" ]; then
  echo "[setup_fastlio] using existing checkout $SRC_DIR"
else
  echo "[setup_fastlio] cloning $FASTLIO_REPO"
  git clone "$FASTLIO_REPO" "$SRC_DIR"
fi

git -C "$SRC_DIR" fetch --all --tags --prune
# Drop local changes (including a previous patch run); the patches below are
# re-applied from scratch so the result is always the pinned tree + patches.
git -C "$SRC_DIR" checkout -- .
git -C "$SRC_DIR" checkout --detach "$FASTLIO_REF"
git -C "$SRC_DIR" submodule update --init --recursive

# Local patches. The pinned port buffers lidar scans without bound: scans that
# pile up while the IMU initialises are never dropped (the mapping loop runs
# at roughly the lidar rate), so /Odometry and the derived odom -> base_link
# TF stay seconds behind the live scan stream and RTAB-Map cannot look up a
# pose at the scan timestamp. The patch keeps only the newest queued scan.
for patch_file in "$PKG"/patches/*.patch; do
  [ -e "$patch_file" ] || continue
  if git -C "$SRC_DIR" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
    echo "[setup_fastlio] patch already applied: $(basename "$patch_file")"
  else
    git -C "$SRC_DIR" apply "$patch_file"
    echo "[setup_fastlio] applied $(basename "$patch_file")"
  fi
done

cd "$FASTLIO_WS"
colcon build --symlink-install --packages-select fork_fast_lio

echo "[setup_fastlio] built $FASTLIO_WS/install/setup.bash"
echo "[setup_fastlio] export FASTLIO_WS=$FASTLIO_WS (lynx_fastlio defaults to ~/fastlio_ws)"
