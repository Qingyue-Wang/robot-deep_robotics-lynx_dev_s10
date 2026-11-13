#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_lidar install — prepare the RoboSense Airy driver (rslidar_sdk + rslidar_msg)
# in a colcon workspace, apply this deployment's config and patches, and build it.
#
# Called automatically by scripts/start.sh when the driver binary is missing, so a
# fresh NX (or a wiped workspace) does not need a manual rslidar install. It is
# idempotent: when the binary already exists it does nothing unless FORCE=1.
#
# Env overrides:
#   RSLIDAR_WS            colcon workspace root        (default: ~/rslidar_ws)
#   RSLIDAR_SDK_REF       rslidar_sdk commit          (default: pinned below)
#   RSLIDAR_MSG_REF       rslidar_msg commit          (default: pinned below)
#   RSLIDAR_GIT_MIRROR    mirror URL prefix for GitHub (default: https://gh-proxy.org/;
#                         set to "" to clone github.com directly)
#   LIDAR_HOST_IP         host_address in config.yaml  (default: 10.21.33.100)
#   FORCE=1               re-apply config/patches and rebuild even if installed
#   RSLIDAR_KEEP_CONFIG=1 keep the workspace's existing config.yaml untouched
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
WS="${RSLIDAR_WS:-$HOME/rslidar_ws}"
SDK_URL="https://github.com/RoboSense-LiDAR/rslidar_sdk.git"
MSG_URL="https://github.com/RoboSense-LiDAR/rslidar_msg.git"
SDK_REF="${RSLIDAR_SDK_REF:-8b4b4b7ff910799260347821084c59e1c73d50d5}"
MSG_REF="${RSLIDAR_MSG_REF:-fe8a95cb242bd294cc3d5e3422f2093fb49a56ee}"
MIRROR="${RSLIDAR_GIT_MIRROR-https://gh-proxy.org/}"
HOST_IP="${LIDAR_HOST_IP:-10.21.33.100}"
SDK_DIR="$WS/src/rslidar_sdk"
MSG_DIR="$WS/src/rslidar_msg"
NODE="$WS/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node"

log() { echo "[lynx_lidar/install] $*"; }
fail() { echo "[lynx_lidar/install] ERROR: $*" >&2; exit 1; }

if [ -x "$NODE" ] && [ "${FORCE:-0}" != "1" ]; then
  log "already installed: $NODE (set FORCE=1 to rebuild)"
  exit 0
fi

[ -f /opt/ros/humble/setup.bash ] || fail "ROS 2 Humble not found at /opt/ros/humble"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

command -v colcon >/dev/null 2>&1 \
  || fail "colcon not found; install python3-colcon-common-extensions or source the ROS setup"
if ! pkg-config --exists libpcap 2>/dev/null; then
  log "WARNING: libpcap headers not found; build may fail (sudo apt install libpcap-dev)"
fi

mkdir -p "$WS/src"

# New GitHub mirrors are tried in order; empty entry means github.com directly.
# GIT_CONFIG_* per command rewrites both the clone and the relative submodule
# URL (../rs_driver.git) through the chosen mirror.
MIRRORS=()
[ -n "$MIRROR" ] && MIRRORS+=("$MIRROR")
MIRRORS+=("https://ghfast.top/" "")

mirror_env() {
  # usage: mirror_env <mirror> git ...
  if [ -n "$1" ]; then
    env GIT_CONFIG_COUNT=1 \
        GIT_CONFIG_KEY_0="url.$1https://github.com/.insteadOf" \
        GIT_CONFIG_VALUE_0="https://github.com/" "${@:2}"
  else
    "${@:2}"
  fi
}

clone_pinned() {
  local url="$1" dir="$2" ref="$3" name="$4"
  if [ -d "$dir/.git" ]; then
    log "reusing existing $name checkout: $dir"
    return 0
  fi
  [ -e "$dir" ] && fail "$dir exists but is not a git checkout; move it away first"

  local mirror attempt
  for mirror in "${MIRRORS[@]}"; do
    for attempt in 1 2 3; do
      log "cloning $name (${mirror:-github.com}, attempt $attempt)"
      if mirror_env "$mirror" git clone --quiet "$url" "$dir"; then
        if ! git -C "$dir" checkout --quiet "$ref" 2>/dev/null; then
          git -C "$dir" fetch --quiet origin "$ref" 2>/dev/null || true
          git -C "$dir" checkout --quiet "$ref" \
            || { log "commit $ref not in $name"; rm -rf "$dir"; continue; }
        fi
        log "$name at $(git -C "$dir" rev-parse --short HEAD)"
        return 0
      fi
      rm -rf "$dir"   # drop a partial clone before retrying
      sleep 2
    done
  done
  fail "git clone failed for $name (tried: ${MIRRORS[*]:-github.com})"
}

clone_pinned "$SDK_URL" "$SDK_DIR" "$SDK_REF" "rslidar_sdk"
clone_pinned "$MSG_URL" "$MSG_DIR" "$MSG_REF" "rslidar_msg"

# rs_driver is a submodule of rslidar_sdk; fetch the commit the superproject pins.
if [ ! -d "$SDK_DIR/src/rs_driver/.git" ]; then
  log "fetching rs_driver submodule"
  ok=0
  for mirror in "${MIRRORS[@]}"; do
    for _ in 1 2 3; do
      if mirror_env "$mirror" git -C "$SDK_DIR" submodule update --init --recursive --quiet; then
        ok=1
        break 2
      fi
      sleep 2
    done
  done
  [ "$ok" = 1 ] || fail "git submodule update failed"
fi

# ── patches ─────────────────────────────────────────────────────────────────
# 1) XYZIRT: per-point timestamps needed by FAST-LIO.
python3 - "$SDK_DIR/CMakeLists.txt" <<'PY'
import pathlib, sys
p = pathlib.Path(sys.argv[1])
s = p.read_text()
if "set(POINT_TYPE XYZIRT)" in s:
    print("[lynx_lidar/install] CMakeLists already XYZIRT")
elif "set(POINT_TYPE XYZI)" in s:
    p.write_text(s.replace("set(POINT_TYPE XYZI)", "set(POINT_TYPE XYZIRT)", 1))
    print("[lynx_lidar/install] CMakeLists: POINT_TYPE XYZI -> XYZIRT")
else:
    sys.exit("[lynx_lidar/install] CMakeLists: unexpected POINT_TYPE line")
PY

# 2) BEST_EFFORT point-cloud publisher: a RELIABLE 1 MB cloud blocks the driver
#    thread under load (unbounded queue -> GBs of RSS -> 100 s stale stamps).
python3 - "$SDK_DIR/src/source/source_pointcloud_ros.hpp" <<'PY'
import pathlib, sys
p = pathlib.Path(sys.argv[1])
s = p.read_text()
if "cloud_qos.best_effort()" in s:
    print("[lynx_lidar/install] point-cloud publisher already BEST_EFFORT")
    raise SystemExit(0)
old = "  pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_send_topic, ros_queue_length);"
new = """  {
    // Point clouds are sensor data: publish BEST_EFFORT so a slow consumer can
    // drop frames instead of blocking this thread (a RELIABLE ~1 MB cloud under
    // load stalls the driver, its queue grows into GBs and stamps go stale).
    rclcpp::QoS cloud_qos{rclcpp::KeepLast(ros_queue_length)};
    cloud_qos.best_effort();
    pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
        ros_send_topic, cloud_qos);
  }"""
if old not in s:
    sys.exit("[lynx_lidar/install] source_pointcloud_ros.hpp: unexpected publisher line")
p.write_text(s.replace(old, new, 1))
print("[lynx_lidar/install] point-cloud publisher -> BEST_EFFORT")
PY

# ── deployment config ───────────────────────────────────────────────────────
if [ "${RSLIDAR_KEEP_CONFIG:-0}" != "1" ]; then
  python3 - "$PKG/config/rslidar_config.yaml" "$SDK_DIR/config/config.yaml" "$HOST_IP" <<'PY'
import pathlib, sys
src, dst, host_ip = sys.argv[1:4]
text = pathlib.Path(src).read_text().replace("__LIDAR_HOST_IP__", host_ip)
pathlib.Path(dst).write_text(text)
print(f"[lynx_lidar/install] config.yaml written (host_address={host_ip})")
PY
else
  log "keeping existing config.yaml (RSLIDAR_KEEP_CONFIG=1)"
fi

# ── build ───────────────────────────────────────────────────────────────────
log "building rslidar_msg + rslidar_sdk (ENABLE_IMU_DATA_PARSE=ON)"
BUILD_LOG="$WS/install.log"
if ! ( cd "$WS" && colcon build --packages-select rslidar_msg rslidar_sdk \
         --cmake-args -DENABLE_IMU_DATA_PARSE=ON ) >"$BUILD_LOG" 2>&1; then
  echo "[lynx_lidar/install] build failed; tail of $BUILD_LOG:" >&2
  tail -40 "$BUILD_LOG" >&2
  exit 1
fi

[ -x "$NODE" ] || fail "build finished but $NODE is missing"
log "installed: $NODE"
log "log: $BUILD_LOG"
