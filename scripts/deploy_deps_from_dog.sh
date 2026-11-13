#!/usr/bin/env bash
# One-shot deploy of the S10 driver workspaces from the bench machine
# dog@10.29.135.168 onto this Orin NX:
#   - drdds message package  -> /home/nvidia/ros2_ws/drdds_humble_ws   (rebuild)
#   - rslidar_sdk + msgs     -> /home/nvidia/rslidar_ws                (rebuild)
#   - FAST-LIO source        -> /home/nvidia/fastlio_ws                (rebuild)
# Rebuilding (instead of copying install trees) makes every colcon
# setup.bash / PROJECT_PATH / RPATH absolute path point at /home/nvidia.
set -o pipefail

LOG_DIR="/home/nvidia/robot-deep_robotics-lynx_s10/logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/deploy-deps-$(date +%Y%m%d-%H%M).log"
exec > >(tee -a "$LOG") 2>&1

echo "=== deploy deps from dog: $(date) ==="
DOG="${DOG_HOST:-dog@10.29.135.168}"
ASK="/tmp/askpass_dog.sh"
printf '#!/bin/sh\necho %s\n' "${DOG_PASS:-123456}" > "$ASK"
chmod 700 "$ASK"
export SSH_ASKPASS="$ASK" SSH_ASKPASS_REQUIRE=force DISPLAY=:0
SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
          -o PreferredAuthentications=password -o PubkeyAuthentication=no \
          -o ConnectTimeout=15)
SSH_CMD="ssh ${SSH_OPTS[*]}"
rssh() { rsync -a -e "$SSH_CMD" "$@"; }

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

FAIL=0
step() { echo; echo "=== $* ==="; }

step "1/6 copy drdds source"
mkdir -p /home/nvidia/ros2_ws/drdds_humble_ws/src
rssh "$DOG:/home/dog/ros2_ws/drdds_humble_ws/src/drdds" \
     /home/nvidia/ros2_ws/drdds_humble_ws/src/ || FAIL=1

step "2/6 copy rslidar_sdk + rslidar_msg source"
mkdir -p /home/nvidia/rslidar_ws/src
rssh "$DOG:/home/dog/rslidar_ws/src/rslidar_sdk" \
     "$DOG:/home/dog/rslidar_ws/src/rslidar_msg" \
     /home/nvidia/rslidar_ws/src/ || FAIL=1

step "3/6 copy FAST-LIO source (no .git)"
mkdir -p /home/nvidia/fastlio_ws/src
rssh --exclude='.git' "$DOG:/home/dog/fastlio_ws/src/fast_lio_fork_ros2" \
     /home/nvidia/fastlio_ws/src/ || FAIL=1

step "4/6 build drdds"
(
  cd /home/nvidia/ros2_ws/drdds_humble_ws || exit 1
  colcon build --event-handlers console_direct-
) || { echo "DRDDS_BUILD_FAILED"; FAIL=1; }

step "5/6 build rslidar_msg + rslidar_sdk"
(
  cd /home/nvidia/rslidar_ws || exit 1
  colcon build --packages-select rslidar_msg rslidar_sdk --event-handlers console_direct-
) || { echo "RSLIDAR_BUILD_FAILED"; FAIL=1; }

step "6/6 build FAST-LIO"
(
  cd /home/nvidia/fastlio_ws || exit 1
  colcon build --symlink-install --packages-select fork_fast_lio --event-handlers console_direct-
) || { echo "FASTLIO_BUILD_FAILED"; FAIL=1; }

step "verify"
bash -lc '
set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/drdds_humble_ws/install/setup.bash 2>/dev/null && \
  python3 -c "from drdds.msg import MotionInfo, MotionState, NavCmd, Gait; print(\"drdds: import OK\")" || \
  echo "drdds: IMPORT FAILED"

source /home/nvidia/fastlio_ws/install/setup.bash 2>/dev/null && \
  echo "fork_fast_lio: $(ros2 pkg prefix fork_fast_lio 2>/dev/null)" || echo "fork_fast_lio: MISSING"
ls -l /home/nvidia/fastlio_ws/install/fork_fast_lio/lib/fork_fast_lio/fastlio_mapping 2>/dev/null

source /home/nvidia/rslidar_ws/install/setup.bash 2>/dev/null && \
  echo "rslidar_sdk: $(ros2 pkg prefix rslidar_sdk 2>/dev/null)" || echo "rslidar_sdk: MISSING"
ls -l /home/nvidia/rslidar_ws/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node 2>/dev/null
strings /home/nvidia/rslidar_ws/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node 2>/dev/null | grep -m1 "/home/nvidia/rslidar_ws/src/rslidar_sdk" && echo "PROJECT_PATH: relocated OK"
' 2>&1

echo
if [ "$FAIL" = "0" ]; then
  echo "=== DEPLOY OK: $(date) ==="
else
  echo "=== DEPLOY HAD FAILURES: $(date) ==="
fi
echo "log: $LOG"
