#!/usr/bin/env bash
# Raise the kernel UDP receive buffers for the RoboSense Airy driver.
#
# rslidar_sdk asks for >=4 MB per receive socket (MSOP/DIFOP/IMU), but the
# kernel caps SO_RCVBUF at net.core.rmem_max. The default (~208 KB after the
# kernel's doubling) leaves ~0.25 s of MSOP data buffered, so any scheduling
# or DDS-publishing hiccup in the driver overflows the socket queue and drops
# packets (Udp RcvbufErrors) — which shows up downstream as multi-second
# /Odometry gaps and FAST-LIO "diverged" suppression.
#
# Run once with sudo; persists across reboots via /etc/sysctl.d. The lidar
# driver must be (re)started afterwards to pick up the new buffer.
#
#   sudo ./scripts/set_udp_buffer.sh
#
# Verify in the driver log:
#   After setting: receive buffer size: 8388608 bytes
# and during a run:
#   grep RcvbufErrors /proc/net/snmp   # should stop increasing
set -euo pipefail

CONF=/etc/sysctl.d/99-rslidar.conf
RMEM_MAX="${RSLIDAR_RMEM_MAX:-16777216}"
RMEM_DEFAULT="${RSLIDAR_RMEM_DEFAULT:-4194304}"

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "run with sudo: sudo $0" >&2
  exit 1
fi

cat > "$CONF" <<EOF
# Larger UDP receive buffers for the RoboSense Airy driver (rslidar_sdk).
# The driver requests >= 4 MB per socket; the stock rmem_max (~212 KB)
# clamps that and MSOP bursts overflow the socket queue (Udp RcvbufErrors).
net.core.rmem_max = $RMEM_MAX
net.core.rmem_default = $RMEM_DEFAULT
EOF

sysctl -p "$CONF"
echo
echo "effective values:"
sysctl net.core.rmem_max net.core.rmem_default
echo
echo "now restart the stack (./start.sh); the driver will log its new SO_RCVBUF."
