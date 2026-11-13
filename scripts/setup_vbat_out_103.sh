#!/usr/bin/env bash
# 在狗本体板（10.21.33.103，user@host）上安装并启用「对外放电开机自启」。
#
# 背景：/sys/class/power/vbat_out_power/power_on 读会返回 EIO，状态要读同目录的
# `status`（off / on）。这块板子每次重启后该开关会回到 off，所以用 systemd oneshot
# 在开机时写一次 1。
#
# 用法（在 NX 上执行，会交互式提示 103 的 sudo 密码）：
#   bash scripts/setup_vbat_out_103.sh
set -euo pipefail

HOST="${HOST:-10.21.33.103}"
USER_NAME="${USER_NAME:-user}"
UNIT_SRC="$(cd "$(dirname "$0")" && pwd)/vbat_out_power.service"

[ -f "$UNIT_SRC" ] || { echo "找不到 $UNIT_SRC" >&2; exit 1; }

echo "== 目标是 $USER_NAME@$HOST =="
ssh -o StrictHostKeyChecking=no "$USER_NAME@$HOST" 'cat /sys/class/power/vbat_out_power/status'

echo "== 拷贝 unit 到 /tmp =="
scp -o StrictHostKeyChecking=no "$UNIT_SRC" "$USER_NAME@$HOST:/tmp/vbat-out-power.service"

echo "== 安装（需要 sudo 密码）=="
# shellcheck disable=SC2029
ssh -t -o StrictHostKeyChecking=no "$USER_NAME@$HOST" '
  set -e
  sudo cp /tmp/vbat-out-power.service /etc/systemd/system/vbat-out-power.service
  sudo systemctl daemon-reload
  sudo systemctl enable --now vbat-out-power.service
  echo "enabled=$(systemctl is-enabled vbat-out-power.service) active=$(systemctl is-active vbat-out-power.service)"
  echo "vbat status=$(cat /sys/class/power/vbat_out_power/status)"
'
