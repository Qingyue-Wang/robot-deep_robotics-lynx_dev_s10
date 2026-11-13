#!/usr/bin/env bash
# can0 现场诊断（需要 root）。一次 sudo 分清三件事：
#   1. 哪些波特率上总线上有活着的节点 —— 对每个速率发一帧：
#        有节点应答（ACK）→ 状态保持 ERROR-ACTIVE，且能看到臂的反馈帧；
#        没有节点       → 没人 ACK，错误计数暴涨，很快 BUS-OFF。
#   2. 如果所有速率都 0 帧，再开 loopback 自发自收，只验证 NX 自己的控制器+收发器
#      （与臂无关）：能自收说明 NX 这侧没问题，问题在接线/臂那侧。
#   3. 收尾把接口按 1Mbps 恢复正常模式。
#
# 用法：
#   sudo bash scripts/can_diag.sh              # 默认扫 1M/500k/250k/125k
#   sudo bash scripts/can_diag.sh can0 1000000 # 只测指定速率
set -uo pipefail
CAN_IF="${1:-can0}"
RATES=("${@:2}")
[[ ${#RATES[@]} -gt 0 ]] || RATES=(1000000 500000 250000 125000)

[[ "$(id -u)" == "0" ]] || { echo "ERR: 需要 root：sudo bash $0" >&2; exit 2; }
command -v candump >/dev/null || { echo "ERR: 缺 can-utils（sudo apt install can-utils）" >&2; exit 2; }

rx()    { cat "/sys/class/net/$CAN_IF/statistics/rx_packets" 2>/dev/null || echo 0; }
state() { ip -d link show "$CAN_IF" 2>/dev/null | grep -o 'can state [A-Z-]*' | awk '{print $3}' | head -1; }
berr()  { ip -s -d link show "$CAN_IF" 2>/dev/null | grep -o 'berr-counter tx [0-9]* rx [0-9]*' | head -1; }

set_rate() { # $1=bitrate  [$2=loopback on]
    ip link set "$CAN_IF" down 2>/dev/null || true
    if [[ "${2:-}" == "loopback" ]]; then
        ip link set "$CAN_IF" type can bitrate "$1" loopback on 2>/dev/null || return 1
    else
        ip link set "$CAN_IF" type can bitrate "$1" 2>/dev/null || return 1
    fi
    ip link set "$CAN_IF" up
}

echo "== 0/3 当前状态"
ip -br link show "$CAN_IF" || { echo "ERR: 没有 $CAN_IF" >&2; exit 1; }
echo "   $(state) $(berr) rx=$(rx)"

echo
echo "== 1/3 逐速率探测：发一帧看有没有节点 ACK"
FOUND=""
for rate in "${RATES[@]}"; do
    set_rate "$rate" || { echo "   ${rate}bps: 设置失败，跳过"; continue; }
    r0=$(rx)
    sleep 0.5
    cansend "$CAN_IF" 7FF#01 2>/dev/null || true
    sleep 1
    r1=$(rx); st=$(state); b=$(berr)
    printf "   %-8s rx+%-4s state=%-13s %s\n" "${rate}bps" "$((r1 - r0))" "${st:-?}" "$b"
    # 有对端 ACK：状态保持 ERROR-ACTIVE 且 tx 错误计数没涨；顺带收到对端帧就更确定。
    if [[ "$st" == "ERROR-ACTIVE" && $((r1 - r0)) -gt 0 ]]; then
        FOUND="$rate"
        echo "   >>> ${rate}bps 上有节点在发帧，就是它"
        break
    fi
    if [[ "$st" == "ERROR-ACTIVE" && "$b" == *"berr-counter tx 0 rx 0"* ]]; then
        FOUND="$rate"
        echo "   >>> ${rate}bps 上帧被 ACK 了（总线上有节点），但没有反馈帧 —— 臂可能还没开始发"
        break
    fi
done

if [[ -n "$FOUND" ]]; then
    echo
    echo "== 结论：臂在 $CAN_IF 上，波特率 ${FOUND}bps。"
    echo "   把 manifest 里的 can_port: $CAN_IF / can_bitrate: ${FOUND} 对齐后重新 boot："
    echo "   ROBONIX_MANIFEST=\$PWD/robonix_manifest.nx.yaml ./start.sh"
    echo "   想先看原始帧：timeout 2 candump -td $CAN_IF"
    set_rate "$FOUND" >/dev/null 2>&1
    exit 0
fi

echo
echo "== 2/3 所有速率都 0 帧 —— 开 loopback 自发自收，只测 NX 这侧"
set_rate 1000000 loopback
timeout 2 candump -n 3 "$CAN_IF" > /tmp/can_loopback.txt 2>&1 &
DUMP_PID=$!
sleep 0.3
cansend "$CAN_IF" 123#DEADBEEF
cansend "$CAN_IF" 124#0102
cansend "$CAN_IF" 125#AABBCCDD
wait "$DUMP_PID" 2>/dev/null || true
LB=$(grep -c " 12[345] " /tmp/can_loopback.txt 2>/dev/null || echo 0)
echo "   回环收到的帧数: $LB"
sed 's/^/   | /' /tmp/can_loopback.txt 2>/dev/null | head -10
echo "   回环后 state=$(state) $(berr)"

echo
echo "== 3/3 恢复正常模式 @1Mbps"
set_rate 1000000 >/dev/null 2>&1 || true
echo "   state=$(state) $(berr) rx=$(rx)"

echo
echo "== 结论"
if [[ "$LB" -ge 1 ]]; then
    echo "   NX 的 can0（控制器+收发器）正常：loopback 能自收，任何速率下都收不到臂的帧。"
    echo "   问题在总线另一侧，按顺序查："
else
    echo "   loopback 都收不到 —— NX 这侧 can0 收发通路本身有问题（carrier 没引出收发器 /"
    echo "   针脚不是 can0 / mttcan 没配好）。先解决这个。同时查："
fi
cat <<'EOF'
   1) 线序：CAN_H ↔ CAN_H、CAN_L ↔ CAN_L 是否接反；接反的典型现象就是 ACK 不到、BUS-OFF、0 帧。
   2) 共地：NX 和臂的 GND 必须连上；两个独立电源只接 CAN_H/L 经常完全通不了。
   3) 终端电阻：断电后万用表量 CAN_H-CAN_L，两端各 120Ω 时约 60Ω；只有一端约 120Ω。
   4) 接口：臂的四芯线到底插在哪 —— 随附的 USB-CAN 小板在本机没有内核驱动
      （CONFIG_CAN_GS_USB 未开），插着也不出 canX 接口，见 scripts/setup_grasp_hw.sh --check。
   5) 臂侧：电源灯亮不等于 CAN 收发器工作，量一下臂 CAN 口在断电/上电时的电阻与电压。
EOF
