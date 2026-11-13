#!/usr/bin/env bash
# 加载 gs_usb 驱动并检验"臂接在随附的 USB-CAN 小板（1d50:606f）上"这条路。
#
# 背景：本机内核 5.15.148-tegra 没开 CONFIG_CAN_GS_USB，这块板插着也不会出 canX。
# 已用官方 5.15.148 源码编出 gs_usb.ko（vermagic 与运行内核完全一致）。
# 本脚本：insmod → 等新 can 接口出现 → 改名 can_piper → 1Mbps 拉起 → 听 2 秒。
#
# 用法：
#   sudo bash scripts/try_gs_usb.sh              # 加载并探测（不动 can0）
#   sudo bash scripts/try_gs_usb.sh --install    # 顺便 modules_install + depmod，之后插拔/重启自动加载
#   sudo bash scripts/try_gs_usb.sh --unload     # 卸载驱动、删掉 can_piper
set -uo pipefail
KO="${GS_USB_KO:-/home/nvidia/gsusb-build/gs_usb.ko}"
TARGET_NAME=can_piper
MODE=probe
for a in "$@"; do
    case "$a" in
        --install) MODE=install ;;
        --unload)  MODE=unload ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    esac
done

[[ "$(id -u)" == "0" ]] || { echo "ERR: 需要 root：sudo bash $0" >&2; exit 2; }
command -v candump >/dev/null || { echo "ERR: 缺 can-utils（sudo apt install can-utils）" >&2; exit 2; }

if [[ "$MODE" == "unload" ]]; then
    ip link show "$TARGET_NAME" >/dev/null 2>&1 && ip link set "$TARGET_NAME" down 2>/dev/null || true
    ip link set "$TARGET_NAME" name can_restore_$$ 2>/dev/null || true
    modprobe -r gs_usb 2>/dev/null || true
    echo "已卸载 gs_usb"
    exit 0
fi

[[ -f "$KO" ]] || { echo "ERR: 找不到 $KO。重新编译：" >&2; echo "  cd ~/gsusb-build && make -C \$(readlink -f /lib/modules/\$(uname -r)/build) M=\$PWD modules" >&2; exit 1; }

echo "== USB-CAN 小板"
lsusb -d 1d50:606f 2>/dev/null | sed 's/^/   /' || echo "   WARN: lsusb 里没有 1d50:606f（板子没插？）"

before=$(ls /sys/class/net | sort)

if lsmod | grep -q '^gs_usb'; then
    echo "== gs_usb 已加载，跳过 insmod"
else
    echo "== insmod $KO"
    if ! insmod "$KO"; then
        echo "ERR: insmod 失败（内核可能拒绝无签名模块）"; dmesg | tail -5 >&2; exit 1
    fi
    sleep 0.5
    dmesg | tail -3 | sed 's/^/   /'
fi

# 等新 can 接口出现（USB 探测 + udev 可能要 1~3s）
NEW=""
for _ in $(seq 1 20); do
    sleep 0.5
    after=$(ls /sys/class/net | sort)
    NEW=$(comm -13 <(echo "$before") <(echo "$after") | grep '^can' || true)
    [[ -n "$NEW" ]] && break
    ls /sys/class/net | grep -qx "$TARGET_NAME" && { NEW=$TARGET_NAME; break; }
done

if [[ -z "$NEW" ]]; then
    echo
    echo "== 没有出现新 can 接口（板子插着但驱动没绑上？）"
    echo "   dmesg 末尾："; dmesg | tail -12 | sed 's/^/   /'
    echo "   USB 树："; lsusb -t 2>/dev/null | grep -A1 -B1 '1d50\|gs_usb' | sed 's/^/   /'
    exit 1
fi
echo "   新接口: $(echo "$NEW" | tr '\n' ' ')"

IFC=$(echo "$NEW" | head -1)
TARGET_NAME="$IFC"
if [[ "$IFC" != "can_piper" ]] && ! ls /sys/class/net | grep -qx can_piper; then
    ip link set "$IFC" down 2>/dev/null || true
    if ip link set "$IFC" name can_piper 2>/dev/null; then
        TARGET_NAME=can_piper
        echo "   已改名 can_piper（清单里 can_port 用它最稳）"
    fi
fi

echo
echo "== 以 1Mbps 拉起 $TARGET_NAME 并听 3 秒"
ip link set "$TARGET_NAME" down 2>/dev/null || true
ip link set "$TARGET_NAME" type can bitrate 1000000 || { echo "ERR: 设置波特率失败" >&2; exit 1; }
ip link set "$TARGET_NAME" up
ip -br link show "$TARGET_NAME" | sed 's/^/   /'
timeout 3 candump -td "$TARGET_NAME" > /tmp/can_piper_probe.txt 2>&1 || true
N=$(wc -l < /tmp/can_piper_probe.txt)
echo "   3 秒内收到 $N 帧"
head -6 /tmp/can_piper_probe.txt | sed 's/^/   | /'
echo "   state=$(ip -d link show $TARGET_NAME | grep -o 'can state [A-Z-]*' | head -1) $(ip -s -d link show $TARGET_NAME | grep -o 'berr-counter tx [0-9]* rx [0-9]*' | head -1)"

echo
if [[ "$N" -gt 0 ]]; then
    echo "== 结论：臂挂在这块 USB-CAN 板上，接口 $TARGET_NAME @1Mbps。"
    echo "   清单 piper_ctl.can_port 应为 $TARGET_NAME。确认后："
    echo "   ROBONIX_MANIFEST=\$PWD/robonix_manifest.nx.yaml ./start.sh"
    if [[ "$MODE" != "install" ]]; then
        echo
        echo "   持久化（重启/插拔后自动加载 + 自动叫 can_piper @1M）："
        echo "   sudo bash scripts/try_gs_usb.sh --install"
    fi
else
    echo "== 结论：这块板上 3 秒也没有帧。"
    echo "   若臂还停在示教模式，先切回正常模式再跑一次；仍 0 帧就是 CAN 线/臂侧收发器问题。"
fi

if [[ "$MODE" == "install" ]]; then
    echo
    echo "== 持久化驱动"
    KS=$(readlink -f /lib/modules/"$(uname -r)"/build)
    if make -C "$KS" M=/home/nvidia/gsusb-build modules_install >/tmp/gsusb_install.log 2>&1 && depmod -a "$(uname -r)"; then
        echo "   已装到 /lib/modules/$(uname -r)/extra/gs_usb.ko，插板子会自动加载"
    else
        echo "   WARN: modules_install 失败，见 /tmp/gsusb_install.log"; tail -3 /tmp/gsusb_install.log | sed 's/^/   /'
    fi
fi
