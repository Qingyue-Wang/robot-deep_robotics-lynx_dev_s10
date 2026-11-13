#!/usr/bin/env bash
# 抓取栈的宿主机前提（装的时候要 root）：D435i 的 USB 访问权限 + Piper 的 CAN 口。
#
# 为什么必须手动跑：本机 sudo 要密码，piper_ctl 的 auto_can_setup 和 realsense
# 的 USB 权限都不能在 boot 里悄悄搞定；不配的话 realsense_camera 的 CMD_INIT
# 打不开设备、piper_ctl 的 CMD_INIT 打不开 CAN，rbnx boot 会直接失败。
#
# 用法：
#   sudo bash scripts/setup_grasp_hw.sh           # 装 udev 规则/驱动 + 立即拉起 can_piper
#   bash scripts/setup_grasp_hw.sh --check        # 只检查，不改（普通用户可跑）
#
# 关于 CAN（2026-11-11 实测这台 NX）：
#   - 臂接在随附的 USB-CAN 小板（gs_usb, 1d50:606f）上，不是 Jetson 自带的 mttcan can0。
#   - 本机内核 5.15.148-tegra 没开 CONFIG_CAN_GS_USB，所以插着也不生成 canX 接口。
#     驱动已用官方 5.15.148 源码编在 ~/gsusb-build/gs_usb.ko，本脚本把它
#     modules_install 进 /lib/modules（之后插板 udev 自动 modprobe）。
#   - udev 规则把接口改名成 **can_piper**（清单里 piper_ctl.can_port 用的名字），
#     并配 1Mbps 拉起，所以 ./start.sh 之前不用再手动弄。
set -uo pipefail

D435I_RULE=/etc/udev/rules.d/99-realsense-d435i.rules
CAN_RULE=/etc/udev/rules.d/90-piper-can.rules
CAN_HELPER=/usr/local/sbin/piper-can-up
CAN_IF="${CAN_IF:-can_piper}"
BITRATE="${CAN_BITRATE:-1000000}"
USB_CAN_ID="1d50:606f"
GS_USB_KO="${GS_USB_KO:-/home/${SUDO_USER:-nvidia}/gsusb-build/gs_usb.ko}"
KERNEL_BUILD=$(readlink -f "/lib/modules/$(uname -r)/build" 2>/dev/null || true)

need_root() { [[ "$(id -u)" == "0" ]] || { echo "ERR: 需要 root：sudo bash $0" >&2; exit 2; }; }
driver_installed() { modinfo gs_usb >/dev/null 2>&1; }
state_of() { ip -d link show "$1" 2>/dev/null | grep -o 'can state [A-Z-]*' | awk '{print $3}' | head -1; }
rx_of() { cat "/sys/class/net/$1/statistics/rx_packets" 2>/dev/null || echo 0; }

check_state() {
    local rc=0
    echo "== D435i =="
    if lsusb -d 8086:0b3a >/dev/null 2>&1; then
        local dev
        dev=$(lsusb -d 8086:0b3a | grep -o "Bus [0-9]* Device [0-9]*" | awk '{printf "/dev/bus/usb/%s/%s", $2, $4}')
        echo "  lsusb: 8086:0b3a 在（$dev）"
        [[ -n "$dev" ]] && ls -l "$dev" | sed 's/^/  /'
        if [[ -n "$dev" && -r "$dev" && -w "$dev" ]]; then
            echo "  权限: 可读写 OK"
        else
            echo "  权限: 不可写 —— 需要 $D435I_RULE（sudo 跑本脚本，然后拔插一次相机）"
            rc=1
        fi
    else
        echo "  没看到 8086:0b3a —— D435i 没插好，或没插在 USB3 口上"; rc=1
    fi

    echo "== Piper CAN ($CAN_IF) =="
    if driver_installed; then
        echo "  gs_usb 驱动: 已装（$(modinfo -n gs_usb 2>/dev/null)），插板自动加载"
    else
        echo "  gs_usb 驱动: 未装 —— 插上 USB-CAN 小板也不会出 canX 接口"; rc=1
    fi
    if [[ -e "/sys/class/net/$CAN_IF" ]]; then
        echo "  $(ip -br link show "$CAN_IF")"
        local bc; bc=$(ip -s -d link show "$CAN_IF" | grep -o 'berr-counter tx [0-9]* rx [0-9]*' | head -1)
        [[ -n "$bc" ]] && echo "  $bc"
        local st; st=$(state_of "$CAN_IF")
        case "$st" in
            BUS-OFF) echo "  ERR: BUS-OFF —— 没有节点应答（臂没上电/线没接/波特率不对）"; rc=1 ;;
            ""|ERROR-ACTIVE) : ;;
            *) echo "  WARN: state=$st（错误计数在涨，接线/终端电阻存疑）" ;;
        esac
        local r0 r1
        r0=$(rx_of "$CAN_IF"); sleep 1; r1=$(rx_of "$CAN_IF")
        if [[ "$r1" -gt "$r0" ]]; then
            echo "  1s 内收到 $((r1 - r0)) 帧 —— 臂在线 OK"
        else
            echo "  WARN: 1s 内 0 帧 —— 臂不在总线上（示教模式？没上电？CAN 线？）"; rc=1
        fi
    else
        echo "  没有 $CAN_IF 接口"
        ip -br link show type can 2>/dev/null | sed 's/^/    /' || true
        echo "  需要: sudo bash scripts/setup_grasp_hw.sh"; rc=1
    fi
    return $rc
}

if [[ "${1:-}" == "--check" ]]; then
    check_state || exit 1
    exit 0
fi
need_root

echo "== 1/4 装 D435i 的 udev 规则 -> $D435I_RULE"
cat > "$D435I_RULE" <<'EOF'
# Intel RealSense D435i (8086:0b3a): 允许 plugdev 组读写，便于非 root 打开设备。
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE="0666", GROUP="plugdev"
EOF
chmod 644 "$D435I_RULE"

echo "== 2/4 装 gs_usb 驱动（USB-CAN 小板）"
if driver_installed; then
    echo "  已装: $(modinfo -n gs_usb 2>/dev/null)，跳过"
else
    if [[ ! -f "$GS_USB_KO" ]]; then
        echo "  ERR: 没有 $GS_USB_KO，先编译：" >&2
        echo "    cd /home/${SUDO_USER:-nvidia}/gsusb-build && make -C $KERNEL_BUILD M=\$PWD modules" >&2
        exit 1
    fi
    if [[ -z "$KERNEL_BUILD" || ! -d "$KERNEL_BUILD" ]]; then
        echo "  ERR: 找不到内核构建目录 /lib/modules/$(uname -r)/build" >&2; exit 1
    fi
    KO_DIR=$(dirname "$GS_USB_KO")
    if make -C "$KERNEL_BUILD" M="$KO_DIR" modules_install >/tmp/gsusb_install.log 2>&1; then
        depmod -a "$(uname -r)"
        echo "  已装到 /lib/modules/$(uname -r)/extra/gs_usb.ko（插板 udev 会自动 modprobe）"
    else
        echo "  ERR: modules_install 失败，见 /tmp/gsusb_install.log" >&2
        tail -5 /tmp/gsusb_install.log >&2; exit 1
    fi
fi

echo "== 3/4 装 udev 规则：USB-CAN 小板 -> $CAN_IF @ ${BITRATE}bps"
cat > "$CAN_HELPER" <<EOF
#!/bin/bash
# 由 /etc/udev/rules.d/90-piper-can.rules 在 USB-CAN 小板插入时调用。
# 把内核给的名字（can1/can2…）改成 ${CAN_IF}，配 ${BITRATE}bps 并拉起。
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
IFC="\${1:-}"
[[ -n "\$IFC" ]] || exit 0
for _ in \$(seq 1 20); do [[ -e "/sys/class/net/\$IFC" ]] && break; sleep 0.1; done
[[ -e "/sys/class/net/\$IFC" ]] || exit 0
ip link set "\$IFC" down 2>/dev/null
if [[ "\$IFC" != ${CAN_IF} && ! -e /sys/class/net/${CAN_IF} ]]; then
    ip link set "\$IFC" name ${CAN_IF} 2>/dev/null && IFC=${CAN_IF}
fi
ip link set "\$IFC" type can bitrate ${BITRATE} 2>/dev/null
ip link set "\$IFC" up 2>/dev/null
exit 0
EOF
chmod 755 "$CAN_HELPER"
cat > "$CAN_RULE" <<EOF
# Piper 臂随附的 USB-CAN 小板（gs_usb, 1d50:606f）：插入后改名 ${CAN_IF} 并配 ${BITRATE}bps。
# 配合 /lib/modules/.../extra/gs_usb.ko（本机内核没开 CONFIG_CAN_GS_USB）。
ACTION=="add", SUBSYSTEM=="net", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", RUN+="$CAN_HELPER %k"
EOF
chmod 644 "$CAN_RULE"

udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --action=change
sleep 1

echo "== 4/4 立即把当前接口拉成 $CAN_IF @ ${BITRATE}bps"
if ! modprobe gs_usb 2>/dev/null; then
    echo "  WARN: modprobe gs_usb 失败（驱动没装上？）"
fi

CUR=""
[[ -e "/sys/class/net/$CAN_IF" ]] && CUR="$CAN_IF"
if [[ -z "$CUR" ]]; then
    # 找 gs_usb 绑定的接口：设备路径里带 usb
    for c in /sys/class/net/can*; do
        [[ -e "$c" ]] || continue
        if readlink -f "$c" | grep -q usb; then CUR=$(basename "$c"); break; fi
    done
fi
if [[ -n "$CUR" ]]; then
    "$CAN_HELPER" "$CUR"
    echo "  已配置 $CAN_IF"
else
    echo "  WARN: 没找到 gs_usb 的 can 接口 —— USB-CAN 小板插着吗？（lsusb -d $USB_CAN_ID）"
fi

echo
echo "== 自检"
check_state || true
echo
echo "然后：ROBONIX_MANIFEST=\$PWD/robonix_manifest.nx.yaml ./start.sh"
