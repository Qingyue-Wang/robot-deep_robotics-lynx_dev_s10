#!/usr/bin/env python3
"""失能 Piper 臂：调用 piper 驱动的 /arm/enable_srv（enable_request=false）。

驱动收到后会执行 DisableArm(7)，并把夹爪打开；服务返回 enable_response=True
表示失能成功。

用法：
    source /opt/ros/humble/setup.bash
    python3 disable_arm.py
    python3 disable_arm.py --piper-ws /path/to/ws/install

注意：如果 can2 掉过线又恢复，piper 驱动的 CAN socket 可能已经坏了（日志里
`send failed`）——先把 can2 拉起来并重启 robonix 栈，再跑本脚本。
"""
from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys

SERVICE = "/arm/enable_srv"
SRV_TYPE = "piper_msgs/srv/Enable"
DEFAULT_PIPER_WS = (
    "/home/dog/.local/share/robonix-console/deployments/default/packages/"
    "primitive-agilex-piper-arm-rbnx/rbnx-build/ws/install"
)
CAN_IFACE = "can2"
CAN_BITRATE = "1000000"


def _can_state(iface: str) -> bool | None:
    """True=UP, False=DOWN, None=查不到这个接口。"""
    try:
        result = subprocess.run(["ip", "-br", "link", "show", iface],
                                capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    fields = result.stdout.split()
    return len(fields) >= 2 and fields[1] == "UP"


def ensure_can_up(iface: str = CAN_IFACE, bitrate: str = CAN_BITRATE) -> bool:
    """确保 CAN 接口 UP；没 UP 就 sudo 拉起。返回是否可用。"""
    state = _can_state(iface)
    if state is None:
        print(f"[disable_arm] 没有 {iface}，跳过 CAN 检查", file=sys.stderr)
        return True
    if state:
        return True

    print(f"[disable_arm] {iface} 未 UP，尝试拉起（可能需要 sudo 密码）")
    commands = [
        ["sudo", "ip", "link", "set", iface, "type", "can", "bitrate", bitrate],
        ["sudo", "ip", "link", "set", iface, "up"],
    ]
    for command in commands:
        try:
            result = subprocess.run(command)
        except OSError as error:
            print(f"[disable_arm] 执行失败：{error}", file=sys.stderr)
            result = None
        if result is None or result.returncode != 0:
            print(
                f"[disable_arm] 拉 {iface} 失败，请手动执行：\n"
                f"  sudo ip link set {iface} type can bitrate {bitrate}\n"
                f"  sudo ip link set {iface} up",
                file=sys.stderr,
            )
            return False
    if _can_state(iface):
        print(f"[disable_arm] {iface} 已 UP @ {bitrate} bit/s")
        return True
    print(f"[disable_arm] {iface} 仍然不是 UP", file=sys.stderr)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Disable the Piper arm.")
    parser.add_argument("--piper-ws", default=os.environ.get("PIPER_WS", DEFAULT_PIPER_WS),
                        help="piper 工作区的 install 目录（含 setup.bash）")
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="服务调用超时秒数，默认 20")
    args = parser.parse_args()

    if not ensure_can_up():
        return 3

    setup_bash = os.path.join(args.piper_ws, "setup.bash")
    if not os.path.isfile(setup_bash):
        print(f"[disable_arm] 找不到 {setup_bash}；用 --piper-ws 指定 piper 工作区",
              file=sys.stderr)
        return 4

    command = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(setup_bash)} && "
        f"ros2 service call {SERVICE} {SRV_TYPE} '{{enable_request: false}}'"
    )
    try:
        result = subprocess.run(["bash", "-c", command],
                                capture_output=True, text=True, timeout=args.timeout)
    except subprocess.TimeoutExpired:
        print(f"[disable_arm] 服务调用超过 {args.timeout:.0f}s 没返回", file=sys.stderr)
        return 1

    stdout = result.stdout.strip()
    stderr = result.stderr.strip()
    if stdout:
        print(stdout)
    if result.returncode != 0:
        if stderr:
            print(stderr, file=sys.stderr)
        print("[disable_arm] 服务调用失败：栈在跑吗？can2 掉过的话需要重启栈。",
              file=sys.stderr)
        return 1

    if "enable_response=True" in stdout:
        print("[disable_arm] 已失能（DisableArm(7)），夹爪已打开")
        return 0
    if "enable_response=False" in stdout:
        print("[disable_arm] 失能失败/超时（enable_response=False）", file=sys.stderr)
        return 1
    print("[disable_arm] 没在服务响应里看到 enable_response，请检查上面的输出",
          file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
