#!/usr/bin/env python3
"""把 Piper 臂送回 home 位：六个关节全零，夹爪张开。

用法（先 source ROS 环境）：
    source /opt/ros/humble/setup.bash
    python3 arm_home.py            # 回 home，等到位
    python3 arm_home.py --no-wait  # 只发命令，不等到位
    python3 arm_home.py --opening-mm 56   # 夹爪只张到 56 mm

依赖：正在运行的 robonix 栈（piper_ctl 会起 piper_single_ctrl，
它订阅 /arm/joint_command 并发布 /arm/joint_states_single）。

脚本启动时会检查 can2（USB-CAN）；如果没 UP，会用 sudo 尝试把它拉到
1 Mbit/s——第一次开机没手动配置过也能直接跑（会提示输入 sudo 密码）。
随后会给臂发一次使能（/arm/enable_flag），再送 home。
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "0")

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
CMD_TOPIC = "/arm/joint_command"
STATE_TOPIC = "/arm/joint_states_single"
ENABLE_TOPIC = "/arm/enable_flag"   # std_msgs/Bool；true -> EnableArm(7)
TOL_RAD = 0.02          # 到位判据：每个关节 0.02 rad ≈ 1.1°
PUBLISH_TIMES = 3       # 驱动按"最后一条"生效；多发几次防丢
PUBLISH_GAP_S = 0.3

CAN_IFACE = "can2"      # 山猫上 USB-CAN 适配器（1-4.1.1）枚举成 can2
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
        print(f"[arm_home] 没有 {iface}，跳过 CAN 检查", file=sys.stderr)
        return True
    if state:
        return True

    print(f"[arm_home] {iface} 未 UP，尝试拉起（可能需要 sudo 密码）")
    commands = [
        ["sudo", "ip", "link", "set", iface, "type", "can", "bitrate", bitrate],
        ["sudo", "ip", "link", "set", iface, "up"],
    ]
    for command in commands:
        try:
            result = subprocess.run(command)
        except OSError as error:
            print(f"[arm_home] 执行失败：{error}", file=sys.stderr)
            result = None
        if result is None or result.returncode != 0:
            print(
                f"[arm_home] 拉 {iface} 失败，请手动执行：\n"
                f"  sudo ip link set {iface} type can bitrate {bitrate}\n"
                f"  sudo ip link set {iface} up",
                file=sys.stderr,
            )
            return False
    if _can_state(iface):
        print(f"[arm_home] {iface} 已 UP @ {bitrate} bit/s")
        return True
    print(f"[arm_home] {iface} 仍然不是 UP", file=sys.stderr)
    return False


def _joint_map(msg: JointState) -> dict[str, float]:
    return dict(zip(msg.name, msg.position))


def main() -> int:
    parser = argparse.ArgumentParser(description="Move the Piper arm to its home pose.")
    parser.add_argument("--opening-mm", type=float, default=102.0,
                        help="夹爪开口宽度 (mm)，默认 102（全开）")
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="等待到位的最长秒数，默认 20")
    parser.add_argument("--no-wait", action="store_true",
                        help="发完命令立即退出，不等关节到位")
    parser.add_argument("--no-enable", action="store_true",
                        help="不发送使能，直接回 home")
    args = parser.parse_args()

    if not ensure_can_up():
        return 3

    travel_m = max(0.0, args.opening_mm) / 2000.0   # 开口宽度 -> 单指行程

    rclpy.init()
    node = Node("arm_home")
    publisher = node.create_publisher(JointState, CMD_TOPIC, 10)
    latest: dict[str, JointState] = {}
    node.create_subscription(
        JointState, STATE_TOPIC, lambda m: latest.__setitem__("msg", m), 10)

    # 等 driver 的订阅出现，确认栈在跑
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        print(f"[arm_home] 没有订阅者：{CMD_TOPIC} 没人接。栈起了吗？", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    if not args.no_enable:
        enable_publisher = node.create_publisher(Bool, ENABLE_TOPIC, 10)
        deadline = time.monotonic() + 2.0
        while (time.monotonic() < deadline
               and enable_publisher.get_subscription_count() == 0):
            rclpy.spin_once(node, timeout_sec=0.1)
        if enable_publisher.get_subscription_count() == 0:
            print(f"[arm_home] 警告：{ENABLE_TOPIC} 没有订阅者，跳过使能",
                  file=sys.stderr)
        else:
            for _ in range(3):
                enable_publisher.publish(Bool(data=True))
                rclpy.spin_once(node, timeout_sec=0.1)
                time.sleep(0.2)
            print("[arm_home] 已发送使能")
            time.sleep(0.5)          # 等 EnableArm(7) 生效后再发运动命令

    command = JointState()
    command.name = JOINTS + ["gripper"]
    command.position = [0.0] * 6 + [travel_m]

    for _ in range(PUBLISH_TIMES):
        command.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(PUBLISH_GAP_S)
    print(f"[arm_home] 已发送 home 命令（gripper 行程 {travel_m * 1000:.1f} mm）")

    if args.no_wait:
        node.destroy_node()
        rclpy.shutdown()
        return 0

    deadline = time.monotonic() + args.timeout
    last = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        msg = latest.get("msg")
        if msg is None:
            continue
        values = _joint_map(msg)
        if not all(name in values for name in JOINTS):
            continue
        last = [values[name] for name in JOINTS]
        if all(abs(value) <= TOL_RAD for value in last):
            opening = values.get("gripper")
            print("[arm_home] 已到位：" + ", ".join(
                f"{name}={value:+.3f}" for name, value in zip(JOINTS, last))
                + (f", gripper={opening * 1000:.1f}mm" if opening is not None else ""))
            node.destroy_node()
            rclpy.shutdown()
            return 0

    shown = ", ".join(f"{v:+.3f}" for v in last) if last else "无反馈"
    print(f"[arm_home] {args.timeout:.0f}s 内没到位（当前 {shown}）", file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
