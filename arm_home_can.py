#!/usr/bin/env python3
# Description: 不依赖 robonix 栈，**直连 CAN** 把 Piper 臂送回 home 位。
#
# 什么时候用这个脚本：
#   - robonix 栈没起（`arm_home.py` 会报"没有订阅者：/arm/joint_command 没人接"）；
#   - 只想快速把臂摆回 home，不想起整套 piper_ctl/robonix。
#
# 什么时候别用：
#   - robonix 栈已经在跑（piper_ctl 占着 can2）：用 `arm_home.py`（走 ROS 话题），
#     两边同时抢 CAN 会互相打架。
#
# 用法：
#   python3 arm_home_can.py                 # 回 home [0,0,0,0,25,0]，保持使能
#   python3 arm_home_can.py --disable       # 回 home 后失能
#   python3 arm_home_can.py --open-gripper  # 回 home 前先把夹爪张开
#   python3 arm_home_can.py --speed 15      # 慢一点
#
# home 位沿用标定脚本里的约定：[0, 0, 0, 0, 25, 0]（和 arm_home.py、DirectArm.home_and_disable 一致）。
import argparse
import sys
import time

# 依赖部署包里的 CAN 直连封装（和 minimal_grasp.py 用同一份）
DEPLOY = "/home/dog/.local/share/robonix-console/deployments/default"
PKG = DEPLOY + "/packages/skill-s10-grasp-dev-rbnx"
sys.path.insert(0, PKG)
sys.path.insert(0, PKG + "/grasp_skill/calib")

from common.robot.direct import (DirectArm, ARM_STATUS_TEXT, CTRL_MODE_TEXT,
                                 TEACH_STATUS_TEXT)

HOME_DEG = [0.0, 0.0, 0.0, 0.0, 25.0, 0.0]


def jstr(joints):
    return "[" + ", ".join("%.2f" % v for v in joints) + "]"


def describe(arm):
    st = arm.status()
    return "arm=%s ctrl=%s teach=%s joints=%s gripper=%.3f" % (
        ARM_STATUS_TEXT.get(st.arm_status, hex(st.arm_status)),
        CTRL_MODE_TEXT.get(st.ctrl_mode, hex(st.ctrl_mode)),
        TEACH_STATUS_TEXT.get(st.teach_status, hex(st.teach_status)),
        jstr(arm.joints()), arm.gripper_open_0to1())


def main() -> int:
    parser = argparse.ArgumentParser(description="CAN 直连：Piper 臂回 home（不依赖 robonix 栈）")
    parser.add_argument("--speed", type=int, default=20, help="速度百分比（默认 20）")
    parser.add_argument("--timeout", type=float, default=90.0, help="单次运动超时（秒）")
    parser.add_argument("--disable", action="store_true", help="回 home 后失能")
    parser.add_argument("--open-gripper", action="store_true", help="回 home 前先张开夹爪")
    args = parser.parse_args()

    arm = DirectArm()
    time.sleep(0.4)
    print("连接：", describe(arm), flush=True)

    st = arm.status()
    if st.arm_status != 0x00:
        print("！！臂状态异常（%s），先别动，检查后重试。"
              % ARM_STATUS_TEXT.get(st.arm_status, hex(st.arm_status)))
        return 2
    if st.teach_status == 0x01:
        print("！！还在示教模式（teach=0x01）：先在臂上按一下示教按钮退出拖拽，再跑本脚本。")
        return 2

    # 使能 + 切 CAN 控制（enable() 是幂等的：已使能会立刻返回）
    if not arm.enable_and_take_control(timeout_s=20, speed=args.speed):
        print("！！使能/切 CAN 控制失败")
        return 3

    if args.open_gripper:
        print("张开夹爪 …", flush=True)
        arm.open_gripper(effort=5000)
        time.sleep(0.8)

    print("回 home 位 %s（速度 %d%%）…" % (HOME_DEG, args.speed), flush=True)
    arm.move_joints(HOME_DEG, speed=args.speed, timeout_s=args.timeout, tolerance_deg=1.5)
    time.sleep(0.5)

    if args.disable:
        print("失能 …", flush=True)
        arm.piper.DisableArm()                 # 与 home_and_disable 内部一致
        arm.piper.GripperCtrl(gripper_code=0x02)
        time.sleep(0.5)

    print("完成：", describe(arm), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
