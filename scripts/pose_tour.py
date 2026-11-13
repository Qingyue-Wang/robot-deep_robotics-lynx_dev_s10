#!/usr/bin/env python3
"""依次走到 4 个观察位姿，并在每个位置抓一帧 /camera_435i/color/image_raw 存图。"""
import os
import sys
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

ROOT = "/home/dog/robot-deep_robotics-lynx_s10"
SKILL = os.path.join(ROOT, "rbnx-boot/cache/skill-pick-depth-grasp-rbnx")
sys.path[:0] = [os.path.join(SKILL, "grasp_skill/calib"), SKILL]
PIPER_WS = os.path.join(ROOT, "rbnx-boot/cache/primitive-agilex-piper-arm-rbnx/rbnx-build/ws/install")
for p in (os.path.join(PIPER_WS, "piper"), os.path.join(PIPER_WS, "piper_msgs"), os.path.join(PIPER_WS, "graspnet_msgs")):
    sys.path.append(p)
sys.path.append(os.path.join(ROOT, "rbnx-boot/cache/primitive-agilex-piper-arm-rbnx/rbnx-build/piper_sdk_parent"))

import can
from common.robot.direct import DirectArm

OUT = os.path.join(ROOT, "logs/pose_shots")
os.makedirs(OUT, exist_ok=True)

POSES = [
    ("1_nominal", [1.13, 86.36, -97.62, 6.34, 69.99, 0.0]),
    ("2_rec1", [7.45, 62.86, -87.57, 4.53, 69.86, 7.80]),
    ("3_rec2", [7.45, 52.60, -53.59, 0.0, 59.35, 7.69]),
    ("4_home", [0.0, 0.0, 0.0, 0.0, 25.0, 0.0]),
]


class Grabber(Node):
    def __init__(self):
        super().__init__("pose_tour_grabber")
        self.latest = None
        self.count = 0
        self.create_subscription(Image, "/camera_435i/color/image_raw", self._on_img, qos_profile_sensor_data)

    def _on_img(self, msg):
        self.latest = msg
        self.count += 1


def main():
    rclpy.init()
    node = Grabber()

    def spin_thread():
        rclpy.spin(node)

    t = threading.Thread(target=spin_thread, daemon=True)
    t.start()

    arm = DirectArm(speed=30)
    st = arm.status("before")
    print("before:", st, flush=True)

    # 示教模式下 CAN 运动指令不生效：先发 0x150 恢复 + 0x151 CAN 模式
    if getattr(st, "ctrl_mode", 0) == 0x02:
        print("arm in teach mode -> sending MotionCtrl_1(0x02) + MotionCtrl_2(0x01,0x01)", flush=True)
        bus = can.interface.Bus(channel="can_piper", bustype="socketcan")
        for _ in range(3):
            bus.send(can.Message(arbitration_id=0x150, data=bytes([0x02, 0, 0, 0, 0, 0, 0, 0]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x151, data=bytes([0x01, 0x01, 0x32, 0, 0, 0, 0, 0]), is_extended_id=False))
            time.sleep(0.1)
        bus.shutdown()
        time.sleep(0.5)
        print("after recover:", arm.status("after-recover"), flush=True)

    arm.enable_and_take_control(timeout_s=20, speed=30)

    for name, target in POSES:
        print(f"\n=== {name} -> {target}", flush=True)
        before = node.count
        ok = arm.move_joints(target, speed=30, timeout_s=60, tolerance_deg=1.5)
        cur = arm.joints()
        print(f"move ok={ok} joints={['%.2f' % v for v in cur]}", flush=True)
        # 等图像刷新并稳定
        t0 = time.time()
        got = None
        while time.time() - t0 < 4.0:
            if node.latest is not None and node.count > before + 2:
                got = node.latest
                break
            time.sleep(0.05)
        if got is None:
            got = node.latest
        if got is None:
            print("!! no image", flush=True)
            continue
        h, w, c = got.height, got.width, 3
        buf = np.frombuffer(bytes(got.data), dtype=np.uint8)
        if got.step != w * c:
            buf = buf.reshape(h, got.step)[:, : w * c]
        img = buf.reshape(h, w, c)
        if got.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        path = os.path.join(OUT, f"{name}.jpg")
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        print(f"saved {path} ({w}x{h} {got.encoding}) status={arm.status('at-pose')}", flush=True)

    print("\ndone", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
