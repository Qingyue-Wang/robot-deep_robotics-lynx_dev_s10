#!/usr/bin/env python3
"""从腕上相机抓帧，存到标注目录（给 label_tool.py 用）。

必须在栈运行、相机被 realsense provider 占着的情况下用（走 ROS 话题，不抢设备）：

    source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=43
    python3 scripts/capture_frames.py                 # 每按一次回车存一张，q 退出
    python3 scripts/capture_frames.py --auto 1.5 --count 6   # 每 1.5 s 自动存一张，共 6 张

别名（--prefix）方便按场景分：--prefix carrot / --prefix potato。
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description="腕上相机抓帧")
    ap.add_argument("--topic", default="/camera_435i/color/image_raw")
    ap.add_argument("--out", default=os.path.expanduser("~/grasp_labels/images"))
    ap.add_argument("--prefix", default="cap")
    ap.add_argument("--auto", type=float, default=0.0, help="每隔 N 秒自动存（0=回车触发）")
    ap.add_argument("--count", type=int, default=0, help="自动模式下最多存几张（0=不限）")
    ap.add_argument("--domain", default=os.environ.get("ROS_DOMAIN_ID", "43"))
    args = ap.parse_args()

    os.environ.setdefault("ROS_DOMAIN_ID", args.domain)
    os.makedirs(args.out, exist_ok=True)

    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("frame_capture")
    latest = {"msg": None}

    def on_image(msg: Image) -> None:
        latest["msg"] = msg

    node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)
    print(f"等待 {args.topic} …（Ctrl-C 退出）")

    saved = 0
    last_auto = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            msg = latest["msg"]
            if msg is None:
                continue
            if args.auto > 0:
                if time.time() - last_auto < args.auto:
                    continue
                last_auto = time.time()
            else:
                try:
                    cmd = input("回车存帧 / q 退出 > ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    break
                if cmd == "q":
                    break
            # 只取最新一帧，避免保存陈旧画面
            rclpy.spin_once(node, timeout_sec=0.05)
            msg = latest["msg"]
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            img = buf.reshape(msg.height, msg.width, -1)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            name = f"{args.prefix}_{time.strftime('%Y%m%d-%H%M%S')}_{saved:03d}.jpg"
            path = os.path.join(args.out, name)
            cv2.imwrite(path, img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            saved += 1
            print(f"saved {path} ({msg.width}x{msg.height})")
            if args.count and saved >= args.count:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f"共存 {saved} 张 → {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
