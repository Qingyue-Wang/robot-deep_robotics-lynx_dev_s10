#!/usr/bin/env python3
"""抓当前 /camera_435i/color/image_raw 一帧存盘。"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/now.jpg"


class Shot(Node):
    def __init__(self):
        super().__init__("snap_now")
        self.done = False
        self.create_subscription(Image, "/camera_435i/color/image_raw", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if self.done:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding.lower() in ("rgb8", "rgb"):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite(out, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        print(f"saved {out} {img.shape} {msg.encoding}")
        self.done = True
        raise SystemExit(0)


rclpy.init()
n = Shot()
try:
    rclpy.spin(n)
except SystemExit:
    pass
finally:
    n.destroy_node()
    rclpy.shutdown()
