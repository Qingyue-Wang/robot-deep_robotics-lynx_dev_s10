#!/usr/bin/env python3
"""采样底盘运动遥测，用于判断「滚轮」还是「抬腿步进」。

记录 /MOTION_INFO（state/gait/height/速度）、/odom（位姿）、/cmd_vel（输入指令），
20Hz 写 JSONL。抬腿时 body height 会以 1.5~2.5 Hz 上下振荡（几 cm），滚轮时只
有毫米级抖动。

用法：source /opt/ros/humble/setup.bash
      source ~/ros2_ws/drdds_humble_ws/install/setup.bash
      ROS_DOMAIN_ID=43 python3 scripts/motion_trace.py [秒数]
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
out_dir = "/home/dog/robot-deep_robotics-lynx_s10/rbnx-boot/data"
os.makedirs(out_dir, exist_ok=True)
path = os.path.join(out_dir, time.strftime("motion_trace_%Y%m%d-%H%M%S.jsonl"))

rows = []


class Trace(Node):
    def __init__(self):
        super().__init__("motion_trace")
        self.create_subscription(MotionInfo, "/MOTION_INFO", self.on_info, 20)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 20)

    def _emit(self, rec):
        rec["t"] = time.time()
        rows.append(rec)

    def on_info(self, m):
        d = m.data
        self._emit({"ev": "info", "state": int(d.motion_state.state), "gait": int(d.gait_state.gait),
                    "height": float(d.height), "vx": float(d.vel_x), "vy": float(d.vel_y),
                    "vyaw": float(d.vel_yaw)})

    def on_odom(self, m):
        p = m.pose.pose.position
        self._emit({"ev": "odom", "x": float(p.x), "y": float(p.y), "z": float(p.z)})

    def on_cmd(self, m):
        self._emit({"ev": "cmd", "vx": float(m.linear.x), "wz": float(m.angular.z)})


rclpy.init()
n = Trace()
t0 = time.time()
try:
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.05)
finally:
    with open(path, "w") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")
    print(f"wrote {len(rows)} rows -> {path}")
    n.destroy_node()
    rclpy.shutdown()
