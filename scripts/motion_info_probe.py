#!/usr/bin/env python3
"""读 /MOTION_INFO（步态/运动状态/离地高度）与 /odom，判断底盘当前是滚轮还是步进。"""
import sys
import time

import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo  # noqa: E402

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0


class Probe(Node):
    def __init__(self):
        super().__init__("motion_info_probe")
        self.last = None
        self.n = 0
        self.create_subscription(MotionInfo, "/MOTION_INFO", self.cb, 10)

    def cb(self, m):
        self.n += 1
        self.last = m


rclpy.init()
p = Probe()
t0 = time.time()
last_key = None
while time.time() - t0 < secs:
    rclpy.spin_once(p, timeout_sec=0.1)
    m = p.last
    if m is None:
        continue
    st = m.motion_state.state
    gait = m.gait_state.gait
    h = m.height if hasattr(m, "height") else None
    key = (st, gait)
    if key != last_key:
        print(f"t={time.time()-t0:5.1f}s state={st} gait={hex(int(gait))} height={h} "
              f"vel=({m.vel_x:.3f},{m.vel_y:.3f},{m.vel_yaw:.3f})")
        last_key = key
print(f"samples={p.n}")
p.destroy_node()
rclpy.shutdown()
