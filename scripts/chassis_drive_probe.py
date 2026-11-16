#!/usr/bin/env python3
"""底盘直测：直接发 /cmd_vel，观察 /NAV_CMD、/MOTION_INFO 与 /odom 是否真的动。

用法（domain 43）：
  source /opt/ros/humble/setup.bash
  source /home/dog/ros2_ws/drdds_humble_ws/install/setup.bash
  ROS_DOMAIN_ID=43 /usr/bin/python3 scripts/chassis_drive_probe.py [m/s] [s]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from drdds.msg import MotionInfo, NavCmd

SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.12
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0


class Probe(Node):
    def __init__(self):
        super().__init__("chassis_drive_probe")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.mi = []
        self.nc = []
        self.od = []
        self.create_subscription(MotionInfo, "/MOTION_INFO", lambda m: self.mi.append(m), 20)
        self.create_subscription(NavCmd, "/NAV_CMD", lambda m: self.nc.append(m), 20)
        self.create_subscription(Odometry, "/odom", lambda m: self.od.append(m), 20)


def main():
    rclpy.init()
    n = Probe()
    t_end = time.time() + 2.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)

    o0 = n.od[-1].pose.pose.position if n.od else None
    print(f"起点 odom=({o0.x:+.2f},{o0.y:+.2f})" if o0 else "没有 /odom")

    t0 = time.time()
    while time.time() - t0 < SECS:
        msg = Twist()
        msg.linear.x = SPEED
        n.pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.02)
        time.sleep(0.08)

    for _ in range(40):  # 停止后再收 2 s
        n.pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.02)
        time.sleep(0.03)

    o1 = n.od[-1].pose.pose.position if n.od else None
    if o0 and o1:
        d = math.hypot(o1.x - o0.x, o1.y - o0.y)
        print(f"终点 odom=({o1.x:+.2f},{o1.y:+.2f}) 走了 {d:.2f} m "
              f"（指令 {SPEED} m/s × {SECS} s = {SPEED * SECS:.2f} m）")

    print(f"/NAV_CMD 收到 {len(n.nc)} 条；/MOTION_INFO 收到 {len(n.mi)} 条")
    if n.mi:
        m = n.mi[-1]
        d = m.data
        print(f"  MotionInfo: state={d.motion_state.state} gait={hex(d.gait_state.gait)} "
              f"height={d.height:.3f} vel_x={d.vel_x:+.3f} vel_y={d.vel_y:+.3f} vel_yaw={d.vel_yaw:+.3f}")
    if n.nc:
        c = n.nc[-1]
        print(f"  NavCmd: vel_x={c.vel_x:+.3f} vel_y={c.vel_y:+.3f} yaw={c.vel_yaw:+.3f}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
