#!/usr/bin/env python3
"""实时对照：DWB 输出(cmd_vel_nav) vs smoother 输出(cmd_vel_guard_input) vs 最终(/cmd_vel)。

用法：ROS_DOMAIN_ID=43 /usr/bin/python3 scripts/nav_cmd_probe.py [前进距离m] [时长s]
默认朝当前航向走 1.0 m，观察 15 s。只用于诊断，不改任何配置。
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

AHEAD = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0


class Probe(Node):
    def __init__(self):
        super().__init__("nav_cmd_probe")
        self.samples = {"cmd_vel_nav": [], "cmd_vel_guard_input": [], "cmd_vel": [], "odom": []}
        self.create_subscription(Twist, "/cmd_vel_nav", lambda m: self._rec("cmd_vel_nav", m), 20)
        self.create_subscription(
            Twist, "/cmd_vel_guard_input", lambda m: self._rec("cmd_vel_guard_input", m), 20)
        self.create_subscription(Twist, "/cmd_vel", lambda m: self._rec("cmd_vel", m), 20)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None

    def _rec(self, key, m):
        self.samples[key].append((m.linear.x, m.angular.z))


def stats(rows):
    if not rows:
        return "无消息"
    nz = sum(1 for v, w in rows if abs(v) > 0.001 or abs(w) > 0.001)
    mv = max(abs(v) for v, _ in rows)
    mw = max(abs(w) for _, w in rows)
    return (f"n={len(rows)} 非零={100.0 * nz / len(rows):.0f}% "
            f"max|v|={mv:.3f} max|ω|={mw:.3f}")


def main():
    rclpy.init()
    n = Probe()
    t_end = time.time() + 3.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)

    t_end = time.time() + 10.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)

    t = None
    for _ in range(60):
        try:
            t = n.tf.lookup_transform("map", "base_link", rclpy.time.Time())
            break
        except Exception:  # noqa: BLE001
            rclpy.spin_once(n, timeout_sec=0.1)
    if t is None:
        print("TF map->base_link 不可用（等了约 16 s）")
        rclpy.shutdown()
        return 1
    px, py = t.transform.translation.x, t.transform.translation.y
    q = t.transform.rotation
    yaw = 2.0 * math.atan2(q.z, q.w)
    gx, gy = px + AHEAD * math.cos(yaw), py + AHEAD * math.sin(yaw)
    print(f"当前 ({px:+.2f},{py:+.2f}) yaw={yaw:+.2f} → 目标 ({gx:+.2f},{gy:+.2f}) 前向 {AHEAD} m")

    from geometry_msgs.msg import PoseStamped
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = n.get_clock().now().to_msg()
    goal.pose.pose.position.x = gx
    goal.pose.pose.position.y = gy
    goal.pose.pose.orientation = q
    if not n.ac.wait_for_server(timeout_sec=5.0):
        print("navigate_to_pose action 不可用")
        rclpy.shutdown()
        return 1
    n.goal_handle = n.ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, n.goal_handle, timeout_sec=5.0)

    t_end = time.time() + DURATION
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)

    for k in ("cmd_vel_nav", "cmd_vel_guard_input", "cmd_vel"):
        print(f"{k:22s}: {stats(n.samples[k])}")

    # 取消目标，避免继续走
    try:
        if n.goal_handle and n.goal_handle.result() and n.goal_handle.result().goal_handle:
            n.goal_handle.result().goal_handle.cancel_goal_async()
            for _ in range(20):
                rclpy.spin_once(n, timeout_sec=0.05)
    except Exception:  # noqa: BLE001
        pass
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
