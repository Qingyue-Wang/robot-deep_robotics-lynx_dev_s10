#!/usr/bin/env python3
"""诊断：机器人位姿处的代价（local/global costmap）与地图覆盖情况。

用法（domain 43）：ROS_DOMAIN_ID=43 /usr/bin/python3 scripts/cost_at_pose_probe.py
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

TRANS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=QoSReliabilityPolicy.RELIABLE)


class Probe(Node):
    def __init__(self):
        super().__init__("cost_at_pose_probe")
        self.map = None
        self.local = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, TRANS)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._on_local, TRANS)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def _on_map(self, m):
        self.map = m

    def _on_local(self, m):
        self.local = m


def cell(grid, x, y):
    info = grid.info
    i = int((x - info.origin.position.x) / info.resolution)
    j = int((y - info.origin.position.y) / info.resolution)
    if i < 0 or j < 0 or i >= info.width or j >= info.height:
        return None, (i, j)
    return grid.data[j * info.width + i], (i, j)


def main():
    rclpy.init()
    n = Probe()
    end = time.time() + 12.0
    while time.time() + 3.0 < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    # 再给 TF 缓冲 3 s：map->odom、odom->base_link 都是 10–20 Hz
    tf_end = time.time() + 3.0
    while time.time() < tf_end:
        rclpy.spin_once(n, timeout_sec=0.05)

    if n.map is None or n.local is None:
        print(f"缺话题：/map={'有' if n.map else '无'} /local_costmap/costmap={'有' if n.local else '无'}")
        rclpy.shutdown()
        return 1

    try:
        tf = n.tf.lookup_transform("map", "base_link", rclpy.time.Time())
        px = tf.transform.translation.x
        py = tf.transform.translation.y
        yaw = 2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)
    except Exception as exc:  # noqa: BLE001
        print(f"TF map->base_link 不可用：{exc}")
        rclpy.shutdown()
        return 1

    print(f"机器人 map 位姿：x={px:+.3f} y={py:+.3f} yaw={yaw:+.3f} rad")
    g = n.map
    print(f"/map：{g.info.width}x{g.info.height} res={g.info.resolution:.3f} "
          f"origin=({g.info.origin.position.x:+.2f},{g.info.origin.position.y:+.2f})")
    v, ij = cell(g, px, py)
    print(f"  /map 机器人处 = {v} {'' if v is not None else '（超出地图范围！）'} cell={ij}")

    # 全局代价地图（如果有）
    print(f"/local_costmap/costmap：{n.local.info.width}x{n.local.info.height} "
          f"res={n.local.info.resolution:.3f}")
    v, ij = cell(n.local, px, py)
    print(f"  全局坐标下：{v} cell={ij}")

    # 滚动窗口：机器人一般在窗口中心
    ci, cj = n.local.info.width // 2, n.local.info.height // 2
    c = n.local.data[cj * n.local.info.width + ci]
    print(f"  窗口中心 (机器人应在的位置) = {c}（>=99 即认为在障碍里）")

    lethal = sum(1 for x in n.local.data if x >= 99)
    unk = sum(1 for x in n.local.data if x < 0)
    print(f"  窗口内 lethal={lethal} unknown={unk} / {len(n.local.data)}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
