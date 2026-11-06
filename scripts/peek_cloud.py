#!/usr/bin/env python3
"""Peek at the first PointCloud2 to check its orientation.

Usage:  python3 scripts/peek_cloud.py [topic] [target_frame]

Reads one cloud, transforms it into `target_frame` (default ``base_link``) via
TF, and prints the x/y/z bounds plus the first few points. If the transform is
missing it falls back to the raw sensor frame and says so.

Points are decoded directly from the ``data`` blob (no ``sensor_msgs_py``
dependency) so it works even where ``sensor_msgs.point_cloud2`` is missing.

Interpreting the front lidar (head-mounted, looking forward, top-vertical):

  * floor points land at  z < 0  (lidar is ~0.5 m above ground) with a large
    x spread  -> pitch sign is CORRECT.
  * the world looks "sideways" (floor appears as a vertical wall: large z
    spread / small x spread)  -> pitch sign is WRONG, flip 1.5708 -> -1.5708.
"""
import math
import struct
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/airy_201/points"
TARGET = sys.argv[2] if len(sys.argv) > 2 else "base_link"

FLOAT32 = 7
FLOAT64 = 8


def _field(msg, name):
    for f in msg.fields:
        if f.name == name:
            return f
    raise KeyError(f"field {name!r} not in PointCloud2")


class Peek(Node):
    def __init__(self):
        super().__init__("peek_cloud")
        self.got = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, TOPIC, self.cb, 1)

    def _decode(self, msg):
        data = bytes(msg.data)
        endian = ">" if msg.is_bigendian else "<"

        def fmt(f):
            if f.datatype == FLOAT32:
                return endian + "f"
            if f.datatype == FLOAT64:
                return endian + "d"
            raise ValueError(f"unsupported datatype {f.datatype} for {f.name}")

        xf, yf, zf = _field(msg, "x"), _field(msg, "y"), _field(msg, "z")
        fx, fy, fz = fmt(xf), fmt(yf), fmt(zf)

        def read(i):
            base = i * msg.point_step
            x = struct.unpack_from(fx, data, base + xf.offset)[0]
            y = struct.unpack_from(fy, data, base + yf.offset)[0]
            z = struct.unpack_from(fz, data, base + zf.offset)[0]
            return (x, y, z)

        pts = []
        for i in range(msg.width * msg.height):
            p = read(i)
            if all(math.isfinite(v) for v in p):
                pts.append(p)
        return pts

    def cb(self, msg):
        if self.got:
            return
        self.got = True
        pts = self._decode(msg)
        if not pts:
            print("(empty cloud / no finite points)")
            return

        tf = None
        try:
            tf = self.tf_buffer.lookup_transform(
                TARGET, msg.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
        except Exception as e:  # noqa: BLE001
            print(f"no transform {msg.header.frame_id} -> {TARGET}: {e}")
            print(f"(showing RAW {msg.header.frame_id} frame)")

        def xform(p):
            if tf is None:
                return p
            ps = PointStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.point.x, ps.point.y, ps.point.z = p
            q = do_transform_point(ps, tf).point
            return (q.x, q.y, q.z)

        frame = TARGET if tf is not None else msg.header.frame_id
        sub = pts[::10]
        xs = [xform(p)[0] for p in sub]
        ys = [xform(p)[1] for p in sub]
        zs = [xform(p)[2] for p in sub]
        print(f"frame={frame}  n_points={len(pts)}  (bounds over every 10th pt)")
        print(
            f"x:[{min(xs):.3f},{max(xs):.3f}]  "
            f"y:[{min(ys):.3f},{max(ys):.3f}]  "
            f"z:[{min(zs):.3f},{max(zs):.3f}]"
        )
        print("first points (x, y, z):")
        for p in pts[:8]:
            q = xform(p)
            print(f"  {q[0]:8.3f} {q[1]:8.3f} {q[2]:8.3f}")


def main():
    rclpy.init()
    node = Peek()
    deadline = time.time() + 8.0
    while time.time() < deadline and rclpy.ok() and not node.got:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.got:
        print(f"no message on {TOPIC} within 8s (is rslidar_sdk running?)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
