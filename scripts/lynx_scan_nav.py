#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Merge the Lynx S10 front/rear Airy clouds into one 360-degree LaserScan.

Mapping keeps using the front cloud through the ``lynx_lidar`` primitive; this
node only feeds the navigation costmaps on ``/scanner/scan_nav``. The two
lidars are mounted vertically (front faces forward, rear faces backward), so
each contributes one half of the 360-degree 2D slice:

    /airy_201/points (front) ─┐
                              ├─ base_link -> height band -> 720 bins -> min range
    /airy_202/points (rear)  ─┘
                              ▼
                      /scanner/scan_nav

The height band is ``-0.25 .. 1.50 m`` in ``base_link``. The measured floor is
at z ~= -0.45 m, so the floor stays out while low obstacles down to ~0.2 m
above the floor are kept. ``range_min`` (default 0.36 m, the chassis radius)
drops the robot's own legs/body.

``/scanner/scan`` (front-only) is deliberately left untouched: it still feeds
mapping's relocalization and the nav2 velocity guard's diagnostics.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


def _apply_ros_env(manifest_path: str) -> None:
    """Use the same ROS domain/RMW as the stack (manifest ``env:`` block).

    Must run before ``rclpy.init()``; the domain id is read when the middleware
    initialises, not at import time.
    """
    try:
        with open(manifest_path, encoding="utf-8") as handle:
            env = (yaml.safe_load(handle) or {}).get("env") or {}
    except OSError as error:
        print(f"[lynx_scan_nav] cannot read manifest {manifest_path}: {error}",
              file=sys.stderr)
        return
    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"):
        value = env.get(key)
        if value is not None:
            os.environ[key] = str(value)


def _quat_to_mat(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _points_from_cloud(msg: PointCloud2) -> np.ndarray:
    """Return an (N, 3) float64 array from an XYZIRT PointCloud2.

    The driver's point_step is 26 bytes with x/y/z at offsets 0/4/8, but the
    offsets are read from the message fields so a point-type change cannot
    silently feed garbage into the scan.
    """
    fields = {field.name: field for field in msg.fields}
    missing = [name for name in ("x", "y", "z") if name not in fields]
    if missing:
        raise ValueError(f"cloud has no {missing} fields")
    for name in ("x", "y", "z"):
        if fields[name].datatype != 7:  # sensor_msgs/PointField.FLOAT32
            raise ValueError(f"cloud field {name} is not FLOAT32")
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
        "itemsize": msg.point_step,
    })
    count = int(msg.width) * int(msg.height)
    if count == 0:
        return np.empty((0, 3), dtype=np.float64)
    try:
        raw = np.frombuffer(msg.data, dtype=dtype, count=count)
    except (TypeError, ValueError):
        raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=count)
    return np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)


class NavScanMerger(Node):
    """Publish one 360-degree scan from the front and rear Airy clouds."""

    def __init__(self) -> None:
        super().__init__("lynx_scan_nav")
        self.base_frame = str(self.declare_parameter(
            "base_frame", "base_link").value)
        self.front_topic = str(self.declare_parameter(
            "front_topic", "/airy_201/points").value)
        self.rear_topic = str(self.declare_parameter(
            "rear_topic", "/airy_202/points").value)
        self.front_frame = str(self.declare_parameter(
            "front_frame", "airy_201").value)
        self.rear_frame = str(self.declare_parameter(
            "rear_frame", "airy_202").value)
        self.output_topic = str(self.declare_parameter(
            "output_topic", "/scanner/scan_nav").value)
        self.height_min = float(self.declare_parameter(
            "height_min", -0.25).value)
        self.height_max = float(self.declare_parameter(
            "height_max", 1.50).value)
        self.range_min = float(self.declare_parameter(
            "range_min", 0.36).value)
        self.range_max = float(self.declare_parameter(
            "range_max", 12.0).value)
        self.bins = int(self.declare_parameter("angle_bins", 720).value)
        self.rate_hz = float(self.declare_parameter(
            "publish_rate", 10.0).value)
        self.max_cloud_age_s = float(self.declare_parameter(
            "max_cloud_age_s", 0.6).value)

        self._front: PointCloud2 | None = None
        self._rear: PointCloud2 | None = None
        self._warn_at: dict[str, float] = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            PointCloud2, self.front_topic, self._on_front,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.rear_topic, self._on_rear,
            qos_profile_sensor_data)
        self._pub = self.create_publisher(
            LaserScan, self.output_topic, qos_profile_sensor_data)
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self._tick)
        self.get_logger().info(
            "lynx_scan_nav ready: %s + %s -> %s "
            "(height %.2f..%.2f m, range %.2f..%.2f m, %d bins, %.1f Hz)"
            % (self.front_topic, self.rear_topic, self.output_topic,
               self.height_min, self.height_max, self.range_min,
               self.range_max, self.bins, self.rate_hz))

    def _warn(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._warn_at.get(key, 0.0) >= 5.0:
            self._warn_at[key] = now
            self.get_logger().warning(message)

    def _on_front(self, msg: PointCloud2) -> None:
        self._front = msg

    def _on_rear(self, msg: PointCloud2) -> None:
        self._rear = msg

    def _transform(self, msg: PointCloud2, fallback_frame: str) -> np.ndarray | None:
        frame = msg.header.frame_id or fallback_frame
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame, frame, Time())
        except Exception as error:  # tf2 raises several exception types
            self._warn(f"tf:{frame}",
                       f"cannot transform {frame} -> {self.base_frame}: {error}")
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = _quat_to_mat(rotation.x, rotation.y, rotation.z, rotation.w)
        try:
            points = _points_from_cloud(msg)
        except ValueError as error:
            self._warn(f"cloud:{frame}", f"bad cloud on {frame}: {error}")
            return None
        if points.size == 0:
            return None
        return points @ matrix.T + np.array(
            [translation.x, translation.y, translation.z])

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        ranges = np.full(self.bins, np.inf, dtype=np.float64)
        used = 0
        for label, msg, fallback_frame in (
                ("front", self._front, self.front_frame),
                ("rear", self._rear, self.rear_frame)):
            if msg is None:
                continue
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if abs(now_s - stamp) > self.max_cloud_age_s:
                self._warn(f"stale:{label}",
                           f"{label} cloud is {now_s - stamp:.2f}s old; skipping")
                continue
            points = self._transform(msg, fallback_frame)
            if points is None:
                continue
            mask = np.isfinite(points).all(axis=1)
            points = points[mask]
            if points.size == 0:
                continue
            mask = ((points[:, 2] >= self.height_min) &
                    (points[:, 2] <= self.height_max))
            points = points[mask]
            if points.size == 0:
                continue
            distance = np.hypot(points[:, 0], points[:, 1])
            mask = ((distance >= self.range_min) &
                    (distance <= self.range_max))
            points = points[mask]
            distance = distance[mask]
            if points.size == 0:
                continue
            index = np.floor(
                (np.arctan2(points[:, 1], points[:, 0]) + math.pi) /
                (2.0 * math.pi) * self.bins).astype(np.int64)
            index = np.clip(index, 0, self.bins - 1)
            np.minimum.at(ranges, index, distance)
            used += 1

        if used == 0:
            self._warn("no-data", "no fresh lidar cloud for /scanner/scan_nav")
            return

        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.base_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / self.bins
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(self.rate_hz, 1.0)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self._pub.publish(scan)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Merge the S10 front/rear lidars into /scanner/scan_nav.")
    parser.add_argument(
        "--manifest", default="robonix_manifest.yaml",
        help="stack manifest; its env: ROS_DOMAIN_ID/RMW are applied")
    args, ros_args = parser.parse_known_args(argv)
    _apply_ros_env(args.manifest)
    rclpy.init(args=ros_args or None)
    node = NavScanMerger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
