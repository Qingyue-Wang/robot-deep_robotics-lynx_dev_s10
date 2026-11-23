#!/usr/bin/env python3
"""Fuse the front and rear Airy scans into ONE cloud for FAST-LIO.

WHY
    FAST-LIO currently runs on the front Airy alone. The Airy is mounted
    VERTICALLY, so it observes a single vertical disc and the LIO only ever sees
    the body it is facing: while the robot rotates the registration has almost
    no horizontal structure, and the resulting /fastlio/odom drifts. The rear
    Airy looks the opposite way, so publishing both discs as one cloud in the
    FRONT lidar frame doubles the horizontal field of view without touching the
    calibrated LiDAR->IMU extrinsic (the cloud frame stays `airy_201`).

HOW
    * rear points are transformed `airy_202 -> airy_201` through TF
      (both lidars are static relative to base_link, so this is constant);
    * all fields of the driver cloud are preserved (x y z intensity ring
      timestamp) so FAST-LIO can still de-skew with the per-point time;
    * both drivers stamp `timestamp` in absolute host-clock seconds and their
      frames are within a few milliseconds of each other (measured), so the
      merged points stay on one time base;
    * the robot body is removed with the same 0.68 x 0.46 m (+0.15 m margin)
      envelope used by lynx_scan_nav, otherwise the legs seed the map.

TOPIC
    /scanner/cloud_lio  (frame = front lidar, sensor-data QoS, ~10 Hz)

Point it at FAST-LIO by setting `common.lid_topic` in
primitives/primitive-lynx_fastlio/config/robosense_airy_s10.yaml.
"""

from __future__ import annotations

import argparse
import array
import math
import sys
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import Buffer, TransformListener
except ImportError as error:  # pragma: no cover - ROS not sourced
    print(f"lio_cloud_merge needs ROS 2 (source /opt/ros/humble/setup.bash): {error}",
          file=sys.stderr)
    raise SystemExit(1)


def _dtype_from_fields(msg: PointCloud2) -> tuple[np.dtype, dict[str, str]]:
    """Build a numpy view over a PointCloud2 and map field name -> dtype name."""
    names, formats, offsets = [], [], []
    for field in msg.fields:
        if field.datatype == PointField.INT8:
            ctype = "i1"
        elif field.datatype == PointField.UINT8:
            ctype = "u1"
        elif field.datatype == PointField.INT16:
            ctype = "<i2"
        elif field.datatype == PointField.UINT16:
            ctype = "<u2"
        elif field.datatype == PointField.INT32:
            ctype = "<i4"
        elif field.datatype == PointField.UINT32:
            ctype = "<u4"
        elif field.datatype == PointField.FLOAT32:
            ctype = "<f4"
        elif field.datatype == PointField.FLOAT64:
            ctype = "<f8"
        else:
            raise ValueError(f"unsupported datatype {field.datatype} for {field.name}")
        if field.count != 1:
            raise ValueError(f"unsupported count {field.count} for {field.name}")
        names.append(field.name)
        formats.append(ctype)
        offsets.append(int(field.offset))
    dtype = np.dtype({"names": names, "formats": formats,
                      "offsets": offsets, "itemsize": int(msg.point_step)})
    return dtype, {name: name for name in names}


def _quat_to_mat(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


class LioCloudMerger(Node):
    """Publish front+rear Airy clouds as one PointCloud2 in the front frame."""

    def __init__(self) -> None:
        super().__init__("lio_cloud_merge")
        self.front_topic = str(self.declare_parameter(
            "front_topic", "/airy_201/points").value)
        self.rear_topic = str(self.declare_parameter(
            "rear_topic", "/airy_202/points").value)
        self.base_frame = str(self.declare_parameter(
            "base_frame", "base_link").value)
        self.front_frame = str(self.declare_parameter(
            "front_frame", "airy_201").value)
        self.rear_frame = str(self.declare_parameter(
            "rear_frame", "airy_202").value)
        self.output_topic = str(self.declare_parameter(
            "output_topic", "/scanner/cloud_lio").value)
        self.rate_hz = float(self.declare_parameter("publish_rate", 10.0).value)
        self.max_cloud_age_s = float(self.declare_parameter(
            "max_cloud_age_s", 0.60).value)
        self.max_points = int(self.declare_parameter(
            "max_points", 40000).value)
        self.range_max = float(self.declare_parameter(
            "range_max", 40.0).value)
        self.self_filter_length = float(self.declare_parameter(
            "self_filter_length", 0.68).value)
        self.self_filter_width = float(self.declare_parameter(
            "self_filter_width", 0.46).value)
        self.self_filter_margin = float(self.declare_parameter(
            "self_filter_margin", 0.15).value)
        self._half_length = max(
            0.0, self.self_filter_length / 2.0 + self.self_filter_margin)
        self._half_width = max(
            0.0, self.self_filter_width / 2.0 + self.self_filter_margin)

        self._front: PointCloud2 | None = None
        self._rear: PointCloud2 | None = None
        self._rear_mat: np.ndarray | None = None
        self._rear_mat_at = 0.0
        self._warn_at: dict[str, float] = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(PointCloud2, self.front_topic,
                                 self._on_front, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.rear_topic,
                                 self._on_rear, qos_profile_sensor_data)
        self._pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self._tick)
        self._stats_at = time.monotonic()
        self._stats_ticks = 0
        self._stats_pubs = 0
        self._stats_points = 0
        self._stats_ms = 0.0
        self.create_timer(5.0, self._log_stats)
        self.get_logger().info(
            "lio_cloud_merge ready: %s + %s -> %s (frame %s, %.1f Hz, "
            "self_filter %.2f x %.2f m)"
            % (self.front_topic, self.rear_topic, self.output_topic,
               self.front_frame, self.rate_hz,
               2.0 * self._half_length, 2.0 * self._half_width))

    # --- subscriptions -------------------------------------------------
    def _on_front(self, msg: PointCloud2) -> None:
        self._front = msg

    def _on_rear(self, msg: PointCloud2) -> None:
        self._rear = msg

    def _warn(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._warn_at.get(key, 0.0) < 10.0:
            return
        self._warn_at[key] = now
        self.get_logger().warn(message)

    # --- helpers -------------------------------------------------------
    def _lookup(self, target: str, source: str):
        try:
            transform = self._tf_buffer.lookup_transform(target, source, Time())
        except Exception as error:  # tf2 raises several types
            self._warn(f"tf:{source}",
                       f"cannot transform {source} -> {target}: {error}")
            return None
        return transform.transform

    def _log_stats(self) -> None:
        now = time.monotonic()
        span = now - self._stats_at
        if span <= 0.0:
            return
        self.get_logger().info(
            "lio merge: %.1f Hz published, %.1f ticks/s, avg %d pts, "
            "%.1f ms/tick"
            % (self._stats_pubs / span, self._stats_ticks / span,
               self._stats_points // max(1, self._stats_pubs),
               self._stats_ms / max(1, self._stats_ticks) * 1e3))
        self._stats_at = now
        self._stats_ticks = 0
        self._stats_pubs = 0
        self._stats_points = 0
        self._stats_ms = 0.0

    def _tick(self) -> None:
        start = time.monotonic()
        self._stats_ticks += 1
        try:
            self._tick_once()
        finally:
            self._stats_ms += time.monotonic() - start

    def _tick_once(self) -> None:
        front, rear = self._front, self._rear
        if front is None:
            self._warn("no-front", "no front Airy cloud yet")
            return
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        for label, msg in (("front", front), ("rear", rear)):
            if msg is None:
                continue
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if abs(now_s - stamp) > self.max_cloud_age_s:
                self._warn(f"stale:{label}",
                           f"{label} cloud is {now_s - stamp:.2f}s old; skipping")
                if label == "front":
                    return

        try:
            front_dtype, names = _dtype_from_fields(front)
        except ValueError as error:
            self._warn("dtype", f"bad front cloud: {error}")
            return
        for required in ("x", "y", "z", "timestamp"):
            if required not in names:
                self._warn("fields", f"front cloud has no '{required}' field")
                return

        front_points = np.frombuffer(bytes(front.data), dtype=front_dtype)
        if front_points.size == 0:
            return
        parts = [front_points]

        if rear is not None:
            try:
                rear_dtype, rear_names = _dtype_from_fields(rear)
            except ValueError as error:
                self._warn("dtype-rear", f"bad rear cloud: {error}")
                rear_dtype = None
            if rear_dtype is not None and all(
                    name in rear_names for name in ("x", "y", "z", "timestamp")):
                rear_points = np.frombuffer(bytes(rear.data), dtype=rear_dtype)
                if rear_points.size:
                    moved = self._rear_to_front(rear_points)
                    if moved is not None:
                        merged_rear = np.zeros(moved.shape[0], dtype=front_dtype)
                        merged_rear["x"] = moved[:, 0]
                        merged_rear["y"] = moved[:, 1]
                        merged_rear["z"] = moved[:, 2]
                        for name in names:
                            if name in ("x", "y", "z"):
                                continue
                            if name in rear_names:
                                merged_rear[name] = rear_points[name]
                        parts.append(merged_rear)

        points = parts[0] if len(parts) == 1 else np.concatenate(parts)
        points = self._self_filter(points)
        if points.size == 0:
            return
        if self.max_points > 0 and points.size > self.max_points:
            stride = int(math.ceil(points.size / float(self.max_points)))
            points = points[::stride][:self.max_points]

        cloud = PointCloud2()
        cloud.header.stamp = front.header.stamp
        cloud.header.frame_id = self.front_frame
        cloud.height = 1
        cloud.width = int(points.size)
        cloud.fields = front.fields
        cloud.is_bigendian = front.is_bigendian
        cloud.point_step = front.point_step
        cloud.row_step = front.point_step * cloud.width
        cloud.is_dense = True
        # rclpy converts a `bytes` payload element-by-element (234 ms for
        # 1.6 MB!); an array.array('B') is accepted as-is (0.4 ms).
        cloud.data = array.array("B", points.tobytes())
        self._pub.publish(cloud)
        self._stats_pubs += 1
        self._stats_points += int(points.size)

    def _rear_matrix(self):
        """(R, t) mapping rear points into the front lidar frame.

        Both lidars are bolted to the chassis, so the transform is constant:
        cache it and only refresh every 30 s (or when a lookup fails).
        """
        now = time.monotonic()
        if self._rear_mat is not None and now - self._rear_mat_at < 30.0:
            return self._rear_mat
        to_base = self._lookup(self.base_frame, self.rear_frame)
        base_from_front = self._lookup(self.base_frame, self.front_frame)
        if to_base is None or base_from_front is None:
            return self._rear_mat
        rot_rear = _quat_to_mat(to_base.rotation.x, to_base.rotation.y,
                                to_base.rotation.z, to_base.rotation.w)
        rot_front = _quat_to_mat(base_from_front.rotation.x,
                                 base_from_front.rotation.y,
                                 base_from_front.rotation.z,
                                 base_from_front.rotation.w)
        t_rear = np.array([to_base.translation.x, to_base.translation.y,
                           to_base.translation.z], dtype=np.float64)
        t_front = np.array([base_from_front.translation.x,
                            base_from_front.translation.y,
                            base_from_front.translation.z], dtype=np.float64)
        rot = rot_front.T @ rot_rear
        trans = rot_front.T @ (t_rear - t_front)
        self._rear_mat = (rot, trans, (rot_front, t_front))
        self._rear_mat_at = now
        return self._rear_mat

    def _rear_to_front(self, rear_points: np.ndarray) -> np.ndarray | None:
        """Transform rear points (airy_202) into the front lidar frame."""
        cached = self._rear_matrix()
        if cached is None:
            return None
        rot, trans, _ = cached
        xyz = np.empty((rear_points.size, 3), dtype=np.float32)
        xyz[:, 0] = rear_points["x"]
        xyz[:, 1] = rear_points["y"]
        xyz[:, 2] = rear_points["z"]
        return xyz @ rot.astype(np.float32).T + trans.astype(np.float32)

    def _self_filter(self, points: np.ndarray) -> np.ndarray:
        xyz = np.empty((points.size, 3), dtype=np.float32)
        xyz[:, 0] = points["x"]
        xyz[:, 1] = points["y"]
        xyz[:, 2] = points["z"]
        good = np.isfinite(xyz).all(axis=1)
        if self.range_max > 0.0:
            good &= np.einsum("ij,ij->i", xyz, xyz) <= self.range_max * self.range_max
        if self._half_length > 0.0 and self._half_width > 0.0:
            cached = self._rear_matrix()
            if cached is not None:
                rot, t = cached[2]
                in_base = xyz @ rot.astype(np.float32).T + t.astype(np.float32)
                inside = ((np.abs(in_base[:, 0]) <= self._half_length) &
                          (np.abs(in_base[:, 1]) <= self._half_width))
                good &= ~inside
        return points[good]


def _apply_ros_env(manifest_path: str) -> None:
    """Export ROS_DOMAIN_ID etc. from the deployment manifest, as start.sh does."""
    try:
        import yaml
    except ImportError:
        return
    try:
        with open(manifest_path, "r", encoding="utf-8") as handle:
            manifest = yaml.safe_load(handle) or {}
    except OSError:
        return
    env = manifest.get("env") or {}
    import os
    for key, value in env.items():
        if not isinstance(value, (str, int, float)):
            continue
        text = str(value)
        if "${" in text:
            continue
        os.environ.setdefault(key, text)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default=None,
                        help="deployment manifest used for ROS env defaults")
    args, _ = parser.parse_known_args(argv)
    if args.manifest:
        _apply_ros_env(args.manifest)
    rclpy.init()
    node = LioCloudMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
