#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""lynx_lidar — DeepRobotics Lynx S10 3D lidar primitive.

The S10 carries two RoboSense Airy lidars, driven by ``rslidar_sdk`` on the NX
(``~/rslidar_ws``). That SDK already publishes standard ``sensor_msgs/PointCloud2``:

    /airy_201/points  (frame ``airy_201``) — front lidar, RELIABLE, VERTICAL scan (default source)
    /airy_202/points  (frame ``airy_202``) — back lidar,  RELIABLE, HORIZONTAL 360-deg scan

The robot's own ``/LIDAR/POINTS`` is the LIO-fused cloud and only publishes
while the onboard perception stack runs; it is the alternative when available.

Because the two lidars live in different frames, this primitive exposes ONE
lidar (configurable) as ``robonix/primitive/lidar/lidar3d`` — merging two
frames into a single cloud needs a TF transform and is left to a fused source
(e.g. ``/LIDAR/POINTS``) rather than done here.
"""
from __future__ import annotations

import logging
import os

from robonix_api import Primitive, Ok, Deferred

from sensor_msgs.msg import PointCloud2  # type: ignore

logging.basicConfig(
    level=os.environ.get("LYNX_LOG_LEVEL", "INFO"),
    format="[lynx_lidar] %(message)s",
)
log = logging.getLogger("lynx_lidar")

lynx_lidar = Primitive(id="lynx_lidar", namespace="robonix/primitive/lidar")


@lynx_lidar.on_init
def init(cfg):
    topic = cfg.get("points_topic", "/airy_201/points")
    qos = cfg.get("qos", "reliable")
    timeout = float(cfg.get("sentinel_timeout_s", 15.0))

    if not lynx_lidar.wait_for_topic(topic, PointCloud2, timeout):
        return Deferred(
            f"no sensor_msgs/PointCloud2 received on {topic} within {timeout}s "
            "(is rslidar_sdk running and the lidar returning points?)"
        )

    lynx_lidar.declare_ros2_topic("robonix/primitive/lidar/lidar3d", topic, qos=qos)
    log.info("lynx_lidar ready: lidar3d=%s (qos=%s)", topic, qos)
    return Ok()


@lynx_lidar.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    lynx_lidar.run()
