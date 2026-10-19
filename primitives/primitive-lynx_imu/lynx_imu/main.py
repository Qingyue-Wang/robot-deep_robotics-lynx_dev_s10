#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""lynx_imu — DeepRobotics Lynx S10 IMU primitive.

The S10 publishes a standard ``sensor_msgs/Imu`` on ``/IMU`` (with a second,
lidar-internal IMU on ``/IMU_103``) but leaves ``header.frame_id`` empty.
rtabmap's ICP odometry cannot initialize without a real frame, so this
primitive re-stamps the stream with a configured frame id (default
``imu_link``), republishes it, and declares the re-stamped topic as the
``robonix/primitive/imu/imu`` data plane. The companion static transform
``base_link -> imu_link`` is published by ``scripts/start.sh``.
"""
from __future__ import annotations

import logging
import os

from robonix_api import Primitive, Ok, Deferred

from sensor_msgs.msg import Imu  # type: ignore

from rclpy.clock import Clock  # type: ignore

logging.basicConfig(
    level=os.environ.get("LYNX_LOG_LEVEL", "INFO"),
    format="[lynx_imu] %(message)s",
)
log = logging.getLogger("lynx_imu")

lynx_imu = Primitive(id="lynx_imu", namespace="robonix/primitive/imu")

_imu_pub = None
_frame_id = "imu_link"


@lynx_imu.on_init
def init(cfg):
    global _imu_pub, _frame_id
    topic = cfg.get("imu_topic", "/IMU")
    out_topic = cfg.get("out_topic", "/IMU/framed")
    qos = cfg.get("qos", "best_effort")
    timeout = float(cfg.get("sentinel_timeout_s", 15.0))
    _frame_id = str(cfg.get("frame_id", "imu_link"))

    # Confirm the robot is actually publishing before declaring the data plane.
    if not lynx_imu.wait_for_topic(topic, Imu, timeout):
        return Deferred(f"no sensor_msgs/Imu received on {topic} within {timeout}s")

    # Re-stamp and republish: the S10 leaves /IMU's frame_id empty, which makes
    # it unusable for TF lookups. Mirror the lynx_quadruped odom-forwarding
    # pattern (fill the missing frame, republish on the declared topic).
    _imu_pub = lynx_imu.create_publisher(
        "robonix/primitive/imu/imu", topic=out_topic, msg_type=Imu,
        qos=qos, declare=False)
    lynx_imu.declare_ros2_topic("robonix/primitive/imu/imu", out_topic, qos=qos)
    lynx_imu.create_subscription(
        "_imu_source", topic=topic, msg_type=Imu, callback=_on_imu,
        qos=qos, declare=False)

    log.info("lynx_imu ready: %s -> %s (frame_id=%s, qos=%s)",
             topic, out_topic, _frame_id, qos)
    return Ok()


def _on_imu(msg):
    # The S10 leaves /IMU's header.frame_id empty; rtabmap's ICP odometry needs
    # a real frame to look up base_link -> imu (the static TF in start.sh).
    msg.header.frame_id = _frame_id
    # Re-stamp to the HOST clock. Like /ODOM, the S10's /IMU carries the
    # robot's own clock, which lags this host. Because an IMU is wired, ICP
    # odometry runs with wait_imu_to_init=true and refuses to process a scan
    # until it has seen an IMU sample newer than it; a robot-clock stamp makes
    # the IMU look permanently stale, so it drops scans mid-motion
    # ("We didn't receive IMU newer than previous image/scan ... dropped!").
    # Matching the odom fix puts IMU and scan in the same time domain.
    msg.header.stamp = Clock().now().to_msg()
    _imu_pub.publish(msg)


@lynx_imu.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    lynx_imu.run()
