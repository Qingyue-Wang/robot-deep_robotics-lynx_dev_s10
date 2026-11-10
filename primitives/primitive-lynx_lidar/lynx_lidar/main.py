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
import fcntl
from pathlib import Path
import signal
import threading

from robonix_api import Primitive, Ok, Err, Deferred
from robonix_api.spawn import SpawnRegistry

from sensor_msgs.msg import PointCloud2  # type: ignore

logging.basicConfig(
    level=os.environ.get("LYNX_LOG_LEVEL", "INFO"),
    format="[lynx_lidar] %(message)s",
)
log = logging.getLogger("lynx_lidar")

lynx_lidar = Primitive(
    id=os.environ.get("RBNX_INSTANCE_NAME", "lynx_lidar"),
    namespace="robonix/primitive/lidar",
)
children = SpawnRegistry()
processes = []
stopping = threading.Event()
monitor = None
device_lock = None


def watch():
    while not stopping.wait(0.2):
        if any(process.poll() is not None for process in processes):
            log.error("Lidar helper exited; stopping this primitive")
            os.kill(os.getpid(), signal.SIGTERM)
            return


@lynx_lidar.on_init
def init(cfg):
    global monitor, device_lock
    if processes:
        return Err("Restart the lidar primitive before changing its configuration")
    topic = cfg.get("points_topic", "/airy_201/points")
    qos = cfg.get("qos", "reliable")
    timeout = float(cfg.get("sentinel_timeout_s", 15.0))

    try:
        managed = cfg.get("managed_driver", True)
        if type(managed) is not bool:
            raise ValueError("managed_driver must be boolean")
        stopping.clear()
        if managed:
            workspace = Path(
                os.environ.get("RSLIDAR_WS", str(Path.home() / "rslidar_ws"))
            )
            executable = cfg.get(
                "lidar_executable",
                str(workspace / "install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node"),
            )
            if not Path(executable).is_file():
                raise ValueError("Missing lidar executable: " + executable)
            device_lock = open(f"/tmp/robonix-lynx-lidar-{os.getuid()}.lock", "a")
            fcntl.flock(device_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            run = Path(cfg.get("run_dir", "rbnx-build/lidar"))
            command = [executable]
            if cfg.get("lidar_config"):
                command += ["--ros-args", "-p", "config_path:=" + cfg["lidar_config"]]
            processes.append(children.spawn(command, log_path=run / "driver.log"))
            processes.append(
                children.spawn(
                    [
                        "ros2",
                        "run",
                        "tf2_ros",
                        "static_transform_publisher",
                        "--x",
                        "0.22341",
                        "--y",
                        "0",
                        "--z",
                        "-0.0001",
                        "--roll",
                        "3.14159",
                        "--pitch",
                        "-1.5708",
                        "--yaw",
                        "0",
                        "--frame-id",
                        "base_link",
                        "--child-frame-id",
                        "airy_201",
                        "--ros-args",
                        "-r",
                        "__node:=lynx_lidar_tf_front",
                    ],
                    log_path=run / "tf.log",
                )
            )
        if not lynx_lidar.wait_for_topic(topic, PointCloud2, timeout):
            shutdown()
            return Deferred(
                f"No PointCloud2 received on {topic} within {timeout}s; check lidar power/UDP"
            )
        if any(process.poll() is not None for process in processes):
            raise RuntimeError("Lidar helper exited; inspect lidar logs")
        lynx_lidar.declare_ros2_topic("robonix/primitive/lidar/lidar3d", topic, qos=qos)
        monitor = threading.Thread(target=watch, daemon=True)
        monitor.start()
        log.info("lynx_lidar ready: lidar3d=%s (qos=%s)", topic, qos)
        return Ok()
    except Exception as error:
        shutdown()
        return Err(str(error))


@lynx_lidar.on_shutdown
def shutdown():
    global monitor, device_lock
    stopping.set()
    if monitor is not None:
        monitor.join(timeout=1)
        monitor = None
    children.shutdown_all()
    processes.clear()
    if device_lock is not None:
        device_lock.close()
        device_lock = None
    return Ok()


if __name__ == "__main__":
    lynx_lidar.run()
