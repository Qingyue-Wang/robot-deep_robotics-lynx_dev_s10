"""Robonix lifecycle for the S10 command translator and native-domain watchdog."""

import fcntl
import logging
import os
from pathlib import Path
import signal
import sys
import threading

from robonix_api import Err, Ok, Primitive
from robonix_api.spawn import SpawnRegistry

chassis = Primitive(
    id=os.environ.get("RBNX_INSTANCE_NAME", "lynx_chassis"),
    namespace="robonix/primitive/chassis",
)
children = SpawnRegistry()
processes = []
stopping = threading.Event()
monitor = None
device_lock = None
settings = None


def watch():
    while not stopping.wait(0.2):
        if any(process.poll() is not None for process in processes):
            logging.error("Chassis helper exited; stopping this primitive")
            os.kill(os.getpid(), signal.SIGTERM)
            return


@chassis.on_deactivate
@chassis.on_shutdown
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


@chassis.on_init
def init(config):
    global settings
    if processes:
        return Err("Deactivate the chassis primitive before changing its configuration")
    settings = None
    try:
        motion = config.get("allow_motion", False)
        gait = config.get("allow_gait", False)
        if type(motion) is not bool or type(gait) is not bool:
            raise ValueError("allow_motion and allow_gait must be boolean")
        if gait and not motion:
            raise ValueError("Gait output requires motion output")
        native = int(config.get("robot_domain_id", 0))
        private = int(os.environ.get("ROS_DOMAIN_ID", "80"))
        if not 0 <= native <= 232 or not 0 <= private <= 232 or native == private:
            raise ValueError("Robot and service ROS domains must be valid and distinct")
        host = str(config.get("robot_ip", "10.21.33.103"))
        port = int(config.get("robot_port", 30004))
        if not host or not 1 <= port <= 65535:
            raise ValueError("Invalid robot address")
        chassis.declare_ros2_topic(
            "robonix/primitive/chassis/twist_in", "/cmd_vel", qos="reliable"
        )
        if gait:
            chassis.declare_ros2_topic(
                "robonix/primitive/chassis/gait_in",
                "/outdoor/gait_request",
                qos="reliable",
            )
        settings = {
            "native": native,
            "run_dir": config.get("run_dir", "rbnx-build/chassis"),
            "env": {
                "LYNX_ALLOW_MOTION": "1" if motion else "0",
                "LYNX_ALLOW_GAIT": "1" if gait else "0",
                "ROS_DOMAIN_ID": str(private),
                "LYNX_ROBOT_DOMAIN_ID": str(native),
                "LYNX_ROBOT_IP": host,
                "LYNX_ROBOT_PORT": str(port),
            },
        }
        return Ok()
    except Exception as error:
        return Err(str(error))


@chassis.on_activate
def activate():
    global monitor, device_lock
    if settings is None:
        return Err("Initialize the chassis configuration first")
    if processes:
        return Ok()
    try:
        device_lock = open(
            f"/tmp/robonix-lynx-chassis-{os.getuid()}-{settings['native']}.lock", "a"
        )
        fcntl.flock(device_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run = Path(settings["run_dir"])
        stopping.clear()
        # Separate processes preserve the native zero-command watchdog if translation fails.
        for module in ("domain_bridge", "velocity_bridge"):
            processes.append(
                children.spawn(
                    [sys.executable, "-m", "lynx_chassis." + module],
                    env=settings["env"],
                    log_path=run / (module + ".log"),
                )
            )
        monitor = threading.Thread(target=watch, daemon=True)
        monitor.start()
        return Ok()
    except Exception as error:
        shutdown()
        return Err(str(error))


if __name__ == "__main__":
    chassis.run()
