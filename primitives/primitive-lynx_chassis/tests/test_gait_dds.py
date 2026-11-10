"""Synthetic DDS domains 229/230 only. Never publishes to the robot's domain 0."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("drdds")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from drdds.msg import MotionInfo, Gait
from std_msgs.msg import String, UInt32


ROOT = Path(__file__).resolve().parents[1]


def test_native_gait_relay_in_isolated_synthetic_domains(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    private_domain, native_domain = 229, 230
    assert private_domain not in (0, 80) and native_domain not in (0, 80)
    env = {
        **os.environ,
        "ROS_DOMAIN_ID": str(private_domain),
        "LYNX_ROBOT_DOMAIN_ID": str(native_domain),
        "ROS_LOCALHOST_ONLY": "1",
        "LYNX_ALLOW_MOTION": "1",
        "LYNX_ALLOW_GAIT": "1",
    }
    log = (tmp_path / "synthetic-gait-bridge.log").open("w")
    child = subprocess.Popen(
        [sys.executable, "-m", "lynx_chassis.domain_bridge"],
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    contexts = []
    nodes = []
    executors = []
    seen = []
    try:
        for domain in (private_domain, native_domain):
            context = Context()
            rclpy.init(args=[], context=context, domain_id=domain)
            contexts.append(context)
            node = rclpy.create_node(
                "synthetic_gait_test_" + str(domain), context=context
            )
            nodes.append(node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            executors.append(executor)
        private, native = nodes
        status = private.create_publisher(String, "/outdoor/chassis_status", 10)
        request = private.create_publisher(UInt32, "/outdoor/gait_request", 10)
        motion = native.create_publisher(MotionInfo, "/MOTION_INFO", 10)
        native.create_subscription(
            Gait, "/GAIT", lambda msg: seen.append(msg.data.gait), 10
        )
        info = MotionInfo()
        info.data.motion_state.state = 17
        info.data.gait_state.gait = 0x3002
        deadline = time.monotonic() + 7.0
        while not seen and time.monotonic() < deadline:
            assert child.poll() is None, "Synthetic relay exited"
            motion.publish(info)
            status.publish(
                String(data=json.dumps({"usage_mode": 1, "usage_mode_age_s": 0.0}))
            )
            request.publish(UInt32(data=0x3003))
            for executor in executors:
                executor.spin_once(timeout_sec=0.02)
        assert seen and set(seen) == {0x3003}, {
            "native_motion_subscribers": native.count_subscribers("/MOTION_INFO"),
            "private_gait_subscribers": private.count_subscribers(
                "/outdoor/gait_request"
            ),
            "private_status_subscribers": private.count_subscribers(
                "/outdoor/chassis_status"
            ),
            "native_gait_publishers": native.count_publishers("/GAIT"),
        }
        assert native.count_publishers("/MOTION_STATE") == 0
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=3)
        for executor in executors:
            executor.shutdown()
        for node in nodes:
            node.destroy_node()
        for context in contexts:
            rclpy.try_shutdown(context=context)
        log.close()
