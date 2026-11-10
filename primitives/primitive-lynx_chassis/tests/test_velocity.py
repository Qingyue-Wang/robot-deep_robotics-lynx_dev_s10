"""Exercise the actual command callback with in-memory ROS publishers."""

import importlib
import json
import sys
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def bridge(monkeypatch):
    class NavCmd:
        def __init__(self):
            self.data = SimpleNamespace(x_vel=0.0, y_vel=0.0, yaw_vel=0.0)

    class String:
        def __init__(self, data):
            self.data = data

    class Node:
        def __init__(self, name):
            pass

        def create_subscription(self, *args):
            pass

        def create_publisher(self, *args):
            samples = []
            return SimpleNamespace(samples=samples, publish=samples.append)

        def create_timer(self, *args):
            pass

    for name, attrs in {
        "rclpy": {},
        "rclpy.node": {"Node": Node},
        "geometry_msgs": {},
        "geometry_msgs.msg": {"Twist": object},
        "drdds": {},
        "drdds.msg": {"NavCmd": NavCmd, "MotionInfo": object},
        "std_msgs": {},
        "std_msgs.msg": {"String": String},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    name = "lynx_chassis.velocity_bridge"
    sys.modules.pop(name, None)
    module = importlib.import_module(name)
    monkeypatch.setattr(
        module,
        "threading",
        SimpleNamespace(
            Event=lambda: SimpleNamespace(),
            Thread=lambda **kwargs: SimpleNamespace(start=lambda: None),
        ),
    )
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: 100.0))
    monkeypatch.setenv("LYNX_ALLOW_MOTION", "1")
    node = module.Bridge()
    node.state, node.gait, node.mode = 17, 0x3002, 1
    node.motion_at = node.mode_at = 100.0
    try:
        yield module, node
    finally:
        sys.modules.pop(name, None)


def command(v, w):
    return SimpleNamespace(linear=SimpleNamespace(x=v), angular=SimpleNamespace(z=w))


@pytest.mark.parametrize(
    "v,w,expected", [(3.0, -4.0, (0.4, -0.4)), (-1.0, 0.2, (0.0, 0.2))]
)
def test_velocity_limits_and_timeout(bridge, v, w, expected):
    module, node = bridge
    node.receive(command(v, w))
    node.tick()
    output = node.publisher.samples[-1].data
    assert (output.x_vel, output.y_vel, output.yaw_vel) == (
        expected[0],
        0.0,
        expected[1],
    )
    module.time.monotonic = lambda: 100.36
    node.tick()
    output = node.publisher.samples[-1].data
    assert (output.x_vel, output.y_vel, output.yaw_vel) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    "change", [dict(state=0), dict(motion_at=98.0), dict(mode=0), dict(mode_at=96.0)]
)
def test_unverified_feedback_blocks_motion(bridge, change):
    _, node = bridge
    node.receive(command(0.2, 0.1))
    for key, value in change.items():
        setattr(node, key, value)
    node.tick()
    output = node.publisher.samples[-1].data
    assert (output.x_vel, output.y_vel, output.yaw_vel) == (0.0, 0.0, 0.0)
    assert json.loads(node.status_pub.samples[-1].data)["ready"] is False


def test_check_mode_has_no_velocity_publisher(bridge, monkeypatch):
    module, _ = bridge
    monkeypatch.setenv("LYNX_ALLOW_MOTION", "0")
    monkeypatch.setenv("LYNX_ALLOW_GAIT", "1")
    node = module.Bridge()
    assert node.publisher is None
    assert node.allow_gait is False
