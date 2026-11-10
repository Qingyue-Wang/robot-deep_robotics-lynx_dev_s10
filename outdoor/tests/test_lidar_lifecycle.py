"""A failed lidar sentinel must release only its own driver and TF processes."""

import importlib
from pathlib import Path
import sys
from types import ModuleType

from robonix_api import Deferred


def test_missing_cloud_reaps_owned_helpers(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(
        str(Path(__file__).resolve().parents[2] / "primitives/primitive-lynx_lidar")
    )
    for name, attrs in {
        "sensor_msgs": {},
        "sensor_msgs.msg": {"PointCloud2": object},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    lidar = importlib.import_module("lynx_lidar.main")
    actual_spawn = lidar.children.spawn
    started = []

    def spawn(argv, **kwargs):
        process = actual_spawn(
            [sys.executable, "-c", "import signal; signal.pause()"], **kwargs
        )
        started.append(process)
        return process

    monkeypatch.setattr(lidar.children, "spawn", spawn)
    monkeypatch.setattr(lidar.lynx_lidar, "wait_for_topic", lambda *a: False)
    try:
        result = lidar.init(
            {"lidar_executable": sys.executable, "run_dir": str(tmp_path)}
        )
        assert isinstance(result, Deferred), result
        assert len(started) == 2
        assert all(p.poll() is not None for p in started)
        assert lidar.processes == [] and lidar.device_lock is None
    finally:
        lidar.shutdown()
        sys.modules.pop("lynx_lidar.main", None)
