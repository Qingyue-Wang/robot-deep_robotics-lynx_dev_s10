"""Real owned subprocess teardown; no ROS or device access."""

import os
import sys

import pytest

from lynx_chassis import main
from robonix_api import Err, Ok
from robonix_api.spawn import SpawnRegistry


def test_shutdown_reaps_only_owned_helpers_and_lock_rejects_second_owner(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("ROS_DOMAIN_ID", "229")
    declared = []
    monkeypatch.setattr(
        main.chassis, "declare_ros2_topic", lambda *a, **kw: declared.append(a)
    )
    actual_spawn = main.children.spawn

    def spawn(argv, **kwargs):
        assert kwargs["env"]["LYNX_ALLOW_MOTION"] == "0"
        assert kwargs["env"]["LYNX_ALLOW_GAIT"] == "0"
        return actual_spawn(
            [sys.executable, "-c", "import signal; signal.pause()"],
            log_path=kwargs["log_path"],
        )

    monkeypatch.setattr(main.children, "spawn", spawn)
    unrelated = SpawnRegistry()
    other = unrelated.spawn([sys.executable, "-c", "import signal; signal.pause()"])
    try:
        result = main.init({"robot_domain_id": 230, "run_dir": str(tmp_path)})
        assert isinstance(result, Ok), result
        assert main.processes == []
        assert isinstance(main.activate(), Ok)
        processes = list(main.processes)
        assert len(processes) == 2
        assert all(p.poll() is None for p in processes)
        assert [item[0] for item in declared] == ["robonix/primitive/chassis/twist_in"]
        # Verify the OS lock using a distinct open, not a Python flag.
        import fcntl

        with open(
            f"/tmp/robonix-lynx-chassis-{os.getuid()}-230.lock", "a"
        ) as duplicate:
            with pytest.raises(BlockingIOError):
                fcntl.flock(duplicate, fcntl.LOCK_EX | fcntl.LOCK_NB)
        main.shutdown()
        assert all(p.poll() is not None for p in processes)
        assert other.poll() is None
        assert main.device_lock is None
        assert isinstance(main.activate(), Ok)
        main.shutdown()
    finally:
        main.shutdown()
        unrelated.shutdown_all()


@pytest.mark.parametrize(
    "config",
    [
        {"allow_motion": "false"},
        {"allow_gait": True},
        {"robot_domain_id": 229},
    ],
)
def test_invalid_arming_never_starts_a_helper(monkeypatch, config):
    monkeypatch.setenv("ROS_DOMAIN_ID", "229")
    monkeypatch.setattr(
        main.children, "spawn", lambda *a, **k: pytest.fail("must not start helpers")
    )
    assert isinstance(main.init(config), Err)
    assert main.processes == []
