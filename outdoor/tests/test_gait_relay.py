import math
import pytest
from robot_outdoor.chassis_status import gait_relay_allowed


def test_gait_relay_accepts_both_navigation_gaits():
    for target in (0x3002, 0x3003):
        assert gait_relay_allowed(target, 17, .1, 1, .1, .1, 1)


@pytest.mark.parametrize("change", [
    {"target":0x1003}, {"state":0}, {"motion_age":2.}, {"motion_age":math.inf},
    {"mode":0}, {"mode_age":None}, {"mode_age":4.}, {"status_age":2.}, {"publishers":2},
])
def test_gait_relay_rejects_unverified_state(change):
    args=dict(target=0x3003,state=17,motion_age=.1,mode=1,mode_age=.1,status_age=.1,publishers=1)
    assert not gait_relay_allowed(**{**args,**change})
