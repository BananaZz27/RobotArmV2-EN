# link.py tests with a simulated servo bus
import math
import time

import config
import pytest
from fake_bus import FakeBus
from link import DirectLink, SimLink, jump_too_big



def _wait(cond, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def fake():
    return FakeBus({j["id"]: j["center"] for j in config.JOINTS})


@pytest.fixture
def link(fake):
    lk = DirectLink(serial_obj=fake, period_s=0.01)
    assert _wait(lambda: lk.q_actual is not None)
    yield lk
    lk.close()


def test_connect_holds_then_torque_on(fake, link):
    assert all(fake.goals[j["id"]] == (j["center"], config.MIN_MOVE_MS) for j in config.JOINTS)
    assert all(fake.torque[j["id"]] for j in config.JOINTS)


def test_stream_small_step_sent_with_short_time(fake, link):
    assert link.send_q([0, 0.4, 0, 0], config.STREAM_MS) is True
    j = config.JOINTS[1]
    assert fake.goals[9] == (config.q_to_raw(j, 0.4), config.STREAM_MS)


def test_stream_big_jump_refused(fake, link):
    before = dict(fake.goals)
    assert link.send_q([0, 30, 0, 0], config.STREAM_MS) is False
    assert fake.goals == before and "jump" in link.message


def test_slow_big_move_allowed(fake, link):
    assert link.send_q([0, 10, 0, 0], 2000) is True


def test_nan_refused(fake, link):
    assert link.send_q([0, math.nan, 0, 0], 2000) is False


def test_removed_head_servo_never_touched(fake, link):
    link.send_q([0, 0.2, 0, 0], config.STREAM_MS)
    link.stop()
    link.torque(False)
    assert not any(sid == 1 for sid, _i, _p in fake.packets)


def test_torque_off_then_on_holds(fake, link):
    link.torque(False)
    assert not any(fake.torque[j["id"]] for j in config.JOINTS)
    fake.positions[9] = 400
    link.torque(True)
    assert fake.goals[9] == (400, config.MIN_MOVE_MS) and fake.torque[9]


def test_offline_when_silent(fake, link):
    fake.silent_ids = {j["id"] for j in config.JOINTS}
    assert _wait(lambda: not link.online(), 2.0)


def test_jump_guard():
    assert jump_too_big([0] * 4, [0, 25, 0, 0], 60)
    assert not jump_too_big([0] * 4, [0, 10, 0, 0], 60)
    assert not jump_too_big([0] * 4, [0, 50, 0, 0], 2000)


def test_simlink_follows():
    s = SimLink()
    assert s.send_q([1, 2, 3, 4], 2000) and s.q_actual == [1, 2, 3, 4]
    assert s.simulated and s.online()


def test_reads_temp_and_load(fake, link):
    assert _wait(lambda: 9 in link.servo)
    assert link.servo[9]["temp"] == 31 and link.servo[9]["load"] == 0
