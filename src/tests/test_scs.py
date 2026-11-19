# SCS15 packet builder/parser tests (scs.py) against simulated servos (fake_bus.py)
import config
import pytest
from fake_bus import FakeBus
from scs import ScsBus, make_packet


def test_make_packet_write_goal_by_hand():
    # write a goal to servo ID 1: raw 511 (0x01FF), time 2000 ms (0x07D0), speed 0
    # LEN = 7 params + 2 = 9 ; sum 1+9+3+42+1+255+7+208 = 526 → 526 & 0xFF = 14 → ~14 = 0xF1
    pkt = make_packet(1, 0x03, bytes([42, 0x01, 0xFF, 0x07, 0xD0, 0, 0]))
    assert pkt == bytes([0xFF, 0xFF, 0x01, 0x09, 0x03, 42, 0x01, 0xFF, 0x07, 0xD0, 0x00, 0x00, 0xF1])


def test_q_to_raw():
    j = {"dir": +1, "center": 511, "raw_min": 100, "raw_max": 940}
    assert config.q_to_raw(j, 0) == 511
    assert config.q_to_raw(j, 10) == round(511 + 10 * config.TICK_PER_DEG)
    assert config.q_to_raw(j, 500) == 940 and config.q_to_raw(j, -500) == 100
    k = {"dir": -1, "center": 556, "raw_min": 20, "raw_max": 1003}
    assert config.q_to_raw(k, 10) == round(556 - 10 * config.TICK_PER_DEG)
    assert config.raw_to_q(k, 556) == 0


def _bus(**kw):
    fake = FakeBus({1: 600, 9: 400}, **kw)
    return fake, ScsBus(fake)


def test_read_pos_volt():
    fake, bus = _bus()
    assert bus.read_pos_volt(1) == (600, pytest.approx(7.6))
    assert bus.read_pos_volt(9)[0] == 400


def test_write_pos_and_torque():
    fake, bus = _bus()
    assert bus.write_pos(9, 700, 2000) is True
    assert fake.goals[9] == (700, 2000)
    assert bus.torque(9, True) is True and fake.torque[9] is True


def test_ping():
    fake, bus = _bus()
    assert bus.ping(1) is True
    assert bus.ping(5) is False


def test_bad_checksum_rejected():
    fake, bus = _bus()
    fake.bad_checksum_ids = {1}
    assert bus.read_pos_volt(1) is None


def test_wrong_id_rejected():
    fake, bus = _bus()
    fake.wrong_id_map = {1: 9}                   # ID 1 replies as ID 9 ("ghost ID")
    assert bus.read_pos_volt(1) is None


def test_silent_returns_none():
    fake, bus = _bus()
    fake.silent_ids = {1}
    assert bus.read_pos_volt(1) is None
    assert bus.write_pos(1, 511, 2000) is False


def test_receive_skips_garbage():
    fake, bus = _bus()
    fake.garbage_prefix = b"\x00\xff\x12\xff"
    assert bus.read_pos_volt(1)[0] == 600


def test_detect_echo_on():
    fake, bus = _bus(echo=True)
    assert bus.detect_echo() is True
    assert bus.read_pos_volt(9)[0] == 400


def test_detect_echo_off():
    fake, bus = _bus()
    assert bus.detect_echo() is False
    assert bus.read_pos_volt(9)[0] == 400
