# Driver tests with a fake arm (tick() is called manually, no thread)
import config
import pytest
from driver import Driver
from link import SimLink
from motion import Mover

DT = config.PERIOD_S


class FakeArm(SimLink):
    """fake arm: shoulder sags by `sag` degrees (stops short) + adjustable voltage/online + records commands"""
    simulated = False

    def __init__(self, sag=0.0):
        super().__init__()
        self.sag, self.volt, self.is_online = sag, 7.6, True
        self.sent, self.stops = [], 0

    def send_q(self, q, time_ms):
        self.sent.append((list(q), time_ms))      # like DirectLink: jump guard compares to the pose sent, not the sagged angle
        self.q_actual = [float(v) for v in q]
        self.q_actual[1] -= self.sag
        return True

    def stop(self):
        self.stops += 1

    def online(self):
        return self.is_online


def _drive(d, seconds, t0=0.0):
    t = t0
    for _ in range(int(seconds / DT)):
        t += DT
        d.tick(now=t)
    return t


def test_streams_while_moving_and_stops_sending_when_idle():
    arm = FakeArm()
    d = Driver(arm, Mover([0] * 4, v=10))
    d.correct = False
    d.do(lambda m: m.goto_q([0, 5, 0, 0]))
    _drive(d, 2.0)
    n = len(arm.sent)
    assert n > 10 and all(t == config.STREAM_MS for _q, t in arm.sent)
    assert arm.sent[-1][0][1] == pytest.approx(5)
    _drive(d, 1.0, 2.0)
    assert len(arm.sent) == n                     # idle → nothing resent


def test_driver_stops_on_low_volt():
    arm = FakeArm()
    d = Driver(arm, Mover([0] * 4, v=10))
    d.do(lambda m: m.goto_q([0, 20, 0, 0]))
    _drive(d, 0.5)
    arm.volt = 5.5
    _drive(d, 0.2, 0.5)
    assert not d.mover.busy and arm.stops >= 1 and "voltage" in d.message


def test_driver_stops_when_offline():
    arm = FakeArm()
    d = Driver(arm, Mover([0] * 4, v=10))
    d.do(lambda m: m.goto_q([0, 20, 0, 0]))
    arm.is_online = False
    _drive(d, 0.2)
    assert not d.mover.busy and arm.sent == [] and "offline" in d.message


def test_refused_send_stops():
    arm = FakeArm()
    arm.send_q = lambda q, t: False
    d = Driver(arm, Mover([0] * 4, v=10))
    d.do(lambda m: m.goto_q([0, 20, 0, 0]))
    _drive(d, 0.2)
    assert not d.mover.busy


def test_correction_removes_sag():
    arm = FakeArm(sag=1.5)
    d = Driver(arm, Mover([0] * 4, v=10))
    d.do(lambda m: m.goto_q([0, 10, 0, 0]))
    _drive(d, 10.0)
    assert abs(10 - arm.q_actual[1]) <= config.CORR_TOL
    assert arm.sent[-1][0][1] - 10 <= config.CORR_MAX
    assert "corrected" in d.message


def test_correction_off():
    arm = FakeArm(sag=1.5)
    d = Driver(arm, Mover([0] * 4, v=10))
    d.correct = False
    d.do(lambda m: m.goto_q([0, 10, 0, 0]))
    _drive(d, 10.0)
    assert arm.q_actual[1] == pytest.approx(8.5)


def test_correction_gives_up_on_big_error():
    arm = FakeArm(sag=8.0)
    d = Driver(arm, Mover([0] * 4, v=10))
    d.do(lambda m: m.goto_q([0, 10, 0, 0]))
    _drive(d, 10.0)
    assert arm.sent[-1][0][1] == pytest.approx(10) and "Not correcting" in d.message


def test_new_command_cancels_correction():
    arm = FakeArm(sag=1.5)
    d = Driver(arm, Mover([0] * 4, v=10))
    d.do(lambda m: m.goto_q([0, 10, 0, 0]))
    t = _drive(d, 2.0)
    d.do(lambda m: m.follow([5, 10, 0, 0]))     # new command (slider) — follow does not correct when it ends
    _drive(d, 5.0, t)
    assert max(q[1] for q, _t in arm.sent) == pytest.approx(10)    # no shoulder correction happened


def test_emergency_stop_resets_to_actual():
    arm = FakeArm(sag=1.5)
    d = Driver(arm, Mover([0] * 4, v=10))
    d.do(lambda m: m.goto_q([0, 10, 0, 0]))
    _drive(d, 1.0)
    d.emergency_stop()
    assert not d.mover.busy and arm.stops == 1 and d.mover.q[1] == pytest.approx(arm.q_actual[1])


def test_simulated_link_needs_no_volt():
    d = Driver(SimLink(), Mover([0] * 4, v=10))
    d.do(lambda m: m.goto_q([0, 3, 0, 0]))
    _drive(d, 2.0)
    assert d.link.q_actual[1] == pytest.approx(3)


def test_peek_does_not_change_mover():
    m = Mover([0] * 4, v=30, a=90)
    m.jog_joint(2, +1)
    for _ in range(20):
        m.step(DT)
    q, vel = list(m.q), list(m.vel)
    ahead = m.peek(0.08)
    assert ahead[2] > q[2] and m.q == q and m.vel == vel


def test_servo_speed_steady_despite_tick_jitter():
    """ticks jitter 27–52 ms (measured) → the speed the servo must run = (goal sent − current pose) / STREAM_MS must stay constant"""
    import random
    rnd = random.Random(0)
    arm = FakeArm()
    d = Driver(arm, Mover([0] * 4, v=30, a=90))
    d.correct = False
    d.do(lambda m: m.jog_joint(2, +1))
    t, speeds = 0.0, []
    for k in range(60):
        t += rnd.uniform(0.027, 0.052)
        d.tick(now=t)
        if k > 15:                                   # cruising (acceleration done)
            speeds.append((arm.sent[-1][0][2] - d.mover.q[2]) / (config.STREAM_MS / 1000))
    assert min(speeds) == pytest.approx(30, rel=0.02) and max(speeds) == pytest.approx(30, rel=0.02)


def test_correction_after_jog_release():
    """release a jog button with a 1.5° sag → correction brings the actual angle to the stop pose (≤ 0.5°)"""
    arm = FakeArm(sag=1.5)
    d = Driver(arm, Mover([0] * 4, v=30, a=90))
    d.do(lambda m: m.jog_joint(1, +1))
    t = _drive(d, 0.5)
    d.do(lambda m: m.release())
    while d.mover.busy:                               # ease to a stop → remember the stop pose
        t = _drive(d, DT, t)
    q_stop = d.mover.q[1]
    assert q_stop - arm.q_actual[1] == pytest.approx(1.5)   # before correction: 1.5° sag
    _drive(d, 6.0, t)
    assert abs(arm.q_actual[1] - q_stop) <= config.CORR_TOL and "corrected" in d.message
