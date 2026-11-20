# A ⇄ B loop (auto_cycle.py) — pure state machine + saving/loading the point file
import pytest
import config
from auto_cycle import DEMO, MAX_POINTS, AutoCycle, load_dwells, load_points, save_points
from kinematics import check_limits, forward_kinematics
from motion import Mover

A, B = [0.0, 10.0, 20.0, 30.0], [40.0, 0.0, 60.0, -10.0]


def _arrive(c, now, q):
    """pretend the arm reached q (no longer busy) → next command (if any)"""
    return c.update(now, False, q)


def test_needs_both_points():
    with pytest.raises(ValueError):
        AutoCycle([A, None]).start(0)


def test_sequence_and_cycle_count():
    c = AutoCycle([A, B], dwell=1.0, cycles=2)
    assert c.start(0) == A                          # go to A first (setup — not counted)
    assert c.update(0.5, True, [5, 5, 5, 5]) is None  # still moving
    assert _arrive(c, 1, A) is None and c.phase == "dwell" and c.done == 0
    assert c.update(1.5, False, A) is None           # 1 s wait not over yet
    assert c.update(2.0, False, A) == B              # wait over → go to B
    assert _arrive(c, 3, B) is None
    assert c.update(4, False, B) == A
    assert _arrive(c, 5, A) is None and c.done == 1 and c.last_cycle_s == pytest.approx(4)   # A(1) → A(5)
    assert c.update(6, False, A) == B
    _arrive(c, 7, B)
    assert c.update(8, False, B) == A
    assert _arrive(c, 9, A) is None
    assert c.done == 2 and c.phase == "done" and not c.running   # 2 cycles → stops by itself at A


def test_unlimited_cycles_keep_going():
    c = AutoCycle([A, B], dwell=0, cycles=0)
    c.start(0)
    t = 0
    for _ in range(20):
        t += 1
        _arrive(c, t, c.points[c.index])
        t += 0.01
        assert c.update(t, False, c.points[c.index]) is not None
    assert c.running and c.done == 9                # 10 arrivals at A − the first (setup) = 9 cycles


def test_stops_when_arm_halts_before_point():
    c = AutoCycle([A, B])
    c.start(0)
    assert _arrive(c, 1, [0.0, 10.0, 25.0, 30.0]) is None     # not busy but 5° away from A = stopped on the way (voltage drop/STOP)
    assert not c.running and c.phase == "stopped" and "A" in c.reason


def test_stop_and_status():
    c = AutoCycle([A, B], dwell=2)
    assert "ready" in c.status(0)
    c.start(0)
    assert "moving to point A" in c.status(0)
    _arrive(c, 1, A)
    assert "waiting at point A" in c.status(1.5)
    c.stop("test")
    assert c.update(10, False, A) is None and "test" in c.status(10)


def test_points_file_roundtrip(tmp_path):
    f = tmp_path / "pts.json"
    assert load_points(f) == [None, None]            # no file yet
    save_points([A, None], f)
    assert load_points(f) == [A, None]
    f.write_text("{bad json", encoding="utf-8")
    assert load_points(f) == [None, None]            # broken file → start fresh, no crash


def test_three_point_sequence():
    C = [10.0, 30.0, 40.0, 0.0]
    c = AutoCycle([A, B, C], dwell=0, cycles=1)
    assert c.start(0) == A
    _arrive(c, 1, A)
    assert c.update(1.1, False, A) == B
    _arrive(c, 2, B)
    assert c.update(2.1, False, B) == C                               # A → B → C → A
    _arrive(c, 3, C)
    assert c.update(3.1, False, C) == A
    _arrive(c, 4, A)
    assert c.phase == "done" and c.done == 1


def test_points_file_old_format_and_many_points(tmp_path):
    f = tmp_path / "pts.json"
    f.write_text('{"A": [1, 2, 3, 4], "B": null}', encoding="utf-8")   # old file format (only A, B)
    assert load_points(f) == [[1.0, 2.0, 3.0, 4.0], None]
    save_points([A, B, A], f)
    assert load_points(f) == [A, B, A]


def test_approaching_only_until_first_point():
    c = AutoCycle([A, B], dwell=0)
    assert not c.approaching
    c.start(0)
    assert c.approaching                              # first run to A
    _arrive(c, 1, A)
    assert not c.approaching


def test_dwell_per_point():
    c = AutoCycle([A, B], dwell=9, dwells=[0.0, 2.0])
    c.start(0)
    _arrive(c, 1, A)
    assert c.update(1.0, False, A) == B               # A waits 0 s → moves on at once
    _arrive(c, 2, B)
    assert c.update(3.9, False, B) is None            # B waits 2 s, not over yet
    assert c.update(4.0, False, B) == A
    assert AutoCycle([A, B], dwell=0.7).dwell_at(1) == 0.7   # not set → default


def test_dwells_file_roundtrip(tmp_path):
    f = tmp_path / "pts.json"
    assert load_dwells(f) == []
    save_points([A, B], f, [0.5, 2.0])
    assert load_points(f) == [A, B] and load_dwells(f) == [0.5, 2.0]
    save_points([A, B], f)                            # file without waits → []
    assert load_dwells(f) == []


def test_demo_poses_safe():
    assert len(DEMO) == 7 and len(DEMO) <= MAX_POINTS
    pts = [q for _, q, _ in DEMO]
    for q in pts:
        assert not check_limits(q)
        assert min(min(v - lo, hi - v) for v, lo, hi in zip(q, config.Q_MIN, config.Q_MAX)) >= 10   # away from limits
        assert forward_kinematics(q).z >= 80                               # clear of the table
    for a, b in zip(pts, pts[1:] + pts[:1]):                               # every segment (incl. G → A) runs straight
        m = Mover(a)
        assert m.goto_line_q(b) == "" and m.message == ""
