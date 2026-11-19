# Mover tests — every test checks that no step jumps faster than the set speed (_run does it)
import math

import config
import numpy as np
import pytest
from kinematics import forward_kinematics
from motion import Mover

DT = config.PERIOD_S


def _run(m, seconds, v_check=None):
    """run the Mover for `seconds` → list of every pose · checks per-joint speed ≤ v·1.05"""
    v = v_check or m.v
    out, prev = [], list(m.q)
    for _ in range(int(seconds / DT) + 1):
        q = m.step(DT)
        for a, b in zip(prev, q):
            assert abs(b - a) <= v * DT * 1.05 + 1e-9, f"jumped {abs(b - a):.3f}° in {DT}s"
        out.append(list(q))
        prev = q
    return out


def _tip(q):
    f = forward_kinematics(q)
    return np.array([f.x, f.y, f.z])


# ---------- go to angles ----------

def test_goto_q_reaches_and_all_joints_arrive_together():
    m = Mover([0, 0, 0, 0, 0], v=10)
    assert m.goto_q([10, -5, 20, 0, 0]) == ""
    T = 1.875 * 20 / 10
    traj = _run(m, T + 0.2)
    assert traj[-1] == pytest.approx([10, -5, 20, 0, 0], abs=1e-6) and not m.busy
    # arrive together: at half time every joint has covered the same fraction
    mid = traj[int(T / 2 / DT)]
    frac = [mid[i] / g for i, g in enumerate([10, -5, 20]) if g]
    assert max(frac) - min(frac) < 1e-6


def test_goto_q_starts_and_ends_slow():
    m = Mover([0, 0, 0, 0, 0], v=10)
    m.goto_q([0, 20, 0, 0, 0])
    traj = _run(m, 4.0)
    steps = np.diff([q[1] for q in traj])
    assert steps[0] < 0.05 and steps[len(steps) // 2 - 5] > 0.3        # slow start, fast middle
    end = [s for s in steps if s > 1e-9][-1]
    assert end < 0.05                                                    # eases in before arriving


def test_goto_q_clamped_to_limits():
    m = Mover([0, 0, 0, 0, 0], v=30)
    m.goto_q([0, 500, 0, 0, 0])
    traj = _run(m, 40, v_check=30)
    assert traj[-1][1] == pytest.approx(config.Q_MAX[1])


def test_unused_joint_locked():
    m = Mover([0, 0, 0, 0, 0])
    m.goto_q([0, 0, 0, 0, 40])
    assert _run(m, 1)[-1][4] == 0.0


def test_new_goal_mid_move_no_jump():
    m = Mover([0, 0, 0, 0, 0], v=10)
    m.goto_q([0, 20, 0, 0, 0])
    _run(m, 1.0)
    m.goto_q([0, -10, 0, 0, 0])               # change of mind mid-move → no jump (_run checks)
    assert _run(m, 8)[-1][1] == pytest.approx(-10)


# ---------- straight line XYZ ----------

def test_goto_xyz_straight_line():
    q0 = [0, 10, 20, 20, 0]
    m = Mover(q0, v=10, v_cart=20)
    p0 = _tip(q0)
    goal = forward_kinematics([5, 15, 25, 15, 0])
    assert m.goto_xyz(goal.x, goal.y, goal.z, goal.phi) == ""
    traj = _run(m, 20)
    p1 = np.array([goal.x, goal.y, goal.z])
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    for q in traj:
        p = _tip(q) - p0
        assert np.linalg.norm(p - np.dot(p, d) * d) < 1.0            # < 1 mm off the straight line
    assert np.linalg.norm(_tip(traj[-1]) - p1) < 0.1 and not m.busy


def test_goto_xyz_unreachable_refused():
    m = Mover([0, 0, 0, 0, 0])
    msg = m.goto_xyz(500, 0, 0, 0)
    assert "Out of reach" in msg and not m.busy
    assert _run(m, 0.5)[-1] == [0, 0, 0, 0, 0]


# ---------- jog ----------

def test_jog_joint_ramps_up_and_down():
    m = Mover([0, 0, 0, 0, 0], v=10, a=30)
    m.jog_joint(1, +1)
    traj = _run(m, 1.0)
    steps = np.diff([q[1] for q in traj])
    assert steps[0] < steps[-1] and steps[-1] == pytest.approx(10 * DT, rel=0.01)
    m.release()
    after = _run(m, 1.0)
    moved = after[-1][1] - traj[-1][1]
    assert 0 < moved <= 10 ** 2 / (2 * 30 * config.JOG_STOP) + 0.5 and not m.busy   # eases (0.5·a) to a stop


def test_jog_stops_at_limit():
    start = config.Q_MAX[1] - 1
    m = Mover([0, start, 0, 0, 0], v=10, a=30)
    m.jog_joint(1, +1)
    traj = _run(m, 2.0)
    assert max(q[1] for q in traj) <= config.Q_MAX[1] + 1e-9


def test_jog_xyz_moves_tip_along_axis():
    q0 = [0, 10, 30, 30, 0]
    m = Mover(q0, v=30, v_cart=20)
    p0 = _tip(q0)
    m.jog_xyz(2, -1)                          # Z down
    traj = _run(m, 1.0, v_check=30)
    m.release()
    traj += _run(m, 1.0, v_check=30)
    p = _tip(traj[-1])
    assert p[2] < p0[2] - 5 and abs(p[0] - p0[0]) < 0.5 and abs(p[1] - p0[1]) < 0.5
    assert not m.busy


def test_jog_xyz_unreachable_stops():
    q0 = [0, 0, 0, 5, 0]                      # almost fully stretched → cannot push further up
    m = Mover(q0, v=30, v_cart=20)
    m.jog_xyz(2, +1)
    _run(m, 3.0, v_check=30)
    assert not m.busy and m.message


# ---------- slider / stop ----------

def test_follow_reaches_without_overshoot():
    m = Mover([0, 0, 0, 0, 0], v=10, a=30)
    m.follow([0, 0, 15, 0, 0])
    traj = _run(m, 4.0)
    assert max(q[2] for q in traj) <= 15 + 1e-6 and traj[-1][2] == pytest.approx(15, abs=1e-3)
    assert not m.busy


def test_follow_target_change_keeps_smooth():
    m = Mover([0, 0, 0, 0, 0], v=10, a=30)
    m.follow([0, 0, 15, 0, 0])
    _run(m, 0.8)
    m.follow([0, 0, -5, 0, 0])
    assert _run(m, 6)[-1][2] == pytest.approx(-5, abs=1e-3)


def test_stop_freezes():
    m = Mover([0, 0, 0, 0, 0])
    m.goto_q([0, 20, 0, 0, 0])
    _run(m, 1.0)
    m.stop()
    q = list(m.q)
    assert _run(m, 0.5)[-1] == q and not m.busy


def test_reset_ignores_nan():
    m = Mover([1, 2, 3, 4, 0])
    m.reset([5, math.nan, 7, 8, 0])
    assert m.q == [5, 2, 7, 8, 0]


def test_finished_flag_once():
    m = Mover([0, 0, 0, 0, 0])
    m.goto_q([0, 2, 0, 0, 0])
    _run(m, 2)
    assert m.pop_finished() == pytest.approx([0, 2, 0, 0, 0]) and m.pop_finished() is None


def test_jog_xyz_stops_before_singularity():
    m = Mover([0, 30, 20, 20, 0], v=150, a=600, v_cart=100)
    m.jog_xyz(0, +1)                          # push forward → arm stretches (θ3 → 0)
    _run(m, 5.0, v_check=150)
    assert not m.busy and "singularity" in m.message


def test_jog_xyz_can_leave_singularity():
    q0 = [0, 30, 3, 10, 0]                    # almost stretched (inside the warning zone)
    m = Mover(q0, v=150, a=600, v_cart=50)
    p0 = _tip(q0)
    m.jog_xyz(0, -1)                          # pull in = away from the singularity → must be allowed
    _run(m, 1.5, v_check=150)                 # near the singularity steps shrink to keep joints ≤ jog v → slow start
    assert _tip(m.q)[0] < p0[0] - 5


def test_jog_arrives_at_limit_slowly():
    """hitting a limit must be slow (was ~14 °/s) — last step before the limit ≤ 6 °/s (40 ms steps leave ~5.6)"""
    lim = config.Q_MAX[2]
    m = Mover([0, 0, lim - 12, 0, 0])
    m.jog_joint(2, +1)
    prev, last_v = m.q[2], None
    for _ in range(int(3 / DT)):
        q = m.step(DT)[2]
        if q >= lim - 1e-9:
            break
        last_v, prev = (q - prev) / DT, q
    assert last_v is not None and last_v <= 6.0


def _z(q):
    return forward_kinematics(q).z


@pytest.mark.parametrize("q0", [[0, 0, 0, 0, 0], [0, -0.5, 0.7, 0.5, 0], [0, 45, 0, 0, 0], [30, 40, 0, 0, 0]])
def test_jog_minus_z_from_stretched_moves_down(q0):
    """a straight arm (pointing up / stretched forward) must go down on Z− — φ may change, joints stay ≤ jog v"""
    m = Mover(q0)
    m.jog_xyz(2, -1)
    traj = _run(m, 2.0, v_check=config.JOG_V)
    assert _z(traj[-1]) < _z(q0) - 5, (m.message, traj[-1])


def test_jog_minus_z_at_horizontal_full_stretch_says_why():
    """fully stretched horizontally: going straight down (same x) needs a longer arm = true edge of reach → stops with a reason"""
    m = Mover([0, 90, 0, 0, 0])
    m.jog_xyz(2, -1)
    _run(m, 1.0, v_check=config.JOG_V)
    assert not m.busy and "fully stretched" in m.message


def test_goto_q_with_slower_speed():
    m = Mover([0, 0, 0, 0, 0], v=150)
    m.goto_q([0, 0, 40, 0, 0], v=20)
    traj = _run(m, 1.875 * 40 / 20 + 0.1, v_check=20)          # never above 20 °/s although the Mover is set to 150
    assert traj[-1][2] == pytest.approx(40) and not m.busy
