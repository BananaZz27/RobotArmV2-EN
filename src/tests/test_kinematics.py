# kinematics.py tests following acceptance criteria 7 of the design

import numpy as np
import pytest

import config
from kinematics import dh_matrix, forward_kinematics, inverse_kinematics, q_to_theta, theta_to_q



# ---------- FK ----------

def test_fk_all_zero_points_straight_up():
    fk = forward_kinematics([0, 0, 0, 0])
    assert (fk.x, fk.y, fk.z) == pytest.approx((0, 0, 382.0), abs=0.1)
    assert fk.phi == pytest.approx(90)


def test_fk_shoulder_30_leans_to_plus_x():
    fk = forward_kinematics([0, 30, 0, 0])
    assert (fk.x, fk.y, fk.z) == pytest.approx((142.65, 0, 343.78), abs=0.1)


def test_fk_hand_example_matches_doc():
    # hand-worked example in RobotControl/v2/docs/01-fk.md
    fk = forward_kinematics([30, 20, 40, 30])
    assert (fk.x, fk.y, fk.z) == pytest.approx((157.61, 90.99, 274.59), abs=0.01)
    assert fk.phi == pytest.approx(0, abs=1e-9)


def test_q_theta_table_6_1():
    assert q_to_theta([10, 20, 30, 40]) == pytest.approx([10, 70, -30, -40])
    assert theta_to_q(q_to_theta([10, 20, 30, 40])) == pytest.approx([10, 20, 30, 40])


def test_dh_matrix_pure_rotation_z():
    A = dh_matrix(90, 0, 0, 0)
    assert A[:3, :3] == pytest.approx(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]), abs=1e-12)


def test_frames_count_and_last_is_T():
    fk = forward_kinematics([5, 10, 15, 20])
    assert len(fk.frames) == 5 and np.allclose(fk.frames[-1], fk.T)


def test_q_limits_from_raw():
    # the q range comes from the raw limits via center/dir (changes with calibration, so computed from config)
    for i, j in enumerate(config.JOINTS):
        ends = sorted([config.raw_to_q(j, j["raw_min"]), config.raw_to_q(j, j["raw_max"])])
        assert (config.Q_MIN[i], config.Q_MAX[i]) == pytest.approx(ends)
        assert config.Q_MIN[i] < 0 < config.Q_MAX[i]          # q = 0 (upright) must be within limits


# ---------- IK ----------

def _front_poses(n, seed=0):
    """random q within limits with wrist and tip in front of the base (IK scope per the slides, θ₁ = atan2)"""
    rng = np.random.default_rng(seed)
    poses = []
    while len(poses) < n:
        q = [rng.uniform(lo, hi) for lo, hi in zip(config.Q_MIN, config.Q_MAX)]
        fk = forward_kinematics(q)
        c, s = np.cos(np.radians(q[0])), np.sin(np.radians(q[0]))
        r_tip = fk.x * c + fk.y * s
        r_wrist = r_tip - config.A4 * np.cos(np.radians(fk.phi))
        if r_tip > 1.0 and r_wrist > 1.0:
            poses.append(q)
    return poses


def test_ik_roundtrip_500_front_poses():
    for q in _front_poses(500):
        fk = forward_kinematics(q)
        ik = inverse_kinematics(fk.x, fk.y, fk.z, fk.phi, elbow_up=q[2] > 0, theta1_prev=q[0])   # like the app: pass the current base angle
        assert ik.ok, ik.reason
        fk2 = forward_kinematics(ik.q)
        assert np.linalg.norm([fk2.x - fk.x, fk2.y - fk.y, fk2.z - fk.z]) < 0.01


def test_ik_known_pose_from_spec():
    fk = forward_kinematics([10, 20, 40, 30])
    ik = inverse_kinematics(fk.x, fk.y, fk.z, fk.phi, elbow_up=True)
    assert ik.q == pytest.approx([10, 20, 40, 30], abs=1e-6)


def test_ik_practical_point_elbow_up():
    ik = inverse_kinematics(200, 0, 50, -90, elbow_up=True)
    assert ik.ok and ik.q == pytest.approx([0, 62.71, 64.54, 52.75], abs=0.01)


def test_ik_unreachable_reports_thai_reason():
    ik = inverse_kinematics(500, 0, 0, -90)
    assert not ik.ok and ik.q is None and "Out of reach" in ik.reason


def test_ik_over_limit_names_joint():
    ik = inverse_kinematics(150, 0, 100, 90, elbow_up=True)   # needs q4 ≈ −171.5°
    assert not ik.ok and "J4" in ik.reason


def test_ik_base_over_limit_behind():
    ik = inverse_kinematics(-150, 50, 100, -90)               # θ₁ ≈ 161.6° > 120°
    assert not ik.ok and "J1" in ik.reason


def test_ik_singular_above_base_keeps_theta1():
    ik = inverse_kinematics(0, 0, 382.0, 90, theta1_prev=25)  # exactly straight, D = 1
    assert ik.ok and ik.q[0] == pytest.approx(25) and not any(np.isnan(ik.q))
    assert ik.q[1:4] == pytest.approx([0, 0, 0], abs=1e-3)


@pytest.mark.parametrize("q0", [[0, -0.5, 0.7, 0.5], [-0.5, -1, 0, 0], [10, -3, 2, 1]])
def test_ik_tip_just_behind_axis_no_base_flip(q0):
    """tip just behind the base axis (the real arm's near-upright pose) → IK must not turn the base 180° (was: base exceeds limit 180°)"""
    f = forward_kinematics(q0)
    assert f.x < 0 or q0[0]
    ik = inverse_kinematics(f.x, f.y, f.z, f.phi, q0[2] >= 0, theta1_prev=q0[0])
    assert ik.ok, ik.reason
    assert abs(ik.q[0] - q0[0]) < 1e-6
    g = forward_kinematics(ik.q)
    assert (g.x, g.y, g.z) == pytest.approx((f.x, f.y, f.z), abs=1e-6)


def test_4_row_dh_same_tip_as_old_5_row_dh():
    """the 4-row DH table (a4 = 50, θ₄ = −q₄) gives the same tip as the old 5-row table (α₄ = 90°, θ₄ = 90° − q₄, d5 = 50, θ₅ = 0)"""
    from kinematics import dh_matrix
    old = [(0.0, 90.0, config.D1), (config.A2, 0.0, 0.0), (config.A3, 0.0, 0.0), (0.0, 90.0, 0.0), (0.0, 0.0, config.A4)]
    rng = np.random.default_rng(7)
    for _ in range(300):
        q = [rng.uniform(lo, hi) for lo, hi in zip(config.Q_MIN, config.Q_MAX)]
        T = np.eye(4)
        for (a, al, d), th in zip(old, [q[0], 90 - q[1], -q[2], 90 - q[3], 0.0]):
            T = T @ dh_matrix(th, d, a, al)
        fk = forward_kinematics(q)
        assert np.allclose(T[:3, 3], [fk.x, fk.y, fk.z], atol=1e-9)
        assert np.allclose(T[:3, 2], fk.T[:3, 0], atol=1e-9)        # tip direction: old z₅ = new x₄


def test_t04_closed_form_matches_code():
    """the T⁰₄ formula in docs/explain.md section 2.1 = the code's A1·A2·A3·A4 product, every pose"""
    c, s = (lambda d: np.cos(np.radians(d))), (lambda d: np.sin(np.radians(d)))
    rng = np.random.default_rng(3)
    for _ in range(300):
        q = [rng.uniform(lo, hi) for lo, hi in zip(config.Q_MIN, config.Q_MAX)]
        t1, t2, t3, t4 = q_to_theta(q)
        t234 = t2 + t3 + t4
        R = config.A2 * c(t2) + config.A3 * c(t2 + t3) + config.A4 * c(t234)
        Z = config.A2 * s(t2) + config.A3 * s(t2 + t3) + config.A4 * s(t234)
        T = np.array([[c(t1) * c(t234), -c(t1) * s(t234), s(t1), c(t1) * R],
                      [s(t1) * c(t234), -s(t1) * s(t234), -c(t1), s(t1) * R],
                      [s(t234), c(t234), 0, config.D1 + Z],
                      [0, 0, 0, 1]])
        assert np.allclose(T, forward_kinematics(q).T, atol=1e-9)


def test_t04_example_in_docs():
    """numeric example in docs/explain.md section 2.1: q = (30, 40, 40, 50) → tip (193.3, 111.6, 186.6), φ = −40°"""
    f = forward_kinematics([30, 40, 40, 50])
    assert [round(f.x, 1), round(f.y, 1), round(f.z, 1), round(f.phi, 1)] == [193.3, 111.6, 186.6, -40.0]
