# tests for the IK explanation (ik_explain.py)
import pytest

import config
from ik_explain import TIP_MAX, diagnose, fmt_ranges, phi_ranges
from kinematics import forward_kinematics, inverse_kinematics


def test_reachable_point_gives_pose_and_no_reason():
    d = diagnose(110, 10, 350, 40)
    assert d.exact and d.headline.startswith("✅") and not d.reasons
    assert d.q == pytest.approx(inverse_kinematics(110, 10, 350, 40).q)
    assert len(d.options) == 4 and sum(o.ok for o in d.options) >= 1


def test_too_far_says_how_much_is_missing():
    d = diagnose(400, 0, 100, 0)
    assert d.q is None and d.headline.startswith("❌")
    assert d.tip_dist == pytest.approx(400 - 0, abs=0.1) and d.tip_dist > TIP_MAX
    assert "Too far" in d.reasons[0] and f"{400 - TIP_MAX:.1f}" in d.reasons[0]
    assert "short" in d.options[0].why()


def test_wrist_too_close():
    d = diagnose(30, 0, 60, -90)
    assert d.q is None
    assert "too close" in d.options[0].why()


def test_joint_limit_reason_names_joint_and_excess():
    d = diagnose(200, 0, 50, 0)                       # the wrist needs −77° but only reaches −73.1°
    assert d.q is not None and not d.exact and d.phi_used != 0
    why = d.options[0].why()
    assert config.JOINTS[3]["name"] in why and "over" in why
    assert "reachable φ" in d.hints[0]


def test_target_behind_base_blames_base_limit():
    d = diagnose(-150, 0, 200, 0)
    assert d.q is None
    assert config.JOINTS[0]["name"] in d.reasons[0] and "behind the base" in d.hints[0]


def test_below_base_warns():
    d = diagnose(150, 0, -20, -90)
    assert d.q is not None and any("below the base" in w for w in d.warnings)


def test_phi_ranges_contain_reachable_phi_and_match_ik():
    f = forward_kinematics([0, 30, 60, 20])
    ranges = phi_ranges(f.x, f.y, f.z)
    assert any(lo <= round(f.phi) <= hi for lo, hi in ranges)
    for lo, hi in ranges:                             # range edges must be reachable, 1° past the edge must not
        assert inverse_kinematics(f.x, f.y, f.z, lo).ok or inverse_kinematics(f.x, f.y, f.z, lo, False).ok
        if hi < 180:
            assert not (inverse_kinematics(f.x, f.y, f.z, hi + 1).ok or inverse_kinematics(f.x, f.y, f.z, hi + 1, False).ok)


def test_fmt_ranges():
    assert fmt_ranges([(-106, -4), (7, 7)]) == "-106…-4°, 7°"
