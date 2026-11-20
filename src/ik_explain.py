# Explain an IK result to the user: reachable or not, why not, and what to change (no GUI → testable)
#
# Follows the IK steps of the slides:
#   1) step back from the tip to the wrist by a4 along φ → the wrist must be |a2 − a3| … a2 + a3 from the
#      shoulder axis (otherwise the law of cosines has no solution)
#   2) try all 4 poses: facing / facing away (θ₁ and π + θ₁) × elbow up/down → which joint exceeds its limit, by how much
#   3) which φ ranges work at this position (with the poses the program really uses)
# The pose choice (choose) keeps the window's old rule: requested φ, elbow up first → elbow down → else the nearest φ that works
from collections import Counter
from dataclasses import dataclass, field

import config
from kinematics import REACH_MAX, REACH_MIN, IKGeo, solve_ik

TIP_MAX = REACH_MAX + config.A4            # farthest the tip can be from the shoulder axis (arm straight) = 285.3 mm
PHI_STEP = 1                               # search reachable φ in 1° steps


@dataclass
class Option:
    """one of the four poses at the requested φ"""
    name: str
    flip: bool
    elbow_up: bool
    geo: IKGeo
    bad: list = field(default_factory=list)   # [(joint i, angle needed, lo, hi)]

    @property
    def ok(self):
        return self.geo.q is not None and not self.bad

    def why(self):
        """short reason this pose cannot be used ("" = usable)"""
        g = self.geo
        if g.q is None:
            if g.dist > REACH_MAX:
                return (f"wrist is {g.dist:.1f} mm from the shoulder but links a2 + a3 reach at most {REACH_MAX:.1f} mm "
                        f"({g.dist - REACH_MAX:.1f} mm short)")
            return (f"wrist is {g.dist:.1f} mm from the shoulder but the links fold to at least {REACH_MIN:.1f} mm "
                    f"({REACH_MIN - g.dist:.1f} mm too close)")
        return " · ".join(
            f"{config.JOINTS[i]['name']} needs {v:.1f}° but can do {lo:.1f}…{hi:.1f}° ({_excess(v, lo, hi):.1f}° over)"
            for i, v, lo, hi in self.bad)


@dataclass
class Diagnosis:
    x: float
    y: float
    z: float
    phi: float
    q: list | None                         # pose that will be used (None = cannot reach at all)
    phi_used: float | None                 # φ of that pose (may differ from the requested φ)
    options: list                          # the 4 poses at the requested φ
    phi_ranges: list                       # [(lo, hi)] reachable φ at this position
    tip_dist: float                        # tip distance from the shoulder axis
    headline: str = ""
    reasons: list = field(default_factory=list)
    hints: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    @property
    def exact(self):
        """reachable with exactly the requested φ"""
        return self.q is not None and self.phi_used == self.phi


def _excess(v, lo, hi):
    return lo - v if v < lo else v - hi


def _limits(q):
    return [(i, v, lo, hi) for i, (v, lo, hi) in enumerate(zip(q, config.Q_MIN, config.Q_MAX)) if not lo <= v <= hi]


def options_at(x, y, z, phi, theta1_prev=0.0):
    """the slides' four poses: θ₁ = Atan2 (facing) / π + Atan2 (facing away) × elbow up/down"""
    out = []
    for flip, side in ((False, "facing"), (True, "facing away")):
        for up in (True, False):
            g = solve_ik(x, y, z, phi, up, theta1_prev, flip=flip)
            out.append(Option(f"{side} · elbow {'up' if up else 'down'}", flip, up, g, _limits(g.q) if g.q else []))
    return out


def _usable(x, y, z, phi, theta1_prev):
    """the pose the program really uses (same rule as inverse_kinematics): elbow up first, then elbow down"""
    for up in (True, False):
        g = solve_ik(x, y, z, phi, up, theta1_prev)
        if g.q is not None and not _limits(g.q):
            return g.q
    return None


def phi_ranges(x, y, z, theta1_prev=0.0):
    """reachable φ ranges (1° steps) → [(lo, hi), ...]"""
    ranges, start, prev = [], None, None
    for p in range(-180, 181, PHI_STEP):
        if _usable(x, y, z, p, theta1_prev) is not None:
            if start is None:
                start = p
            prev = p
        elif start is not None:
            ranges.append((start, prev))
            start = None
    if start is not None:
        ranges.append((start, prev))
    return ranges


def fmt_ranges(ranges):
    return ", ".join(f"{lo}°" if lo == hi else f"{lo}…{hi}°" for lo, hi in ranges)


def diagnose(x, y, z, phi, theta1_prev=0.0):
    r = (x * x + y * y) ** 0.5
    tip_dist = (r * r + (z - config.D1) ** 2) ** 0.5
    opts = options_at(x, y, z, phi, theta1_prev)
    ranges = phi_ranges(x, y, z, theta1_prev)
    q, used = _usable(x, y, z, phi, theta1_prev), phi
    if q is None and ranges:                # requested φ fails → nearest φ that works (as the window did before)
        for p in sorted(range(-180, 181, PHI_STEP), key=lambda p: abs(p - phi)):
            q = _usable(x, y, z, p, theta1_prev)
            if q is not None:
                used = p
                break
    d = Diagnosis(x, y, z, phi, q, used if q is not None else None, opts, ranges, tip_dist)
    front = [o for o in opts if not o.flip]

    if d.exact:
        d.headline = "✅ Reachable"
    elif q is not None:
        d.headline = f"⚠ φ = {phi:.0f}° is not reachable → using φ = {used}° instead (nearest reachable)"
        d.reasons.append(f"at φ = {phi:.0f}°: " + _front_reason(front))
        d.hints.append(f"reachable φ at this position: {fmt_ranges(ranges)}")
    else:
        d.headline = "❌ Cannot reach this point"
        if tip_dist > TIP_MAX:
            d.reasons.append(f"Too far: the tip is {tip_dist:.1f} mm from the shoulder axis but the straight arm reaches "
                             f"{TIP_MAX:.1f} mm (a2 + a3 + a4) — {tip_dist - TIP_MAX:.1f} mm short")
            d.hints.append(f"move the target towards the shoulder axis (0, 0, {config.D1:.1f}) by at least "
                           f"{tip_dist - TIP_MAX:.1f} mm")
        else:
            why, hint = _dominant_block(x, y, z, theta1_prev)
            d.reasons.append(f"no φ works — mostly because {why}")
            d.reasons.append(f"at φ = {phi:.0f}°: " + _front_reason(front))
            d.hints.append(hint)
        back = [o for o in opts if o.flip and o.ok]
        if back:
            d.hints.append(f"the {back[0].name} pose (base turned 180°, reaching over the top) works, "
                           "but the program does not use it on the real arm yet")

    if z < 0:
        d.warnings.append("target is below the base (z < 0) — the limits do not protect the table yet, the tip may hit it")
    return d


def _front_reason(front):
    """reason for the facing poses (elbow up/down) — merged into one when identical"""
    whys = [(o.name, o.why()) for o in front if not o.ok]
    if len({w for _, w in whys}) == 1:
        return whys[0][1]
    return " | ".join(f"{n}: {w}" for n, w in whys)


def _dominant_block(x, y, z, theta1_prev):
    """scan every φ and count what blocks the facing poses most → (reason, hint)"""
    count = Counter()
    for p in range(-180, 181, 5):
        for up in (True, False):
            g = solve_ik(x, y, z, p, up, theta1_prev, flip=False)
            if g.q is None:
                count["reach"] += 1
            else:
                for i, *_ in _limits(g.q):
                    count[i] += 1
    if not count:
        return "unknown", "try another target"
    what, _ = count.most_common(1)[0]
    if what == "reach":
        return (f"the wrist cannot be reached (links a2, a3 span {REACH_MIN:.1f}…{REACH_MAX:.1f} mm)",
                "the target is too close to the arm's body — move it away from the base or higher")
    j, lo, hi = config.JOINTS[what], config.Q_MIN[what], config.Q_MAX[what]
    why = f"{j['name']} exceeds its limit ({lo:.1f}…{hi:.1f}°)"
    if what == 0:
        return why, f"the target is behind the base, but the base only turns {lo:.1f}…{hi:.1f}° — try a target in front or to the side"
    return why, f"try a target that does not need {j['name']} beyond {lo:.1f}…{hi:.1f}°, e.g. closer or higher"
