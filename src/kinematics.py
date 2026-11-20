# Kinematics of the 4-DOF arm (base, shoulder, elbow, wrist) — written to follow the course slides 1352205 (Forward / Inverse Kinematics)
# Units at every function boundary: angles in degrees (both q and θ), distances in mm
#   q = servo angle (0 = straight up)   θ = DH angle as in the slides   converted only in q_to_theta / theta_to_q
from dataclasses import dataclass

import numpy as np

from config import A2, A3, A4, D1, JOINTS, Q_MAX, Q_MIN

NEAR_AXIS_MM = 30.0   # tip closer than this to the base axis → allow the θ₁ + 180° solution (lean back) if nearer the current base angle

# ---------- DH table (FK slides: Link | aᵢ | αᵢ | dᵢ | θᵢ) ----------
#            (a,  alpha, d)
DH_TABLE = [
    (0.0, 90.0, D1),   # Link 1: base rotates about z₀, rises d1, α₁ = 90° tips the axis horizontal
    (A2, 0.0, 0.0),    # Link 2: shoulder → elbow, length a2
    (A3, 0.0, 0.0),    # Link 3: elbow → wrist, length a3
    (A4, 0.0, 0.0),    # Link 4: wrist → arm tip, length a4 (no head joint any more → x₄ points along the tip)
]


def dh_matrix(theta, d, a, alpha):
    """FK slides: Aᵢ = Rot(z,θᵢ) · Trans(z,dᵢ) · Trans(x,aᵢ) · Rot(x,αᵢ)  (angles in degrees)"""
    ct, st = np.cos(np.radians(theta)), np.sin(np.radians(theta))
    ca, sa = np.cos(np.radians(alpha)), np.sin(np.radians(alpha))
    # the full Aᵢ from the slides (the four matrices above multiplied out)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def q_to_theta(q):
    """servo angle q → DH angle θ (table 6.1 in the design) — the only place angle offsets live"""
    q1, q2, q3, q4 = q
    return [
        q1,          # θ₁ = q₁       base positive CCW = θ positive about z₀
        90.0 - q2,   # θ₂ = 90° − q₂ upright = 90° from horizontal, leaning forward = θ₂ decreases
        -q3,         # θ₃ = −q₃      elbow folding forward = clockwise in the r–s plane
        -q4,         # θ₄ = −q₄      wrist leaning forward = clockwise, like the elbow
    ]


def theta_to_q(theta):
    """DH angle θ → servo angle q (inverse of table 6.1)"""
    t1, t2, t3, t4 = theta
    return [t1, 90.0 - t2, -t3, -t4]


@dataclass
class FK:
    x: float          # tip position (mm) in the base frame
    y: float
    z: float
    phi: float        # tip angle vs horizontal in the arm plane (degrees): 90 = up, −90 = down
    T: np.ndarray     # T⁰₄ (4×4)
    frames: list      # [T⁰₀, T⁰₁, ..., T⁰₄] for drawing and the Jacobian


def wrap180(angle):
    """wrap an angle into (−180°, 180°]"""
    return -((-angle + 180.0) % 360.0 - 180.0)


def forward_kinematics(q):
    """FK slides: T⁰₄ = A₁ A₂ A₃ A₄"""
    theta = q_to_theta(q)
    T = np.eye(4)                       # T⁰₀ = identity (base frame)
    frames = [T]
    for (a, alpha, d), th in zip(DH_TABLE, theta):
        T = T @ dh_matrix(th, d, a, alpha)   # T⁰ᵢ = T⁰ᵢ₋₁ · Aᵢ
        frames.append(T)
    x, y, z = T[:3, 3]                  # tip position = 4th column of T⁰₄
    # tip angle φ: the three pitch axes are parallel → the angles simply add (direction of x₄ vs horizontal)
    phi = wrap180(theta[1] + theta[2] + theta[3])
    return FK(x, y, z, phi, T, frames)


# ======================================================================
# Inverse Kinematics — IK slides: geometric approach + kinematic decoupling
# A 4-DOF arm controls 4 values, so it takes (x, y, z) + φ (tip angle vs horizontal)
# Scope: θ₁ = atan2(y, x), one solution → for poses with wrist and tip in front of the base
#        (near the base axis the slides' second solution θ₁ + 180° is also allowed, see below)
# ======================================================================

@dataclass
class IK:
    ok: bool
    q: list | None    # 5 servo angles (degrees) when ok
    reason: str       # human-readable reason when not ok ("" when ok)


def check_limits(q):
    """None if every joint is within its limit, otherwise a message naming each joint that is out"""
    bad = [
        f"{j['name']} exceeds limit: {v:.1f}° (range {lo:.1f}…{hi:.1f}°)"
        for j, v, lo, hi in zip(JOINTS, q, Q_MIN, Q_MAX)
        if not lo <= v <= hi
    ]
    return " · ".join(bad) if bad else None


def inverse_kinematics(x, y, z, phi=-90.0, elbow_up=True, theta1_prev=0.0):
    """tip position (x, y, z) mm + tip angle φ (degrees) → servo angles q"""
    phi_r = np.radians(phi)

    # θ₁: the slides write θ₁ = Atan2(xc, yc) in Spong's convention (arguments = cos, sin)
    #     = numpy's np.arctan2(yc, xc) · the wrist centre lies in the same vertical plane as the tip, so x, y work directly
    r = np.sqrt(x * x + y * y)               # slides: r² = xc² + yc² (horizontal distance from the base axis)
    if x * x + y * y < 1e-9:
        theta1 = theta1_prev      # slides: singular configuration — θ₁ can be anything, so keep the previous one
    else:
        theta1 = np.degrees(np.arctan2(y, x))
        # near the base axis: if the tip is just behind the axis, atan2 jumps to ~180° (base out of limit — seen on the
        # real arm when jogging Z− from upright) → use the slides' second solution θ₁ + 180° with negative r
        # (lean back instead of turning the base) when it is closer to the current base angle
        flipped = wrap180(theta1 + 180.0)
        if r < NEAR_AXIS_MM and abs(wrap180(flipped - theta1_prev)) < abs(wrap180(theta1 - theta1_prev)):
            theta1, r = flipped, -r
    # kinematic decoupling (slides: oc = o − d·R·k): step back from the tip to the wrist by a4 along the tip direction φ
    rc = r - A4 * np.cos(phi_r)              # horizontal distance of the wrist
    s = z - A4 * np.sin(phi_r) - D1          # slides: s = zc − d1 (wrist height above the shoulder)

    # slides: law of cosines  D = cos θ₃ = (r² + s² − a2² − a3²) / (2·a2·a3)
    D = (rc * rc + s * s - A2 * A2 - A3 * A3) / (2 * A2 * A3)
    if abs(D) > 1 + 1e-9:
        dist = np.hypot(rc, s)
        return IK(False, None, f"Out of reach: wrist is {dist:.1f} mm from the shoulder "
                               f"but the links span {abs(A2 - A3):.1f}…{A2 + A3:.1f} mm")
    D = float(np.clip(D, -1.0, 1.0))         # trim rounding when the arm is exactly straight (D = ±1)

    # slides: θ₃ = Atan2(D, ±√(1 − D²)) → numpy: arctan2(±√(1 − D²), D)
    #   elbow up = minus sign (θ₃ < 0 = elbow folds forward = positive q₃)
    sign = -1.0 if elbow_up else 1.0
    theta3 = np.arctan2(sign * np.sqrt(1 - D * D), D)

    # slides: θ₂ = Atan2(r, s) − Atan2(a2 + a3·cos θ₃, a3·sin θ₃) → numpy swaps the argument order
    theta2 = np.arctan2(s, rc) - np.arctan2(A3 * np.sin(theta3), A2 + A3 * np.cos(theta3))

    # θ₄: tip angle φ = θ₂ + θ₃ + θ₄ (see forward_kinematics) → rearranged
    theta4 = phi_r - theta2 - theta3

    theta = [theta1, np.degrees(theta2), np.degrees(theta3), np.degrees(theta4)]
    q = [wrap180(v) for v in theta_to_q(theta)]
    reason = check_limits(q)
    if reason:
        return IK(False, None, reason)
    return IK(True, [float(v) for v in q], "")
