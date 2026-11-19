# Provable singularity check for the 4-joint arm (base, shoulder, elbow, wrist)
#
# The arm controls 4 values p = (x, y, z, φ) with 4 joints → Jacobian J = ∂p/∂θ is 4×4 (square)
# singularity = det J = 0 : some tip direction cannot be produced by any joint motion
#
# Closed-form proof: split p into two layers
#   1) base: (x, y) = (r cos θ1, r sin θ1)  →  det ∂(x,y)/∂(θ1,r) = r
#   2) planar 3-joint arm (θ2, θ3, θ4) → (r, z, φ)  →  det = a2 · a3 · sin θ3   (planar 3R result, d5 cancels)
#   multiply:  |det J| = r · a2 · a3 · |sin θ3|
# ⇒ two kinds of singularity:
#   • r = 0       tip on the base axis       → turning the base does not move the tip (one direction lost)
#   • sin θ3 = 0  arm fully stretched/folded → cannot pull in / push out along the arm (one direction lost)
# Evidence: det_numeric (J from nudging each joint slightly) must equal det_formula — the tests check every pose
import math

import numpy as np

import config
from kinematics import forward_kinematics, q_to_theta

R_WARN_MM = 20.0          # tip closer than this to the base axis = near the base singularity
ELBOW_WARN_DEG = 10.0     # elbow closer than this to straight/fully folded = near the elbow singularity
STEP_DEG = 1e-3           # nudge size for the numerical Jacobian


def _pose(q):
    f = forward_kinematics(q)
    return np.array([f.x, f.y, f.z, math.radians(f.phi)])


def jacobian(q):
    """J 4×4 = ∂(x, y, z, φ)/∂(θ1..θ4) [mm/rad, rad/rad] by central difference"""
    J = np.zeros((4, 4))
    for k in range(4):
        dq = [0.0] * 5
        dq[k] = STEP_DEG
        plus = _pose([a + b for a, b in zip(q, dq)])
        minus = _pose([a - b for a, b in zip(q, dq)])
        d = plus - minus
        d[3] = (d[3] + math.pi) % (2 * math.pi) - math.pi        # φ wrapping across ±180°
        J[:, k] = d / math.radians(2 * STEP_DEG)
    return J


def analyze(q):
    """result: det both ways, r, θ3, smallest singular value, and warnings (empty list = safe)"""
    f = forward_kinematics(q)
    r = math.hypot(f.x, f.y)
    theta3 = q_to_theta(q)[2]
    J = jacobian(q)
    sv = np.linalg.svd(J[:3], compute_uv=False)                   # position part (3×4) — mm/rad
    out = {
        "det_numeric": float(abs(np.linalg.det(J))),
        "det_formula": r * config.A2 * config.A3 * abs(math.sin(math.radians(theta3))),
        "r": r,
        "theta3": theta3,
        "sigma_min": float(sv[-1]),
        "warnings": [],
    }
    if r < R_WARN_MM:
        out["warnings"].append(f"Tip almost on the base axis (r = {r:.1f} mm < {R_WARN_MM:.0f}) — turning the base does not move the tip")
    bend = min(abs(theta3) % 180, 180 - abs(theta3) % 180)
    if bend < ELBOW_WARN_DEG:
        out["warnings"].append(f"Arm almost fully stretched/folded (θ3 = {theta3:.1f}°) — cannot pull in / push out along the arm")
    return out


def getting_worse(q_old, q_new):
    """warning text when q_new is in a warning zone and closer to that singularity than q_old ("" = fine, moving away is allowed)"""
    a, b = analyze(q_old), analyze(q_new)
    for w in b["warnings"]:
        if "base axis" in w and b["r"] < a["r"] - 1e-6:          # unchanged (e.g. moving down the base axis) is not worse
            return w
        if "stretched" in w and abs(math.sin(math.radians(b["theta3"]))) < abs(math.sin(math.radians(a["theta3"]))) - 1e-9:
            return w
    return ""


def report(q):
    """multi-line text for the GUI: result + evidence"""
    a = analyze(q)
    head = "⚠ NEAR SINGULARITY\n" + "\n".join("• " + w for w in a["warnings"]) if a["warnings"] else "✅ Not near a singularity"
    return (f"{head}\n\n"
            f"|det J| (numerical)        = {a['det_numeric']:12.0f}\n"
            f"r·a2·a3·|sin θ3| (formula) = {a['det_formula']:12.0f}\n"
            f"   r = {a['r']:.1f} mm,  θ3 = {a['theta3']:.1f}°\n"
            f"σ_min (position)           = {a['sigma_min']:.1f} mm/rad  (≈ 0 = a direction is lost)")
