# Builds the charts for docs/explain.md (SVG + PNG) — re-run after changing a formula/value:
#   cd src && ../.venv/Scripts/python.exe ../docs/make_charts.py      (needs matplotlib: pip install matplotlib)
# Colours follow the dataviz skill's categorical order (blue, orange, aqua) — every line has a direct label
import json
import math
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))
import config  # noqa: E402
from motion import Mover, min_jerk  # noqa: E402
from singularity import analyze  # noqa: E402

OUT = Path(__file__).resolve().parent / "figures"
C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({
    "font.family": ["Segoe UI", "DejaVu Sans"], "font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 2, "figure.facecolor": "#fcfcfb",
    "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb",
})


def label(ax, x, y, text, color):
    """direct label on a line (ink-coloured text + a dot in the line's colour)"""
    ax.plot([x], [y], "o", color=color, ms=7, mec="#fcfcfb", mew=2)
    ax.annotate(text, (x, y), xytext=(6, 4), textcoords="offset points", color=INK, fontsize=9)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / f"{name}.svg")
    fig.savefig(OUT / f"{name}.png", dpi=110)
    plt.close(fig)


# ---------- 1) min-jerk: position/velocity/acceleration + 3 joints arriving together ----------
tau = np.linspace(0, 1, 201)
s = np.array([min_jerk(t) for t in tau])
ds = 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4
dds = 60 * tau - 180 * tau ** 2 + 120 * tau ** 3
fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.8))
a1.plot(tau, s, color=C1)
a1.plot(tau, ds / 1.875, color=C2)
a1.plot(tau, dds / 5.774, color=C3)
label(a1, 0.62, min_jerk(0.62), "position s", C1)
label(a1, 0.5, 1.0, "velocity (peak 1.875·Δ/T)", C2)
label(a1, 0.21, 1.0, "acceleration", C3)
a1.axhline(0, color=MUTED, lw=0.8)
a1.set_xlabel("time τ = t / T")
a1.set_title("s(τ) = 10τ³ − 15τ⁴ + 6τ⁵  (velocity = acceleration = 0 at both ends)", fontsize=10)
a1.set_ylim(-1.15, 1.25)
goal = [20, -10, 35]
names = ["base +20°", "shoulder −10°", "elbow +35°"]
T = 1.875 * max(abs(g) for g in goal) / 30
t = np.linspace(0, T, 200)
for g, n, c in zip(goal, names, (C1, C2, C3)):
    a2.plot(t, [g * min_jerk(x / T) for x in t], color=c)
    label(a2, T * 0.98, g, n, c)
a2.set_xlabel("time (s)")
a2.set_ylabel("angle (°)")
a2.set_title(f"goto_q: every joint shares s(τ) → all arrive at T = 1.875·35/30 = {T:.2f} s", fontsize=10)
a2.set_xlim(0, T * 1.35)
save(fig, "min_jerk")

# ---------- 2) jog: hold → ramp up, release → ease out, near a limit → brake ----------
dt = config.PERIOD_S
fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.6))
m = Mover([0] * 5)
m.jog_joint(2, +1)
ts, vs = [], []
for k in range(int(2.2 / dt)):
    if k == int(1.0 / dt):
        m.release()
    q0 = m.q[2]
    m.step(dt)
    ts.append(k * dt)
    vs.append((m.q[2] - q0) / dt)
a1.plot(ts, vs, color=C1)
a1.axvline(1.0, color=MUTED, lw=0.8, ls="--")
label(a1, 0.6, config.JOG_V, f"jog speed {config.JOG_V:.0f} °/s", C1)
a1.annotate("button released", (1.0, 3), xytext=(6, 0), textcoords="offset points", color=MUTED, fontsize=9)
a1.set_xlabel("time (s)")
a1.set_ylabel("joint speed (°/s)")
a1.set_title(f"hold: speed rises by a·dt (a = {config.JOG_A:.0f} °/s²) · release: falls by {config.JOG_STOP}·a·dt", fontsize=10)
lim = config.Q_MAX[2]
m = Mover([0, 0, lim - 12, 0, 0])
m.jog_joint(2, +1)
qs, vs = [], []
for _ in range(int(1.5 / dt)):
    q0 = m.q[2]
    m.step(dt)
    qs.append(lim - m.q[2])
    vs.append((m.q[2] - q0) / dt)
room = np.linspace(0, 12, 100)
a2.plot(room, np.minimum(config.JOG_V, np.sqrt(config.JOG_A * room)), color=C2, ls="--")
a2.plot(qs, vs, color=C1)
label(a2, 3, math.sqrt(config.JOG_A * 3), "braking ceiling v ≤ √(a·distance to limit)", C2)
k = int(len(qs) * 0.15)
label(a2, qs[k], vs[k], "actual jog (starting 12° away)", C1)
a2.invert_xaxis()
a2.set_xlabel("distance left to the limit (°)")
a2.set_ylabel("speed (°/s)")
a2.set_title("near a limit: brake to stop right at it, no impact", fontsize=10)
save(fig, "jog_ramp")

# ---------- 3) look-ahead: real-arm measurements before/after the fix ----------
fig, axs = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
for ax, (title, f) in zip(axs, (("before: current pose + fixed 60 ms", "jog.json"),
                                ("after: pose 80 ms ahead + 80 ms", "jog_after.json"))):
    a = np.array(json.load(open(OUT / f)), dtype=float)
    t, cmd, act = a[:, 0], a[:, 1], a[:, 2]
    g = np.arange(t[0], t[-1], 0.08)
    vc, va = np.diff(np.interp(g, t, cmd)) / 0.08, np.diff(np.interp(g, t, act)) / 0.08
    mv = np.abs(vc) > 20
    ax.plot(g[1:], va, color=C2, marker="o", ms=4)
    ax.axhline(0, color=MUTED, lw=0.8)
    label(ax, g[1:][np.argmax(va)], va.max(), "actual elbow speed", C2)
    ax.set_title(f"{title}\ncruising: {np.abs(va[mv]).mean():.1f} ± {np.abs(va[mv]).std():.1f} °/s", fontsize=10)
    ax.set_xlabel("time (s)")
axs[0].set_ylabel("°/s (averaged over 80 ms)")
save(fig, "lookahead_measured")

# ---------- 4) singularity: |det J| formula vs numerical ----------
fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.6))
q3s = np.linspace(-120, 120, 61)
num = [analyze([0, 40, q, 20, 0])["det_numeric"] / 1e6 for q in q3s]
form = [analyze([0, 40, q, 20, 0])["det_formula"] / 1e6 for q in q3s]
a1.plot(q3s, form, color=C1)
a1.plot(q3s, num, "o", color=C2, ms=4)
label(a1, 60, form[45], "formula r·a2·a3·|sin θ3|", C1)
label(a1, -115, num[2], "numerical Jacobian", C2)
a1.axvspan(-10, 10, color="#e34948", alpha=0.08)
a1.annotate("arm straight θ3 = 0", (0, 0.02), xytext=(35, 12), textcoords="offset points",
            color=INK, fontsize=9, arrowprops=dict(arrowstyle="->", color=MUTED))
k0 = int(np.argmin(form[:30]))
a1.annotate("tip swings back onto the base axis r = 0", (q3s[k0], 0.02), xytext=(-20, 150), textcoords="offset points",
            color=INK, fontsize=9, arrowprops=dict(arrowstyle="->", color=MUTED))
a1.set_xlabel("elbow angle q3 (°)  [shoulder 40°, wrist 20°]")
a1.set_ylabel("|det J| (×10⁶)")
a1.set_title("both methods give the same value = evidence the formula is right", fontsize=10)
q2s = np.linspace(-60, 60, 61)
rs = [analyze([0, q, 90, 60, 0])["r"] for q in q2s]
dets = [analyze([0, q, 90, 60, 0])["det_numeric"] / 1e6 for q in q2s]
a2.plot(rs, dets, "o", color=C2, ms=4)
a2.plot(rs, np.array(rs) * config.A2 * config.A3 / 1e6, color=C1)
label(a2, rs[45], rs[45] * config.A2 * config.A3 / 1e6, "formula r·a2·a3 (sin θ3 = 1)", C1)
label(a2, rs[20], dets[20], "numerical Jacobian", C2)
a2.axvspan(0, 20, color="#e34948", alpha=0.08)
a2.annotate("tip on the base axis r = 0", (0, 0), xytext=(10, 110), textcoords="offset points",
            color=INK, fontsize=9, arrowprops=dict(arrowstyle="->", color=MUTED))
a2.set_xlabel("r = tip distance from the base axis (mm)  [elbow 90°]")
a2.set_ylabel("|det J| (×10⁶)")
a2.set_title("det grows linearly with r", fontsize=10)
save(fig, "singularity_det")
print("ok")
