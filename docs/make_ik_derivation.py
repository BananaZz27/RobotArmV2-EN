# Builds figures/ik_derivation.svg: IK step by step + equations + numbers from a real example
#   every number is computed by kinematics.solve_ik / forward_kinematics (the same formulas the arm uses)
#   run: python docs/make_ik_derivation.py [en|th]   then convert to PNG with a browser
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "src"))
import config  # noqa: E402
from config import A2, A3, A4, D1  # noqa: E402
from kinematics import REACH_MAX, REACH_MIN, forward_kinematics, solve_ik  # noqa: E402

LANG = sys.argv[1] if len(sys.argv) > 1 else "en"
X, Y, Z, PHI = 150.0, 90.0, 120.0, -60.0        # example (elbow up)

T = {
    "th": dict(
        title="วิธีหา Inverse Kinematics ทีละขั้น",
        sub=f"ตัวอย่าง: เป้าปลายแขน (x, y, z) = ({X:.0f}, {Y:.0f}, {Z:.0f}) mm · มุมปลายแขน φ = {PHI:.0f}° · ศอกขึ้น  "
            f"(a₂ = {A2}, a₃ = {A3}, a₄ = {A4}, d₁ = {D1} mm)",
        s1="ขั้น 1 · หมุนฐาน θ₁ (มองจากบน)", s2="ขั้น 2 · ถอยจากปลายแขนไปข้อมือ (kinematic decoupling)",
        s3="ขั้น 3 · กฎ cosine หา θ₃ (ศอก)", s4="ขั้น 4 · หา θ₂ (ไหล่)",
        s5="ขั้น 5 · หา θ₄ (คอ) จากมุมปลายแขน", s6="ขั้น 6 · แปลง θ → q, ตรวจ limit, ตรวจกลับด้วย FK",
        top="มองจากบน", side="มองข้าง (ระนาบแขน)", flip="คำตอบที่ 2: θ₁ + 180°",
        n1="คำตอบที่ 2 (สไลด์ θ₁ = π + Atan2): หันหลัง + r ติดลบ — ใช้เมื่อเป้าใกล้แกนฐาน (r < 30 mm)",
        n1b="สไลด์เขียน Atan2(x, y) แบบ Spong = np.arctan2(y, x)",
        target="เป้า", wrist="ข้อมือ", shoulder="ไหล่", elbow="ศอก",
        n2="ปลายแขนชี้ทิศ φ เสมอ → ข้อมืออยู่ห่างปลายแขน a₄ ในทิศตรงข้าม",
        n2b="เหลือโจทย์แขน 2 ท่อน (a₂, a₃) ไปหาข้อมือ (rc, s)",
        up="ศอกขึ้น", down="ศอกลง", inner="มุมใน",
        n3="ไปได้เมื่อ |D| ≤ 1  ⇔  |a₂ − a₃| ≤ L ≤ a₂ + a₃",
        n3b="ศอกขึ้น = เครื่องหมาย − (θ₃ < 0) · ศอกลง = +",
        n4="α = มุมเส้นไหล่→ข้อมือ, β = มุมระหว่างท่อน a₂ กับเส้นนั้น",
        n4b="ศอกลง: θ₃ = +{t3d:.1f}° → β = +{bd:.1f}° → θ₂ = {t2d:.1f}°",
        n5="มุมของแต่ละท่อนเทียบแนวนอนบวกกันต่อ ๆ (แกน z₁ z₂ z₃ ขนานกัน):",
        n5b="ท่อน a₂: θ₂ · ท่อน a₃: θ₂ + θ₃ · ท่อน a₄: θ₂ + θ₃ + θ₄ = φ",
        hdr=["ข้อ", "θ (DH)", "q =", "q", "limit", ""],
        fk="ตรวจกลับ: ใส่ q เข้า FK (สูตร T⁰₄ ข้อ 2.1) ต้องได้เป้าเดิม",
        ok="ตรงกับเป้า ✓", n6="ถ้าข้อใดเกิน limit → ลองศอกลง → ยังไม่ได้ ลอง φ ใกล้เคียง (ข้อ 3.1)",
        deg="ทุกมุมเป็นองศา · θ = มุมตาม DH · q = มุม servo ที่หน้าต่างแสดง",
    ),
    "en": dict(
        title="Solving Inverse Kinematics step by step",
        sub=f"Example: tip target (x, y, z) = ({X:.0f}, {Y:.0f}, {Z:.0f}) mm · tip angle φ = {PHI:.0f}° · elbow up  "
            f"(a₂ = {A2}, a₃ = {A3}, a₄ = {A4}, d₁ = {D1} mm)",
        s1="Step 1 · base angle θ₁ (top view)", s2="Step 2 · step back from tip to wrist (kinematic decoupling)",
        s3="Step 3 · law of cosines → θ₃ (elbow)", s4="Step 4 · θ₂ (shoulder)",
        s5="Step 5 · θ₄ (wrist) from the tip angle", s6="Step 6 · θ → q, limit check, FK check",
        top="top view", side="side view (arm plane)", flip="2nd solution: θ₁ + 180°",
        n1="2nd solution (slides π + Atan2): face away, r < 0 — near the base axis (r < 30 mm)",
        n1b="the slides' Spong-style Atan2(x, y) = np.arctan2(y, x)",
        target="target", wrist="wrist", shoulder="shoulder", elbow="elbow",
        n2="the tip always points along φ → the wrist is a₄ behind the tip",
        n2b="what is left: a 2-link arm (a₂, a₃) reaching the wrist (rc, s)",
        up="elbow up", down="elbow down", inner="inner angle",
        n3="solvable when |D| ≤ 1  ⇔  |a₂ − a₃| ≤ L ≤ a₂ + a₃",
        n3b="elbow up = minus sign (θ₃ < 0) · elbow down = plus",
        n4="α = angle of the shoulder→wrist line, β = angle between link a₂ and that line",
        n4b="elbow down: θ₃ = +{t3d:.1f}° → β = +{bd:.1f}° → θ₂ = {t2d:.1f}°",
        n5="link angles from horizontal add up (axes z₁ z₂ z₃ are parallel):",
        n5b="link a₂: θ₂ · link a₃: θ₂ + θ₃ · link a₄: θ₂ + θ₃ + θ₄ = φ",
        hdr=["joint", "θ (DH)", "q =", "q", "limit", ""],
        fk="check: put q into FK (T⁰₄ formula, section 2.1) → must give the target back",
        ok="matches the target ✓", n6="a joint over its limit → try elbow down → still no: try a nearby φ (section 3.1)",
        deg="all angles in degrees · θ = DH angle · q = servo angle shown in the window",
    ),
}[LANG]
EN_NAMES = ["J1 base", "J2 shoulder", "J3 elbow", "J4 wrist"]

# ---------- compute ----------
g = solve_ik(X, Y, Z, PHI, True, 0.0)
q = g.q
t1, t2, t3, t4 = q[0], 90 - q[1], -q[2], -q[3]
r, rc, s, L = g.r, g.rc, g.s, g.dist
D = (rc * rc + s * s - A2 * A2 - A3 * A3) / (2 * A2 * A3)
alpha = math.degrees(math.atan2(s, rc))
t3r = math.radians(t3)
by, bx = A3 * math.sin(t3r), A2 + A3 * math.cos(t3r)
beta = math.degrees(math.atan2(by, bx))
t2_down = alpha + beta                      # elbow down: θ₃ → −θ₃, β → −β
fk = forward_kinematics(q).frames[-1][:3, 3]

c = math.radians
S = (0.0, D1)
E = (A2 * math.cos(c(t2)), D1 + A2 * math.sin(c(t2)))
W = (E[0] + A3 * math.cos(c(t2 + t3)), E[1] + A3 * math.sin(c(t2 + t3)))
P = (W[0] + A4 * math.cos(c(PHI)), W[1] + A4 * math.sin(c(PHI)))
E_dn = (A2 * math.cos(c(t2_down)), D1 + A2 * math.sin(c(t2_down)))

# ---------- SVG ----------
PW, PH, GAP, TOP = 470, 456, 18, 78
WID = GAP * 3 + PW * 2
HEI = TOP + 3 * (PH + GAP)
out = []
LINK, CORAL, PURPLE, TEAL, BLUE, GREY = "#888780", "#D85A30", "#7F77DD", "#0F6E56", "#185FA5", "#5F5E5A"


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text(x, y, t, cls="t", anchor="start", color=None):
    fill = f' fill="{color}"' if color else ""
    out.append(f'<text class="{cls}" x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}"{fill}>{esc(t)}</text>')


def line(a, b, color=GREY, w=1.0, dash=None, arrow=False, cap=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    m = ' marker-end="url(#arrow)"' if arrow else ""
    k = f' stroke-linecap="{cap}"' if cap else ""
    out.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{color}" '
               f'stroke-width="{w}"{d}{m}{k}/>')


def dot(p, r=4.0, fill="#FFFFFF", stroke=GREY, w=1.5):
    out.append(f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{w}"/>')


def arc(cn, R, a0, a1, color=CORAL, w=1.5, label=None, lr=None, cls="ts"):
    """arc around cn (screen coords) from world angle a0 → a1 degrees (CCW = positive)"""
    p0 = (cn[0] + R * math.cos(c(a0)), cn[1] - R * math.sin(c(a0)))
    p1 = (cn[0] + R * math.cos(c(a1)), cn[1] - R * math.sin(c(a1)))
    large = 1 if abs(a1 - a0) > 180 else 0
    sweep = 0 if a1 > a0 else 1
    out.append(f'<path d="M{p0[0]:.1f} {p0[1]:.1f} A{R} {R} 0 {large} {sweep} {p1[0]:.1f} {p1[1]:.1f}" fill="none" '
               f'stroke="{color}" stroke-width="{w}"/>')
    if label:
        m, lr = (a0 + a1) / 2, lr or R + 14
        text(cn[0] + lr * math.cos(c(m)), cn[1] - lr * math.sin(c(m)) + 4, label, cls, "middle", color)


def panel(i, title):
    col, row = i % 2, i // 2
    x0, y0 = GAP + col * (PW + GAP), TOP + row * (PH + GAP)
    out.append(f'<rect class="box" x="{x0}" y="{y0}" width="{PW}" height="{PH}" rx="8" stroke-width="0.8"/>')
    out.append(f'<rect class="c-purple" x="{x0}" y="{y0}" width="{PW}" height="30" rx="8" stroke-width="0.8"/>')
    text(x0 + 12, y0 + 20, title, "th", color="#3C3489")
    return x0, y0


def eqs(x0, y0, lines, start=328):
    y = y0 + start
    for ln in lines:
        cls, t = ln if isinstance(ln, tuple) else ("eq", ln)
        text(x0 + 14, y, t, cls)
        y += 21 if cls == "eq" else 18


K = 1.12                                     # px per mm in the side views


def side_map(x0, y0):
    ox, oy = x0 + 110, y0 + 300

    def M(p):
        return (ox + K * p[0], oy - K * p[1])
    return M


def side_base(M, x0):
    line((x0 + 20, M((0, 0))[1]), (x0 + PW - 20, M((0, 0))[1]), GREY, 0.6)
    a, b = M((-14, 12)), M((14, 0))
    out.append(f'<rect class="c-gray" x="{a[0]:.1f}" y="{a[1]:.1f}" width="{b[0]-a[0]:.1f}" height="{b[1]-a[1]:.1f}" '
               f'rx="2" stroke-width="0.5"/>')
    line(M((0, 12)), M(S), LINK, 6, cap="round")


def arm(M, pts, color=LINK, w=6, dash=None):
    for a, b in zip(pts, pts[1:]):
        line(M(a), M(b), color, w, dash, cap="round")
    for p in pts[1:-1]:
        dot(M(p), 3.5, stroke=color)


out.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{WID}" height="{HEI}" viewBox="0 0 {WID} {HEI}" role="img">'
           f'<title>{esc(T["title"])}</title><desc>{esc(T["sub"])}</desc>')
out.append("""<style>
text{font-family:'Sarabun','Noto Sans Thai','Leelawadee UI','Tahoma',sans-serif}
.t{font-size:14px;fill:#2C2C2A}.ts{font-size:12px;fill:#444441}.th{font-size:14px;font-weight:600}
.h1{font-size:20px;font-weight:600;fill:#2C2C2A}.eq{font-size:14px;fill:#2C2C2A;font-family:'Cambria Math','Cambria','Segoe UI',sans-serif}
.nt{font-size:12px;fill:#5F5E5A}.c-purple{fill:#EEEDFE;stroke:#534AB7}.c-gray{fill:#F1EFE8;stroke:#5F5E5A}
.box{fill:#FCFBF8;stroke:#B4B2A9}.ok{font-size:14px;fill:#27500A;font-weight:600}.bad{font-size:14px;fill:#A32D2D}
</style>""")
out.append(f'<rect width="{WID}" height="{HEI}" fill="#FFFFFF"/>')
out.append('<defs><marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" '
           'orient="auto-start-reverse"><path d="M2 1L8 5L2 9" fill="none" stroke="context-stroke" stroke-width="1.5" '
           'stroke-linecap="round" stroke-linejoin="round"/></marker></defs>')
text(GAP, 34, T["title"], "h1")
text(GAP, 58, T["sub"], "ts")

# ---------- step 1 ----------
x0, y0 = panel(0, T["s1"])
ox, oy = x0 + 200, y0 + 250


def TM(px, py):
    return (ox + 0.95 * px, oy - 0.95 * py)


line(TM(-170, 0), TM(205, 0), GREY, 1, arrow=True)
line(TM(0, -40), TM(0, 150), GREY, 1, arrow=True)
text(*TM(208, -14), "x", "ts")
text(*TM(6, 146), "y", "ts")
line(TM(0, 0), TM(X, Y), CORAL, 2.5, arrow=True)
line(TM(0, 0), TM(-X * 0.4, -Y * 0.4), CORAL, 1.2, "5 4", arrow=True)
text(*TM(-X * 0.4 - 4, -Y * 0.4 - 4), T["flip"], "nt", "end")
line(TM(X, 0), TM(X, Y), TEAL, 1, "4 3")
line(TM(0, Y), TM(X, Y), TEAL, 1, "4 3")
dot(TM(X, Y), 6, "#FFFFFF", CORAL, 2.5)
text(*TM(X + 10, Y + 8), f"{T['target']} ({X:.0f}, {Y:.0f})", "ts")
text(*TM(X / 2 - 18, Y / 2 + 14), f"r = {r:.1f}", "ts", "end", CORAL)
text(*TM(X, -16), f"x = {X:.0f}", "ts", "middle", TEAL)
text(*TM(-8, Y - 4), f"y = {Y:.0f}", "ts", "end", TEAL)
arc(TM(0, 0), 46, 0, t1, CORAL, 1.8, f"θ₁ = {t1:.1f}°", 78)
dot(TM(0, 0), 5, "#F1EFE8", GREY)
text(x0 + PW - 14, y0 + 50, T["top"], "nt", "end")
eqs(x0, y0, [
    f"r = √(x² + y²) = √({X:.0f}² + {Y:.0f}²) = {r:.1f} mm",
    f"θ₁ = atan2(y, x) = atan2({Y:.0f}, {X:.0f}) = {t1:.1f}°",
    ("nt", T["n1b"]),
    ("nt", T["n1"]),
])

# ---------- step 2 ----------
x0, y0 = panel(1, T["s2"])
M = side_map(x0, y0)
side_base(M, x0)
arm(M, [S, E, W, P], "#C9C7BE", 6)
line(M(S), M((W[0], S[1])), TEAL, 1, "4 3")
line(M((W[0], S[1])), M(W), TEAL, 1, "4 3")
line(M((P[0], 0)), M(P), GREY, 0.6, "2 3")
line(M(P), M(W), BLUE, 2.2, arrow=True)
line(M(P), M((P[0] + 40, P[1])), GREY, 0.8, "3 3")
arc(M(P), 30, 0, PHI, CORAL, 1.6)
text(M(P)[0] + 50, M(P)[1] + 4, f"φ = {PHI:.0f}°", "ts", color=CORAL)
dot(M(P), 6, "#FFFFFF", CORAL, 2.5)
dot(M(W), 5, "#FFFFFF", TEAL, 2.2)
dot(M(S), 5, "#F1EFE8", GREY)
text(M(P)[0] + 6, M(P)[1] + 46, T["target"], "ts")
text(M(W)[0] + 9, M(W)[1] - 8, T["wrist"], "ts", color=TEAL)
text((M(P)[0] + M(W)[0]) / 2 + 8, (M(P)[1] + M(W)[1]) / 2 - 6, "a₄", "ts", color=BLUE)
text((M(S)[0] + M(W)[0]) / 2, M(S)[1] + 16, f"rc = {rc:.1f}", "ts", "middle", TEAL)
text(M(W)[0] - 6, (M(S)[1] + M(W)[1]) / 2 + 4, f"s = {s:.1f}", "ts", "end", TEAL)
line(M((-8, 0)), M((-8, D1)), GREY, 0.8, arrow=True)
text(*M((-12, D1 / 2)), "d₁", "ts", "end")
text(x0 + PW - 14, y0 + 50, T["side"], "nt", "end")
eqs(x0, y0, [
    f"rc = r − a₄·cos φ = {r:.1f} − {A4:.0f}·cos({PHI:.0f}°) = {rc:.1f} mm",
    f"s = z − d₁ − a₄·sin φ = {Z:.0f} − {D1} − {A4:.0f}·sin({PHI:.0f}°) = {s:.1f} mm",
    f"L = √(rc² + s²) = {L:.1f} mm   (ข้อมือ ↔ ไหล่)" if LANG == "th" else f"L = √(rc² + s²) = {L:.1f} mm   (wrist ↔ shoulder)",
    ("nt", T["n2"]),
    ("nt", T["n2b"]),
])

# ---------- step 3 ----------
x0, y0 = panel(2, T["s3"])
M = side_map(x0, y0)
side_base(M, x0)
arm(M, [S, E_dn, W], "#B9B5E8", 4, "6 4")
out.append(f'<polygon points="{M(S)[0]:.1f},{M(S)[1]:.1f} {M(E)[0]:.1f},{M(E)[1]:.1f} {M(W)[0]:.1f},{M(W)[1]:.1f}" '
           f'fill="#EEEDFE" fill-opacity="0.7" stroke="none"/>')
arm(M, [S, E, W], LINK, 6)
line(M(S), M(W), PURPLE, 2, "5 4")
ext = (E[0] + 45 * math.cos(c(t2)), E[1] + 45 * math.sin(c(t2)))
line(M(E), M(ext), GREY, 0.8, "3 3")
arc(M(E), 26, t2, t2 + t3, CORAL, 1.8, f"θ₃ = {t3:.1f}°", 62)
arc(M(E), 14, t2 + t3, t2 - 180, PURPLE, 1.2)
text(M(E)[0] - 24, M(E)[1] + 30, f"{T['inner']} 180° − |θ₃|", "nt", "end", PURPLE)
dot(M(W), 5, "#FFFFFF", TEAL, 2.2)
dot(M(S), 5, "#F1EFE8", GREY)
mid = lambda a, b: ((M(a)[0] + M(b)[0]) / 2, (M(a)[1] + M(b)[1]) / 2)  # noqa: E731
text(mid(S, E)[0] - 12, mid(S, E)[1] - 4, f"a₂ = {A2:.0f}", "ts", "end")
text(mid(E, W)[0] + 10, mid(E, W)[1] - 8, f"a₃ = {A3}", "ts")
text(mid(S, W)[0] + 18, mid(S, W)[1] + 18, f"L = {L:.1f}", "ts", "middle", PURPLE)
text(M(E)[0] - 14, M(E)[1] - 4, T["up"], "nt", "end")
text(M(E_dn)[0] + 4, M(E_dn)[1] + 18, T["down"], "nt", "middle", PURPLE)
eqs(x0, y0, [
    "L² = a₂² + a₃² − 2a₂a₃·cos(180° − |θ₃|) = a₂² + a₃² + 2a₂a₃·cos θ₃",
    "D = cos θ₃ = (rc² + s² − a₂² − a₃²) / (2·a₂·a₃)",
    f"   = ({L:.1f}² − {A2:.0f}² − {A3}²) / (2·{A2:.0f}·{A3}) = {D:.3f}",
    f"θ₃ = atan2(−√(1 − D²), D) = {t3:.1f}°",
    ("nt", T["n3b"]),
    ("nt", T["n3"] + f"  =  {REACH_MIN:.1f} ≤ L ≤ {REACH_MAX:.1f} mm"),
])

# ---------- step 4 ----------
x0, y0 = panel(3, T["s4"])
M = side_map(x0, y0)
side_base(M, x0)
arm(M, [S, E, W], LINK, 6)
line(M(S), M(W), PURPLE, 2, "5 4")
line(M(S), M((S[0] + 200, S[1])), GREY, 0.8, "3 3")
arc(M(S), 100, 0, alpha, TEAL, 1.8, f"α = {alpha:.1f}°", 140)
arc(M(S), 70, alpha, t2, BLUE, 1.8, f"−β = {-beta:.1f}°", 112)
arc(M(S), 36, 0, t2, CORAL, 2.2)
text(M(S)[0] - 8, M(S)[1] - 44, f"θ₂ = {t2:.1f}°", "ts", "end", CORAL)
dot(M(W), 5, "#FFFFFF", TEAL, 2.2)
dot(M(S), 5, "#F1EFE8", GREY)
text(M(E)[0], M(E)[1] - 12, T["elbow"], "ts", "middle")
text(M(W)[0] + 9, M(W)[1] - 8, T["wrist"], "ts", color=TEAL)
eqs(x0, y0, [
    f"α = atan2(s, rc) = atan2({s:.1f}, {rc:.1f}) = {alpha:.1f}°",
    "β = atan2(a₃·sin θ₃, a₂ + a₃·cos θ₃)",
    f"   = atan2({by:.1f}, {bx:.1f}) = {beta:.1f}°",
    f"θ₂ = α − β = {alpha:.1f} − ({beta:.1f}) = {t2:.1f}°",
    ("nt", T["n4"]),
    ("nt", T["n4b"].format(t3d=-t3, bd=-beta, t2d=t2_down)),
])

# ---------- step 5 ----------
x0, y0 = panel(4, T["s5"])
M = side_map(x0, y0)
side_base(M, x0)
arm(M, [S, E, W, P], LINK, 6)
ext3 = (W[0] + 46 * math.cos(c(t2 + t3)), W[1] + 46 * math.sin(c(t2 + t3)))
line(M(W), M(ext3), GREY, 0.8, "3 3")
line(M(W), M((W[0] + 60, W[1])), GREY, 0.8, "3 3")
line(M(E), M((E[0] + 60, E[1])), GREY, 0.8, "3 3")
line(M(S), M((S[0] + 50, S[1])), GREY, 0.8, "3 3")
arc(M(S), 30, 0, t2, CORAL, 1.6, "θ₂", 44)
arc(M(E), 30, 0, t2 + t3, BLUE, 1.6)
text(M(E)[0] + 40, M(E)[1] - 8, f"θ₂ + θ₃ = {t2 + t3:.1f}°", "ts", color=BLUE)
arc(M(W), 40, t2 + t3, PHI, CORAL, 2.2, f"θ₄ = {t4:.1f}°", 82)
arc(M(W), 24, 0, PHI, TEAL, 1.4)
text(M(W)[0] + 28, M(W)[1] + 4, "φ", "ts", color=TEAL)
dot(M(P), 6, "#FFFFFF", CORAL, 2.5)
text(M(P)[0] - 12, M(P)[1] + 18, T["target"], "ts", "end")
eqs(x0, y0, [
    "φ = θ₂ + θ₃ + θ₄",
    f"θ₄ = φ − θ₂ − θ₃ = {PHI:.0f} − {t2:.1f} − ({t3:.1f}) = {t4:.1f}°",
    ("nt", T["n5"]),
    ("nt", T["n5b"]),
])

# ---------- step 6 ----------
x0, y0 = panel(5, T["s6"])
cols = [x0 + 14, x0 + 112, x0 + 182, x0 + 262, x0 + 322, x0 + 436]
y = y0 + 58
for cx, h in zip(cols, T["hdr"]):
    text(cx, y, h, "th", color="#3C3489")
line((x0 + 10, y + 7), (x0 + PW - 10, y + 7), GREY, 0.6)
thetas = [t1, t2, t3, t4]
forms = ["θ₁", "90° − θ₂", "−θ₃", "−θ₄"]
for i in range(4):
    y += 26
    name = config.JOINTS[i]["name"] if LANG == "th" else EN_NAMES[i]
    ok = config.Q_MIN[i] <= q[i] <= config.Q_MAX[i]
    for cx, v in zip(cols, [name, f"{thetas[i]:.1f}", forms[i], f"{q[i]:.1f}",
                            f"{config.Q_MIN[i]:.1f} … {config.Q_MAX[i]:.1f}"]):
        text(cx, y, v, "t")
    text(cols[5], y, "✓" if ok else "✗", "ok" if ok else "bad")
y += 34
text(x0 + 14, y, T["fk"], "nt")
c1, s1 = math.cos(c(t1)), math.sin(c(t1))
R = A2 * math.cos(c(t2)) + A3 * math.cos(c(t2 + t3)) + A4 * math.cos(c(PHI))
Zr = A2 * math.sin(c(t2)) + A3 * math.sin(c(t2 + t3)) + A4 * math.sin(c(PHI))
eqs(x0, y0, [
    "R = a₂c₂ + a₃c₂₃ + a₄c₂₃₄ = "
    f"{A2 * math.cos(c(t2)):.1f} + {A3 * math.cos(c(t2 + t3)):.1f} + {A4 * math.cos(c(PHI)):.1f} = {R:.1f}",
    "Z = a₂s₂ + a₃s₂₃ + a₄s₂₃₄ = "
    f"{A2 * math.sin(c(t2)):.1f} + ({A3 * math.sin(c(t2 + t3)):.1f}) + ({A4 * math.sin(c(PHI)):.1f}) = {Zr:.1f}",
    f"x = c₁R = {fk[0]:.1f} · y = s₁R = {fk[1]:.1f} · z = d₁ + Z = {fk[2]:.1f}",
    ("ok", T["ok"]),
    ("nt", T["n6"]),
    ("nt", T["deg"]),
], start=y - y0 + 26)

out.append("</svg>")
dst = ROOT / "figures" / "ik_derivation.svg"
dst.write_text("\n".join(out), encoding="utf-8")
print(dst, f"q = {[round(v, 1) for v in q]}", f"FK = {[round(float(v), 1) for v in fk]}")
