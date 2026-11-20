# Side view (arm plane r–z) in the IK tab: draws why a target is reachable or not
#   green ring = where the wrist can be (|a2 − a3| … a2 + a3 from the shoulder axis) · dashed circle = farthest tip a2 + a3 + a4
#   target → step back a4 along φ = wrist · wrist outside the ring = out of reach (red line shows the gap)
#   orange = pose that will be used · red dashed = pose that solves but hits a limit (red circle on that joint)
import math

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import QWidget

import config
from kinematics import REACH_MAX, REACH_MIN, forward_kinematics
from ik_explain import TIP_MAX

ORANGE, RED, GREEN, GREY = QColor("#ff8c42"), QColor("#d1453b"), QColor(46, 160, 120), QColor("#8a9499")
JOINT_AT = {0: 0, 1: 1, 2: 2, 3: 3}          # joint i turns at frame origin i (J1 base, J2 shoulder, J3 elbow, J4 wrist)


class IKSketch(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.d = None
        self.setMinimumHeight(180)

    def set_diagnosis(self, d):
        self.d = d
        self.update()

    # ---------- coordinates ----------
    def _setup(self):
        d = self.d
        self.u = (d.x, d.y) if math.hypot(d.x, d.y) > 1e-6 else (1.0, 0.0)   # arm-plane direction (facing the target)
        n = math.hypot(*self.u)
        self.u = (self.u[0] / n, self.u[1] / n)
        r = math.hypot(d.x, d.y)
        lo_x, hi_x = min(-TIP_MAX * 0.75, r - 40), max(TIP_MAX + 20, r + 40)
        lo_z, hi_z = min(-40, d.z - 40), max(config.D1 + TIP_MAX + 10, d.z + 40)
        m = 14
        s = min((self.width() - 2 * m) / (hi_x - lo_x), (self.height() - 2 * m) / (hi_z - lo_z))
        self.s = s
        self.ox = m + (self.width() - 2 * m - s * (hi_x - lo_x)) / 2 - s * lo_x
        self.oy = self.height() - m - (self.height() - 2 * m - s * (hi_z - lo_z)) / 2 + s * lo_z

    def P(self, r, z):
        return QPointF(self.ox + self.s * r, self.oy - self.s * z)

    def _proj(self, q):
        """joint points of pose q projected onto the arm plane → [(r, z), ...]"""
        return [(T[0, 3] * self.u[0] + T[1, 3] * self.u[1], T[2, 3]) for T in forward_kinematics(q).frames]

    # ---------- drawing ----------
    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        ink = self.palette().color(self.foregroundRole())
        p.fillRect(self.rect(), self.palette().color(self.backgroundRole()))
        if self.d is None:
            p.setPen(GREY)
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Press Preview or Go to see whether the arm can reach this point, and why")
            return
        self._setup()
        d, s = self.d, self.s
        shoulder = self.P(0, config.D1)

        # floor + base
        p.setPen(QPen(GREY, 1.5))
        p.drawLine(self.P(-TIP_MAX, 0), self.P(TIP_MAX * 1.3, 0))
        p.fillRect(QRectF(self.P(-25, 14), self.P(25, 0)), QColor(150, 155, 160, 80))

        # wrist ring + farthest-tip circle
        ring = QPainterPath()
        ring.addEllipse(shoulder, REACH_MAX * s, REACH_MAX * s)
        hole = QPainterPath()
        hole.addEllipse(shoulder, REACH_MIN * s, REACH_MIN * s)
        p.fillPath(ring.subtracted(hole), QColor(46, 160, 120, 38))
        p.setPen(QPen(GREEN, 1))
        p.drawPath(ring.subtracted(hole))
        p.setPen(QPen(GREY, 1, Qt.PenStyle.DashLine))
        p.drawEllipse(shoulder, TIP_MAX * s, TIP_MAX * s)

        front = [o for o in d.options if not o.flip]
        g = front[0].geo                                   # wrist of the facing pose (elbow up/down share the same wrist)
        tip = self.P(math.hypot(d.x, d.y), d.z)
        wrist = self.P(g.rc, g.s + config.D1)
        wrist_ok = REACH_MIN - 1e-6 <= g.dist <= REACH_MAX + 1e-6

        # pose that hits a limit (requested φ fails but the angles solve)
        if not d.exact:
            blocked = next((o for o in front if o.geo.q is not None), None)
            if blocked:
                pts = self._proj(blocked.geo.q)
                p.setPen(QPen(RED, 3, Qt.PenStyle.DashLine))
                for a, b in zip(pts, pts[1:]):
                    p.drawLine(self.P(*a), self.P(*b))
                p.setPen(QPen(RED, 2.5))
                for k, (i, v, lo, hi) in enumerate(blocked.bad):
                    p.drawEllipse(self.P(*pts[JOINT_AT[i]]), 9, 9)
                    p.drawText(QPointF(10, 52 + 16 * k), f"○ limit: {config.JOINTS[i]['name']} needs {v:.1f}° "
                                                         f"but can do {lo:.1f}…{hi:.1f}° ({blocked.name})")

        # pose that will be used
        if d.q is not None:
            pts = self._proj(d.q)
            p.setPen(QPen(ORANGE, 6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            for a, b in zip(pts, pts[1:]):
                p.drawLine(self.P(*a), self.P(*b))
            p.setBrush(QBrush(self.palette().color(self.backgroundRole())))
            p.setPen(QPen(ORANGE, 2))
            for pt in pts[1:4]:
                p.drawEllipse(self.P(*pt), 4, 4)
            p.setBrush(Qt.BrushStyle.NoBrush)

        # target → wrist along the requested φ
        bad_phi = not d.exact
        p.setPen(QPen(RED if bad_phi else ORANGE, 2, Qt.PenStyle.DashLine if bad_phi else Qt.PenStyle.SolidLine))
        p.drawLine(tip, wrist)
        if not wrist_ok:                                   # wrist outside the ring: line from the shoulder + the gap
            p.setPen(QPen(RED, 1.5, Qt.PenStyle.DashLine))
            p.drawLine(shoulder, wrist)
            gap = g.dist - REACH_MAX if g.dist > REACH_MAX else REACH_MIN - g.dist
            word = "short" if g.dist > REACH_MAX else "too close"
            p.drawText(wrist + QPointF(-44, 36), f"wrist {gap:.1f} mm {word}")
        p.setPen(QPen(RED if not wrist_ok else GREEN, 2))
        p.drawEllipse(wrist, 5, 5)
        p.setPen(QPen(ORANGE, 2.5))
        p.drawEllipse(tip, 7, 7)

        # labels
        p.setPen(ink)
        p.drawText(tip + QPointF(10, -12), f"target φ {d.phi:.0f}°")
        p.drawText(wrist + QPointF(-44, 20), "wrist")
        p.setPen(GREY)
        p.drawText(QPointF(10, 18), "Side view (arm plane) · green ring = wrist can be here · dashed = farthest tip 285 mm")
        p.drawText(QPointF(10, 34), "orange = pose to be used · red = what cannot be reached")
