# Arm control window in the style of ws-control (PyQt6 + 3D PyVista) — but driving the servos directly via the adapter
#   (no ESP32), and every move is smooth (Mover/Driver) · RobotArm.bat  or  cd src && ../.venv/Scripts/python.exe qt_app.py
#
#  ┌ [Connect COM11] ● status ──────────────────────── [Torque OFF] [■ STOP (Space)] ┐
#  │ ┌──────── 3D ────────┐ ┌ tabs: Joint | Jog | Inverse Kinematics | Status ┐     │
#  │ └────────────────────┘ └──────────────────────────────────────────────────┘     │
#  │ Forward Kinematics (Target pose): X Y Z φ Yaw + T⁰₄ + singularity check          │
#  │ Log                                                                              │
#  └──────────────────────────────────────────────────────────────────────────────────┘
import math
import sys
import time
from datetime import datetime
from html import escape

import serial
from PyQt6.QtCore import QLocale, Qt, QTimer
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtWidgets import (QApplication, QCheckBox, QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout, QHeaderView,
                             QLabel, QMainWindow, QMessageBox, QPlainTextEdit, QSlider, QSplitter, QTableWidget,
                             QTableWidgetItem, QTabWidget, QTextEdit, QVBoxLayout, QWidget)

import config
import theme
from driver import Driver
from ik_explain import TIP_MAX, diagnose, fmt_ranges
from ik_sketch import IKSketch
from kinematics import check_limits, forward_kinematics
from link import DirectLink, SimLink
from motion import Mover, clamp_q
from singularity import analyze, report
from servo_calibrate import save_standby

USED = range(len(config.JOINTS))     # every joint (base, shoulder, elbow, wrist)
SLIDER_SCALE = 10      # sliders are int → ×10 for 0.1° resolution


def _spin(lo, hi, step, suffix, value=0.0, decimals=1):
    s = QDoubleSpinBox(decimals=decimals, minimum=lo, maximum=hi, singleStep=step, suffix=suffix)
    s.setValue(value)
    return s


def _ik_html(d):
    """IK result + reasons (ik_explain.Diagnosis) → HTML for the result box"""
    color = theme.OK if d.exact else (theme.WARN if d.q is not None else theme.BAD)
    out = [f"<p style='font-size:15px;font-weight:600;color:{color};margin:0'>{escape(d.headline)}</p>"]
    for title, items, c in (("Why", d.reasons, theme.BAD), ("Try", d.hints, theme.ACCENT), ("Careful", d.warnings, theme.WARN)):
        if items:
            out.append(f"<p style='margin:6px 0 0'><b style='color:{c}'>{title}</b></p><ul style='margin:0'>" +
                       "".join(f"<li>{escape(t)}</li>" for t in items) + "</ul>")
    rows = []
    for o in d.options:
        chosen = d.exact and o.ok and d.q is not None and all(abs(a - b) < 1e-6 for a, b in zip(o.geo.q, d.q))
        mark, c = ("★", theme.OK) if chosen else (("✓", theme.OK) if o.ok else ("✗", theme.BAD))
        q = "  ".join(f"{v:6.1f}" for v in o.geo.q) if o.geo.q is not None else "—"
        rows.append(f"<tr><td style='color:{c}'>{mark}</td><td>{o.name}</td><td style='font-family:Consolas'>{q}</td>"
                    f"<td style='color:{theme.BAD}'>{escape(o.why())}</td></tr>")
    out.append(f"<p style='margin:8px 0 2px'><b>4 poses at φ = {d.phi:.0f}°</b> (IK slides: θ₁ = Atan2 / π + Atan2 × elbow up/down"
               " · ★ = used · the program only uses the facing poses)</p>"
               "<table cellspacing='0' cellpadding='3'><tr><th></th><th align='left'>Pose</th>"
               "<th align='left'>q1 … q4 (°)</th><th align='left'>Why it cannot be used</th></tr>" + "".join(rows) + "</table>")
    out.append(f"<p style='margin:6px 0 0'>Reachable φ at this position: <b>{fmt_ranges(d.phi_ranges) or 'none'}</b>"
               f" · tip is {d.tip_dist:.1f} mm from the shoulder axis (max {TIP_MAX:.1f})</p>")
    if d.q is not None:
        out.append(f"<pre style='font-family:Consolas'>{escape(report(d.q))}</pre>")
    return "".join(out)


def _set_quiet(widget, value):
    """set a value without emitting signals (so it does not loop back into commanding the arm)"""
    widget.blockSignals(True)
    widget.setValue(value)
    widget.blockSignals(False)


class MainWindow(QMainWindow):
    def __init__(self, link=None, connect_fn=None, viewer=None, save_standby_fn=save_standby):
        super().__init__()
        self.connect_fn = connect_fn or (lambda: DirectLink(config.DIRECT_PORT))
        self.save_standby = save_standby_fn          # tests pass a fake so config.py is not written
        self.link, self.driver = None, None
        self.q_target = [0.0] * len(config.JOINTS)
        self.torque_on = True
        self.setWindowTitle("RobotArm 4DOF — Control GUI (USB adapter)")
        self.resize(1400, 900)

        if viewer is None:
            from viewer3d import Viewer3D            # slow import (VTK) — tests pass a fake viewer instead
            viewer = Viewer3D()
        self.viewer = viewer
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_joint_tab(), "Joint")
        self.tabs.addTab(self._build_jog_tab(), "Jog")
        self.tabs.addTab(self._build_ik_tab(), "Inverse Kinematics")
        self.tabs.addTab(self._build_status_tab(), "Status")
        self.tabs.setDocumentMode(True)
        self.log_view = QPlainTextEdit(readOnly=True, maximumBlockCount=500)
        self.log_view.setObjectName("log")
        self.log_view.setFont(theme.mono(9))
        view, vl = theme.card("3D view", "orange = Target · translucent blue = actual")
        vl.addWidget(self.viewer, 1)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(view)
        split.addWidget(self.tabs)
        split.setSizes([780, 620])
        log, ll = theme.card("Log")
        ll.addWidget(self.log_view)
        log.setMaximumHeight(170)

        central = theme.Backdrop()                  # พื้นหลัง (sci-fi: ตาราง HUD)
        lay = QVBoxLayout(central)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(10)
        lay.addWidget(self._build_toolbar())
        lay.addWidget(split, stretch=1)
        lay.addWidget(self._build_fk_box())
        lay.addWidget(log)
        self.setCentralWidget(central)
        QShortcut(QKeySequence("Space"), self, activated=self.stop)

        self._use_link(link or SimLink())
        self.set_target(self.q_target)
        self.log("Ready (Simulation mode) — press Connect to drive the arm via COM11, Space = STOP")
        self.timer = QTimer(self, interval=50)
        self.timer.timeout.connect(self.poll)
        self.timer.start()

    # ================= build the window =================
    def _build_toolbar(self):
        """header: name + port + connection status (left) · Standby / Torque / STOP (right)"""
        self.connect_btn = theme.button("Connect", "primary")
        self.connect_btn.clicked.connect(self.toggle_connect)
        self.conn_label = QLabel()
        self.torque_btn = theme.button("Torque OFF", tip="Turn torque off to move the arm by hand — the GUI pose follows the real arm")
        self.torque_btn.clicked.connect(lambda: self.set_torque(not self.torque_on))
        stop = theme.button("■  STOP  (Space)", "danger")
        stop.clicked.connect(self.stop)
        standby = theme.button("⌂ Standby", tip=f"Go to the Standby pose slowly ({config.STANDBY_V:.0f} °/s) — "
                                                "set the pose with the 📌 button in the Joint tab")
        standby.clicked.connect(self.go_standby)

        title = QVBoxLayout()
        title.setSpacing(0)
        title.addWidget(theme.glow(theme.label("RobotArm 4DOF", "app")))
        title.addWidget(theme.label(f"Port {config.DIRECT_PORT} · USB adapter → Feetech SCS15", "hint"))
        frame = theme.Card()
        bar = QHBoxLayout(frame)
        bar.setContentsMargins(16, 10, 12, 10)
        bar.setSpacing(10)
        bar.addLayout(title)
        bar.addSpacing(18)
        bar.addWidget(self.connect_btn)
        bar.addWidget(self.conn_label)
        bar.addStretch(1)
        self.clock = theme.label(theme.clock_text(), "clock")   # นาฬิกาแบบหน้าจอระบบ (เฉพาะ sci-fi)
        bar.addWidget(self.clock)
        bar.addSpacing(8)
        for w in (standby, self.torque_btn, stop):
            bar.addWidget(w)
        return frame

    @staticmethod
    def _joint_name(i):
        """joint name + its range (small, under the name)"""
        box = QVBoxLayout()
        box.setSpacing(0)
        box.addWidget(theme.label(config.JOINTS[i]["name"], "name"))
        box.addWidget(theme.label(f"{config.Q_MIN[i]:.1f} … {config.Q_MAX[i]:.1f}°", "range"))
        return box

    @staticmethod
    def _page(*widgets):
        """a tab page: cards stacked downwards"""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 10, 0, 0)
        lay.setSpacing(10)
        for x in widgets:
            lay.addWidget(x) if isinstance(x, QWidget) else lay.addLayout(x)
        lay.addStretch(1)
        return w

    def _build_joint_tab(self):
        frame, lay = theme.card("Target pose per joint", "drag a slider / type a number → the arm follows immediately")
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(10)
        for col, text in enumerate(("Joint", "", "Target", "Actual")):
            grid.addWidget(theme.label(text, "hint"), 0, col)
        self.rows = {}
        for r, i in enumerate(USED, start=1):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(config.Q_MIN[i] * SLIDER_SCALE), int(config.Q_MAX[i] * SLIDER_SCALE))
            spin = _spin(config.Q_MIN[i], config.Q_MAX[i], 1.0, " °")
            spin.setKeyboardTracking(False)             # applied after typing + Enter
            spin.setMinimumWidth(92)
            actual = theme.label("—", "actual")
            actual.setFont(theme.mono(10, bold=True))
            actual.setMinimumWidth(78)
            actual.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            grid.addLayout(self._joint_name(i), r, 0)
            for col, w in enumerate((slider, spin, actual), start=1):
                grid.addWidget(w, r, col)
            slider.valueChanged.connect(lambda v, i=i: self.on_joint(i, v / SLIDER_SCALE))
            spin.valueChanged.connect(lambda v, i=i: self.on_joint(i, v))
            self.rows[i] = (slider, spin, actual)
        grid.setColumnStretch(1, 1)
        lay.addLayout(grid)

        zero = theme.button("Zero (straight up)")
        zero.clicked.connect(lambda: (self.set_target([0.0] * len(config.JOINTS)), self.send()))
        pin = theme.button("📌 Set this pose as Standby", "ghost",
                           "Save the current Target pose into config.py — the next Connect moves the arm here by itself")
        pin.clicked.connect(self.set_standby_here)
        bar = QHBoxLayout()
        bar.addWidget(pin)
        bar.addStretch(1)
        bar.addWidget(zero)
        lay.addSpacing(4)
        lay.addLayout(bar)
        lay.addWidget(theme.label("In 3D: orange = Target · translucent blue = actual pose", "hint"))
        return self._page(frame)

    def _build_jog_tab(self):
        self.jog_buttons = {}

        def hold(grid, text, row, col, fn):
            b = theme.button(text, "jog")
            b.setAutoRepeat(False)
            b.pressed.connect(lambda: self.jog(fn))          # hold = speed up
            b.released.connect(self.jog_release)             # release = ease to a stop
            grid.addWidget(b, row, col)
            return b

        joints, jl = theme.card("Joints", "+ = lean forward / base turns left")
        grid = QGridLayout()
        grid.setColumnStretch(0, 1)
        for r, i in enumerate(USED):
            grid.addLayout(self._joint_name(i), r, 0)
            for col, d in ((1, -1), (2, +1)):
                self.jog_buttons[(i, d)] = hold(grid, "−" if d < 0 else "+", r, col,
                                                lambda m, i=i, d=d: m.jog_joint(i, d))
        jl.addLayout(grid)

        tip, tl = theme.card("Tip", "straight line, φ kept")
        grid = QGridLayout()
        grid.setColumnStretch(0, 1)
        for k, axis in enumerate("XYZ"):
            grid.addWidget(theme.label(f"{axis}  (mm)", "name"), k, 0)
            for col, d in ((1, -1), (2, +1)):
                self.jog_buttons[(axis, d)] = hold(grid, f"{axis}{'−' if d < 0 else '+'}", k, col,
                                                   lambda m, k=k, d=d: m.jog_xyz(k, d))
        tl.addLayout(grid)
        hint = theme.label("Hold = the arm keeps moving · release = smooth stop\n"
                           "Tip jog stops by itself before a singularity (tip above the base axis / arm fully stretched)\n"
                           "— the |det J| evidence is in the Singularity card below · moving away is always allowed", "hint")
        return self._page(joints, tip, hint)

    def jog(self, fn):
        if self._ready(self.driver.mover.q):
            self.driver.do(fn)

    def jog_release(self):
        self.driver.do(lambda m: m.release())

    def _build_ik_tab(self):
        reach = config.D1 + config.A2 + config.A3 + config.A4
        self.ik_x = _spin(-reach, reach, 5, " mm", 110)
        self.ik_y = _spin(-reach, reach, 5, " mm", 10)
        self.ik_z = _spin(-reach / 2, reach, 5, " mm", 350)
        self.ik_phi = _spin(-180, 180, 5, " °", 40)
        target, tl = theme.card("Tip target", "φ: 0° = straight ahead · −90° = down · +90° = up")
        form = QGridLayout()
        form.setHorizontalSpacing(10)
        for k, (name, w) in enumerate((("X", self.ik_x), ("Y", self.ik_y), ("Z", self.ik_z),
                                        ("φ tip angle", self.ik_phi))):
            form.addWidget(theme.label(name, "name"), k // 2, (k % 2) * 2)
            form.addWidget(w, k // 2, (k % 2) * 2 + 1)
        form.setColumnStretch(1, 1)
        form.setColumnStretch(3, 1)
        tl.addLayout(form)
        btns = QHBoxLayout()
        for text, kind, fn in (("Copy current pose", "ghost", self.ik_copy),
                               ("👁 Preview (no send)", None, lambda: self.ik_solve(False)),
                               ("▶ Go (send to arm)", "primary", lambda: self.ik_solve(True))):
            b = theme.button(text, kind)
            b.clicked.connect(fn)
            btns.addWidget(b)
        tl.addLayout(btns)
        tl.addWidget(theme.label("A 4-joint arm can only control φ (yaw is set by the X, Y position itself)", "hint"))

        self.ik_sketch = IKSketch()                  # side view: why reachable or not
        sketch, sl = theme.card("Reachable? — side view")
        sl.addWidget(self.ik_sketch, 1)
        self.ik_result = QTextEdit(readOnly=True)
        result, rl = theme.card("IK result")
        rl.addWidget(self.ik_result, 1)
        split = QSplitter(Qt.Orientation.Vertical)
        split.addWidget(sketch)
        split.addWidget(result)
        split.setSizes([300, 330])
        w = self._page(target, split)
        w.layout().takeAt(w.layout().count() - 1)    # no trailing stretch: the sketch + result fill the page
        return w

    def _build_status_tab(self):
        cols = ["Joint", "Online", "Raw", "Actual °", "Error °", "Temp °C", "Load %"]
        self.status_table = QTableWidget(len(USED), len(cols))
        self.status_table.setHorizontalHeaderLabels(cols)
        self.status_table.verticalHeader().setVisible(False)
        self.status_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.status_table.setAlternatingRowColors(True)
        self.status_table.setShowGrid(False)
        self.status_table.setFont(theme.mono(10))
        self.status_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for r, i in enumerate(USED):
            self.status_table.setRowHeight(r, 32)
            self.status_table.setItem(r, 0, QTableWidgetItem(config.JOINTS[i]["name"]))
        self.status_table.setFixedHeight(32 * len(USED) + 36)
        self.summary = theme.label("", "name")
        self.correct_box = QCheckBox("Correct sag after arriving")
        self.correct_box.setChecked(True)
        self.correct_box.toggled.connect(lambda on: setattr(self.driver, "correct", on))
        box, bl = theme.card("Servo")
        for x in (self.summary, self.status_table, self.correct_box,
                  theme.label("Error = Target − actual pose · after arriving the program corrects sag automatically (≤ ±5°)", "hint")):
            bl.addWidget(x)
        return self._page(box)

    def _build_fk_box(self):
        """bottom row: tip position (FK) · matrix T⁰₄ · singularity check — of the Target pose"""
        pos, pl = theme.card("Tip (FK of the Target pose)")
        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        self.fk_values = {}
        for k, (name, unit) in enumerate([("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("φ", "°"), ("Yaw", "°")]):
            val = theme.glow(theme.label("0.0", "tile-value"), radius=14)
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            val.setMinimumWidth(84)
            grid.addWidget(theme.label(f"{name}  ({unit})", "tile-name"), 0, k, Qt.AlignmentFlag.AlignRight)
            grid.addWidget(val, 1, k)
            self.fk_values[name] = val
        pl.addLayout(grid)
        pl.addStretch(1)

        mat, ml = theme.card("T⁰₄", "tip pose relative to the base")
        self.fk_table = QTableWidget(4, 4)
        self.fk_table.setFont(theme.mono(10))
        self.fk_table.horizontalHeader().setVisible(False)
        self.fk_table.verticalHeader().setVisible(False)
        self.fk_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.fk_table.setShowGrid(False)
        self.fk_table.setFixedSize(4 * 70 + 4, 4 * 23 + 4)
        self.fk_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.fk_table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        for c in range(4):
            self.fk_table.setColumnWidth(c, 70)
            self.fk_table.setRowHeight(c, 23)
        ml.addWidget(self.fk_table)

        sing, sl = theme.card("Singularity")
        self.sing_label = QLabel()                    # singularity check of the Target pose + evidence (singularity.py)
        self.sing_label.setFont(theme.mono(9))
        self.sing_label.setToolTip("|det J| = r·a2·a3·|sin θ3| — computed two ways (numerical/formula), they must match\n"
                                   "= 0 when the tip is on the base axis (r = 0) or the arm is fully stretched/folded (θ3 = 0°/180°)")
        sl.addWidget(self.sing_label)
        sl.addStretch(1)

        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)
        lay.addWidget(pos, 3)
        lay.addWidget(mat, 0)
        lay.addWidget(sing, 2)
        return w

    # ================= Target pose =================
    def set_target(self, q):
        """set the pose to command → update sliders/spin boxes, FK, 3D view (does not send)"""
        # joints that cannot be read (nan — servo not answering) keep their value: a nan slider value used to crash Connect
        self.q_target = clamp_q([old if v is None or math.isnan(v) else float(v) for old, v in zip(self.q_target, q)])
        for i, (slider, spin, _a) in self.rows.items():
            _set_quiet(slider, int(round(self.q_target[i] * SLIDER_SCALE)))
            _set_quiet(spin, self.q_target[i])
        fk = forward_kinematics(self.q_target)
        yaw = math.degrees(math.atan2(fk.y, fk.x)) if math.hypot(fk.x, fk.y) > 1e-6 else self.q_target[0]
        for name, v in (("X", fk.x), ("Y", fk.y), ("Z", fk.z), ("φ", fk.phi), ("Yaw", yaw)):
            self.fk_values[name].setText(f"{v:.1f}")
        for r in range(4):
            for c in range(4):
                item = QTableWidgetItem(f"{fk.T[r, c]:.1f}" if c == 3 else f"{fk.T[r, c]:.3f}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.fk_table.setItem(r, c, item)
        warn = bool(analyze(self.q_target)["warnings"])
        self.sing_label.setText(report(self.q_target))
        self.sing_label.setStyleSheet("color:#e63946; font-weight:bold" if warn else f"color:{theme.OK}")
        self.viewer.show_target(self.q_target)

    def on_joint(self, i, deg):
        q = list(self.q_target)
        q[i] = deg
        self.set_target(q)
        if self._ready():                              # sliders drive the arm immediately (no Send needed)
            target = list(self.q_target)
            self.driver.do(lambda m: m.follow(target))

    # ================= command the arm =================
    def _ready(self, q=None):
        """pre-command check — True if it may be sent (otherwise the reason goes to the log)"""
        why = check_limits(q or self.q_target)
        if not why and not self.torque_on:
            why = "Torque is off — press Torque ON first"
        if not why and not self.link.online():
            why = "Arm offline: servos not answering"
        if not why and not self.link.simulated and not (self.link.volt or 0) >= config.VOLT_MIN:
            why = f"Servo voltage low ({self.link.volt} V < {config.VOLT_MIN} V) — not sent"
        if why:
            self.log("❌ " + why)
        return not why

    def send(self):
        if self._ready():
            target = list(self.q_target)
            self.driver.do(lambda m: m.goto_q(target))
            self.log("▶ Send: " + "  ".join(f"{target[i]:.1f}" for i in USED))

    def stop(self):
        self.driver.emergency_stop()
        self.set_target(self.driver.mover.q)
        self.log("■ STOP — the arm holds at its actual position")

    def set_torque(self, on):
        self.driver.emergency_stop()
        self.link.torque(on)
        self.torque_on = on
        self.torque_btn.setText("Torque OFF" if on else "Torque ON")
        if on and self.link.q_actual is not None:
            self.driver.do(lambda m: m.reset(self.link.q_actual))
            self.set_target(self.link.q_actual)
        self.log("Torque ON — the arm holds this pose" if on
                 else "Torque OFF — the arm is limp, support it (the GUI pose follows the real arm)")

    # ================= IK =================
    def ik_copy(self):
        fk = forward_kinematics(self.q_target)
        for w, v in ((self.ik_x, fk.x), (self.ik_y, fk.y), (self.ik_z, fk.z), (self.ik_phi, fk.phi)):
            w.setValue(float(v))

    def ik_solve(self, send):
        x, y, z, phi = self.ik_x.value(), self.ik_y.value(), self.ik_z.value(), self.ik_phi.value()
        self.viewer.show_ik_marker((x, y, z))
        d = diagnose(x, y, z, phi, theta1_prev=self.q_target[0])     # reachable? + reasons (ik_explain.py)
        self.ik_sketch.set_diagnosis(d)
        self.ik_result.setHtml(_ik_html(d))
        best = d.q
        if best is None:
            self.log(f"❌ IK ({x:.0f}, {y:.0f}, {z:.0f}) φ {phi:.0f}°: " + d.reasons[0])
            return
        if not d.exact:
            self.log(d.headline + " — " + d.reasons[0])
        for w in d.warnings:
            self.log("⚠ " + w)
        if analyze(best)["warnings"]:
            self.log(f"⚠ Point ({x:.0f}, {y:.0f}, {z:.0f}) is near a singularity — see the evidence in the IK tab")
        self.set_target(best)
        if send:
            self.send()

    # ================= connection =================
    def _use_link(self, link):
        if self.driver:
            self.driver.close()
        if self.link:
            self.link.close()
        self.link = link
        q0 = link.q_actual if link.q_actual is not None else [0.0] * len(config.JOINTS)
        self.driver = Driver(link, Mover(q0))
        self.driver.correct = self.correct_box.isChecked()
        self.driver.start()
        self.torque_on = True
        self.torque_btn.setText("Torque OFF")
        self.connect_btn.setText("Connect" if link.simulated else "Disconnect")
        self._label("● Simulation" if link.simulated else "● Connected", theme.MUTED if link.simulated else theme.OK)

    def startup(self):
        """program start: connect to the arm by itself → go to Standby (cannot connect → the simulated arm goes to Standby)"""
        self.log(f"Starting: connecting to the arm on {config.DIRECT_PORT}, then going to Standby ...")
        self.toggle_connect()                        # connected → go_standby() already called inside
        if self.link.simulated:
            self.log("Could not connect → showing the simulated arm at Standby instead (press Connect to retry)")
            self.go_standby()

    def toggle_connect(self):
        if not self.link.simulated:
            self._use_link(SimLink())
            self.log("Disconnected — back to Simulation mode (the arm holds its last pose)")
            return
        try:
            link = self.connect_fn()
        except (serial.SerialException, OSError) as e:
            self._label("● Cannot connect", theme.BAD, str(e))
            self.log(f"❌ Cannot open {config.DIRECT_PORT} (adapter plugged in? another program using the port?): {e}")
            return
        end = time.monotonic() + 2.0
        while link.q_actual is None and time.monotonic() < end:       # wait for the first reading from the servos
            QApplication.processEvents()
            time.sleep(0.02)
        if link.q_actual is None:
            link.close()
            self._label("● Servos not answering", theme.BAD)
            self.log("❌ The port opened but no servo answers — servo power on at 7.4 V? jumper on B?")
            return
        self._use_link(link)
        self.set_target(link.q_actual)
        self.log(f"Connected to {config.DIRECT_PORT} — the arm holds in place, Target = actual pose")
        self.go_standby()                            # every start → go to Standby by itself (slowly, after voltage/servo checks)

    # ================= Standby pose =================
    def go_standby(self):
        """move to the Standby pose at STANDBY_V (slow) — not if voltage is low / a servo is missing / torque is off"""
        q = config.STANDBY_Q
        act = self.link.q_actual
        if act is None or any(math.isnan(act[i]) for i in USED):
            self.log("❌ Not going to Standby: not every servo is answering")
            return False
        if not self._ready(q):
            return False
        self.set_target(q)
        self.driver.do(lambda m: m.goto_q(q, v=config.STANDBY_V))
        self.log(f"⌂ Going to Standby ({config.STANDBY_V:.0f} °/s): "
                 + "  ".join(f"{q[i]:.1f}" for i in USED) + " — press STOP / Space to stop")
        return True

    def set_standby_here(self):
        """save the current Target pose as the Standby pose (writes config.py + takes effect now)"""
        q = list(self.q_target)
        text = "  ".join(f"{config.JOINTS[i]['name']} {q[i]:.1f}°" for i in USED)
        if QMessageBox.question(self, "Set Standby pose", f"{text}\n\nUse this as the Standby pose?") \
                != QMessageBox.StandardButton.Yes:
            return
        try:
            self.save_standby(q)
        except ValueError as e:
            self.log(f"❌ Standby pose not saved: {e}")
            return
        config.STANDBY_Q = [round(v, 1) for v in q]
        self.log("📌 Standby pose saved: " + text)

    def _label(self, text, color, tip=""):
        self.conn_label.setText(text)
        self.conn_label.setToolTip(tip)
        self.conn_label.setStyleSheet(theme.pill(color))

    # ================= refresh every 50 ms =================
    def poll(self):
        link, m = self.link, self.driver.mover
        self.clock.setText(theme.clock_text())
        if self.driver.message:
            self.log(self.driver.message)
            self.driver.message = ""
        if m.mode in ("jog_joint", "jog_xyz"):                      # jog: Target/sliders follow the moving pose
            self.set_target(m.q)
        q_act = link.q_actual if not link.simulated else m.q        # simulation: show the moving pose
        if not self.torque_on and link.q_actual is not None:          # torque off: Target follows the real arm
            self.set_target(link.q_actual)
        self.viewer.show_actual(q_act)
        for r, i in enumerate(USED):
            a = None if q_act is None or math.isnan(q_act[i]) else q_act[i]
            self.rows[i][2].setText("—" if a is None else f"{a:6.1f}°")
            s = getattr(link, "servo", {}).get(config.JOINTS[i]["id"])
            cells = ["✅" if (a is not None and (link.simulated or s)) else "❌",
                     str(s["raw"]) if s else "-",
                     "-" if a is None else f"{a:.1f}",
                     "-" if a is None or m.busy else f"{self.q_target[i] - a:+.2f}",
                     str(s["temp"]) if s else "-",
                     f"{s['load']:+.1f}" if s else "-"]
            for c, text in enumerate(cells, start=1):
                self.status_table.setItem(r, c, QTableWidgetItem(text))
        if link.simulated:
            self.summary.setText("Simulation mode (no arm connected)")
        else:
            v = link.volt
            low = v is None or v < config.VOLT_MIN
            self.summary.setText(f"Supply {v if v is None else round(v, 1)} V   •   Torque: "
                                 f"{'ON' if self.torque_on else 'OFF (can be moved by hand)'}   •   "
                                 f"{'offline' if not link.online() else 'moving' if m.busy else 'still'}")
            if not link.online():
                self._label("● Offline (servos not answering)", theme.BAD)
            elif low:
                self._label(f"● Connected — servo voltage low {v} V (needs ≥ {config.VOLT_MIN:.0f} V)", theme.WARN)
            else:
                self._label(f"● Connected  {v:.1f} V", theme.OK)

    def log(self, text):
        self.log_view.appendPlainText(f"[{datetime.now():%H:%M:%S}] {text}")

    def closeEvent(self, event):
        self.timer.stop()
        self.driver.close()
        self.link.close()
        self.viewer.close()
        super().closeEvent(event)


def main():
    # On a Windows machine with a Thai region format, Qt shows Thai digits in spin boxes → force Western digits
    QLocale.setDefault(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))
    app = QApplication(sys.argv)
    theme.apply(app)
    w = MainWindow()
    w.show()
    QTimer.singleShot(300, w.startup)                # wait for the window (STOP visible), then connect + go to Standby
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
